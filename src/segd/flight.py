"""6-DoF MuJoCo model, cascaded control, and complete geometric mission.

Ground truth and noisy-state modes are control experiments, NOT navigation.
"""

import csv
import hashlib
import json
import platform
import time
from pathlib import Path
from collections import deque
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[2]
COLLISION = json.loads((ROOT / "configs/collision.json").read_text())


def collision_clearance(p, R, ceiling=True):
    """Rotated cylinder clearance to aisle walls, floor and optional ceiling."""
    axis = R[:, 2]
    centre = p + COLLISION["centre_z_m"] * axis
    extent = COLLISION["radius_m"] * np.sqrt(np.maximum(0.0, 1 - axis**2)) + COLLISION[
        "half_height_m"
    ] * np.abs(axis)
    distances = [1.25 - abs(centre[1]) - extent[1], centre[2] - extent[2]]
    if ceiling:
        distances.append(8 - centre[2] - extent[2])
    return float(min(distances))


def load_config(path=None):
    c = json.loads(Path(path or ROOT / "configs/default.json").read_text())
    assert 0 < c["mass_kg"] < 3
    assert c["speed_mps"] > 0 and c["physics_dt"] > 0
    assert (
        abs(
            c["control_dt"] / c["physics_dt"] - round(c["control_dt"] / c["physics_dt"])
        )
        < 1e-8
    )
    assert 0 < c["usable_fraction"] <= 1 and 0 < c["reserve_fraction"] < 1
    return c


def create_model(c):
    # Shelves are outside the 2.5 m clear aisle. End regions are explicit assumptions.
    shelves = []
    for side in (-1, 1):
        for k in range(6):
            shelves.append(
                f'<geom type="box" pos="40 {side * 1.65} {k * 1.6}" size="40 .4 .035" rgba=".3 .4 .5 1"/>'
            )
        for x in np.arange(0, 80.1, 4):
            shelves.append(
                f'<geom type="box" pos="{x} {side * 1.3} 4" size=".035 .035 4" rgba=".4 .4 .4 1"/>'
            )
        for layer in range(5):
            for i in range(40):
                x = 1 + 2 * i
                shelves.append(
                    f'<geom type="box" pos="{x} {side * 1.65} {layer * 1.6 + 0.7}" size=".65 .35 .5" rgba=".65 .48 .27 1"/>'
                )
    rotor_sites = "".join(
        f'<site name="r{i}" pos="{x} {y} 0" size=".025"/>'
        for i, (x, y) in enumerate([(0.2, 0.2), (-0.2, 0.2), (-0.2, -0.2), (0.2, -0.2)])
    )
    motors = "".join(
        f'<general site="r{i}" gear="0 0 1 0 0 {s * 0.018}" ctrllimited="true" ctrlrange="0 {c["max_motor_thrust_n"]}"/>'
        for i, s in enumerate([1, -1, 1, -1])
    )
    xml = f'''<mujoco model="warehouse"><option timestep="{c["physics_dt"]}" gravity="0 0 -9.81" integrator="RK4"/>
    <visual><global offwidth="960" offheight="540"/></visual>
    <worldbody><light pos="0 0 10"/><geom name="floor" type="plane" size="100 10 .1" rgba=".8 .8 .8 1"/>
    {"".join(shelves)}
    <body name="uav" pos="-2 0 .15"><freejoint/>
    <inertial pos="0 0 0" mass="{c["mass_kg"]}" diaginertia=".055 .055 .09"/>
    <geom name="guard" type="cylinder" pos="0 0 {COLLISION["centre_z_m"]}" size="{COLLISION["radius_m"]} {COLLISION["half_height_m"]}" rgba=".12 .3 .7 1"/>
    {rotor_sites}<site name="imu"/>
    <camera name="left" pos="0 .1 0" xyaxes="1 0 0 0 0 1" fovy="45"/>
    <camera name="right" pos="0 -.1 0" xyaxes="-1 0 0 0 0 1" fovy="45"/>
    </body></worldbody><actuator>{motors}</actuator>
    <sensor><accelerometer site="imu"/><gyro site="imu"/></sensor></mujoco>'''
    if c.get("visual_assets", False):
        from .visuals import decorate

        xml = decorate(xml)
    return mujoco.MjModel.from_xml_string(xml), xml


class Route:
    """Trapezoidal stop-to-stop segments with explicit takeoff and landing."""

    def __init__(self, speed=0.8, short=False):
        start = np.array([-2.0, 0.0, 0.15])
        targets = [([-2, 0, 0.8], "takeoff")]
        if short:
            targets += [([4, 0, 0.8], "scan"), ([-2, 0, 0.8], "return")]
        else:
            targets += [([0, 0, 0.8], "approach")]
            for layer in range(5):
                z = 0.8 + 1.6 * layer
                if layer:
                    targets.append(([0 if layer % 2 == 0 else 80, 0, z], "climb"))
                targets.append(([80 if layer % 2 == 0 else 0, 0, z], "scan"))
            targets += [
                ([0, 0, 7.2], "return"),
                ([-2, 0, 7.2], "return"),
                ([-2, 0, 0.8], "descend"),
            ]
        targets += [([-2, 0, 0.15], "land")]
        self.segments = []
        t = 0.0
        self.distance = 0.0
        for p, state in targets:
            p = np.asarray(p, float)
            distance = np.linalg.norm(p - start)
            # max derivative of quintic is 1.875; bound velocity and acceleration.
            vmax = (
                min(speed, 0.35)
                if state in ("climb", "takeoff", "descend", "land")
                else speed
            )
            ramp = min(vmax / 0.6, np.sqrt(distance / 0.6))
            cruise = max(0.0, distance / (0.6 * ramp) - ramp)
            duration = 2 * ramp + cruise
            self.segments.append((t, duration, start.copy(), p.copy(), state, ramp))
            t += duration
            self.distance += distance
            start = p
        self.duration = t

    def at(self, t):
        for t0, duration, a, b, name, ramp in self.segments:
            if t <= t0 + duration:
                tau = np.clip(t - t0, 0, duration)
                length = np.linalg.norm(b - a)
                direction = (b - a) / length
                peak = 0.6 * ramp
                if tau < ramp:
                    distance = 0.3 * tau * tau
                    vel = 0.6 * tau
                    accel = 0.6
                elif tau < duration - ramp:
                    distance = 0.3 * ramp * ramp + peak * (tau - ramp)
                    vel = peak
                    accel = 0.0
                else:
                    remaining = duration - tau
                    distance = length - 0.3 * remaining * remaining
                    vel = 0.6 * remaining
                    accel = -0.6
                return (
                    a + direction * distance,
                    direction * vel,
                    direction * accel,
                    name,
                )
        return self.segments[-1][3].copy(), np.zeros(3), np.zeros(3), "done"


class Controller:
    def __init__(self, c):
        self.c = c
        self.integral = np.zeros(3)
        self.motor = np.full(4, c["mass_kg"] * 9.81 / 4)
        self.allocation = np.array(
            [
                [1, 1, 1, 1],
                [0.2, 0.2, -0.2, -0.2],
                [-0.2, 0.2, 0.2, -0.2],
                [0.018, -0.018, 0.018, -0.018],
            ]
        )
        self.inv = np.linalg.inv(self.allocation)

    def command(self, p, v, R, omega, ref, residual=None):
        dt = self.c["control_dt"]
        pr, vr, ar = ref[:3]
        self.integral = np.clip(self.integral + (pr - p) * dt, -0.4, 0.4)
        acc = (
            ar
            + np.array([2.5, 2.5, 4.0]) * (pr - p)
            + np.array([2.6, 2.6, 3.2]) * (vr - v)
            + 0.25 * self.integral
        )
        if residual is not None:
            acc += np.clip(residual, -1, 1)
        acc[:2] = np.clip(acc[:2], -2, 2)
        acc[2] = np.clip(acc[2], -3, 3)
        force = self.c["mass_kg"] * (acc + np.array([0, 0, 9.81]))
        z = force / np.linalg.norm(force)
        y = np.cross(z, [1, 0, 0])
        y /= np.linalg.norm(y)
        desired = np.column_stack((np.cross(y, z), y, z))
        E = 0.5 * (desired.T @ R - R.T @ desired)
        e = np.array([E[2, 1], E[0, 2], E[1, 0]])
        torque = -np.array([3.5, 3.5, 1.5]) * e - np.array([0.65, 0.65, 0.45]) * omega
        wrench = np.r_[max(0, force @ R[:, 2]), torque]
        target = np.clip(self.inv @ wrench, 0, self.c["max_motor_thrust_n"])
        return target

    def advance_motor(self, target):
        self.motor += (1 - np.exp(-self.c["physics_dt"] / self.c["motor_tau_s"])) * (
            target - self.motor
        )
        return self.motor


def state(data):
    R = data.xmat[1].reshape(3, 3).copy()
    return data.qpos[:3].copy(), data.qvel[:3].copy(), R, data.qvel[3:6].copy()


def power(c, thrust):
    # shortcut: uncalibrated induced-power scaling; replace with motor/prop bench measurements before hardware claims.
    ratio = np.sum(np.maximum(thrust, 0) ** 1.5) / (4 * (2.65 * 9.81 / 4) ** 1.5)
    return c["hover_power_w"] * ratio + c["hotel_power_w"]


def provenance(c):
    import importlib.metadata as md

    files = sorted((ROOT / "src").rglob("*.py"))
    return {
        "config": c,
        "python": platform.python_version(),
        "host": platform.node(),
        "source_sha256": hashlib.sha256(
            b"".join(p.read_bytes() for p in files)
        ).hexdigest(),
        "versions": {p: md.version(p) for p in ["mujoco", "numpy", "scipy"]},
    }


def snapshot_source(out):
    import zipfile

    with zipfile.ZipFile(
        Path(out) / "source_snapshot.zip", "w", compression=zipfile.ZIP_DEFLATED
    ) as archive:
        paths = (
            list((ROOT / "src").rglob("*.py"))
            + list((ROOT / "configs").glob("*.json"))
            + list((ROOT / "configs").glob("*.npz"))
            + [ROOT / "pyproject.toml", ROOT / "requirements.lock.txt"]
        )
        for path in paths:
            if path.exists():
                archive.write(path, path.relative_to(ROOT))


def simulate(c, out, short=False, video=False, policy=None):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    snapshot_source(out)
    metadata = provenance(c)
    if policy:
        artifact = Path(policy)
        if not artifact.exists():
            artifact = Path(str(policy) + ".zip")
        metadata["policy"] = {
            "path": str(artifact),
            "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
        }
    (out / "provenance.json").write_text(json.dumps(metadata, indent=2))
    model, xml = create_model(c)
    data = mujoco.MjData(model)
    (out / "scene.xml").write_text(xml)
    mujoco.mj_forward(model, data)
    route = Route(c["speed_mps"], short)
    ctrl = Controller(c)
    rng = np.random.default_rng(c["seed"])
    queue = deque(maxlen=6)
    rows = []
    energy = 0.0
    collision = False
    reason = "completed"
    minimum = np.inf
    renderer = writer = None
    if video:
        import imageio.v2 as imageio

        renderer = mujoco.Renderer(model, height=540, width=960)
        writer = imageio.get_writer(out / "flight.mp4", fps=15)
    cameras = inventory = None
    last_capture = None
    if c.get("inventory", False):
        if not c.get("visual_assets", False):
            raise ValueError("Scene inventory requires visual_assets")
        from .barcodes import SceneCameras, EvidenceInventory

        cameras = SceneCameras(model)
        inventory = EvidenceInventory(out / "evidence")
    nav = lidar = stereo = None
    nav_rows = []
    nav_failed = 0
    if c["state_source"] == "lidar":
        from .navigation import Navigator, SimulatedLidar

        nav = Navigator([-2, 0, 0.15])
        lidar = SimulatedLidar(model, c["seed"])
        data.ctrl[:] = ctrl.motor
        mujoco.mj_forward(model, data)
        nav.update(lidar.read(data))
        if c.get("stereo_odometry", False):
            from .navigation import StereoOdometry

            stereo = StereoOdometry(model)
            stereo.update(data, nav)
    started = time.monotonic()
    target = ctrl.motor.copy()
    policy_model = None
    if policy:
        from stable_baselines3 import PPO

        policy_model = PPO.load(policy, device="cpu")
    try:
        for k in range(int((route.duration + 2) / c["control_dt"])):
            t = data.time
            p, v, R, w = state(data)
            queue.append((p.copy(), v.copy()))
            if nav is not None:
                if k % 5 == 0:
                    cloud = lidar.read(data)
                    nav.update_height(lidar.downward_range)
                    valid = nav.update(cloud)
                    nav_failed = 0 if valid else nav_failed + 1
                if stereo is not None and k % 10 == 0:
                    if stereo.update(data, nav):
                        nav_failed = 0
                        if np.linalg.norm(nav.p - nav.last_key) > 0.35:
                            nav._map(cloud @ nav.R.T + nav.p)
                p_est, v_est = nav.p.copy(), nav.v.copy()
                R_est, w_est = nav.R.copy(), data.sensordata[3:6].copy() - nav.bg
            elif c["state_source"] == "proxy":
                p_est, v_est = queue[0]
                p_est = p_est + rng.normal(0, 0.015, 3)
                v_est = v_est + rng.normal(0, 0.025, 3)
            elif c["state_source"] == "truth":
                p_est, v_est = p, v
            else:
                raise ValueError("state_source must be truth, proxy or lidar")
            if nav is None:
                R_est, w_est = R, w
            ref = route.at(t)
            if (
                cameras is not None
                and ref[3] == "scan"
                and (
                    last_capture is None or np.linalg.norm(p_est - last_capture) > 0.55
                )
            ):
                inventory.update(cameras.capture(data, v_est[0]), p_est, t)
                last_capture = p_est.copy()
            residual = None
            if policy_model is not None:
                obs = observation(p_est, v_est, R_est, w_est, ref, ctrl.motor, c)
                residual = policy_model.predict(obs, deterministic=True)[0]
            target = ctrl.command(p_est, v_est, R_est, w_est, ref, residual)
            wind = np.array([0, c["wind_n"] * np.sin(0.7 * t), 0])
            data.xfrc_applied[1, :3] = wind - 0.15 * v
            for j in range(round(c["control_dt"] / c["physics_dt"])):
                data.ctrl[:] = ctrl.advance_motor(target)
                mujoco.mj_step(model, data)
                if nav is not None:
                    acc = (
                        data.sensordata[:3]
                        + np.array([0.01, -0.008, 0.006])
                        + rng.normal(0, 0.012, 3)
                    )
                    gyro = (
                        data.sensordata[3:6]
                        + np.array([0.0003, -0.0002, 0.0002])
                        + rng.normal(0, 0.001, 3)
                    )
                    nav.propagate(acc, gyro, c["physics_dt"])
                energy += power(c, data.ctrl) * c["physics_dt"] / 3600
                for contact in data.contact:
                    if contact.dist < -0.002:
                        # Ground contact is expected only at the landing pad near launch/landing.
                        ground = contact.geom1 == 0 or contact.geom2 == 0
                        if not (
                            ground
                            and p[0] < -1
                            and ref[3] in ("takeoff", "land", "done")
                        ):
                            collision = True
            p, v, R, w = state(data)
            clearance = collision_clearance(p, R, ceiling=0 <= p[0] <= 80)
            minimum = min(minimum, clearance)
            logged_ref = route.at(data.time)
            rows.append(
                [
                    data.time,
                    *p,
                    *logged_ref[0],
                    *v,
                    *data.ctrl,
                    energy,
                    clearance,
                    logged_ref[3],
                    *data.qpos[3:7],
                ]
            )
            if renderer and k % max(1, round(1 / (15 * c["control_dt"]))) == 0:
                cam = mujoco.MjvCamera()
                cam.lookat[:] = p
                cam.distance = 3.0
                cam.azimuth = 180
                cam.elevation = -15
                renderer.update_scene(data, camera=cam)
                frame = renderer.render()
                import cv2

                cv2.putText(
                    frame,
                    f"CONTROL ONLY | t={data.time:.1f}s | {ref[3]}",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1,
                )
                writer.append_data(frame)
            if nav is not None:
                nav_rows.append(
                    [
                        data.time,
                        *nav.p,
                        *p,
                        float(np.linalg.norm(nav.p - p)),
                        nav.quality["valid"],
                        nav.quality["rmse"],
                    ]
                )
            if nav_failed >= 20:
                reason = "navigation_degraded"
                break
            if collision:
                reason = "collision"
                break
            if (
                not np.all(np.isfinite(data.qpos))
                or collision_clearance(p, R, ceiling=0 <= p[0] <= 80) < 0
            ):
                reason = "unsafe_state"
                break
            available = c["battery_wh"] * c["usable_fraction"]
            if energy > available * (1 - c["reserve_fraction"]):
                reason = "reserve_threshold"
                break
    finally:
        if writer:
            writer.close()
        if renderer:
            renderer.close()
        if cameras:
            cameras.close()
        if stereo:
            stereo.close()
    header = [
        "t",
        "x",
        "y",
        "z",
        "rx",
        "ry",
        "rz",
        "vx",
        "vy",
        "vz",
        "f0",
        "f1",
        "f2",
        "f3",
        "energy_wh",
        "clearance_m",
        "phase",
        "qw",
        "qx",
        "qy",
        "qz",
    ]
    with (out / "trajectory.csv").open("w") as f:
        writer_csv = csv.writer(f)
        writer_csv.writerow(header)
        writer_csv.writerows(rows)
    arr = np.array([r[:16] for r in rows], float)
    errors = np.linalg.norm(arr[:, 1:4] - arr[:, 4:7], axis=1)
    actual_distance = float(np.linalg.norm(np.diff(arr[:, 1:4], axis=0), axis=1).sum())
    result = {
        "experiment": "control_only_" + c["state_source"],
        "mission_inventory_success": False,
        "reason": reason,
        "route_completed": reason == "completed",
        "collision": collision,
        "duration_s": float(data.time),
        "planned_duration_s": route.duration,
        "reference_distance_m": route.distance,
        "actual_distance_m": actual_distance,
        "position_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "position_p95_m": float(np.quantile(errors, 0.95)),
        "minimum_clearance_m": float(minimum),
        "energy_wh": energy,
        "remaining_usable_fraction": 1 - energy / available,
        "wall_time_s": time.monotonic() - started,
        "inventory_note": "No image-decoded full mission inventory; route completion is not mission success.",
    }
    if nav is not None:
        with (out / "navigation.csv").open("w") as f:
            cw = csv.writer(f)
            cw.writerow(
                [
                    "t",
                    "est_x",
                    "est_y",
                    "est_z",
                    "truth_x",
                    "truth_y",
                    "truth_z",
                    "position_error_m",
                    "valid",
                    "scan_rmse",
                ]
            )
            cw.writerows(nav_rows)
        np.save(out / "local_map.npy", nav.points)
        result["navigation_ate_rmse_m"] = float(
            np.sqrt(np.mean([r[7] ** 2 for r in nav_rows]))
        )
        if stereo is not None:
            result["stereo_valid_updates"] = stereo.valid_updates
            result["stereo_rejected_updates"] = stereo.rejected_updates
        result["navigation_method"] = (
            "IMU error-state filter + online point-to-plane LiDAR submap + downward range on level floor"
            + (" + stereo triangulation/PnP" if stereo else "")
            + "; no loop closure"
        )
        result["experiment"] = "sensor_estimated_route_lidar"
    if inventory is not None:
        inventory.export(out)
        # Truth inventory is read only here by the scorer, after the flight has ended.
        expected = {
            row["id"]
            for row in json.loads(
                (ROOT / "assets/generated/inventory_truth.json").read_text()
            )
        }
        found = set(inventory.records)
        result["expected_items"] = len(expected)
        result["read_items"] = len(expected & found)
        result["false_items"] = len(found - expected)
        result["unresolved_items"] = sorted(expected - found)
        result["inventory_read_fraction"] = len(expected & found) / len(expected)
        result["sensor_frames_processed"] = inventory.frames
        result["mission_inventory_success"] = bool(
            nav is not None
            and result["route_completed"]
            and expected <= found
            and not found - expected
        )
        result["inventory_note"] = (
            "Scene-rendered images decoded by zxing-cpp; geometry/dimensions prescribed; no exploration or loop closure."
        )
    (out / "metrics.json").write_text(json.dumps(result, indent=2))
    return result


def observation(p, v, R, w, ref, thrust, c):
    # Both proxy training and full-flight policy use this exact observation layout.
    return np.r_[
        ref[0] - p, ref[1] - v, R.ravel(), w, ref[2], thrust / c["max_motor_thrust_n"]
    ].astype(np.float32)
