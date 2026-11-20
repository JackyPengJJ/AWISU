"""Separate, current-limited rigid-body test of the dimensioned 18 V candidate.

Known reference and delayed position/velocity proxy; no navigation or scan claim.
"""

import json
import math
from pathlib import Path
import numpy as np
import mujoco
from .candidate_airframe import build_model, config

G = 9.80665


class Allocator:
    def __init__(self, mass, com):
        a = 0.6 / (2 * math.sqrt(2))
        xy = np.array([[a, a], [-a, a], [-a, -a], [a, -a]]) - np.array(com[:2])
        self.A = np.array(
            [np.ones(4), xy[:, 1], -xy[:, 0], [0.018, -0.018, 0.018, -0.018]]
        )
        self.inv = np.linalg.inv(self.A)
        self.force = np.array([710, 1100, 1350, 1580, 1730]) * G / 1000
        self.power = np.array([64.38, 117.66, 166.5, 217.56, 255.3])
        self.cap = self.force[-1] * (18 / 22.2) ** 2 * 0.85
        self.mass = mass

    def watts(self, t):
        return float(
            60 + 1.25 * np.interp(t, self.force, self.power, left=self.power[0]).sum()
        )

    def allowed(self, t):
        return bool(
            np.min(t) >= -1e-10
            and np.max(t) <= self.cap + 1e-10
            and self.watts(t) <= 540 + 1e-8
        )

    def allocate(self, collective, torque):
        # Preserve collective first, then roll/pitch, finally yaw. Allocation
        # includes the nominal centre-of-mass shift; no independent clipping.
        lo, hi = 0.0, min(max(collective, 0.0), 4 * self.cap)
        for _ in range(25):
            mid = (lo + hi) / 2
            if self.allowed(self.inv @ np.array([mid, 0, 0, 0])):
                lo = mid
            else:
                hi = mid
        base = self.inv @ np.array([lo, 0, 0, 0])
        scales = []
        for tau in [[torque[0], torque[1], 0], [0, 0, torque[2]]]:
            diff = self.inv @ np.r_[0, tau]
            low, high = 0.0, 1.0
            for _ in range(22):
                mid = (low + high) / 2
                if self.allowed(base + mid * diff):
                    low = mid
                else:
                    high = mid
            base += low * diff
            scales.append(low)
        return base, scales


def reference(t):
    p = np.array([10.0, 0, 2.0])
    v = np.zeros(3)
    acc = np.zeros(3)
    speed = 0.85
    at = speed / 0.6
    cruise = 3.5
    u = t - 3
    if u > 0:
        if u < at:
            p[0] += 0.3 * u * u
            v[0] = 0.6 * u
            acc[0] = 0.6
        elif u < at + cruise:
            p[0] += 0.5 * speed * at + speed * (u - at)
            v[0] = speed
        elif u < 2 * at + cruise:
            z = u - at - cruise
            p[0] += 0.5 * speed * at + speed * cruise + speed * z - 0.3 * z * z
            v[0] = speed - 0.6 * z
            acc[0] = -0.6
        else:
            p[0] += speed * (at + cruise)
    # Smooth local recovery and climb, then a 4 s endpoint yaw manoeuvre.
    for start, duration, axis, distance in [(12, 2, 1, 0.1), (17, 4, 2, 1.6)]:
        z = np.clip((t - start) / duration, 0, 1)
        p[axis] += distance * (3 * z * z - 2 * z * z * z)
        if 0 < z < 1:
            v[axis] = distance * (6 * z - 6 * z * z) / duration
            acc[axis] = distance * (6 - 12 * z) / duration**2
    z = np.clip((t - 23) / 4, 0, 1)
    yaw = math.pi * (3 * z * z - 2 * z * z * z)
    return p, v, acc, yaw


def run(seed=0, reference_fn=None, duration_s=32, warehouse=False):
    model, xml, mech = build_model(warehouse=warehouse, open_ends=warehouse)
    data = mujoco.MjData(model)
    if reference_fn is not None:
        data.qpos[:3] = reference_fn(0)[0]
    mujoco.mj_forward(model, data)
    settings = config()["flight"]
    allocator = Allocator(mech["mass_kg"], mech["centre_of_mass_m"])
    J = np.array(mech["inertia_kgm2"])
    wn = 2 * math.pi * settings["attitude_bandwidth_hz"]
    damping = settings["attitude_damping"]
    motor = np.full(4, mech["mass_kg"] * G / 4)
    rng = np.random.default_rng(seed)
    history = []
    rows = []
    limited = 0
    peak_current = 0.0
    peak_tilt = 0.0
    dt = model.opt.timestep
    target = motor.copy()
    scales = [1, 1]
    for k in range(round(duration_s / dt)):
        t = data.time
        R = data.xmat[1].reshape(3, 3).copy()
        p = data.qpos[:3].copy()
        v = data.qvel[:3].copy()
        omega = data.qvel[3:6].copy()
        history.append((p, v))
        ref = (reference_fn or reference)(t)
        if k % 2 == 0:
            delay = round(settings["state_latency_s"] / dt)
            pp, vv = history[max(0, k - delay)]
            pp = pp + vv * min(t, settings["state_latency_s"]) + rng.normal(0, 0.005, 3)
            vv = vv + rng.normal(0, 0.01, 3)
            acc = ref[2] + 2.5 * (ref[0] - pp) + 2.6 * (ref[1] - vv)
            acc[2] = np.clip(acc[2], -1, 1)
            maxh = (G + acc[2]) * math.tan(math.radians(settings["tilt_limit_deg"]))
            if np.linalg.norm(acc[:2]) > maxh:
                acc[:2] *= maxh / np.linalg.norm(acc[:2])
            force = mech["mass_kg"] * (acc + [0, 0, G])
            z = force / np.linalg.norm(force)
            heading = np.array([math.cos(ref[3]), math.sin(ref[3]), 0])
            y = np.cross(z, heading)
            y /= np.linalg.norm(y)
            desired = np.column_stack([np.cross(y, z), y, z])
            E = 0.5 * (desired.T @ R - R.T @ desired)
            e = np.array([E[2, 1], E[0, 2], E[1, 0]])
            torque = -J @ (wn * wn * e + 2 * damping * wn * omega) + np.cross(
                omega, J @ omega
            )
            target, scales = allocator.allocate(max(0, force @ R[:, 2]), torque)
            limited += int(min(scales) < 0.999)
        motor += (1 - math.exp(-dt / settings["motor_tau_s"])) * (target - motor)
        peak_current = max(peak_current, allocator.watts(motor) / 18)
        peak_tilt = max(peak_tilt, math.degrees(math.acos(np.clip(R[2, 2], -1, 1))))
        data.ctrl[:] = motor
        mujoco.mj_step(model, data)
        if k % 10 == 0:
            rows.append(
                [
                    t,
                    *p,
                    *ref[0],
                    *v,
                    math.degrees(math.acos(np.clip(R[2, 2], -1, 1))),
                    allocator.watts(motor) / 18,
                    *scales,
                    math.atan2(R[1, 0], R[0, 0]),
                ]
            )
        if not np.all(np.isfinite(data.qpos)) or data.qpos[2] < (
            0.05 if reference_fn is not None else 0.5
        ):
            raise RuntimeError("Unstable candidate test")
    a = np.array(rows)
    err = np.linalg.norm(a[:, 1:4] - a[:, 4:7], axis=1)
    braking = a[(a[:, 0] >= 3 + 0.85 / 0.6 + 3.5) & (a[:, 0] < 12)]
    stopped = braking[np.abs(braking[:, 7]) < 0.05]
    stop = stopped[0] if len(stopped) else None
    result = {
        "seed": seed,
        "duration_s": float(data.time),
        "position_rmse_m": float(np.sqrt(np.mean(err**2))),
        "max_position_error_m": float(err.max()),
        "max_current_a": float(peak_current),
        "max_tilt_deg": float(peak_tilt),
        "minimum_z_m": float(a[:, 3].min()),
        "braking_stop_time_s": None if stop is None else float(stop[0] - braking[0, 0]),
        "braking_stop_distance_m": None
        if stop is None
        else float(stop[1] - braking[0, 1]),
        "final_yaw_error_deg": float(
            math.degrees(
                abs(
                    math.atan2(
                        math.sin(a[-1, -1] - math.pi), math.cos(a[-1, -1] - math.pi)
                    )
                )
            )
        ),
        "allocation_limited_control_steps": limited,
        "control_steps": len(history) // 2,
        "pass_current": bool(peak_current <= 30 + 1e-6),
        "pass_tracking_015m": bool(err.max() < 0.15),
        "pass_tilt_12deg": bool(peak_tilt <= 12),
        "assumptions": [
            "18 V fixed worst-endpoint bus, not coupled depletion",
            "known trajectory and 100 ms position/velocity proxy with constant-velocity prediction",
            "true attitude estimate; full physical rotational dynamics",
            "quasi-static manufacturer power interpolation and voltage-squared thrust ceiling",
            "0.018 m reaction-torque ratio is an assumed yaw map",
            "no aerodynamics, external disturbance, SLAM or image decoding",
        ],
    }
    return result, a, xml, mech


def main(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    results = []
    for seed in [0, 1, 2]:
        result, a, xml, mech = run(seed)
        results.append(result)
        np.savetxt(
            out / f"control_seed{seed}.csv",
            a,
            delimiter=",",
            header="time_s,x,y,z,ref_x,ref_y,ref_z,vx,vy,vz,tilt_deg,current_a,rp_scale,yaw_scale,yaw_rad",
            comments="",
        )
    (out / "control_validation.json").write_text(json.dumps(results, indent=2) + "\n")
    (out / "installation.json").write_text(json.dumps(mech, indent=2) + "\n")
    (out / "scene.xml").write_text(xml)
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--out", default="runs/candidate18")
    main(p.parse_args().out)
