"""Render recorded physical trajectories; replay is never an additional flight test."""

import json
from pathlib import Path
import numpy as np
import mujoco


def replay(args):
    from .flight import create_model
    import imageio.v2 as imageio
    import cv2

    run = Path(args.run)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    rows = np.genfromtxt(
        run / "trajectory.csv", delimiter=",", names=True, dtype=None, encoding="utf8"
    )
    if not {"qw", "qx", "qy", "qz"} <= set(rows.dtype.names):
        raise ValueError("Recorded quaternion required for faithful attitude replay")
    metadata = json.loads((run / "provenance.json").read_text())
    metrics = json.loads((run / "metrics.json").read_text())
    config = metadata["config"]
    config["visual_assets"] = True
    model, xml = create_model(config)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=720, width=1280)
    mode = metrics["experiment"]
    policy_name = (
        Path(metadata["policy"]["path"]).parent.name if metadata.get("policy") else None
    )
    indices = np.linspace(0, len(rows) - 1, int(args.seconds * 20)).astype(int)
    events = (
        json.loads((run / "decode_events.json").read_text())
        if (run / "decode_events.json").exists()
        else []
    )
    first_seen = {}
    for event in events:
        for code in event["decoded"]:
            first_seen.setdefault(code, event["t"])
    try:
        with imageio.get_writer(
            out / "mission.mp4", fps=20, quality=8, codec="libx264", macro_block_size=16
        ) as writer:
            for number, index in enumerate(indices):
                row = rows[index]
                data.qpos[:3] = [row[k] for k in ["x", "y", "z"]]
                data.qpos[3:7] = [row[k] for k in ["qw", "qx", "qy", "qz"]]
                mujoco.mj_forward(model, data)
                cam = mujoco.MjvCamera()
                cam.lookat[:] = data.qpos[:3]
                cam.distance = 2.1
                cam.azimuth = 180
                cam.elevation = (
                    -17
                )  # Fixed world direction: no camera flips at layer reversals.
                original_groups = model.geom_group.copy()
                distant = (original_groups == 2) & (
                    np.linalg.norm(data.geom_xpos - data.qpos[:3], axis=1) > 15.0
                )
                try:
                    model.geom_group[distant] = 4
                    renderer.update_scene(data, camera=cam)
                    frame = renderer.render().copy()
                finally:
                    model.geom_group[:] = original_groups
                shade = frame[:90].copy()
                frame[:90] = cv2.addWeighted(shade, 0.25, np.zeros_like(shade), 0.75, 0)
                cv2.putText(
                    frame,
                    f"{'PPO INFERENCE REPLAY' if policy_name else 'RECORDED FLIGHT'} | {mode} | t={row['t']:.1f}s | {row['phase']}",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (240, 245, 250),
                    1,
                )
                line = f"z={row['z']:.2f}m | energy model={row['energy_wh']:.1f}Wh | playback={float(rows[-1]['t']) / args.seconds:.1f}x"
                if events:
                    line += f" | decoded={sum(t <= row['t'] for t in first_seen.values())}/400"
                cv2.putText(
                    frame,
                    line,
                    (20, 62),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (200, 220, 235),
                    1,
                )
                draw_route_map(frame, rows, index)
                writer.append_data(frame)
                if number in (
                    0,
                    len(indices) // 5,
                    len(indices) // 2,
                    4 * len(indices) // 5,
                ):
                    imageio.imwrite(out / f"frame_{number:04d}.png", frame)
    finally:
        renderer.close()
    result = {
        "source_run": str(run),
        "source_experiment": mode,
        "route_completed": metrics["route_completed"],
        "video": str(out / "mission.mp4"),
        "rendered_frames": len(indices),
        "duration_s": args.seconds,
        "detail_visibility_range_m": 15.0,
        "camera": "fixed world azimuth 180 degrees; continuous x-z route map",
        "note": "Recorded position and quaternion replay; not a new flight test or proof of unimplemented autonomy.",
    }
    (out / "provenance.json").write_text(json.dumps(result, indent=2))
    return result


def draw_route_map(frame, rows, index):
    """Show the recorded x-z trajectory in a fixed world frame."""
    import cv2

    height, width = frame.shape[:2]
    left, top = width - 490, height - 185
    panel = frame[top : height - 10, left : width - 10]
    panel[:] = cv2.addWeighted(panel, 0.15, np.zeros_like(panel), 0.85, 0)

    def points(xs, zs):
        return np.column_stack(
            [
                left + 30 + (np.asarray(xs) + 3) / 86 * 420,
                top + 145 - np.asarray(zs) / 8 * 105,
            ]
        ).astype(np.int32)

    for z in [0.8, 2.4, 4.0, 5.6, 7.2]:
        q = points([0, 80], [z, z])
        cv2.line(frame, tuple(q[0]), tuple(q[1]), (55, 65, 70), 1)
    # Downsample only the display polyline; flight data remains untouched.
    past = rows[: index + 1 : 8]
    if len(past) > 1:
        cv2.polylines(frame, [points(past["x"], past["z"])], False, (225, 190, 65), 2)
    row = rows[index]
    q = points([row["x"]], [row["z"]])[0]
    cv2.circle(frame, tuple(q), 5, (30, 230, 250), -1)
    for text, y in [
        ("CONTINUOUS ROUTE | fixed world x-z view", top + 20),
        ("A: 0m                                      B: 80m", top + 166),
    ]:
        cv2.putText(
            frame,
            text,
            (left + 14, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (235, 240, 245),
            1,
        )
    direction = (
        "A -> B"
        if row["vx"] > 0.1
        else "B -> A"
        if row["vx"] < -0.1
        else "CLIMB"
        if row["vz"] > 0.1
        else "DESCEND"
        if row["vz"] < -0.1
        else "HOLD"
    )
    cv2.putText(
        frame,
        f"x={row['x']:.1f}m | z={row['z']:.1f}m | {direction}",
        (left + 14, top + 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43,
        (30, 230, 250),
        1,
    )
