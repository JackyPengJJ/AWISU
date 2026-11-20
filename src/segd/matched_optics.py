"""Pose-specific finite-label sampling bounds using the selected physical optics.

A direction-norm bound was unnecessarily pessimistic: optical depth is the
projection onto the chosen camera's axis, not the full camera-to-corner range.
"""

import copy
import itertools
import json
import math
from pathlib import Path
import numpy as np
from barcode import Code128
from .design_options import options
from .optical_model import normals, safe_halfwidth


def depth_and_sampling(o, z, pitch, half_window):
    _, forward = normals(o, pitch)
    dz = (
        o["label_outer_height_m"] / 2
        + o["vertical_position_bound_m"]
        + o["height_grid_step_m"] / 2
    )
    w = o["label_outer_width_m"] / 2 + o["along_track_position_bound_m"]
    q = np.array(
        list(
            itertools.product(
                [-half_window - w, half_window + w], o["standoff_m"], [z - dz, z + dz]
            )
        )
    )
    delta = math.radians(o["orientation_cone_deg"])
    beta = math.radians(o["label_plane_normal_error_deg"])
    displacement = (
        2
        * math.hypot(o["label_outer_width_m"] / 2, o["label_outer_height_m"] / 2)
        * math.sin(beta / 2)
    )
    # Rotating the camera axis by <=delta changes its dot product by at most
    # 2 |q| sin(delta/2). Add the independent finite-label rotation displacement.
    upper_depth = (
        float(np.max(q @ forward + 2 * np.linalg.norm(q, axis=1) * math.sin(delta / 2)))
        + displacement
    )
    fx = o["focal_mm"] / o["pixel_mm"]
    tx = (o["pixels"][0] / 2 - o["image_border_px"]) / fx
    # du/ds = fx (a_x - (X/Z)c_x)/Z. Combined label/camera angle
    # gamma bounds a_x from below and |c_x| from above. Frustum containment
    # bounds |X/Z| <= tx for every point on a module edge.
    gamma = beta + delta
    ray_error = 2 * np.linalg.norm(q, axis=1) * math.sin(delta / 2) + displacement
    lower_depth = float(np.min(q @ forward - ray_error))
    upper_x = float(np.max(np.abs(q[:, 0]) + ray_error))
    tx = min(tx, upper_x / lower_depth)
    scale = fx * (math.cos(gamma) - tx * math.sin(gamma)) / upper_depth
    return upper_depth, scale


def evaluate(o, modules):
    o = copy.deepcopy(o)
    o["module_width_m"] = o["label_outer_width_m"] / modules
    horizon = o["cycle_s"] + 2 * o["timestamp_jitter_s"]
    half = o["scan_speed_mps"] * horizon / 2
    zs = np.linspace(-0.8, 0.8, round(1.6 / o["height_grid_step_m"]) + 1)
    rows = []
    for z in zs:
        valid = []
        for pitch in o["pitch_deg"]:
            width = 2 * safe_halfwidth(o, float(z), pitch)
            if width + 1e-9 >= 2 * half:
                depth, scale = depth_and_sampling(o, float(z), pitch, half)
                valid.append((scale, pitch, depth, width))
        if valid:
            scale, pitch, depth, width = max(valid)
            rows.append(
                [
                    float(z),
                    pitch,
                    scale,
                    scale * o["label_outer_width_m"] / modules,
                    depth,
                    width,
                ]
            )
        else:
            rows.append([float(z), None, 0.0, 0.0, None, 0.0])
    a = np.array([r[3] for r in rows])
    worst = int(np.argmin(a))
    return {
        "module_count": modules,
        "width_mm": o["label_outer_width_m"] * 1000,
        "height_mm": o["label_outer_height_m"] * 1000,
        "two_observation_horizon_s": horizon,
        "required_window_m": 2 * half,
        "minimum_pixels_per_module": float(a.min()),
        "sampling_target": o["pixels_per_module_target"],
        "geometry_and_sampling_pass": bool(a.min() >= o["pixels_per_module_target"]),
        "worst_height_m": rows[worst][0],
        "by_height": rows,
        "assumptions": o,
    }


def study(out):
    _, c, _ = options()
    o = c["optics"]
    rows = []
    for payload in ["W0000", "INVENTORYABC"]:
        modules = len(Code128(payload).build()[0]) + 20
        for width in [0.1, 0.2, 0.25]:
            case = copy.deepcopy(o)
            case.update(label_outer_width_m=width, label_outer_height_m=0.044)
            r = evaluate(case, modules)
            r["payload"] = payload
            rows.append(r)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "optical_bounds.json").write_text(
        json.dumps(
            {
                "method": "Complete-frustum containment followed by pose-specific optical-depth Jacobian lower bound; quiet zones included",
                "results": rows,
            },
            indent=2,
        )
        + "\n"
    )
    print(
        json.dumps(
            [
                {k: v for k, v in r.items() if k not in ["by_height", "assumptions"]}
                for r in rows
            ],
            indent=2,
        )
    )


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--out", required=True)
    study(p.parse_args().out)
