"""Check assignment geometry and selected optics in the compiled candidate scene."""

import json
import mujoco
import numpy as np
from segd.candidate_airframe import build_model
from segd.flight import ROOT


def audit():
    m, xml, installation = build_model(warehouse=True)

    def gid(n):
        return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)

    guard = gid("guard")
    ceiling = gid("candidate_ceiling")
    uav = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "uav")
    assert m.body_mass[uav] < 3 and abs(m.body_mass[uav] - 2.9016) < 1e-8
    assert np.allclose(m.geom_size[guard, :2], [0.5, 0.25])
    assert abs(m.geom_pos[ceiling, 2] - m.geom_size[ceiling, 2] - 8) < 1e-8
    labels = [
        i
        for i in range(m.ngeom)
        if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith(
            "label_W"
        )
    ]
    assert len(labels) == 400
    assert all(np.allclose(m.geom_size[i, :2], [0.05, 0.022]) for i in labels)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    loc = np.array([d.geom_xpos[i] for i in labels])
    levels = np.unique(np.round(loc[:, 2], 5))
    assert np.allclose(levels, [0.8, 2.4, 4, 5.6, 7.2])
    assert sum(loc[:, 1] < 0) == sum(loc[:, 1] > 0) == 200
    cameras = {}
    for side, sign in [("left", 1), ("right", -1)]:
        c = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, side)
        relative = d.cam_xpos[c] - d.xpos[uav]
        assert np.allclose(relative, [0, sign * 0.4, 0])
        fy = 3000 / (2 * np.tan(np.deg2rad(m.cam_fovy[c]) / 2))
        assert abs(fy - 12 / 0.00345) < 1e-8
        cameras[side] = {
            "body_position_m": relative.tolist(),
            "fovy_deg": float(m.cam_fovy[c]),
            "focal_pixels": float(fy),
        }
    return {
        "aircraft_mass_kg": float(m.body_mass[uav]),
        "envelope_radius_and_halfheight_m": m.geom_size[guard, :2].tolist(),
        "ceiling_underside_m": 8,
        "aisle_m": [80, 2.5, 8],
        "labels": 400,
        "labels_per_side": 200,
        "label_centres_m": levels.tolist(),
        "label_size_mm": [100, 44],
        "camera_resolution": [4096, 3000],
        "cameras": cameras,
        "passed": True,
        "scope": "Compiled dimensions and optical extrinsics; no claim of integrated unknown-layout navigation",
        "end_condition": "Candidate adds end planes and ceiling for static geometry checks; reference flight uses observed end staging volumes as an installation condition.",
    }


if __name__ == "__main__":
    r = audit()
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="runs/candidate18")
    args = parser.parse_args()
    out = ROOT / args.out / "scene_audit.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(r, indent=2) + "\n")
    print(json.dumps(r, indent=2))
