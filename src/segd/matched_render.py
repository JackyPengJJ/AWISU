"""Native-resolution evidence using the candidate optics in the original warehouse.

These static exposure pairs test optics, not flight, exploration or servo timing.
"""

import json
import math
from pathlib import Path
import numpy as np
import mujoco
import zxingcpp
from PIL import Image
from .candidate_airframe import build_model
from .flight import ROOT


def run(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    model, xml, mech = build_model(warehouse=True)
    data = mujoco.MjData(model)
    # Camera images exclude the renderer's opaque lens envelope: a pinhole is
    # inside the allocated real lens. Warehouse geometry stays visible.
    uav = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "uav")
    owned = {uav}
    for i in range(model.nbody):
        if model.body_parentid[i] in owned:
            owned.add(i)
    for i in range(model.ngeom):
        if model.geom_bodyid[i] in owned:
            model.geom_rgba[i, 3] = 0
    renderer = mujoco.Renderer(model, height=3000, width=4096)
    truth = json.loads((ROOT / "assets/generated/inventory_truth.json").read_text())
    events = []
    for side in ["left", "right"]:
        sign = 1 if side == "left" else -1
        head = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "head_" + side)
        for layer, offsets in [(0, [-0.7, 0]), (2, [-0.7, 0, 0.7]), (4, [0, 0.7])]:
            code = f"W{(200 if sign == 1 else 0) + layer * 40:04d}"
            # Cargo truth is used only to select a test object, not to decode it.
            for offset in offsets:
                pitch = 28 if offset > 0 else -28 if offset < 0 else 0
                theta = sign * math.radians(pitch)
                model.body_quat[head] = [math.cos(theta / 2), math.sin(theta / 2), 0, 0]
                pair = []
                for shot in [0, 1]:
                    x = 1 + (-0.24 if shot == 0 else -0.24 + 0.85 * 0.05)
                    data.qpos[:3] = [x, 0, 0.8 + 1.6 * layer - offset]
                    data.qpos[3:7] = [1, 0, 0, 0]
                    mujoco.mj_forward(model, data)
                    renderer.update_scene(data, camera=side)
                    img = renderer.render().copy()
                    decoded = zxingcpp.read_barcodes(img)
                    texts = [v.text for v in decoded]
                    name = f"{side}_level{layer}_offset{offset:+.1f}_shot{shot}.jpg"
                    # Save representative native-size views and all decoded crops.
                    if shot == 0:
                        Image.fromarray(img).save(out / name, quality=95)
                    crops = []
                    for j, d in enumerate(decoded):
                        pos = d.position
                        xs = [
                            pos.top_left.x,
                            pos.top_right.x,
                            pos.bottom_left.x,
                            pos.bottom_right.x,
                        ]
                        ys = [
                            pos.top_left.y,
                            pos.top_right.y,
                            pos.bottom_left.y,
                            pos.bottom_right.y,
                        ]
                        box = (
                            max(0, min(xs) - 20),
                            max(0, min(ys) - 20),
                            min(4096, max(xs) + 20),
                            min(3000, max(ys) + 20),
                        )
                        crop = f"{side}_{layer}_{offset:+.1f}_{shot}_{j}.png"
                        Image.fromarray(img).crop(box).save(out / crop)
                        crops.append(crop)
                    pair.append(
                        {
                            "shot": shot,
                            "x_m": x,
                            "decoded": texts,
                            "expected_found": code in texts,
                            "crops": crops,
                        }
                    )
                event = {
                    "side": side,
                    "layer": layer,
                    "label_vertical_offset_m": offset,
                    "pitch_deg": pitch,
                    "expected": code,
                    "pair": pair,
                    "both_confirm": all(p["expected_found"] for p in pair),
                }
                events.append(event)
                print(
                    json.dumps({k: v for k, v in event.items() if k != "pair"}),
                    flush=True,
                )
    renderer.close()
    result = {
        "image_size": [4096, 3000],
        "camera_optical_centres_y_m": [-0.4, 0.4],
        "focal_length_mm": 12,
        "pixel_pitch_um": 3.45,
        "label_size_mm": [100, 44],
        "events": events,
        "double_confirmed": sum(e["both_confirm"] for e in events),
        "test_pairs": len(events),
        "status": "static native warehouse exposures; no motion blur, lens distortion or servo settling; paired positions differ by 42.5 mm, not independent trials",
        "original_cargo_count": len(truth),
    }
    (out / "render_validation.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--out", required=True)
    run(p.parse_args().out)


def degradation_study(out):
    """Pixel perturbations of saved native crops; not independent acquisitions."""
    import cv2

    out = Path(out)
    record = json.loads((out / "render_validation.json").read_text())
    rng = np.random.default_rng(5110)
    trials = []
    for event in record["events"]:
        for pair in event["pair"]:
            expected = event["expected"]
            crop = None
            filename = None
            for filename in pair["crops"]:
                image = np.array(Image.open(out / filename).convert("L"))
                if expected in [d.text for d in zxingcpp.read_barcodes(image)]:
                    crop = image
                    break
            if crop is None:
                raise ValueError("No preserved expected crop")
            height, width = crop.shape
            blur = np.zeros_like(crop, dtype=float)
            for dx in np.linspace(-0.75 / 2, 0.75 / 2, 9):
                blur += (
                    cv2.warpAffine(
                        crop,
                        np.array([[1.0, 0.0, dx], [0.0, 1.0, 0.0]]),
                        (width, height),
                        flags=cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_REPLICATE,
                    )
                    / 9
                )
            for level in [1.0, 0.4]:
                degraded = np.clip(
                    blur * level + rng.normal(0, 3, crop.shape), 0, 255
                ).astype("uint8")
                decoded = [d.text for d in zxingcpp.read_barcodes(degraded)]
                trials.append(
                    {
                        "source_crop": filename,
                        "expected": expected,
                        "brightness_scale": level,
                        "blur_width_px": 0.75,
                        "noise_sigma_dn": 3,
                        "decoded": decoded,
                        "correct": expected in decoded,
                        "wrong": [x for x in decoded if x != expected],
                    }
                )
    summary = {
        "method": "Saved native crop; nine subpixel translations approximate 0.75 px horizontal exposure blur; brightness scaling and additive Gaussian noise",
        "trials": trials,
        "total": len(trials),
        "correct": sum(x["correct"] for x in trials),
        "wrong_candidates": sum(len(x["wrong"]) for x in trials),
        "interpretation": "Sensitivity of already located code crops, not full-frame detection timing, optical MTF qualification, independent trials or measured illumination.",
    }
    (out / "degradation_validation.json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )
    print(summary["correct"], summary["total"], summary["wrong_candidates"])
