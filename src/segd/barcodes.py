"""Code128 imaging study, scene-camera decoding and inventory verification."""

import csv
import re
import json
from pathlib import Path
import numpy as np
import cv2
import zxingcpp
from barcode import Code128
from barcode.writer import ImageWriter
from PIL import Image


def label_image(text):
    return np.array(
        Code128(text, writer=ImageWriter())
        .render(
            writer_options={
                "module_width": 0.3,
                "module_height": 12,
                "quiet_zone": 3,
                "font_size": 0,
                "write_text": False,
                "dpi": 300,
            }
        )
        .convert("L")
    )


def imaging_study(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    rng = np.random.default_rng(9102)
    rows = []
    width = 4096
    fov = np.deg2rad(45)
    distance = 1.15
    focal = width / (2 * np.tan(fov / 2))
    for size in (0.05, 0.10, 0.15):
        for speed in (0.4, 0.8, 1.1, 1.4):
            for exposure in (0.0001, 0.001, 0.005):
                for brightness in (1.0, 0.4):
                    correct = wrong = 0
                    for i in range(10):
                        text = f"W{i:04d}"
                        label = label_image(text)
                        pixels = round(focal * size / distance)
                        h = max(12, round(pixels * label.shape[0] / label.shape[1]))
                        image = np.full((max(160, h + 40), pixels + 100), 255, np.uint8)
                        image[20 : 20 + h, 50 : 50 + pixels] = cv2.resize(
                            label, (pixels, h), interpolation=cv2.INTER_AREA
                        )
                        blur = max(1, round(focal * speed * exposure / distance))
                        kernel = np.ones((1, blur), np.float32) / blur
                        image = cv2.filter2D(image, -1, kernel)
                        image = np.clip(
                            image.astype(float) * brightness
                            + rng.normal(0, 3, image.shape),
                            0,
                            255,
                        ).astype(np.uint8)
                        decoded = zxingcpp.read_barcodes(image)
                        texts = [d.text for d in decoded]
                        correct += text in texts
                        wrong += sum(t != text for t in texts)
                        if i == 0 and speed == 1.1 and exposure in (0.0001, 0.005):
                            Image.fromarray(image).save(
                                out
                                / f"sample_{int(size * 1000)}mm_{int(exposure * 1e6)}us_{brightness}.png"
                            )
                        rows.append(
                            {
                                "size_mm": size * 1000,
                                "speed_mps": speed,
                                "exposure_s": exposure,
                                "brightness": brightness,
                                "id": text,
                                "pixels": pixels,
                                "blur_px": blur,
                                "correct": text in texts,
                                "wrong": sum(t != text for t in texts),
                            }
                        )
    with (out / "decodes.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "experiment": "isolated_code128_image_study",
        "samples": len(rows),
        "correct": sum(r["correct"] for r in rows),
        "wrong": sum(r["wrong"] for r in rows),
        "camera_width_px": width,
        "horizontal_fov_deg": 45,
        "distance_m": distance,
        "limitations": "Frontoparallel cropped image, no perspective/occlusion, no scene-rendered inventory; not full-mission read rate.",
    }
    (out / "metrics.json").write_text(json.dumps(summary, indent=2))
    return summary


class Inventory:
    """Reads only image pixels; instance identifiers must differ from repeated SKUs."""

    def __init__(self):
        self.records = {}

    def update(self, image, timestamp, side, position):
        for item in zxingcpp.read_barcodes(image):
            # Payload is an item instance ID; identical SKU labels cannot identify separate objects.
            key = item.text
            if key not in self.records:
                self.records[key] = {
                    "barcode": key,
                    "first_seen": timestamp,
                    "side": side,
                    "position": list(position),
                    "observations": 0,
                }
            self.records[key]["last_seen"] = timestamp
            self.records[key]["observations"] += 1
        return self.records


class SceneCameras:
    """Simulated global-shutter cameras; pixels are the only inventory input."""

    def __init__(self, model):
        import mujoco

        self.model = model
        self.renderer = mujoco.Renderer(model, height=2160, width=4096)

    def capture(self, data, speed):
        images = {}
        for side in ["left", "right"]:
            from .rendering import sensor_frame

            image = sensor_frame(self.renderer, self.model, data, side)
            # 0.1 ms exposure, nominal standoff 1.23 m; resize is never used to invent detail.
            blur = max(1, round(4944 * abs(speed) * 0.0001 / 1.23))
            if blur > 1:
                image = cv2.filter2D(image, -1, np.ones((1, blur), np.float32) / blur)
            images[side] = image
        return images

    def close(self):
        self.renderer.close()


def verify_barcode_crop(image, expected):
    """Require the declared ID syntax and agreement across crop preprocessing variants."""
    if not re.fullmatch(r"W[0-9]{4}", expected):
        return False
    if image is None or image.size == 0:
        return False
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    variants = [
        gray,
        cv2.resize(gray, None, fx=2, fy=2),
        cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1],
    ]
    return (
        sum(expected in {d.text for d in zxingcpp.read_barcodes(v)} for v in variants)
        >= 2
    )


class EvidenceInventory:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.records = {}
        self.events = []
        self.frames = 0

    def update(self, images, estimated_position, timestamp):
        for side, image in images.items():
            items = zxingcpp.read_barcodes(image)
            texts = []
            for item in items:
                text = item.text
                texts.append(text)
                if text not in self.records:
                    # File names come from counters, never unsanitised barcode payloads.
                    evidence = f"{len(self.records):04d}.jpg"
                    corners = [
                        getattr(item.position, k)
                        for k in [
                            "top_left",
                            "top_right",
                            "bottom_left",
                            "bottom_right",
                        ]
                    ]
                    xs = [p.x for p in corners]
                    ys = [p.y for p in corners]
                    x0 = max(0, min(xs) - 20)
                    x1 = min(image.shape[1], max(xs) + 20)
                    y0 = max(0, min(ys) - 20)
                    y1 = min(image.shape[0], max(ys) + 20)
                    Image.fromarray(image[y0:y1, x0:x1]).save(
                        self.folder / evidence, quality=92
                    )
                    self.records[text] = {
                        "barcode": text,
                        "first_seen_s": timestamp,
                        "last_seen_s": timestamp,
                        "side": side,
                        "estimated_uav_position": list(map(float, estimated_position)),
                        "observations": 0,
                        "evidence": evidence,
                    }
                record = self.records[text]
                record["last_seen_s"] = timestamp
                record["observations"] += 1
            self.events.append(
                {"t": timestamp, "side": side, "decoded": texts, "frame": self.frames}
            )
            self.frames += 1

    def export(self, folder):
        folder = Path(folder)
        (folder / "raw_inventory.json").write_text(
            json.dumps(list(self.records.values()), indent=2)
        )
        (folder / "raw_decode_events.json").write_text(
            json.dumps(self.events, indent=2)
        )
        rejected = []
        accepted = {}
        for code, record in self.records.items():
            image = cv2.imread(str(self.folder / record["evidence"]))
            if verify_barcode_crop(image, code):
                accepted[code] = record
            else:
                rejected.append(record)
        self.records = accepted
        self.events = [
            {**e, "decoded": [code for code in e["decoded"] if code in accepted]}
            for e in self.events
        ]
        (folder / "inventory.json").write_text(
            json.dumps(list(accepted.values()), indent=2)
        )
        (folder / "decode_events.json").write_text(json.dumps(self.events, indent=2))
        (folder / "barcode_verification.json").write_text(
            json.dumps(
                {
                    "method": "ID syntax plus saved pixel crop agreement on at least two of original, 2x resized and Otsu variants; not independent temporal observations",
                    "accepted": len(accepted),
                    "rejected": rejected,
                    "truth_inventory_used": False,
                },
                indent=2,
            )
        )
