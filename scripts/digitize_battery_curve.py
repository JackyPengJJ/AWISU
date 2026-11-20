"""Digitise the known Murata VTC6 Version 001 rate chart from a supplied PDF.

Requires Poppler pdftoppm. No PDF is downloaded or redistributed by this script.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import numpy as np
from PIL import Image

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--pdf", type=Path, required=True)
parser.add_argument("--out", type=Path, required=True)
args = parser.parse_args()
expected = "ff42082f0d94c311bfd9a8b24211c5d522aeee161adb7473731e9fe94fcc9f08"
if hashlib.sha256(args.pdf.read_bytes()).hexdigest() != expected:
    raise ValueError(
        "Different source PDF; recalibrate rather than reusing pixel coordinates"
    )
with tempfile.TemporaryDirectory(prefix="awisu-vtc6-") as tmp:
    prefix = Path(tmp) / "source"
    subprocess.run(
        [
            "pdftoppm",
            "-f",
            "1",
            "-singlefile",
            "-r",
            "450",
            "-png",
            str(args.pdf),
            str(prefix),
        ],
        check=True,
    )
    image = Image.open(prefix.with_suffix(".png")).convert("RGB")
    w, h = image.size
    image = np.asarray(image.crop((w * 0.145, h * 0.389, w * 0.43, h * 0.56)))
    capacities = [100, 300, 600, 900, 1200, 1500, 1800, 2100, 2400, 2550, 2700]
    rows = []
    for q in capacities:
        row = [q]
        for color in [[146, 208, 80], [0, 176, 80]]:
            x = round(27 + q / 3300 * 928)
            samples = image[40:430, x - 2 : x + 3].astype(float)
            mask = np.linalg.norm(samples - np.array(color), axis=2) < 45
            ys = np.flatnonzero(mask.any(axis=1)) + 40
            if not len(ys):
                raise ValueError(f"Curve missing at {q} mAh")
            group = [ys[0]]
            for y in ys[1:]:
                if y - group[-1] > 2:
                    break
                group.append(y)
            row.append(round(4.5 - (np.median(group) - 28) / 801 * 4, 3))
        rows.append(row)
    rows.insert(0, [0, rows[0][1], rows[0][2]])
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["capacity_mah", "voltage_10a_v", "voltage_15a_v"])
        writer.writerows(rows)
    print(
        json.dumps(
            {"source_sha256": expected, "output": str(args.out), "rows": len(rows)}
        )
    )
