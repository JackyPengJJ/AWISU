"""Check the packaged evidence, final artifacts and release links."""

import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    manifest = json.loads((ROOT / "docs/artifacts.json").read_text())
    for group in ["experiment_sha256", "candidate_sha256", "submission_sha256"]:
        for name, expected in manifest.get(group, {}).items():
            path = ROOT / name
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
            if actual != expected:
                raise ValueError(f"Changed artifact: {name}")
    for name in ["baseline", "inference"]:
        folder = ROOT / "results/final" / name
        rows = np.genfromtxt(
            folder / "trajectory.csv",
            names=True,
            delimiter=",",
            dtype=None,
            encoding="utf8",
        )
        measured = float(
            np.sqrt(
                np.mean(
                    sum(
                        (rows[a] - rows[b]) ** 2
                        for a, b in [("x", "rx"), ("y", "ry"), ("z", "rz")]
                    )
                )
            )
        )
        metrics = json.loads((folder / "metrics.json").read_text())
        if abs(measured - metrics["position_rmse_m"]) > 1e-8:
            raise ValueError(f"RMSE disagrees with saved trajectory: {name}")
    inventory = json.loads(
        (ROOT / "results/final/inference/inventory.json").read_text()
    )
    if len(inventory) != 400 or len({r["barcode"] for r in inventory}) != 400:
        raise ValueError("Inventory identities disagree with report")
    for item in inventory:
        if not (ROOT / "results/final/inference/evidence" / item["evidence"]).is_file():
            raise FileNotFoundError(item["evidence"])
    candidate = ROOT / "results/candidate18"
    if candidate.exists():
        route = json.loads((candidate / "full_route/metrics.json").read_text())
        assert (
            route["pass_current"]
            and route["pass_tracking_015m"]
            and route["pass_tilt_12deg"]
        )
        optical = json.loads(
            (candidate / "optical_render/render_validation.json").read_text()
        )
        assert optical["double_confirmed"] == optical["test_pairs"] == 14
        assert optical["label_size_mm"] == [100, 44]
        scene = json.loads((candidate / "scene_audit.json").read_text())
        assert scene["passed"] and scene["labels"] == 400
    metadata = json.loads((ROOT / "docs/report_metadata.json").read_text())
    if metadata["video"] != "https://youtu.be/NSdeKHdK3a8":
        raise ValueError("Unexpected video URL")
    draft_dirs = [
        "docs/review",
        "reports/review",
        "results/final/presentation",
        "results/final/technical_film_v2",
    ]
    if any((ROOT / p).exists() for p in draft_dirs):
        raise ValueError("Draft directory in release")
    print(
        f"Verified {len(manifest['experiment_sha256'])} preserved experimental files, "
        f"{len(manifest['submission_sha256'])} submission files and 400 inventory records."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
