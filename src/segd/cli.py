"""Command-line entry points for engineering analysis and reference experiments."""

import argparse
import importlib.metadata
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="AWISU warehouse inventory UAV")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("doctor", help="Show installed numerical and simulation versions")
    calc = sub.add_parser("calculate", help="Export the selected engineering design")
    calc.add_argument("--out", default="reports/design_calculations.json")
    report = sub.add_parser("report", help="Build the technical report with XeLaTeX")
    report.add_argument("--results", default="results")
    report.add_argument("--out", default="reports")
    sim = sub.add_parser("simulate", help="Run the reference-plant control experiment")
    sim.add_argument("--out", required=True)
    sim.add_argument("--config")
    sim.add_argument("--short", action="store_true")
    sim.add_argument("--video", action="store_true")
    sim.add_argument("--policy")
    sim.add_argument("--seed", type=int, default=0)
    replay = sub.add_parser("replay", help="Render a saved trajectory")
    replay.add_argument("--run", required=True)
    replay.add_argument("--out", required=True)
    replay.add_argument("--seconds", type=float, default=30)
    barcode = sub.add_parser(
        "barcode-study", help="Reproduce the synthetic optical study"
    )
    barcode.add_argument("--out", required=True)
    train = sub.add_parser(
        "train", help="Train a residual controller in a new directory"
    )
    train.add_argument("--out", required=True)
    train.add_argument("--steps", type=int, default=1000000)
    train.add_argument("--seed", type=int, default=0)
    train.add_argument("--resume")
    train.add_argument("--hours", type=float, default=24)
    args = parser.parse_args()
    if args.command == "doctor":
        result = {
            name: importlib.metadata.version(name)
            for name in [
                "mujoco",
                "numpy",
                "scipy",
                "matplotlib",
                "opencv-python-headless",
                "zxing-cpp",
            ]
        }
    elif args.command == "calculate":
        from .final_design import calculate

        result = calculate()
        path = Path(args.out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, indent=2) + "\n")
        result = {"output": str(path), "installed_mass_g": result["mass_g"]}
    elif args.command == "report":
        from .report import build

        result = build(Path(args.results), Path(args.out))
    elif args.command == "simulate":
        from .flight import load_config, simulate

        config = load_config(args.config)
        config["seed"] = args.seed
        result = simulate(config, args.out, args.short, args.video, args.policy)
    elif args.command == "replay":
        if args.seconds <= 0:
            parser.error("--seconds must be positive")
        from .replay import replay

        result = replay(args)
    elif args.command == "barcode-study":
        from .barcodes import imaging_study

        result = imaging_study(args.out)
    else:
        from .training import train

        result = train(args)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
