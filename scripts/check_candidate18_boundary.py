"""Reproduce the recovery-count boundary for the declared 18 V screening case."""

import json
import argparse
from pathlib import Path
from segd.final_design import calculate
from segd.candidate_airframe import config
from segd.battery_model import mission_screen

parser = argparse.ArgumentParser()
parser.add_argument("--out", default="runs/candidate18")
args = parser.parse_args()
out = Path(args.out)
out.mkdir(parents=True, exist_ok=True)
s = config()["battery"]
base = calculate()["cases"][0]
rows = []
for n in range(21):
    phases = [{**p} for p in base["phases"]]
    for p in phases:
        if p["phase"] in ["targeted_retries", "lateral_recovery"]:
            for key in ["seconds", "nominal_wh", "adverse_wh"]:
                p[key] *= n / 2
    r = mission_screen({**base, "phases": phases}, s)
    rows.append({"recoveries": n, **r})
json.dump(
    {
        "settings": s,
        "results": rows,
        "maximum_passing_recoveries": max(
            x["recoveries"] for x in rows if x["reserve_pass"]
        ),
        "interpretation": "Workload bound under fixed geometry and stated power/temperature/assembly assumptions; not a measured failure-rate estimate.",
    },
    open(out / "recovery_boundary.json", "w"),
    indent=2,
)
print(
    [
        (r["recoveries"], round(r["reserve_fraction"] * 100, 3))
        for r in rows
        if r["recoveries"] in [8, 9]
    ]
)
