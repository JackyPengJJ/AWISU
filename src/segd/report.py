"""Build the technical report from editable Markdown, engineering inputs and final results."""

import json, re
from pathlib import Path
import numpy as np
from .flight import ROOT


def resolved_source(results):
    from .final_design import calculate

    data = calculate()
    c = data["base"]
    r = data["cases"][0]
    rows = np.genfromtxt(
        results / "final/inference/trajectory.csv",
        names=True,
        delimiter=",",
        dtype=None,
        encoding="utf8",
    )
    airborne = np.isin(rows["phase"], ["scan", "climb", "return"])
    d = {
        "geometry_outer_diameter_m": 0.6 + 0.3556 + 0.04,
        "propeller_gap_m": 0.6 / np.sqrt(2) - 0.3556 - 0.04,
        "mass_g": data["mass_g"],
        "safe_width": data["followup"]["coverage"]["minimum_safe_width_m"],
        "targeted_width": data["targeted_recovery_safe_width_m"],
        "mass_margin_g": 3000 - data["mass_g"],
        "mission_s": r["total_seconds"],
        "nominal_energy_wh": r["nominal_wh"],
        "adverse_energy_wh": r["adverse_wh"],
        "required_delivered_wh": r["required_delivered_wh_adverse"],
        "required_eight": data["cases"][1]["required_delivered_wh_adverse"],
        "required_twenty": data["cases"][2]["required_delivered_wh_adverse"],
        "side_clearance_budget_m": data["centre_clearance_m"],
        "ceiling_clearance_budget_m": 8
        - 7.2
        - (0.5 * np.sin(np.deg2rad(12)) + 0.25 * np.cos(np.deg2rad(12)))
        - 0.1
        - 0.075
        - 0.025,
        "airborne_clearance_m": float(np.min(rows["clearance_m"][airborne])),
    }
    screen = json.loads((results / "candidate18/battery_screen.json").read_text())
    selected = next(
        x
        for x in screen["results"]
        if x["cutoff_v"] == 18
        and x["interconnect_ohm"] == 0.01
        and abs(x["capacity_scale"] - 3000 / 3120) < 1e-8
        and x["recoveries"] == 2
    )
    aged = next(
        x
        for x in screen["results"]
        if x["cutoff_v"] == 18
        and x["interconnect_ohm"] == 0.02
        and x["capacity_scale"] == 0.9
        and x["recoveries"] == 2
    )
    eight = next(
        x
        for x in screen["results"]
        if x["cutoff_v"] == 18
        and x["interconnect_ohm"] == 0.02
        and abs(x["capacity_scale"] - 3000 / 3120) < 1e-8
        and x["recoveries"] == 8
    )
    d.update(
        delivered_two=selected["deliverable_under_mission_then_tail_wh"],
        aged_two_reserve=100 * aged["reserve_fraction"],
        eight_high_resistance=100 * eight["reserve_fraction"],
    )
    matched = json.loads((results / "candidate18/full_route/metrics.json").read_text())
    assert (
        matched["pass_current"]
        and matched["pass_tracking_015m"]
        and matched["pass_tilt_12deg"]
    ), "Matched route acceptance failed"
    d.update(
        matched_duration=matched["mission_duration_s"],
        matched_rmse=matched["position_rmse_m"],
        matched_error=matched["max_position_error_m"],
        matched_current=matched["max_current_a"],
        matched_clearance=matched["minimum_nominal_wall_ceiling_clearance_m"],
    )
    content = (ROOT / "docs/report_source.md").read_text()
    content = re.sub(
        r"\{\{([a-z_]+):([^}]+)\}\}", lambda m: format(d[m[1]], m[2]), content
    )
    mass = "| Item | Mass (g) | Basis |\n| --- | ---: | --- |\n"
    names = {
        "Carbon frame, landing supports": "Frame / landing supports",
        "4 x MN3508 KV380, with leads": "4 x MN3508 KV380",
        "4 x AIR40A ESC, installed allowance": "4 x AIR40A ESC",
        "4 x P14x4.8, upper mass tolerance": "4 x P14x4.8",
        "Pixhawk 6C, plastic case": "Pixhawk 6C (plastic)",
        "Orin Nano compute assembly": "Orin Nano assembly",
        "2 x BFS-U3-122S6M-C bodies": "2 x BFS-U3-122S6M-C",
        "2 x RT-V1228-MPY2 lenses, installed": "2 x RT-V1228-MPY2",
        "2 x pitch servos and brackets": "Paired pitch supports",
        "GEPRC VTC6 6S2P 6 Ah, installed": "VTC6 6S2P 6 Ah",
        "Regulators, distribution, LEDs, harness": "Regulators / LEDs / harness",
    }
    compact_basis = {
        "Engineering geometry estimate": "Geometry allocation",
        "77g carrier + module/cooling/storage allocation [14,15]": "Module / carrier / cooling [14,15]",
        "110 g each allocation [36]": "110 g each installed [36]",
        "70 g paired installed allocation [18]": "70 g installed [18]",
        "607 g product + 20 g allowance [37]": "607 g + 20 g installed [37]",
    }
    mass += "\n".join(
        f"| {names.get(name, name)} | {weight:g} | {compact_basis.get(basis, basis)} |"
        for name, weight, basis in c["mass_items"]
    )

    mass += f"\n| Total installed | {data['mass_g']:.1f} | Includes 100 g contingency |"
    energy = "Table 5. Mission budget with two full local recoveries.\n\n| Phase | Time (s) | Nominal (Wh) | Adverse (Wh) |\n| --- | ---: | ---: | ---: |\n"
    grouped = {}
    for phase in r["phases"]:
        key = phase["phase"].replace("_", " ")
        if key not in grouped:
            grouped[key] = [0.0, 0.0, 0.0]
        for i, k in enumerate(["seconds", "nominal_wh", "adverse_wh"]):
            grouped[key][i] += phase[k]
    energy += "\n".join(
        f"| {k.capitalize()} | {v[0]:.1f} | {v[1]:.2f} | {v[2]:.2f} |"
        for k, v in grouped.items()
    )
    energy += f"\n| Total | {r['total_seconds']:.1f} | {r['nominal_wh']:.2f} | {r['adverse_wh']:.2f} |"
    selection = json.loads((results / "final/selection.json").read_text())
    scores = "| Controller / checkpoint | Mean RMSE (m) | Selection outcome |\n| --- | ---: | --- |\n"
    scores += "\n".join(
        f"| {k.replace('_', ' ')} | {v:.4f} | "
        + (
            "Selected"
            if k == selection["selected"]
            else "Baseline"
            if k == "baseline"
            else "Comparison"
        )
        + " |"
        for k, v in selection["scores_rmse_m"].items()
    )
    screen = json.loads((results / "candidate18/battery_screen.json").read_text())
    retry = "| Recoveries | Duration (s) | Mission (Wh) | Reserve, 10 mΩ | Reserve, 20 mΩ |\n| ---: | ---: | ---: | ---: | ---: |\n"
    for case in data["cases"]:
        n = case["retry_instances"]
        entries = [
            x
            for x in screen["results"]
            if x["cutoff_v"] == 18
            and abs(x["capacity_scale"] - 3000 / 3120) < 1e-8
            and x["recoveries"] == n
        ]
        entries = {x["interconnect_ohm"]: x for x in entries}
        assert abs(entries[0.01]["mission_energy_wh"] - case["adverse_wh"]) < 1e-7
        retry += f"| {n} | {case['total_seconds']:.1f} | {case['adverse_wh']:.2f} | {entries[0.01]['reserve_fraction'] * 100:.2f}% | {entries[0.02]['reserve_fraction'] * 100:.2f}% |\n"
    bm = json.loads((results / "final/baseline/metrics.json").read_text())
    pm = json.loads((results / "final/inference/metrics.json").read_text())
    full = "| Measure | Baseline | Residual | Interpretation |\n| --- | ---: | ---: | --- |\n"
    full += f"| RMSE (m) | {bm['position_rmse_m']:.4f} | {pm['position_rmse_m']:.4f} | {(1 - pm['position_rmse_m'] / bm['position_rmse_m']) * 100:.1f}% reduction |\n"
    full += f"| Position p95 (m) | {bm['position_p95_m']:.4f} | {pm['position_p95_m']:.4f} | Within-route percentile |\n"
    full += f"| Duration (s) | {bm['duration_s']:.2f} | {pm['duration_s']:.2f} | Same prescribed route |\n"
    full += "| Collision | None | None | MuJoCo contact log |"
    content = (
        content.replace("{{MASS_TABLE}}", mass)
        .replace("{{ENERGY_TABLE}}", energy)
        .replace("{{POLICY_TABLE}}", scores)
        .replace("{{RETRY_TABLE}}", retry)
        .replace("{{FULL_ROUTE_TABLE}}", full)
    )
    table_number = iter(range(1, 30))
    content = re.sub(
        r"^Table \d+\.",
        lambda m: "Table " + str(next(table_number)) + ".",
        content,
        flags=re.M,
    )
    assert "{{" not in content
    used = set()
    for group in re.findall(r"\[([0-9, –-]+)\]", content):
        for item in group.split(","):
            bounds = re.split("[–-]", item.strip())
            used.update(range(int(bounds[0]), int(bounds[-1]) + 1))
    sources = {s["id"]: s for s in json.loads((ROOT / "docs/sources.json").read_text())}
    assert used <= sources.keys()
    refs = [
        f"[{i}] {sources[i].get('bibliography', sources[i]['title'])}. "
        + sources[i]["url"]
        for i in sorted(used)
    ]
    return content + "\n\n## References\n\n" + "\n\n".join(refs) + "\n", c, d


def blocks(text):
    """Read the small Markdown subset used by the report source."""
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if not line:
            i += 1
            continue
        if line.startswith("#"):
            level = len(line) - len(line.lstrip("#"))
            yield ("heading", level, line[level:].strip())
            i += 1
        elif line.startswith("|"):
            table = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                cells = [s.strip() for s in lines[i].strip().strip("|").split("|")]
                if not all(re.fullmatch(r":?-+:?", cell) for cell in cells):
                    table.append(cells)
                i += 1
            assert all(len(row) == len(table[0]) for row in table), "Ragged table"
            yield ("table", table)
        elif line.startswith("!["):
            m = re.fullmatch(r"!\[(.*)\]\(([^)]+)\)", line)
            assert m, f"Invalid figure: {line}"
            yield ("figure", m.group(1), m.group(2))
            i += 1
        else:
            paras = [line]
            i += 1
            while (
                i < len(lines)
                and lines[i].strip()
                and not lines[i].startswith(("#", "|", "!["))
            ):
                paras.append(lines[i].strip())
                i += 1
            yield ("paragraph", " ".join(paras))


def build(results, out):
    from .latex_report import build_latex

    return build_latex(Path(results), Path(out))
