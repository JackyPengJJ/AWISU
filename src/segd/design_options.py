"""Camera, label and battery alternatives under a common mission model."""

import copy, json
from .optical_model import load, coverage, sampling, mission
from .flight import ROOT


def options():
    base, old = load()
    settings = json.loads((ROOT / "docs/design/followup_candidate.json").read_text())
    candidate = copy.deepcopy(old)
    candidate["optics"].update(settings["optics"])
    candidate["mission"].update(settings["mission"])
    for row in base["mass_items"]:
        if row[0].startswith("2 x LM12JC5MC"):
            row[1] = 2 * settings["lens_allocation_g_each"]
        if row[0].startswith("2 x pitch servos"):
            row[1] = settings["pitch_assembly_allocation_g"]
    return base, candidate, settings


def results():
    base, candidate, s = options()
    o = candidate["optics"]
    c = coverage(o)
    sample = sampling(o)
    cases = {}
    for battery in s["batteries"]:
        rows = []
        for count in s["retry_counts"]:
            r = mission(
                base,
                candidate,
                battery_g=battery["installed_allocation_g"],
                capacity_ah=battery["capacity_ah"],
                retry_fraction=count / candidate["mission"]["cargo_instances"],
            )
            assert r["hover_interpolation_valid"] and r["retry_instances"] == count
            # The older model's 0.8 usable factor is not evidence about a Li-ion
            # pack. Export required delivered energy instead of a false pass.
            rows.append(
                {
                    k: r[k]
                    for k in [
                        "retry_instances",
                        "retry_seconds_each",
                        "total_seconds",
                        "mass_g",
                        "mass_margin_g",
                        "hover_propulsion_w",
                        "nominal_wh",
                        "adverse_wh",
                        "low_voltage_twr",
                    ]
                }
            )
            rows[-1]["required_delivered_wh_nominal"] = r["nominal_wh"] / 0.8
            rows[-1]["required_delivered_wh_adverse"] = r["adverse_wh"] / 0.8
            rows[-1]["battery_cutoff_qualification"] = (
                "not established by nameplate capacity"
            )
        cases[battery["name"]] = rows
    n = s["code128_b_data_symbols"]
    modules = 11 * n + 55
    labels = []
    for width_mm in s["label_width_comparison_mm"]:
        labels.append(
            {
                "width_mm": width_mm,
                "modules_including_quiet_zones": modules,
                "module_mm": width_mm / modules,
                "pixels_per_module_lower_bound": sample["min_pixels_per_metre"]
                * width_mm
                / modules
                / 1000,
            }
        )
    valid = (
        o["scan_speed_mps"] * c["two_observation_horizon_s"]
        <= c["minimum_safe_width_m"]
    )
    return {
        "status": "research comparison; installed configuration and 18 V shared-current screen are exported by final_design",
        "thrust_screen_voltage_v": 19.8,
        "thrust_screen_excludes_shared_current": True,
        "coverage": c,
        "sampling": sample,
        "capture_schedule_geometry_pass": valid,
        "labels": labels,
        "batteries": cases,
        "raw_camera_mb_s": 2 * o["pixels"][0] * o["pixels"][1] * 10 / 1e6,
        "local_retry_policy": [
            "Keep confirmed items on both sides; do not re-read them after a local retry.",
            "Try extra exposures / illumination while the current label is still inside its valid projected footprint.",
            "If still unconfirmed, pause coverage progression and brake within a verified stopping corridor.",
            "Recenter locally, take settled observations, then resume the same unfinished coverage boundary.",
            "Do not mark an item complete or schedule an aisle-end return merely because its station was passed.",
            "Persistent failure or inadequate return reserve yields an exception/abort, never a complete-inventory claim.",
        ],
        "important_limit": "Window-only recovery cannot be guaranteed for every arbitrary label phase; local brake/recenter time is still charged to every failed item in the budget.",
    }
