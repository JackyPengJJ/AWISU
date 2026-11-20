"""Consistent sizing for the selected report design, including lateral recovery."""

import math, copy
from .design_options import options, results
from .optical_model import mission, segment_time, safe_halfwidth


def recovery_offset(side, unresolved, free_clearance, energy_ok=True, max_offset=0.10):
    """A known-free, stopped recovery; retain confirmed opposite-side tracks."""
    if side not in ("left", "right"):
        raise ValueError("Unknown rack side")
    if not unresolved:
        return {"action": "continue", "offset_m": 0.0}
    if not energy_ok:
        return {"action": "return_with_exception", "offset_m": 0.0}
    offset = min(max_offset, max(0.0, free_clearance - 0.25))
    return {
        "action": "local_offset" if offset > 0 else "settled_retry",
        "offset_m": offset * (1 if side == "left" else -1),
    }


def calculate():
    base, candidate, settings = options()
    candidate["status"] = (
        "selected report design; installed hardware qualification pending"
    )
    follow = results()
    battery = settings["batteries"][0]
    mass = (
        sum(r[1] for r in base["mass_items"]) - 755 + battery["installed_allocation_g"]
    )
    angle = math.radians(12)
    extent = 0.5 * math.cos(angle) + 0.25 * math.sin(angle)
    centre_clearance = 1.25 - extent - 0.15 - 0.15 - 0.05
    offset = 0.10
    assert centre_clearance - offset >= 0.25
    targeted = copy.deepcopy(candidate["optics"])
    targeted["standoff_m"] = [x - offset for x in targeted["standoff_m"]]
    targeted_width = 2 * safe_halfwidth(targeted, 0.0, 0.0)
    assert targeted_width > 0, "Full targeted label must fit after approach"
    # Full out-and-back is charged for every recovery, even if window retry succeeds.
    lateral_s = 2 * segment_time(offset, 0.2, 0.3) + 0.3
    cases = []
    for n in settings["retry_counts"]:
        r = mission(
            base,
            candidate,
            battery_g=battery["installed_allocation_g"],
            capacity_ah=6,
            retry_fraction=n / 400,
        )
        pn = r["hover_propulsion_w"] * 1.1 + 55
        pa = r["hover_propulsion_w"] * 1.25 + 60
        lateral = {
            "phase": "lateral_recovery",
            "seconds": n * lateral_s,
            "nominal_wh": n * lateral_s * pn / 3600,
            "adverse_wh": n * lateral_s * pa / 3600,
        }
        r["phases"].append(lateral)
        for key, total in [("nominal_wh", "nominal_wh"), ("adverse_wh", "adverse_wh")]:
            r[key] += lateral[total]
        r["total_seconds"] += lateral["seconds"]
        r["required_delivered_wh_nominal"] = r["nominal_wh"] / 0.8
        r["required_delivered_wh_adverse"] = r["adverse_wh"] / 0.8
        for key in ["allowed_wh", "energy_headroom_wh", "passes_mass_thrust_energy"]:
            r.pop(key)
        r["battery_cutoff_energy_qualified"] = False
        cases.append(r)
    for row in base["mass_items"]:
        if "camera bodies" in row[0]:
            row[0] = "2 x BFS-U3-122S6M-C bodies"
            row[2] = "Manufacturer [35]"
        if "lenses" in row[0]:
            row[0] = "2 x RT-V1228-MPY2 lenses, installed"
            row[2] = "110 g each allocation [36]"
        if "pitch servos" in row[0]:
            row[2] = "70 g paired installed allocation [18]"
        if "battery" in row[0]:
            row[:] = [
                "GEPRC VTC6 6S2P 6 Ah, installed",
                627,
                "607 g product + 20 g allowance [37]",
            ]
    inertia_x = sum(
        (row[1] / 1000) * (0.212132**2)
        for row in base["mass_items"]
        if "MN3508" in row[0] or "ESC" in row[0] or "P14" in row[0]
    )
    return {
        "base": base,
        "candidate": candidate,
        "settings": settings,
        "followup": follow,
        "cases": cases,
        "mass_g": mass,
        "centre_clearance_m": centre_clearance,
        "recovery_clearance_m": centre_clearance - offset,
        "targeted_recovery_safe_width_m": targeted_width,
        "targeted_recovery_standoff_m": targeted["standoff_m"],
        "offset_m": offset,
        "lateral_recovery_seconds_each": lateral_s,
        "motor_prop_inertia_lower_bound_kgm2": inertia_x,
        "battery_qualification": "delivered energy above 19.8 V under mission load remains an admission condition",
    }
