"""Audit assignment geometry, barcode assumptions and battery/current closure.

This reads the retained experiment and the selected engineering inputs separately.
Manufacturer discharge curves are typical single-cell data, not pack test results.
"""

import json
import math
from pathlib import Path
import numpy as np
from barcode import Code128
from .flight import ROOT, create_model


def delivered_energy(capacity_mah, voltage_v, cutoff_v, voltage_shift_v=0.0):
    """Piecewise-linear integral, truncated at the first loaded-voltage cutoff."""
    q = np.asarray(capacity_mah, dtype=float) / 1000
    v = np.asarray(voltage_v, dtype=float) - voltage_shift_v
    if np.any(np.diff(q) <= 0) or np.any(np.diff(v) > 1e-8):
        raise ValueError(
            "Discharge trace must have increasing capacity and decreasing voltage"
        )
    if v[0] <= cutoff_v:
        return {"cell_wh": 0.0, "cell_ah": 0.0}
    below = np.flatnonzero(v <= cutoff_v)
    if not len(below):
        raise ValueError("Trace does not reach requested cutoff; no extrapolation")
    k = int(below[0])
    end = q[k - 1] + (q[k] - q[k - 1]) * (v[k - 1] - cutoff_v) / (v[k - 1] - v[k])
    qq = np.r_[q[:k], end]
    vv = np.r_[v[:k], cutoff_v]
    return {"cell_wh": float(np.trapezoid(vv, qq)), "cell_ah": float(end)}


def battery_audit(design):
    rows = np.genfromtxt(
        ROOT / "docs/audit/vtc6_discharge.csv", delimiter=",", names=True
    )
    output = []
    required = design["cases"][0]["required_delivered_wh_adverse"]
    # Series interconnect loss is displayed separately: not a measured pack value.
    for cutoff in [19.8, 18.0]:
        for amps in [10, 15]:
            for shift in [0.0, 0.04]:
                e = delivered_energy(
                    rows["capacity_mah"], rows[f"voltage_{amps}a_v"], cutoff / 6, shift
                )
                pack = 12 * e["cell_wh"]
                output.append(
                    {
                        "pack_cutoff_v": cutoff,
                        "cell_current_a": amps,
                        "curve_voltage_shift_v": shift,
                        "cell_capacity_to_cutoff_ah": e["cell_ah"],
                        "typical_curve_pack_wh": pack,
                        "after_20_percent_reserve_wh": 0.8 * pack,
                        "required_delivered_wh": required,
                        "two_recovery_adverse_comparison_pass": pack >= required,
                    }
                )
    return {
        "source": "docs/audit/vtc6_source.json",
        "series_cells": 6,
        "parallel_cells": 2,
        "temperature_c": 23,
        "results": output,
        "status": "screening from typical constant-current cell curves; no pack guarantee",
        "selection_warning": "19.8 V reserve closure must not be inferred from 129.6 Wh nameplate energy or an assumed 100 Wh delivery",
    }


def current_audit(design):
    mass = design["mass_g"] / 1000
    g = 9.80665
    hover = mass / 4
    b = design["base"]
    force = np.asarray(b["motor_test_thrust_g"]) / 1000
    power = np.asarray(b["motor_test_power_w"])
    arm = 0.6 / (2 * math.sqrt(2))
    loss = 1.25
    aux = 60.0
    out = []

    def per_rotor(t):
        # Below the first measured point, use its power as an upper bound,
        # never a linear extrapolation through zero. This assumes monotone power.
        return np.interp(t, force, power, left=power[0], right=np.nan) * loss

    for voltage in [22.2, 19.8, 18.0]:
        cap = force[-1] * min(1.0, (voltage / 22.2) ** 2) * 0.85

        def allowed(ts):
            return bool(
                max(ts) <= cap + 1e-10
                and (sum(per_rotor(t) for t in ts) + aux) <= 30 * voltage
            )

        lo, hi = hover, min(cap, force[-1])
        for _ in range(50):
            mid = (lo + hi) / 2
            if allowed([mid] * 4):
                lo = mid
            else:
                hi = mid
        balanced = lo
        lo, hi = 0.0, min(hover, cap - hover)
        for _ in range(50):
            mid = (lo + hi) / 2
            if allowed([hover + mid] * 2 + [hover - mid] * 2):
                lo = mid
            else:
                hi = mid
        delta = lo
        testdelta = 0.2 * hover
        tilted = hover / math.cos(math.radians(12))
        out.append(
            {
                "bus_voltage_v": voltage,
                "maximum_collective_twr_model": 4 * balanced / mass,
                "constant_collective_roll_pitch_torque_bound_nm": 4 * arm * delta * g,
                "20_percent_differential_torque_nm": 4 * arm * testdelta * g,
                "20_percent_differential_current_a": (
                    2 * per_rotor(hover + testdelta)
                    + 2 * per_rotor(hover - testdelta)
                    + aux
                )
                / voltage,
                "20_percent_differential_admitted": allowed(
                    [hover + testdelta] * 2 + [hover - testdelta] * 2
                ),
                "12_deg_tilt_collective_admitted": allowed([tilted] * 4),
                "12_deg_horizontal_acceleration_mps2": g * math.tan(math.radians(12)),
            }
        )
    return {
        "manufacturer_map_voltage_v": 22.2,
        "shared_limit_a": 30,
        "installation_power_multiplier": loss,
        "aux_w": aux,
        "voltage_ceiling_model": "maximum thrust scales with (V/22.2)^2 and 0.85 thrust derating; not low-voltage bench data",
        "low_thrust_power_bound": "Power <= first measured point for 0..710 g, assuming monotonicity; no claimed measured low-load curve",
        "roll_pitch_only": True,
        "combined_yaw_authority_not_established": True,
        "results": out,
    }


def scene_audit():
    import mujoco

    run = ROOT / "results/final/inference"
    provenance = json.loads((run / "provenance.json").read_text())
    c = provenance["config"].copy()
    model, xml = create_model(c)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ids = [
        i
        for i in range(model.ngeom)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith(
            "label_W"
        )
    ]
    positions = data.geom_xpos[ids]
    sizes = model.geom_size[ids] * 2
    levels = sorted(set(np.round(positions[:, 2], 6)))
    cargo = [
        i
        for i in range(model.ngeom)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").endswith(
            "_collision"
        )
        and "cargo_" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "")
    ]
    nearest = min(abs(data.geom_xpos[i, 1]) - model.geom_size[i, 1] for i in cargo)
    ycount = {
        str(side): int(np.sum(np.sign(positions[:, 1]) == side)) for side in [-1, 1]
    }
    bounds = []
    for i in range(model.ngeom):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if n.startswith("rack_deck_"):
            bounds.append(
                {
                    "x_min": float(data.geom_xpos[i, 0] - model.geom_size[i, 0]),
                    "x_max": float(data.geom_xpos[i, 0] + model.geom_size[i, 0]),
                    "inner_y": float(abs(data.geom_xpos[i, 1]) - model.geom_size[i, 1]),
                    "top_z": float(data.geom_xpos[i, 2] + model.geom_size[i, 2]),
                }
            )
    names = ["left", "right"]
    camera = []
    for n in names:
        cam = model.camera(n)
        camera.append(
            {
                "name": n,
                "position_body_m": cam.pos.tolist(),
                "fovy_deg": float(cam.fovy[0]),
            }
        )
    metric = json.loads((run / "metrics.json").read_text())
    modules = len(Code128("W0000").build()[0]) + 20
    checks = {
        "rack_length_80m": all(
            abs(b["x_min"]) < 1e-8 and abs(b["x_max"] - 80) < 1e-8 for b in bounds
        ),
        "clear_aisle_2_5m": all(b["inner_y"] >= 1.25 - 1e-8 for b in bounds)
        and nearest >= 1.25 - 1e-8,
        "rack_top_8m": abs(max(b["top_z"] for b in bounds) - 8) < 1e-8,
        "five_levels_1_6m_apart": len(levels) == 5
        and np.allclose(np.diff(levels), 1.6),
        "both_sides_200_each": all(n == 200 for n in ycount.values()),
        "reference_mass_below_3kg": c["mass_kg"] < 3,
        "recorded_route_completed": metric["route_completed"],
    }
    return {
        "checks": {k: bool(v) for k, v in checks.items()},
        "reference_geometry_pass": all(checks.values()),
        "label_count": len(ids),
        "cargo_count": len(cargo),
        "labels_per_side": ycount,
        "label_centre_levels_m": levels,
        "label_dimensions_m": np.unique(sizes[:, :2], axis=0).tolist(),
        "nearest_cargo_edge_abs_y_m": float(nearest),
        "barcode_example": "W0000",
        "encoded_modules_including_quiet_zones": modules,
        "side_cameras": camera,
        "recorded_sensor_image_size_px": [4096, 2160],
        "recorded_state_source": c["state_source"],
        "known_prescribed_route": True,
        "unknown_environment_end_to_end_validated": False,
        "matches_final_hardware_optics": False,
        "matches_final_hardware_mass": False,
        "assignment_label_size_specified": False,
        "assignment_cargo_count_specified": False,
        "end_zone_assumption": "Launch/landing and turning space outside the 80 m aisle must be allocated; not specified by assignment",
        "generated_scene_xml_sha256": __import__("hashlib")
        .sha256(xml.encode())
        .hexdigest(),
    }


def audit(out):
    from .final_design import calculate

    design = calculate()
    result = {
        "scene": scene_audit(),
        "battery": battery_audit(design),
        "current": current_audit(design),
    }
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "technical_audit.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--out", default=str(ROOT / "docs/audit"))
    a = p.parse_args()
    r = audit(a.out)
    print(json.dumps(r, indent=2))
