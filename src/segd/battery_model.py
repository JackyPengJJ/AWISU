"""Loaded VTC6 pack screening with traced typical-cell curves and explicit losses."""

import json
from pathlib import Path
import numpy as np
from .flight import ROOT


class LoadedPack:
    def __init__(self, settings):
        self.s = settings.copy()
        rows = np.genfromtxt(
            ROOT / "docs/audit/vtc6_discharge.csv", delimiter=",", names=True
        )
        self.q = rows["capacity_mah"] / 1000
        self.v10 = rows["voltage_10a_v"]
        self.v15 = rows["voltage_15a_v"]
        self.used_cell_ah = 0.0
        self.delivered_wh = 0.0
        self.interconnect_loss_wh = 0.0
        self.max_current_a = 0.0
        self.last_voltage_v = None

    def cell_voltage(self, cell_current_a, capacity_ah=None):
        q = (self.used_cell_ah if capacity_ah is None else capacity_ah) / self.s[
            "capacity_scale"
        ]
        if q > self.q[-1]:
            raise ValueError("Discharge curve exhausted; no extrapolation")
        if cell_current_a > 15 + 1e-8:
            raise ValueError("Cell current exceeds digitised 15 A curve")
        a = np.clip((cell_current_a - 10) / 5, 0.0, 1.0)
        # Below 10 A use the loaded 10 A curve as a monotonic rate bound.
        return float(
            (1 - a) * np.interp(q, self.q, self.v10)
            + a * np.interp(q, self.q, self.v15)
            - self.s["curve_voltage_shift_v"]
        )

    def voltage(self, pack_current_a, capacity_ah=None):
        return (
            self.s["series"]
            * self.cell_voltage(pack_current_a / self.s["parallel"], capacity_ah)
            - pack_current_a * self.s["interconnect_resistance_ohm"]
        )

    def operating_point(self, power_w):
        if power_w <= 0:
            raise ValueError("Positive electrical load required")
        q = self.used_cell_ah / self.s["capacity_scale"]
        if q > self.q[-1]:
            return None
        v10 = float(np.interp(q, self.q, self.v10)) - self.s["curve_voltage_shift_v"]
        v15 = float(np.interp(q, self.q, self.v15)) - self.s["curve_voltage_shift_v"]
        ns = self.s["series"]
        np_ = self.s["parallel"]
        r = self.s["interconnect_resistance_ohm"]

        def root(a, b):
            discriminant = a * a - 4 * b * power_w
            return (
                2 * power_w / (a + discriminant**0.5)
                if discriminant >= 0
                else float("inf")
            )

        current = root(ns * v10, r)
        if current > 10 * np_:
            slope = (v10 - v15) / (5 * np_)
            current = root(ns * (v10 + 10 * np_ * slope), ns * slope + r)
        if current > self.s["pack_current_limit_a"] or current > 15 * np_:
            return None
        voltage = self.voltage(current)
        if voltage < self.s["loaded_cutoff_v"]:
            return None
        return current, voltage

    def step(self, power_w, dt):
        point = self.operating_point(power_w)
        if point is None:
            return None
        current, voltage = point
        qnew = self.used_cell_ah + current / self.s["parallel"] * dt / 3600
        if (
            qnew / self.s["capacity_scale"] > self.q[-1]
            or self.voltage(current, qnew) < self.s["loaded_cutoff_v"]
        ):
            return None
        self.used_cell_ah = qnew
        self.delivered_wh += power_w * dt / 3600
        self.interconnect_loss_wh += (
            current**2 * self.s["interconnect_resistance_ohm"] * dt / 3600
        )
        self.max_current_a = max(self.max_current_a, current)
        self.last_voltage_v = voltage
        return point


def mission_screen(case, settings, adverse=True, tail_power_w=None):
    pack = LoadedPack(settings)
    kind = "adverse_wh" if adverse else "nominal_wh"
    elapsed = 0.0
    failures = []
    for phase in case["phases"]:
        duration = phase["seconds"]
        if duration <= 0:
            continue
        load = phase[kind] * 3600 / duration
        local = 0.0
        while local < duration - 1e-9:
            dt = min(settings["integration_step_s"], duration - local)
            if pack.step(load, dt) is None:
                failures.append(
                    {"phase": phase["phase"], "time_s": elapsed, "load_w": load}
                )
                break
            local += dt
            elapsed += dt
        if failures:
            break
    mission_energy = pack.delivered_wh
    mission_q = pack.used_cell_ah
    mission_voltage = pack.last_voltage_v
    loss_at_mission = pack.interconnect_loss_wh
    # Remaining reserve is evaluated under a specified continued electrical load,
    # not inferred by multiplying nominal Ah. This is a load-history model.
    tail_power_w = tail_power_w or max(
        p[kind] * 3600 / p["seconds"] for p in case["phases"] if p["seconds"] > 0
    )
    extra = 0.0
    while not failures and extra < 3600:
        if pack.step(tail_power_w, settings["integration_step_s"]) is None:
            break
        extra += settings["integration_step_s"]
    remaining = pack.delivered_wh - mission_energy
    reserve = remaining / pack.delivered_wh if pack.delivered_wh else 0.0
    return {
        "mission_finished": not failures,
        "mission_energy_wh": mission_energy,
        "mission_time_s": elapsed,
        "mission_end_voltage_v": mission_voltage,
        "used_cell_ah": mission_q,
        "interconnect_loss_during_mission_wh": loss_at_mission,
        "tail_power_w": tail_power_w,
        "deliverable_under_mission_then_tail_wh": pack.delivered_wh,
        "remaining_tail_energy_wh": remaining,
        "remaining_tail_time_s": extra,
        "reserve_fraction": reserve,
        "reserve_pass": not failures and reserve >= settings["reserve_fraction"],
        "maximum_pack_current_a": pack.max_current_a,
        "failures": failures,
    }


def screen_all(out):
    from .final_design import calculate

    design = calculate()
    config = json.loads((ROOT / "configs/candidate18.json").read_text())
    s = config["battery"]
    rows = []
    for cutoff in [19.8, 18.0]:
        for r in [0.0, 0.01, 0.02, 0.03]:
            for scale in [1.0, 3000 / 3120, 0.90]:
                ss = {
                    **s,
                    "loaded_cutoff_v": cutoff,
                    "interconnect_resistance_ohm": r,
                    "capacity_scale": scale,
                }
                for case in design["cases"]:
                    result = mission_screen(case, ss)
                    rows.append(
                        {
                            "cutoff_v": cutoff,
                            "interconnect_ohm": r,
                            "capacity_scale": scale,
                            "recoveries": case["retry_instances"],
                            **result,
                        }
                    )
    ordering = []
    for case in design["cases"]:
        for order, reverse in [("low_then_high", False), ("high_then_low", True)]:
            ordered = {
                **case,
                "phases": sorted(
                    case["phases"],
                    key=lambda p: p["adverse_wh"] / p["seconds"] if p["seconds"] else 0,
                    reverse=reverse,
                ),
            }
            ordering.append(
                {
                    "recoveries": case["retry_instances"],
                    "order": order,
                    **mission_screen(ordered, s),
                }
            )
    result = {
        "ordering_sensitivity": ordering,
        "model": "Typical 23 C cell curves, quasi-static constant power, explicit interconnect drop/loss; no cell-resistance double counting",
        "settings": config,
        "results": rows,
        "not_modelled": [
            "temperature other than 23 C",
            "parallel-cell current imbalance",
            "cell mismatch or hysteresis",
            "electrothermal transient response",
            "guaranteed minimum-cell voltage curve",
        ],
        "profile_order": "Existing engineering phase list; recovery and admission blocks follow motion blocks. Tail at maximum phase-average adverse load. Additional chronological placement is a separate sensitivity.",
    }
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "battery_screen.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--out", default="runs/candidate18")
    a = p.parse_args()
    r = screen_all(a.out)
    print(
        json.dumps(
            [
                x
                for x in r["results"]
                if x["recoveries"] == 2 and x["capacity_scale"] == 3000 / 3120
            ],
            indent=2,
        )
    )
