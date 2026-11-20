"""Finite-label visibility, projection bounds and action-based mission budgets."""

import itertools
import json
import math
import numpy as np
from .flight import ROOT, Route


def load():
    candidate = json.loads((ROOT / "docs/design/candidate.json").read_text())
    base = json.loads((ROOT / candidate["base_design"]).read_text())
    return base, candidate


def normals(optics, pitch):
    fx = optics["focal_mm"] / optics["pixel_mm"]
    tx = (optics["pixels"][0] / 2 - optics["image_border_px"]) / fx
    ty = (optics["pixels"][1] / 2 - optics["image_border_px"]) / fx
    t = math.radians(pitch)
    forward = np.array([0, math.cos(t), math.sin(t)])
    right = np.array([1, 0, 0])
    up = np.array([0, -math.sin(t), math.cos(t)])
    ns = np.array(
        [
            tx * forward + right,
            tx * forward - right,
            ty * forward + up,
            ty * forward - up,
        ]
    )
    return ns / np.linalg.norm(ns, axis=1)[:, None], forward


def safe_halfwidth(optics, z, pitch):
    """Cone-robust frustum containment of all vertices of the label/position box.

    n.q >= |q| sin(delta) guarantees the entire direction-uncertainty cone
    stays inside a frustum half-space. This concave margin attains its minimum
    over a rectangular uncertainty box at a vertex; height cells are enclosed.
    """
    ns, _ = normals(optics, pitch)
    z -= optics.get("camera_vertical_offset_m", 0.0)
    dz = (
        optics["label_outer_height_m"] / 2
        + optics["vertical_position_bound_m"]
        + optics["height_grid_step_m"] / 2
    )
    w = optics["label_outer_width_m"] / 2 + optics["along_track_position_bound_m"]
    delta = math.sin(math.radians(optics["orientation_cone_deg"]))
    label_radius = math.hypot(
        optics["label_outer_width_m"] / 2, optics["label_outer_height_m"] / 2
    )
    label_error = (
        2
        * label_radius
        * math.sin(math.radians(optics["label_plane_normal_error_deg"]) / 2)
    )

    def valid(h):
        q = np.array(
            list(
                itertools.product(
                    [-h - w, h + w], optics["standoff_m"], [z - dz, z + dz]
                )
            )
        )
        return bool(
            np.all(
                q @ ns.T
                - np.linalg.norm(q, axis=1)[:, None] * delta
                - label_error * (1 + delta)
                >= 0
            )
        )

    if not valid(0):
        return 0.0
    lo, hi = 0.0, 1.5
    for _ in range(30):
        mid = (lo + hi) / 2
        if valid(mid):
            lo = mid
        else:
            hi = mid
    return lo


def coverage(optics):
    zs = np.linspace(-0.8, 0.8, round(1.6 / optics["height_grid_step_m"]) + 1)
    widths = np.array(
        [
            [2 * safe_halfwidth(optics, float(z), p) for z in zs]
            for p in optics["pitch_deg"]
        ]
    )
    best = np.max(widths, axis=0)
    guaranteed = float(np.min(best))
    period = optics["cycle_s"] + 2 * optics["timestamp_jitter_s"]
    return {
        "height_centres_m": zs.tolist(),
        "safe_width_by_pose_m": widths.tolist(),
        "minimum_safe_width_m": guaranteed,
        "two_observation_horizon_s": period,
        "speed_limit_mps": guaranteed / period,
        "coverage_ok": bool(guaranteed >= optics["scan_speed_mps"] * period),
    }


def sampling(optics):
    """Conservative projection scale bound over the admitted viewing envelope."""
    # All pair exposures occur within the selected safe rectangle. Use its maximum x.
    c = coverage(optics)
    max_x = (
        max(max(w) for w in c["safe_width_by_pose_m"]) / 2
        + optics["label_outer_width_m"] / 2
        + optics["along_track_position_bound_m"]
    )
    radius = math.sqrt(
        optics["standoff_m"][1] ** 2
        + (
            0.8
            + optics["vertical_position_bound_m"]
            + optics["label_outer_height_m"] / 2
            + optics["height_grid_step_m"] / 2
        )
        ** 2
        + max_x**2
    )
    radius += (
        2
        * math.hypot(
            optics["label_outer_width_m"] / 2, optics["label_outer_height_m"] / 2
        )
        * math.sin(math.radians(optics["label_plane_normal_error_deg"]) / 2)
    )
    fx = optics["focal_mm"] / optics["pixel_mm"]
    # ||q|| bounds optical depth; cos accounts for label-plane obliquity. This
    # sufficient scale is deliberately below the centre-ray sampling estimate.
    gamma = math.radians(
        optics["label_plane_normal_error_deg"] + optics["orientation_cone_deg"]
    )
    tx = (optics["pixels"][0] / 2 - optics["image_border_px"]) / fx
    scale = fx * (math.cos(gamma) - tx * math.sin(gamma)) / radius
    return {
        "min_pixels_per_metre": scale,
        "pixels_per_module_bound": scale * optics["module_width_m"],
        "module_for_target_mm": 1000 * optics["pixels_per_module_target"] / scale,
        "normal_blur_px": fx
        * optics["scan_speed_mps"]
        * optics["exposure_s"]
        / optics["standoff_m"][0],
    }


def segment_time(distance, speed, acc):
    ramp = min(speed / acc, math.sqrt(distance / acc))
    return 2 * ramp + max(0.0, distance / (acc * ramp) - ramp) if distance else 0.0


def mission(
    base,
    candidate,
    battery_g=755,
    capacity_ah=5,
    prop="P14",
    retry_fraction=None,
    scan_speed=None,
):
    o = candidate["optics"]
    m = candidate["mission"]
    speed = scan_speed or o["scan_speed_mps"]
    mass = sum(x[1] for x in base["mass_items"]) - 755 + battery_g
    # P15 needs wider motor spacing and explicit extra structural mass.
    if prop == "P15":
        mass += 24 + 30
    thrust, power = (
        (base["motor_test_thrust_g"], base["motor_test_power_w"])
        if prop == "P14"
        else ([820, 1200, 1500, 1700, 1880], [79.92, 135.42, 210.9, 250.86, 295.26])
    )
    # Lower P15 load falls below its first 6S point: do not extrapolate.
    hover_valid = thrust[0] <= mass / 4 <= thrust[-1]
    if not hover_valid:
        return {
            "mass_g": mass,
            "hover_interpolation_valid": False,
            "reason": "Hover below first matched 22.2 V P15 data point; no extrapolation.",
        }
    hover = 4 * float(np.interp(mass / 4, thrust, power))
    kg = mass / 1000
    g = 9.80665
    area = 4 * math.pi * (0.3556 / 2) ** 2
    induced_velocity = math.sqrt(kg * g / (2 * m["air_density_kg_m3"] * area))
    phases = []
    for _, _, a, b, phase, _ in Route(speed).segments:
        a = a.copy()
        b = b.copy()
        if phase == "takeoff":
            a[2] = base["launch_body_height_m"]
        if phase == "land":
            b[2] = base["launch_body_height_m"]
        distance = float(np.linalg.norm(b - a))
        vertical = phase in ("takeoff", "climb", "descend", "land")
        vmax = (
            min(speed, m["vertical_speed_mps"])
            if vertical
            else speed
            if phase == "scan"
            else m["transit_speed_mps"]
        )
        duration = segment_time(distance, vmax, m["acceleration_mps2"])
        # Tilt-induced hover increase plus drag work. Positive vertical work is
        # added for ascent; no descent energy credit is taken near racks.
        tilt_factor = (
            (1 + (m["acceleration_mps2"] / g) ** 2) ** 0.75 if not vertical else 1.0
        )
        drag = (
            0.5 * m["air_density_kg_m3"] * m["cd_area_m2"] * vmax**3
            if not vertical
            else 0.0
        )
        up = max(0, float(b[2] - a[2]))
        climb_j = kg * g * up / m["vertical_efficiency"]
        maneuver = phase in ("takeoff", "climb", "descend", "land")
        pn = (
            hover * m["propulsion_loss_nominal"] * tilt_factor
            + m["aux_nominal_w"]
            + drag
        )
        pa = (
            hover
            * m["propulsion_loss_adverse"]
            * tilt_factor
            * (m["maneuver_induced_factor_adverse"] if maneuver else 1)
            + m["aux_adverse_w"]
            + drag
        )
        phases.append(
            {
                "phase": phase,
                "seconds": duration,
                "nominal_wh": (pn * duration + climb_j) / 3600,
                "adverse_wh": (pa * duration + climb_j) / 3600,
            }
        )
    fraction = m["adverse_retry_fraction"] if retry_fraction is None else retry_fraction
    retries = math.ceil(m["cargo_instances"] * fraction)
    acc = m["acceleration_mps2"]
    r = m["retry_actions_s"]
    brake = speed / acc
    overshoot = speed * r["decode_delay_s"] + speed**2 / (2 * acc)
    # A failed second observation may occur at the far edge of its readable
    # interval. Bound the label offset by the largest camera footprint.
    fx = o["focal_mm"] / o["pixel_mm"]
    radius = math.hypot(
        o["standoff_m"][1],
        0.8 + o["vertical_position_bound_m"] + o["label_outer_height_m"] / 2,
    )
    edge = (o["pixels"][0] / 2 / fx) * radius
    reposition = segment_time(overshoot + edge, r["recenter_speed_mps"], acc)
    retry_each = (
        r["decode_delay_s"]
        + 2 * brake
        + reposition
        + m["max_attempts_per_retry"] * r["settle_and_pair_s"]
    )
    extra = [
        ("initialization", sum(m["initialization_actions_s"].values())),
        ("endpoint_observation", m["endpoint_count"] * m["endpoint_confirmation_s"]),
        ("yaw_and_reacquisition", m["endpoint_count"] * m["yaw_per_endpoint_s"]),
        ("boundary_pairs", 5 * m["boundary_pair_holds_per_level"] * o["cycle_s"]),
        ("targeted_retries", retries * retry_each),
    ]
    for name, seconds in extra:
        phases.append(
            {
                "phase": name,
                "seconds": seconds,
                "nominal_wh": seconds
                * (hover * m["propulsion_loss_nominal"] + m["aux_nominal_w"])
                / 3600,
                "adverse_wh": seconds
                * (hover * m["propulsion_loss_adverse"] + m["aux_adverse_w"])
                / 3600,
            }
        )
    energy = sum(p["adverse_wh"] for p in phases)
    allowed = 22.2 * capacity_ah * 0.8 * 0.8
    twr = 4 * thrust[-1] * (19.8 / 22.2) ** 2 * 0.85 / mass
    return {
        "mass_g": mass,
        "mass_margin_g": 3000 - mass,
        "hover_interpolation_valid": True,
        "hover_propulsion_w": hover,
        "induced_velocity_mps": induced_velocity,
        "capacity_ah": capacity_ah,
        "allowed_wh": allowed,
        "nominal_wh": sum(p["nominal_wh"] for p in phases),
        "adverse_wh": energy,
        "total_seconds": sum(p["seconds"] for p in phases),
        "energy_headroom_wh": allowed - energy,
        "retry_instances": retries,
        "retry_seconds_each": retry_each,
        "retry_braking_overshoot_m": overshoot,
        "retry_reposition_distance_bound_m": overshoot + edge,
        "phases": phases,
        "low_voltage_twr": twr,
        "passes_mass_thrust_energy": bool(
            mass < 3000 and twr >= 1.5 and energy <= allowed
        ),
    }
