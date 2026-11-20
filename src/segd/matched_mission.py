"""Known-route warehouse test of the revised plant; no learned or SLAM state."""

import json
import math
from pathlib import Path
import numpy as np
from .optical_model import segment_time
from .candidate_control import run
from .final_design import calculate


class Mission:
    def __init__(self):
        self.segments = []
        self.clock = 0.0
        self.position = np.array([-2.0, 0, 0.25])
        self.yaw = 0.0

        def hold(seconds, phase):
            self.add(
                self.position, self.position, seconds, phase, self.yaw, self.yaw, 0.0
            )

        def move(target, speed, phase):
            target = np.array(target, float)
            d = np.linalg.norm(target - self.position)
            duration = segment_time(d, speed, 0.6)
            self.add(
                self.position,
                target,
                duration,
                phase,
                self.yaw,
                self.yaw,
                min(speed / 0.6, math.sqrt(d / 0.6)),
            )
            self.position = target

        d = calculate()
        recovery = d["cases"][0]["retry_seconds_each"]
        lateral = d["lateral_recovery_seconds_each"]
        hold(10, "initialization")
        move([-2, 0, 0.8], 0.35, "takeoff")
        move([0, 0, 0.8], 1, "approach")
        for layer in range(5):
            z = 0.8 + 1.6 * layer
            hold(0.6, "boundary_pair")
            end = 80 if layer % 2 == 0 else 0
            if layer in [1, 3]:
                # Declared failure injections test immediate local recovery; no
                # claim that the detector measured this failure frequency.
                move([41, 0, z], 0.85, "scan")
                hold(recovery - 0.85 / 0.6, "retry_processing_and_recenter_allowance")
                # Small triangular recovery, using the allocated 0.3 m/s² limit.
                self.lateral(0.1, z)
                hold(0.3, "settled_pair")
                self.lateral(0, z)
                move([end, 0, z], 0.85, "scan")
            else:
                move([end, 0, z], 0.85, "scan")
            hold(0.6, "boundary_pair")
            hold(0.5, "endpoint_observation")
            newyaw = self.yaw + math.pi
            self.add(
                self.position,
                self.position,
                4,
                "yaw_and_reacquisition",
                self.yaw,
                newyaw,
                0.0,
            )
            self.yaw = newyaw
            if layer < 4:
                move([end, 0, z + 1.6], 0.35, "climb")
        move([0, 0, 7.2], 1, "return")
        move([-2, 0, 7.2], 1, "return")
        move([-2, 0, 0.8], 0.35, "descend")
        move([-2, 0, 0.25], 0.35, "land")
        self.duration = self.clock

    def add(self, a, b, duration, phase, y0, y1, ramp, acc=0.6):
        self.segments.append(
            (
                self.clock,
                duration,
                np.array(a).copy(),
                np.array(b).copy(),
                phase,
                y0,
                y1,
                ramp,
                acc,
            )
        )
        self.clock += duration

    def lateral(self, y, z):
        target = np.array([self.position[0], y, z])
        dist = np.linalg.norm(target - self.position)
        ramp = math.sqrt(dist / 0.3)
        self.add(
            self.position,
            target,
            2 * ramp,
            "lateral_recovery",
            self.yaw,
            self.yaw,
            ramp,
            0.3,
        )
        self.position = target

    def __call__(self, t):
        if t >= self.duration:
            return self.position.copy(), np.zeros(3), np.zeros(3), self.yaw
        s = self.segments[-1]
        for candidate in self.segments:
            if t <= candidate[0] + candidate[1] + 1e-10:
                s = candidate
                break
        t0, duration, a, b, phase, y0, y1, ramp, acc = s
        u = np.clip(t - t0, 0, duration)
        length = np.linalg.norm(b - a)
        p = a.copy()
        v = np.zeros(3)
        ac = np.zeros(3)
        if length > 1e-9:
            direction = (b - a) / length
            peak = acc * ramp
            if u < ramp:
                distance = 0.5 * acc * u * u
                vel = acc * u
                aa = acc
            elif u < duration - ramp:
                distance = 0.5 * acc * ramp * ramp + peak * (u - ramp)
                vel = peak
                aa = 0
            else:
                remaining = duration - u
                distance = length - 0.5 * acc * remaining**2
                vel = acc * remaining
                aa = -acc
            p = a + direction * distance
            v = direction * vel
            ac = direction * aa
        z = u / duration if duration else 1.0
        yaw = y0 + (y1 - y0) * (3 * z * z - 2 * z * z * z)
        return p, v, ac, yaw


def main(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    mission = Mission()
    result, rows, xml, mech = run(
        seed=703, reference_fn=mission, duration_s=mission.duration + 1, warehouse=True
    )
    result.update(
        test="Full known-route matched warehouse rigid-body test",
        mission_duration_s=mission.duration,
        injected_recoveries=2,
        unknown_navigation=False,
        image_decode_during_flight=False,
        energy_model="Current is evaluated at a fixed 18 V endpoint; loaded depletion/reserve is a separate forecast",
        end_space="Observed-free open staging regions beyond rack ends; no end wall is assumed in this flight scene",
    )
    result.pop("braking_stop_time_s")
    result.pop("braking_stop_distance_m")
    result.pop("final_yaw_error_deg")
    phase = [
        next(s[4] for s in mission.segments if row[0] <= s[0] + s[1] + 1e-9)
        if row[0] <= mission.duration
        else "done"
        for row in rows
    ]
    airborne = np.array(
        [p in ["scan", "climb", "return", "lateral_recovery"] for p in phase]
    )
    margin = np.minimum(
        1.25
        - np.abs(rows[:, 2])
        - (
            0.5 * np.cos(np.deg2rad(rows[:, 10]))
            + 0.25 * np.sin(np.deg2rad(rows[:, 10]))
        ),
        8
        - rows[:, 3]
        - (
            0.25 * np.cos(np.deg2rad(rows[:, 10]))
            + 0.5 * np.sin(np.deg2rad(rows[:, 10]))
        ),
    )
    result["minimum_nominal_wall_ceiling_clearance_m"] = float(margin[airborne].min())
    (out / "metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    np.savetxt(
        out / "trajectory.csv",
        rows,
        delimiter=",",
        header="time_s,x,y,z,ref_x,ref_y,ref_z,vx,vy,vz,tilt_deg,current_a,rp_scale,yaw_scale,yaw_rad",
        comments="",
    )
    (out / "schedule.json").write_text(
        json.dumps(
            [
                {
                    "start_s": s[0],
                    "duration_s": s[1],
                    "from_m": s[2].tolist(),
                    "to_m": s[3].tolist(),
                    "phase": s[4],
                    "yaw_from_rad": s[5],
                    "yaw_to_rad": s[6],
                }
                for s in mission.segments
            ],
            indent=2,
        )
        + "\n"
    )
    (out / "scene.xml").write_text(xml)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--out", required=True)
    main(p.parse_args().out)
