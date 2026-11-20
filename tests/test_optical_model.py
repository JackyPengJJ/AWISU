"""Review calculations exercise uncertainty, sampling and mission rejection."""

import copy
import math
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from segd.optical_model import load, coverage, safe_halfwidth, normals, mission


class DesignReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base, cls.candidate = load()
        cls.o = cls.candidate["optics"]
        cls.cov = coverage(cls.o)

    def test_height_cells_and_double_observation_speed(self):
        self.assertTrue(self.cov["coverage_ok"])
        bad = copy.deepcopy(self.o)
        bad["label_outer_height_m"] = 0.3
        self.assertEqual(safe_halfwidth(bad, 0.28, 0), 0.0)
        self.assertLess(self.cov["speed_limit_mps"], 1.0)

    def test_containment_survives_rotated_finite_labels(self):
        rng = np.random.default_rng(18)
        for _ in range(160):
            z = float(rng.uniform(-0.8, 0.8))
            widths = [safe_halfwidth(self.o, z, p) for p in self.o["pitch_deg"]]
            pitch = self.o["pitch_deg"][int(np.argmax(widths))]
            h = max(widths)
            self.assertGreater(h, 0)
            x = rng.uniform(-h, h)
            d = rng.uniform(*self.o["standoff_m"])
            dz = rng.uniform(-0.05, 0.05)
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            rotation = Rotation.from_rotvec(
                axis * rng.uniform(0, math.radians(2))
            ).as_matrix()
            labelrot = Rotation.from_euler("xyz", rng.uniform(-1, 1, 3)).as_rotvec()
            labelrot = Rotation.from_rotvec(
                labelrot / np.linalg.norm(labelrot) * math.radians(5) * rng.random()
            ).as_matrix()
            ns, _ = normals(self.o, pitch)
            for sx in [-1, 1]:
                for sz in [-1, 1]:
                    q = np.array(
                        [x + rng.uniform(-0.01, 0.01), d, z + dz]
                    ) + labelrot @ np.array([sx * 0.1, 0, sz * 0.02])
                    self.assertTrue(np.all(ns @ (rotation @ q) > -1e-9))

    def test_periodic_pairs_cover_arbitrary_phase_and_jitter(self):
        cycle = self.o["cycle_s"]
        j = self.o["timestamp_jitter_s"]
        width = self.cov["minimum_safe_width_m"]
        v = self.o["scan_speed_mps"]
        for phase in np.linspace(0, cycle, 301):
            # Include extreme timing jitter combinations for three cycles.
            for sign in [-1, 1]:
                events = np.array(
                    [
                        n * cycle + t + sign * j * (-1) ** i
                        for n in range(-2, 4)
                        for i, t in enumerate([0.14, 0.19])
                    ]
                )
                inview = events[(events >= phase) & (events <= phase + width / v)]
                self.assertGreaterEqual(len(inview), 2)
                self.assertGreaterEqual(float(inview[-1] - inview[0]), 0.046 - 1e-10)
        # Independent jitter: count events guaranteed to stay within each window.
        events = np.array([n * cycle + t for n in range(-2, 4) for t in [0.14, 0.19]])
        for phase in np.linspace(0, cycle, 301):
            guaranteed = events[
                (events >= phase + j) & (events <= phase + width / v - j)
            ]
            self.assertGreaterEqual(len(guaranteed), 2)
        # An accepted pair can straddle cycles; it need not be one 50 ms burst.
        inview = events[(events >= 0.15) & (events <= 0.15 + width / v)]
        self.assertGreaterEqual(len(inview), 2)
        self.assertGreater(inview[1] - inview[0], 0.5)
        # Original one-per-pose sampling has a real phase with only one image.
        events = np.arange(-2, 4) * 0.4
        self.assertEqual(int(np.sum((events >= 0.01) & (events <= 0.01 + 0.463))), 1)

    def test_adverse_energy_conflict_is_retained(self):
        r = mission(self.base, self.candidate, battery_g=852, capacity_ah=6)
        self.assertLess(r["mass_g"], 3000)
        self.assertGreater(r["low_voltage_twr"], 1.5)
        self.assertGreater(r["adverse_wh"], r["allowed_wh"])
        self.assertFalse(r["passes_mass_thrust_energy"])
        self.assertGreater(r["retry_seconds_each"], 6)
        zero = mission(
            self.base, self.candidate, battery_g=852, capacity_ah=6, retry_fraction=0
        )
        self.assertLess(zero["adverse_wh"], r["adverse_wh"])
        p15 = mission(self.base, self.candidate, prop="P15")
        self.assertFalse(p15["hover_interpolation_valid"])
