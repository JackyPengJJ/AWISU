"""Checks for the tighter projection bound, without assuming decoder success."""

import copy
import math
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from segd.design_options import options
from segd.optical_model import safe_halfwidth
from segd.matched_optics import evaluate, depth_and_sampling


class MatchedOpticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, c, _ = options()
        cls.o = copy.deepcopy(c["optics"])
        cls.o["height_grid_step_m"] = 0.01

    def test_short_original_labels_and_dense_counterexample(self):
        short = evaluate(self.o, 99)
        dense = evaluate(self.o, 187)
        self.assertTrue(short["geometry_and_sampling_pass"])
        self.assertFalse(dense["geometry_and_sampling_pass"])
        large = copy.deepcopy(self.o)
        large["label_outer_width_m"] = 0.2
        self.assertTrue(evaluate(large, 187)["geometry_and_sampling_pass"])

    def test_jacobian_bound_against_rotated_rays(self):
        o = self.o
        rng = np.random.default_rng(5110)
        half = 0.85 * 0.604 / 2
        fx = 12 / 0.00345
        for _ in range(300):
            z = rng.uniform(-0.8, 0.8)
            valid = [p for p in o["pitch_deg"] if safe_halfwidth(o, z, p) >= half]
            self.assertTrue(valid)
            pitch = valid[rng.integers(len(valid))]
            depth, bound = depth_and_sampling(o, z, pitch, half)

            def randrot(deg):
                axis = rng.normal(size=3)
                axis /= np.linalg.norm(axis)
                return Rotation.from_rotvec(
                    axis * math.radians(deg) * rng.random()
                ).as_matrix()

            camera = randrot(2)
            label = randrot(5)
            t = math.radians(pitch)
            basis = (
                np.array(
                    [
                        [1, 0, 0],
                        [0, -math.sin(t), math.cos(t)],
                        [0, math.cos(t), math.sin(t)],
                    ]
                )
                @ camera
            )
            point = np.array(
                [
                    rng.uniform(-half, half) + rng.uniform(-0.01, 0.01),
                    rng.uniform(0.8, 0.95),
                    z + rng.uniform(-0.05, 0.05),
                ]
            ) + label @ np.array(
                [rng.uniform(-0.05, 0.05), 0, rng.uniform(-0.022, 0.022)]
            )
            q = basis @ point
            a = basis @ label[:, 0]
            actual = fx * (a[0] - (q[0] / q[2]) * a[2]) / q[2]
            self.assertLessEqual(q[2], depth + 1e-9)
            self.assertGreaterEqual(actual, bound - 1e-9)
            tx = (o["pixels"][0] / 2 - o["image_border_px"]) / fx
            ty = (o["pixels"][1] / 2 - o["image_border_px"]) / fx
            # Check all four finite-label corners, rather than a centre ray.
            centre = np.array(
                [
                    rng.uniform(-half, half),
                    rng.uniform(0.8, 0.95),
                    z + rng.uniform(-0.05, 0.05),
                ]
            )
            for sx in [-1, 1]:
                for sz in [-1, 1]:
                    corner = basis @ (
                        centre + label @ np.array([sx * 0.05, 0, sz * 0.022])
                    )
                    self.assertLessEqual(abs(corner[0] / corner[2]), tx + 1e-8)
                    self.assertLessEqual(abs(corner[1] / corner[2]), ty + 1e-8)
