import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from segd.navigation import Navigator


class NavigationTests(unittest.TestCase):
    def test_scan_registration_uses_no_scene_map(self):
        rng = np.random.default_rng(21)
        points = np.vstack(
            [
                np.column_stack(
                    (rng.uniform(-3, 3, 800), np.ones(800) * 2, rng.uniform(0, 4, 800))
                ),
                np.column_stack(
                    (np.ones(800) * 3, rng.uniform(-2, 2, 800), rng.uniform(0, 4, 800))
                ),
                np.column_stack(
                    (rng.uniform(-3, 3, 800), rng.uniform(-2, 2, 800), np.zeros(800))
                ),
            ]
        )
        nav = Navigator([0, 0, 0])
        self.assertTrue(nav.update(points))
        displacement = np.array([0.08, 0.04, 0.03])
        rot = Rotation.from_rotvec([0.003, 0.002, 0.01]).as_matrix()
        scan = (points - displacement) @ rot
        nav.p = displacement + np.array([0.025, -0.01, 0.01])
        nav.R = rot.copy()
        self.assertTrue(nav.update(scan))
        self.assertLess(np.linalg.norm(nav.p - displacement), 0.03)

    def test_stationary_imu(self):
        nav = Navigator([0, 0, 1])
        for _ in range(200):
            nav.propagate([0, 0, 9.81], [0, 0, 0], 0.005)
        np.testing.assert_allclose(nav.p, [0, 0, 1], atol=1e-10)


if __name__ == "__main__":
    unittest.main()
