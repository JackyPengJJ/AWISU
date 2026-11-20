import unittest
import numpy as np
from segd.flight import collision_clearance, create_model, load_config


class CollisionTests(unittest.TestCase):
    def test_compiled_geometry_and_rotated_clearance(self):
        model, _ = create_model(load_config())
        g = model.geom("guard")
        np.testing.assert_allclose(g.size[:2], [0.4, 0.145])
        np.testing.assert_allclose(g.pos, [0, 0, 0.08])
        self.assertAlmostEqual(
            collision_clearance(np.array([10.0, 0, 4]), np.eye(3)), 0.85
        )
        self.assertAlmostEqual(
            collision_clearance(np.array([10.0, 0, 0.065]), np.eye(3)), 0.0
        )
        # 90 degree roll: cylinder centre and axial extent rotate toward the wall.
        R = np.array([[1.0, 0, 0], [0, 0, -1], [0, 1, 0]])
        self.assertAlmostEqual(
            collision_clearance(np.array([10.0, 0, 4]), R), 1.25 - 0.08 - 0.145
        )
