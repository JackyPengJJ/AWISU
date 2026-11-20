import unittest
import numpy as np
from segd.flight import Route, Controller, load_config, create_model
from segd.barcodes import Inventory, label_image


class CoreTests(unittest.TestCase):
    def test_route(self):
        r = Route(1.1)
        self.assertAlmostEqual(r.distance, 498.1, places=4)
        for t, d, a, b, name, ramp in r.segments:
            np.testing.assert_allclose(r.at(t + 1e-7)[0], a, atol=1e-5)
            np.testing.assert_allclose(r.at(t + d - 1e-7)[0], b, atol=1e-5)
        scan = [s for s in r.segments if s[4] == "scan"]
        self.assertEqual(len(scan), 5)
        for i, s in enumerate(scan):
            self.assertEqual(s[2][0], 0 if i % 2 == 0 else 80)
            self.assertEqual(s[3][0], 80 if i % 2 == 0 else 0)
            if i:
                self.assertEqual(scan[i - 1][3][0], s[2][0])
        for previous, current in zip(r.segments, r.segments[1:]):
            np.testing.assert_allclose(previous[3], current[2])
        self.assertAlmostEqual(sum(np.linalg.norm(s[3] - s[2]) for s in scan), 400)

    def test_hover_allocation(self):
        c = load_config()
        ctrl = Controller(c)
        u = ctrl.command(
            np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3), (np.zeros(3),) * 3
        )
        np.testing.assert_allclose(
            ctrl.allocation @ u, [c["mass_kg"] * 9.81, 0, 0, 0], atol=1e-8
        )
        model, _ = create_model(c)
        self.assertEqual(model.nu, 4)

    def test_decode_and_deduplicate(self):
        inv = Inventory()
        image = label_image("ITEM0001")
        inv.update(image, 0, "left", [1, 0, 1])
        inv.update(image, 1, "left", [1, 0, 1])
        self.assertEqual(len(inv.records), 1)
        self.assertEqual(inv.records["ITEM0001"]["observations"], 2)
        inv.update(label_image("ITEM0002"), 2, "left", [2, 0, 1])
        self.assertEqual(len(inv.records), 2)


if __name__ == "__main__":
    unittest.main()
