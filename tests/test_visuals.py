import unittest
import xml.etree.ElementTree as ET
import numpy as np
from segd.flight import load_config, create_model, ROOT


class VisualSceneTests(unittest.TestCase):
    def test_real_asset_scene_keeps_plant_and_inventory_count(self):
        c = load_config()
        c["visual_assets"] = True
        model, xml = create_model(c)
        root = ET.fromstring(xml)
        labels = [
            g
            for g in root.iter("geom")
            if g.get("material", "").startswith("label_mat_")
        ]
        self.assertEqual(len(labels), 400)
        self.assertEqual(model.nu, 4)
        self.assertAlmostEqual(model.body_mass[1], c["mass_kg"])
        self.assertTrue(np.isfinite(model.body_inertia).all())
        self.assertTrue((ROOT / "assets/vendor/menagerie/skydio_x2/LICENSE").exists())
        self.assertEqual(len({g.get("material") for g in labels}), 400)
        self.assertEqual(
            root.find("worldbody/geom[@name='floor']").get("material"), "floor_mat"
        )


if __name__ == "__main__":
    unittest.main()
