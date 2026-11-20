"""Checks for the revised comparison; no simulated-flight claims are introduced."""

import unittest
from segd.design_options import results


class FollowupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r = results()

    def test_larger_complete_label_passes_sampling_and_schedule(self):
        r = self.r
        self.assertTrue(r["capture_schedule_geometry_pass"])
        self.assertGreater(r["coverage"]["speed_limit_mps"], 0.85)
        small, _, large = r["labels"]
        self.assertLess(small["pixels_per_module_lower_bound"], 2.5)
        self.assertGreater(large["pixels_per_module_lower_bound"], 2.5)
        self.assertAlmostEqual(
            large["pixels_per_module_lower_bound"]
            / small["pixels_per_module_lower_bound"],
            2.5,
        )
        self.assertEqual(large["modules_including_quiet_zones"], 187)

    def test_mass_counts_lenses_mounts_and_battery_together(self):
        rows = self.r["batteries"]["GEPRC VTC6 6S2P 6000"]
        self.assertAlmostEqual(rows[0]["mass_g"], 2899.6 + 130 + 627 - 755)
        self.assertLess(rows[0]["mass_g"], 3000)
        self.assertGreater(rows[0]["low_voltage_twr"], 1.5)

    def test_two_items_are_not_eight_or_twenty_and_battery_is_not_certified(self):
        rows = self.r["batteries"]["GEPRC VTC6 6S2P 6000"]
        self.assertEqual([r["retry_instances"] for r in rows], [2, 8, 20])
        self.assertLess(rows[0]["total_seconds"], rows[1]["total_seconds"])
        self.assertLess(rows[1]["adverse_wh"], rows[2]["adverse_wh"])
        for row in rows:
            self.assertAlmostEqual(
                row["required_delivered_wh_adverse"] * 0.8, row["adverse_wh"]
            )
            self.assertNotIn("passes_mass_thrust_energy", row)
            self.assertNotIn("allowed_wh", row)
            self.assertEqual(
                row["battery_cutoff_qualification"],
                "not established by nameplate capacity",
            )
