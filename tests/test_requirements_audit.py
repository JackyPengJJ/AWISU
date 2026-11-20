"""Checks for the design audit's cutoff integral and rejection behaviour."""

import unittest
from segd.requirements_audit import delivered_energy, battery_audit, current_audit
from segd.final_design import calculate


class RequirementsAuditTests(unittest.TestCase):
    def test_cutoff_is_interpolated_before_integration(self):
        r = delivered_energy([0, 1000, 2000], [4.2, 3.6, 3.0], 3.3)
        self.assertAlmostEqual(r["cell_ah"], 1.5)
        self.assertAlmostEqual(r["cell_wh"], 5.625)
        self.assertEqual(delivered_energy([0, 1000], [3.0, 2.5], 3.3)["cell_wh"], 0)
        with self.assertRaises(ValueError):
            delivered_energy([0, 1000], [4.2, 4.0], 3.3)

    def test_voltage_uncertainty_reduces_energy_and_failure_is_not_hidden(self):
        result = battery_audit(calculate())["results"]
        a = next(
            r
            for r in result
            if r["pack_cutoff_v"] == 19.8
            and r["cell_current_a"] == 10
            and r["curve_voltage_shift_v"] == 0
        )
        b = next(
            r
            for r in result
            if r["pack_cutoff_v"] == 19.8
            and r["cell_current_a"] == 10
            and r["curve_voltage_shift_v"] == 0.04
        )
        self.assertLess(b["typical_curve_pack_wh"], a["typical_curve_pack_wh"])
        self.assertFalse(a["two_recovery_adverse_comparison_pass"])
        self.assertLess(a["typical_curve_pack_wh"], a["required_delivered_wh"])

    def test_shared_current_limit_replaces_unconstrained_twr(self):
        result = current_audit(calculate())["results"]
        low = next(r for r in result if r["bus_voltage_v"] == 19.8)
        self.assertLess(low["maximum_collective_twr_model"], 1.5)
        self.assertTrue(low["20_percent_differential_admitted"])
        self.assertLess(low["20_percent_differential_current_a"], 30)
        self.assertGreater(low["20_percent_differential_torque_nm"], 1)
