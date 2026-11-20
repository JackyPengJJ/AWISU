import unittest
from segd.final_design import calculate, recovery_offset
from segd.latex_report import typeset, prose
from segd.report import resolved_source
from segd.flight import ROOT
import json


class FinalDesignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d = calculate()

    def test_lateral_recovery_preserves_margin_and_counts_energy(self):
        d = self.d
        self.assertAlmostEqual(d["mass_g"], 2901.6)
        self.assertGreaterEqual(d["recovery_clearance_m"], 0.25)
        self.assertGreater(d["lateral_recovery_seconds_each"], 2)
        self.assertGreater(d["targeted_recovery_safe_width_m"], 0.4)
        self.assertEqual(d["targeted_recovery_standoff_m"], [0.7000000000000001, 0.85])
        for case in d["cases"]:
            self.assertAlmostEqual(
                sum(p["seconds"] for p in case["phases"]), case["total_seconds"]
            )
            self.assertAlmostEqual(
                sum(p["adverse_wh"] for p in case["phases"]), case["adverse_wh"]
            )
            self.assertAlmostEqual(
                case["required_delivered_wh_adverse"] * 0.8, case["adverse_wh"]
            )
            self.assertFalse(case["battery_cutoff_energy_qualified"])
        self.assertGreater(d["cases"][0]["adverse_wh"], 72.57)

    def test_offset_is_side_specific_and_requires_space_and_energy(self):
        self.assertAlmostEqual(recovery_offset("right", True, 0.359)["offset_m"], -0.1)
        self.assertAlmostEqual(recovery_offset("left", True, 0.28)["offset_m"], 0.03)
        self.assertEqual(recovery_offset("right", True, 0.2)["action"], "settled_retry")
        self.assertEqual(
            recovery_offset("right", True, 0.359, False)["action"],
            "return_with_exception",
        )
        self.assertEqual(recovery_offset("left", False, 0.359)["action"], "continue")

    def test_final_document_has_resolved_numbers_and_references(self):
        content, _, _ = resolved_source(ROOT / "results")
        source, n = typeset(
            content, json.loads((ROOT / "docs/sources.json").read_text())
        )
        self.assertTrue(all(ord(c) >= 32 or c == "\n" for c in content))
        self.assertNotIn("{{", source)
        self.assertIn(
            f"{self.d['cases'][0]['required_delivered_wh_adverse']:.2f}", source
        )
        self.assertIn(f"{self.d['cases'][0]['adverse_wh']:.2f}", source)
        self.assertIn(r"\begin{equation}", source)
        self.assertEqual(source.count(r"\label{fig:"), 7)
        self.assertGreaterEqual(n, 35)
        self.assertEqual(prose("Gain $K_p$ [40]."), r"Gain $K_p$ \cite{r40}.")
        with self.assertRaises(ValueError):
            prose("Unmatched $ delimiter")
        self.assertEqual(
            prose("[Project code](../README.md)"), r"\href{../README.md}{Project code}"
        )
        self.assertIn("Controller", content)
        self.assertIn("0.1364", content)
        self.assertIn("0.1233", content)
        self.assertIn(f"{self.d['cases'][1]['adverse_wh']:.2f}", content)
        self.assertIn(f"{self.d['cases'][2]['adverse_wh']:.2f}", content)
