import unittest
from segd.battery_model import LoadedPack
from segd.candidate_airframe import config
from segd.candidate_control import Allocator
import numpy as np


class Candidate18Tests(unittest.TestCase):
    def test_constant_power_solution_and_energy(self):
        s = config()["battery"]
        p = LoadedPack(s)
        for load in [200, 400, 500]:
            for q in [0, 1, 2]:
                p.used_cell_ah = q
                point = p.operating_point(load)
                if point is not None:
                    i, v = point
                    self.assertAlmostEqual(i * v, load, places=8)
                    self.assertAlmostEqual(v, p.voltage(i), places=9)
                    self.assertLessEqual(i, 30)
        p = LoadedPack(s)
        i, v = p.step(400, 1)
        self.assertAlmostEqual(p.delivered_wh, 400 / 3600)
        self.assertAlmostEqual(p.interconnect_loss_wh, i * i * 0.01 / 3600)
        self.assertAlmostEqual(p.used_cell_ah, i / 2 / 3600)

    def test_loaded_curve_does_not_double_count_cell_resistance(self):
        s = {
            **config()["battery"],
            "curve_voltage_shift_v": 0,
            "interconnect_resistance_ohm": 0,
        }
        p = LoadedPack(s)
        self.assertAlmostEqual(p.voltage(20), 6 * 3.831)
        p.used_cell_ah = 1
        self.assertAlmostEqual(
            p.voltage(30), 6 * np.interp(1 / s["capacity_scale"], p.q, p.v15)
        )
        self.assertIsNone(p.operating_point(1000))

    def test_allocator_shared_limit_and_wrench(self):
        a = Allocator(2.9016, [0.0025, 0, 0])
        for torque in [[0, 0, 0], [1.2, 0, 0], [0, 1.2, 0], [1, 1, 0.5], [10, -10, 1]]:
            t, scales = a.allocate(2.9016 * 9.80665, torque)
            self.assertTrue(a.allowed(t))
            self.assertAlmostEqual((a.A @ t)[0], 2.9016 * 9.80665, places=4)
            self.assertTrue(all(0 <= x <= 1 for x in scales))


class MatchedMissionTests(unittest.TestCase):
    def test_complete_schedule_matches_budget_and_terminal_acceleration_is_zero(self):
        from segd.matched_mission import Mission
        from segd.final_design import calculate

        m = Mission()
        design = calculate()
        self.assertAlmostEqual(
            m.duration, design["cases"][0]["total_seconds"], places=7
        )
        turns = [x for x in m.segments if x[4] == "yaw_and_reacquisition"]
        self.assertEqual(len(turns), 5)
        p, v, a, yaw = m(m.duration + 1)
        np.testing.assert_allclose(p, [-2, 0, 0.25])
        np.testing.assert_allclose(v, 0)
        np.testing.assert_allclose(a, 0)
        self.assertAlmostEqual(np.cos(yaw), -1)
