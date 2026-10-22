"""Validation-contract tests, not actuator or flight performance evidence."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import validate_x650_ts20_prediction as validation


class Ts20PredictionValidationTests(unittest.TestCase):
    def test_fixture_and_threshold_identity_is_frozen(self):
        declaration = validation.protocol()
        self.assertEqual(validation.digest(declaration),
            "1a95e9703bffd3c5bfac82766d7ecd9453266c4e6361f9219d1d55c61bc06db2")
        self.assertEqual(len(declaration["fixtures"]), 8)
        self.assertFalse(declaration["validation_trajectories_may_be_used_for_tuning"])
        self.assertFalse(declaration["interpretation"]["supply_24v_qualified"])
        self.assertFalse(declaration["interpretation"]["actual_aerial_arm_load_qualified"])

    def test_fresh_protocol_copies_do_not_silently_carry_mutations(self):
        modified = validation.protocol()
        modified["gates"]["q_peak_abs_rad"] = 1.
        modified["fixtures"][0]["dq0"][0] = 1.
        self.assertEqual(validation.protocol()["gates"]["q_peak_abs_rad"], .0003)
        self.assertEqual(validation.protocol()["fixtures"][0]["dq0"], [0., 0.])

    def test_independent_nongrid_integral_and_prehistory(self):
        declaration = validation.protocol()
        declaration["bus_period_s"] = .005
        times = np.array([0., .007, .013, .02])
        velocity = np.array([[.2, -.1], [-.1, .3], [.4, -.2], [0., 0.]])
        bus, command = validation.independent_bridge(times, velocity, [0., 0.], declaration)
        np.testing.assert_array_equal(bus, [-.005, 0., .005, .01, .015, .02])
        # Values before quantization: exact integral across source switches.
        expected_q = np.array([[0., 0.], [0., 0.], [.001, -.0005],
                               [.0011, .0002], [.0016, .0007], [.0036, -.0003]])
        self.assertLessEqual(float(abs(command["q_ref"] - expected_q).max()), np.pi / 2**24)
        np.testing.assert_array_equal(command["q_ref"][0], command["q_ref"][1])
        validation.check_bridge(times, velocity, [0., 0.], bus, command, declaration)

    def test_half_away_quantization_and_wire_field_detection(self):
        declaration = validation.protocol()
        step = 1 / 512
        times = np.array([0., .005, .01])
        velocity = np.array([[-.5, .5], [-1.5, 1.5], [0., 0.]]) * step
        bus, command = validation.independent_bridge(times, velocity, [0., 0.], declaration)
        np.testing.assert_array_equal(command["dq_ref"][1:3], [[-step, step], [-2 * step, 2 * step]])
        changed = {key: value.copy() for key, value in command.items()}
        changed["dq_ref"][2, 0] += step
        with self.assertRaisesRegex(ValueError, "wire field: dq_ref"):
            validation.check_bridge(times, velocity, [0., 0.], bus, changed, declaration)

    def test_exact_horizons_and_whole_window_have_independent_gates(self):
        declaration = validation.protocol()
        t = np.arange(1201) * .001
        oracle = np.zeros((len(t), 2))
        q = oracle.copy()
        # A late error must fail even though all four early horizon points pass.
        q[900, 1] = .000301
        result = validation.compare_prediction(t, q, oracle, oracle, oracle, declaration)
        self.assertFalse(result["pass"])
        for horizon in result["first_state_free_prediction_horizons"].values():
            self.assertEqual(horizon["q_abs_error_rad"], [0., 0.])
        q[900, 1] = .0003
        self.assertTrue(validation.compare_prediction(t, q, oracle, oracle, oracle, declaration)["pass"])

    def test_initial_state_mismatch_is_not_hidden_in_metrics(self):
        declaration = validation.protocol()
        t = np.arange(1201) * .001
        oracle = np.zeros((len(t), 2))
        q = oracle.copy()
        q[0, 0] = 1e-9  # Far below q gate, above first-state identity gate.
        self.assertFalse(validation.compare_prediction(t, q, oracle, oracle, oracle, declaration)["pass"])

    def test_missing_horizon_nonfinite_and_shape_are_rejected(self):
        declaration = validation.protocol()
        t = np.arange(1201) * .001
        state = np.zeros((len(t), 2))
        bad = state.copy()
        bad[10, 0] = np.nan
        with self.assertRaisesRegex(ValueError, "Finite"):
            validation.compare_prediction(t, bad, state, state, state, declaration)
        with self.assertRaisesRegex(ValueError, "Finite"):
            validation.compare_prediction(t, state[:, 0], state, state, state, declaration)
        keep = np.flatnonzero(t != .02)
        with self.assertRaisesRegex(ValueError, "exact query point"):
            validation.compare_prediction(t[keep], state[keep], state[keep], state[keep], state[keep], declaration)

    def test_static_threshold_and_single_delay_first_release(self):
        model = {"fixed_parameters": {"offset_over_ka": 0.,
                                     "combined_interface_delay_s": .003346379092576424},
                 "friction_parameters": {"static_threshold": .075}}
        t = np.arange(21) * .001
        bus = np.array([-.005, 0., .005, .01])
        qref = np.array([[0., 0.], [0., 0.], [.02, -.02], [.02, -.02]])
        command = {"q_ref": qref, "dq_ref": np.zeros((4, 2)),
                   "tau_ff": np.zeros((4, 2)), "kp": np.full((4, 2), 6.),
                   "kd": np.zeros((4, 2))}
        correct = np.zeros((len(t), 2))
        correct[9:] = [.001, -.001]
        result = validation.first_release_check(t, [0., 0.], [0., 0.], bus, command, model, correct, correct)
        self.assertTrue(result["pass"])
        self.assertAlmostEqual(result["joints"][0]["expected_delayed_release_s"], .008346379092576424)
        early = correct.copy()
        early[8] = [.001, -.001]
        self.assertFalse(validation.first_release_check(t, [0., 0.], [0., 0.], bus, command, model, early, correct)["pass"])
        # Zero net command and exactly static threshold both remain stuck.
        command["kp"][:] = 0.
        command["tau_ff"][:, 1] = .075
        still = np.zeros_like(correct)
        self.assertTrue(validation.first_release_check(t, [0., 0.], [0., 0.], bus, command, model, still, still)["pass"])


if __name__ == "__main__":
    unittest.main()
