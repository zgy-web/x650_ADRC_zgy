"""Synthetic numerical contracts only, never Gazebo acceptance evidence."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from x650_wholebody import prediction_chain as chain
from x650_wholebody.api import ControlState, EndpointTarget
from x650_wholebody.mpc import MpcController
from x650_wholebody.model import tip_world
from test_model import example_model
from test_mixed_mpc import RESPONSE, GAINS

CHAIN = {"kind": chain.KIND, "joint_delay_s": .024, "joint_delay_guard_s": .05,
         "gravity_m_s2": 9.8, "attitude_residual_decay_s": .15}


def setup():
    model = example_model(elbow_offset_body_m=(0., .04, 0.))
    controller = MpcController(model, GAINS, config={"response_model": RESPONSE, "prediction_chain": CHAIN})
    state = ControlState([0., 0., 1.], np.eye(3), np.zeros(3), np.zeros(3), np.zeros(2),
                         np.zeros(2), 1., acceleration_enu=np.zeros(3), control_time_s=1., joint_stamp_s=1.)
    controller.reset(state.position, state.rotation, state.q)
    target = EndpointTarget(tip_world(state.position, state.rotation, state.q, model))
    return model, controller, state, target


class PredictionChainTests(unittest.TestCase):
    def test_prime_uses_actual_prestream_command_and_pending_nonzero_age(self):
        _, controller, state, target = setup()
        old = np.array([.01, -.02, .005, .04, -.03])
        recent = old+np.array([.001, .001, .001, .005, .005])
        controller.prime_command_history([(0.96, old), (.98, recent)], 1.)
        np.testing.assert_array_equal(controller.previous_command, recent)
        self.assertAlmostEqual(controller.last_t, -.02)
        state = replace(state, joint_stamp_s=.988, dq=old[3:])
        problem = controller._problem(0., .02, state, target)
        # Previous command at .98 applies at 1.004, after this consumption.
        np.testing.assert_allclose(problem["initial_state"][6:8], .012*old[3:], atol=1e-15)
        np.testing.assert_array_equal(problem["initial_state"][8:10], old[3:])
        first = problem["joint_prediction"]["nodes"][0]
        np.testing.assert_allclose(first[1], .016*old[3:]+.016*recent[3:], atol=1e-15)
        out = controller.step(0., .02, state, target)
        sent = out["diagnostics"]["predicted_commands"][0]
        self.assertTrue(np.all(abs(sent-recent) <= controller.acceleration_limit*.02+1e-9))
        self.assertTrue(out["diagnostics"]["command_delay_compensated"])
        self.assertFalse(out["diagnostics"]["base_command_delay_compensated"])
        with self.assertRaises(ValueError):
            controller.prime_command_history([(.98, recent)], 1.)

    def test_prime_rejects_missing_future_reordered_or_out_of_bounds_history_atomically(self):
        _, controller, state, _ = setup()
        for history in ([], [(1.001, np.zeros(5))], [(.98, np.zeros(5)), (.97, np.zeros(5))],
                        [(.98, np.ones(5))], [(.5, np.zeros(5))]):
            with self.assertRaises(ValueError):
                controller.prime_command_history(history, 1.)
            self.assertEqual(controller._joint_history, [])
            np.testing.assert_array_equal(controller.previous_command, np.zeros(5))
        controller.prime_command_history([(.98, np.zeros(5))], 1.)
        with self.assertRaises(ValueError):
            controller.prime_command_history([(.98, np.zeros(5))], 1.)
        controller.reset(state.position, state.rotation, state.q)
        controller.prime_command_history([(.98, np.zeros(5))], 1.)

    def test_delay_and_measurement_age_preserve_measured_dq_and_exact_history(self):
        _, _, state, _ = setup()
        state = replace(state, joint_stamp_s=.99, dq=[.1, -.1])
        result = chain.joint_prediction(0., [.02, .06, .10], [.02, .04, .04], state,
                                       [(-.02, np.array([.2, -.2]))], CHAIN, [.45, .35])
        np.testing.assert_allclose(result["q_current"], [.001, -.001], atol=1e-14)
        np.testing.assert_allclose(result["dq_current"], [.1, -.1], atol=1e-14)
        np.testing.assert_allclose(result["nodes"][0][1], [.0046, -.0046], atol=1e-14)
        np.testing.assert_allclose(result["nodes"][0][2], [.2, -.2], atol=1e-14)
        np.testing.assert_array_equal(result["nodes"][0][3], 0.)
        # At 60 ms, u0 was active 20 ms and u1 16 ms (delay is seconds,
        # not a guessed one/two control step offset).
        np.testing.assert_allclose(result["nodes"][1][3][:, 3:5], .02*np.diag([.45, .35]), atol=1e-14)
        np.testing.assert_allclose(result["nodes"][1][3][:, 8:10], .016*np.diag([.45, .35]), atol=1e-14)

    def test_output_state_and_input_jacobians_match_independent_differences(self):
        model, _, state, _ = setup()
        times = np.array([.02, .18])
        context = chain.attitude_context(state, times, CHAIN)
        x = np.tile(np.r_[state.position, [.03, -.04, .01], [-.03, .08], [.02, -.03], [.05, -.06]], (2, 1))
        u = np.array([[.08, -.05, .02, .02, -.03], [.02, .06, -.01, -.03, .04]])
        result = chain.endpoint_outputs(x, u, context, model, RESPONSE["parameters"], jacobians=True)
        for axis in range(17):
            xp, xm, up, um = x.copy(), x.copy(), u.copy(), u.copy()
            epsilon = 2e-6
            if axis < 12:
                xp[:, axis] += epsilon
                xm[:, axis] -= epsilon
                names, index = ("px", "vx"), axis
            else:
                up[:, axis-12] += epsilon
                um[:, axis-12] -= epsilon
                names, index = ("pu", "vu"), axis-12
            plus = chain.endpoint_outputs(xp, up, context, model, RESPONSE["parameters"])
            minus = chain.endpoint_outputs(xm, um, context, model, RESPONSE["parameters"])
            for field, derivative in zip(("position", "velocity"), names):
                np.testing.assert_allclose((plus[field]-minus[field])/(2*epsilon), result[derivative][:, :, index], atol=2e-7, rtol=2e-6)

    def test_y_acceleration_rotates_low_tip_and_is_inside_qp_cost(self):
        model, controller, state, target = setup()
        problem = controller._problem(0., .02, state, target)
        self.assertEqual(problem["H"].shape, (75, 75))
        x = np.tile(np.r_[state.position, np.zeros(9)], (1, 1))
        context = chain.attitude_context(state, np.array([0.]), CHAIN)
        zero = chain.endpoint_outputs(x, np.zeros((1, 5)), context, model, RESPONSE["parameters"], jacobians=True)
        x[0, 11] = .1
        tilted = chain.endpoint_outputs(x, np.zeros((1, 5)), context, model, RESPONSE["parameters"])
        self.assertLess(tilted["position"][0, 1], zero["position"][0, 1])
        self.assertAlmostEqual(zero["px"][0, 1, 11], -.33/CHAIN["gravity_m_s2"], delta=1e-8)
        # Same optimization inputs must produce a different objective versus
        # the legacy attitude-independent model, not only different plots.
        legacy = MpcController(model, GAINS, config={"response_model": RESPONSE})
        legacy.reset(state.position, state.rotation, state.q)
        self.assertGreater(np.max(abs(problem["H"]-legacy._problem(0., .02, state, target)["H"])), 1e-4)

    def test_measurement_times_and_stopping_envelope_fail_closed(self):
        _, controller, state, target = setup()
        for bad in (replace(state, control_time_s=None), replace(state, joint_stamp_s=1.001)):
            with self.assertRaises(ValueError):
                controller.step(0., .02, bad, target)
        controller.previous_command[3:] = [.14470754, -.12327183]
        edge = replace(state, q=[-.14930077, -.33117312], dq=[.10870754, -.14381841])
        with self.assertRaises(ValueError):
            controller.step(0., .032, edge, target)
        self.assertEqual(controller._joint_history, [])

    def test_real_solve_has_original_limits_timing_and_rotation_diagnostics(self):
        _, controller, state, target = setup()
        state = replace(state, position=state.position+[0., .005, 0.])
        out = controller.step(0., .02, state, target)
        d = out["diagnostics"]
        self.assertTrue(d["accepted_optimum"])
        self.assertFalse(d["previous_plan_fallback"])
        self.assertEqual(d["prediction_chain"], CHAIN)
        self.assertEqual(d["predicted_rotations_flu_to_enu"].shape, (15, 3, 3))
        np.testing.assert_allclose(d["prediction_initial_joint_velocity"], state.dq)
        self.assertTrue(np.all(abs(d["predicted_commands"][0]) <= controller.acceleration_limit*.02+1e-9))
        self.assertEqual(len(controller._joint_history), 1)
        controller.reset(state.position, state.rotation, state.q)
        self.assertEqual(controller._joint_history, [])


if __name__ == "__main__":
    unittest.main()
