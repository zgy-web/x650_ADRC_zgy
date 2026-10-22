"""MPC integration contracts; these tests do not certify a native flight."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from x650_wholebody.api import ControlState, EndpointTarget
from x650_wholebody.model import tip_world
from x650_wholebody.mpc import MpcSolveError
from x650_wholebody.ts20_mpc import Ts20MpcController, CHAIN_CONFIG
from x650_wholebody.ts20_prediction import Ts20PredictionModel
from test_model import example_model
from test_mixed_mpc import RESPONSE, GAINS
from test_ts20_prediction import metadata, packet


def setup():
    model = example_model(elbow_offset_body_m=(0., .04, 0.))
    predictor = Ts20PredictionModel(metadata())
    def context(state, t):
        return {"reference_state": {"time_s": -.01, "r": np.zeros(2)},
                "command_history": [(-.01, np.zeros(2))],
                "packet_history": [packet(-.01), packet(-.005)], "bus_origin_s": 0.}
    controller = Ts20MpcController(model, GAINS, predictor=predictor, context_provider=context,
                                  config={"nodes": 4, "response_model": RESPONSE})
    state = ControlState([0., 0., 2.], np.eye(3), [0.]*3, [0.]*3, [0.]*2, [0.]*2, 1.,
                         acceleration_enu=[0.]*3, control_time_s=1., joint_stamp_s=1.)
    controller.reset(state.position, state.rotation, state.q)
    target = EndpointTarget(tip_world(state.position, state.rotation, state.q, model))
    return controller, predictor, state, target


class Ts20MpcTests(unittest.TestCase):
    def test_hold_uses_new_chain_and_nonlinear_candidate(self):
        controller, predictor, state, target = setup()
        out = controller.step(0., .02, state, target)
        np.testing.assert_allclose(out["qd_cmd"], 0., atol=1e-10)
        self.assertTrue(out["diagnostics"]["ts20_nonlinear_check_passed"])
        self.assertFalse(out["diagnostics"]["supply_24v_qualified"])
        self.assertEqual(controller.metadata()["prediction_chain"], CHAIN_CONFIG)
        self.assertTrue(controller.metadata()["base_response_is_prior"])

    def test_nonlinear_failure_does_not_advance_controller_or_send_plan(self):
        controller, predictor, state, target = setup()
        previous = controller.previous_command.copy()
        original = predictor.candidate_rollout
        def bad(prediction, plan):
            out = original(prediction, plan)
            out["envelope"]["q_upper"] = np.full(2, 2.)
            return out
        with patch.object(predictor, "candidate_rollout", side_effect=bad):
            with self.assertRaisesRegex(MpcSolveError, "rejected before command"):
                controller.step(0., .02, state, target)
        self.assertEqual(controller.step_count, 0)
        self.assertIsNone(controller._plan)
        self.assertEqual(controller._joint_history, [])
        np.testing.assert_array_equal(controller.previous_command, previous)

    def test_reference_bounds_and_joint_trust_region_enter_qp(self):
        controller, _, state, target = setup()
        problem = controller._problem(0., .02, state, target)
        self.assertTrue(problem["joint_guard"]["actual_continuous_bounds_checked_before_publish"])
        self.assertAlmostEqual(problem["joint_guard"]["trust_radius_normalized"], .02)
        z = np.zeros(controller.nodes*5)
        z[3] = .2
        self.assertGreater(controller._quality(problem, z)["primal_violation"], .04)

    def test_relinearization_freezes_feedback_without_committing_rejected_plan(self):
        controller, predictor, state, target = setup()
        original = controller._validate_candidate
        contexts, observations = [], []
        provider = controller.context_provider
        def context(*args):
            contexts.append(args)
            return provider(*args)
        controller.context_provider = context
        def validate(t, state, problem, plan, predicted):
            observations.append((controller.previous_command.copy(), controller._joint_history.copy()))
            if len(observations) == 1:
                raise MpcSolveError("synthetic local-model mismatch", {
                    "ts20_candidate_actual_bounds_passed": True,
                    "ts20_rejected_candidate_plan": plan.copy(),
                    "ts20_candidate_q_linearization_error_rad": .007,
                    "ts20_candidate_dq_linearization_error_rad_s": .01})
            return original(t, state, problem, plan, predicted)
        # This test concerns transactional state only. Explicitly substitute
        # the reference Jacobian because a compiled backend is optional in a
        # local unit-test environment; numerical native coverage is separate.
        original_predict = predictor.predict
        def reference_predict(*args, **kwargs):
            kwargs["exact_local"] = False
            return original_predict(*args, **kwargs)
        with patch.object(controller, "_validate_candidate", side_effect=validate), \
                patch.object(predictor, "predict", side_effect=reference_predict):
            out = controller.step(0., .02, state, target)
        self.assertEqual(len(contexts), 1)
        self.assertEqual(len(observations), 2)
        for previous, sent in observations:
            np.testing.assert_array_equal(previous, 0.)
            self.assertEqual(sent, [])
        self.assertEqual(controller.step_count, 1)
        self.assertEqual(len(controller._joint_history), 1)
        self.assertEqual(len(out["diagnostics"]["ts20_relinearization_attempts"]), 2)

    def test_total_deadline_rolls_back_even_a_nonlinear_accepted_plan(self):
        controller, _, state, target = setup()
        with patch("x650_wholebody.ts20_mpc.perf_counter", side_effect=[0., .1]):
            with self.assertRaisesRegex(MpcSolveError, "deadline exceeded"):
                controller.step(0., .02, state, target)
        self.assertEqual(controller.step_count, 0)
        self.assertEqual(controller._joint_history, [])
        self.assertIsNone(controller._plan)
        self.assertIsNone(controller.last_t)
        np.testing.assert_array_equal(controller.previous_command, 0.)


if __name__ == "__main__":
    unittest.main()
