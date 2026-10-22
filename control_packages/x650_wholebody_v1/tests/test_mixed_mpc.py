"""Mixed response numerical/constraint contracts; no native-flight claim."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.integrate import solve_ivp

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from x650_wholebody.api import ControlState, EndpointTarget
from x650_wholebody.mpc import (MpcController, MIXED_RESPONSE_KIND,
                               mixed_response_matrices, normalize_response_model, response_matrices)
from x650_wholebody.model import tip_world
from test_model import example_model


# Independently accepted values are fixtures only, never runtime defaults.
RESPONSE = {"kind": MIXED_RESPONSE_KIND, "parameters": {
    "x": {"wn_rad_s": 2.8471958701784543, "zeta": .695123475648465},
    "y": {"wn_rad_s": 2.7956770526957433, "zeta": .69189021293641},
    "z": {"tau_s": .19588596825267549}}}
GAINS = {"MPC_XY_P": 1.7, "MPC_Z_P": .9}


class MixedMpcTests(unittest.TestCase):
    def make(self, *, acceleration=(0., 0., 0.), config=None):
        model = example_model(elbow_offset_body_m=(0., .04, 0.))
        controller = MpcController(model, GAINS, config={"nodes": 6, "response_model": RESPONSE, **(config or {})})
        state = ControlState([0., 0., 2.], np.eye(3), np.zeros(3), np.zeros(3),
                             np.zeros(2), np.zeros(2), 1., acceleration_enu=acceleration)
        controller.reset(state.position, state.rotation, state.q)
        return controller, state, EndpointTarget(tip_world(state.position, state.rotation, state.q, model))

    def test_exact_zoh_matches_independent_integration_with_nonzero_acceleration(self):
        for h in (.02, .04, .3):
            A, B = mixed_response_matrices(h, RESPONSE)
            self.assertEqual(A.shape, (12, 12))
            self.assertEqual(B.shape, (12, 5))
            for axis, name in enumerate(("x", "y")):
                wn, zeta = (RESPONSE["parameters"][name][key] for key in ("wn_rad_s", "zeta"))
                initial, command = np.array([.14, -.09, .27]), .08
                def derivative(t, state):
                    p, v, a = state
                    return [v, a, wn**2*(command-v)-2*zeta*wn*a]
                expected = solve_ivp(derivative, (0., h), initial, rtol=2e-12, atol=2e-14).y[:, -1]
                indices = [axis, 3+axis, 10+axis]
                predicted = A[np.ix_(indices, indices)]@initial+B[indices, axis]*command
                np.testing.assert_allclose(predicted, expected, atol=3e-13, rtol=3e-12)
            legacy_A, legacy_B = response_matrices(h, [.2, .2, RESPONSE["parameters"]["z"]["tau_s"]])
            np.testing.assert_array_equal(A[np.ix_([2, 5], [2, 5])], legacy_A[np.ix_([2, 5], [2, 5])])
            np.testing.assert_array_equal(B[[2, 5], 2], legacy_B[[2, 5], 2])
            np.testing.assert_array_equal(A[6:10, 6:10], legacy_A[6:10, 6:10])
            np.testing.assert_array_equal(B[6:10, 3:5], legacy_B[6:10, 3:5])

    def test_held_command_semigroup_and_stable_velocity_poles(self):
        a, b = mixed_response_matrices(.02, RESPONSE)
        c, d = mixed_response_matrices(.04, RESPONSE)
        full_a, full_b = mixed_response_matrices(.06, RESPONSE)
        np.testing.assert_allclose(c@a, full_a, atol=4e-15)
        np.testing.assert_allclose(c@b+d, full_b, atol=4e-15)
        for indices in ([3, 10], [4, 11]):
            self.assertTrue(np.all(abs(np.linalg.eigvals(a[np.ix_(indices, indices)])) < 1.))
        self.assertLess(b[3, 0], .01)  # velocity cannot jump to its command
        np.testing.assert_array_equal(b[[0, 3, 10], 1:], 0.)

    def test_control_state_acceleration_is_optional_copied_and_readonly(self):
        _, state, _ = self.make(acceleration=None)
        self.assertIsNone(state.acceleration_enu)
        values = np.array([.2, -.1, .03])
        measured = replace(state, acceleration_enu=values)
        values[:] = 99.
        np.testing.assert_array_equal(measured.acceleration_enu, [.2, -.1, .03])
        with self.assertRaises(ValueError):
            measured.acceleration_enu[0] = 1.
        for bad in ([0., 0.], [0., np.nan, 0.], [0., 0., np.inf]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                replace(state, acceleration_enu=bad)

    def test_prediction_initializes_from_current_measurement_every_solve(self):
        controller, state, target = self.make(acceleration=(.31, -.22, 9.))
        state = replace(state, velocity=[.04, -.03, .02], q=[.01, -.02], dq=[.03, -.01])
        first = controller._problem(0., .02, state, target)
        x = np.r_[state.position, state.velocity, state.q, state.dq, state.acceleration_enu[:2]]
        for i, step in enumerate(first["h"]):
            x = mixed_response_matrices(step, RESPONSE)[0]@x
            np.testing.assert_allclose(first["offsets"][i], x, atol=2e-15)
        second = controller._problem(.02, .02, replace(state, acceleration_enu=[-.16, .18, -99.]), target)
        np.testing.assert_array_equal(first["jacobians"], second["jacobians"])
        self.assertGreater(np.linalg.norm(first["offsets"]-second["offsets"]), .1)
        # Z acceleration is measured/recorded but is not a Z first-order state.
        same_xy = controller._problem(.02, .02, replace(state, acceleration_enu=[.31, -.22, -100.]), target)
        np.testing.assert_array_equal(first["offsets"], same_xy["offsets"])

    def test_missing_or_invalid_measured_acceleration_never_reaches_solver(self):
        for value in (None, [0., 0.], [np.nan, 0., 0.], [0., np.inf, 0.]):
            controller, state, target = self.make()
            raw = SimpleNamespace(**state.__dict__)
            raw.acceleration_enu = value
            with patch("x650_wholebody.mpc.minimize", side_effect=AssertionError("solver called")):
                with self.subTest(value=value), self.assertRaisesRegex(ValueError, "acceleration_enu"):
                    controller.step(0., .02, raw, target)
            self.assertIsNone(controller.last_t)
            np.testing.assert_array_equal(controller.previous_command, 0.)

    def test_response_schema_rejects_ambiguous_unstable_or_nonfinite_values(self):
        bad = [None, {}, {**RESPONSE, "kind": "unknown"}, {**RESPONSE, "gain": 1.}]
        for axis, name in (("x", "wn_rad_s"), ("y", "zeta"), ("z", "tau_s")):
            for number in (0., -1., np.nan, np.inf, True, "2.0", None, 1+2j):
                value = deepcopy(RESPONSE)
                value["parameters"][axis][name] = number
                bad.append(value)
        for value in bad:
            with self.subTest(value=value), self.assertRaises(ValueError):
                normalize_response_model(value)
        with self.assertRaisesRegex(ValueError, "cannot be combined"):
            self.make(config={"base_time_constant_s": [.2, .2, .25]})
        external = deepcopy(RESPONSE)
        controller, _, _ = self.make(config={"response_model": external})
        external["parameters"]["x"]["wn_rad_s"] = 99.
        self.assertEqual(controller.response_model, RESPONSE)

    def test_cache_keys_include_all_mixed_parameters_and_first_interval(self):
        controller, _, _ = self.make()
        previous = controller._horizon(.02)
        self.assertIs(previous, controller._horizon(.02))
        self.assertIsNot(previous, controller._horizon(.024))
        for axis, name in (("x", "wn_rad_s"), ("x", "zeta"), ("y", "wn_rad_s"), ("y", "zeta"), ("z", "tau_s")):
            controller.response_model["parameters"][axis][name] *= 1.01
            current = controller._horizon(.02)
            self.assertIsNot(previous, current)
            self.assertFalse(np.array_equal(previous["transitions"][0], current["transitions"][0]))
            previous = current

    def test_mixed_plan_preserves_joint_indices_speed_slew_and_terminal_constraints(self):
        controller, state, target = self.make(acceleration=(.05, -.04, 0.))
        target = replace(target, position=target.position+[.02, 0., .01])
        out = controller.step(0., .01, state, target)
        diagnostic = out["diagnostics"]
        commands, states, h = (diagnostic[key] for key in ("predicted_commands", "predicted_states", "node_intervals_s"))
        self.assertEqual(states.shape, (6, 12))
        self.assertTrue(diagnostic["accepted_optimum"])
        self.assertTrue(np.all(abs(commands) <= controller.velocity_limit+1e-9))
        changes = np.diff(np.vstack((np.zeros(5), commands)), axis=0)
        self.assertTrue(np.all(abs(changes) <= controller.acceleration_limit*h[:, None]+1e-9))
        self.assertTrue(np.all(states[:, 6:8] >= controller.q_lower-1e-9))
        self.assertTrue(np.all(states[:, 6:8] <= controller.q_upper+1e-9))
        np.testing.assert_allclose(states[:, 8:10], commands[:, 3:5], atol=1e-14)
        reserve = controller.velocity_limit[3:]/(2*controller.acceleration_limit[3:])+controller.max_dt
        terminal = states[-1, 6:8]+reserve*states[-1, 8:10]
        self.assertTrue(np.all(terminal >= controller.q_lower-1e-9))
        self.assertTrue(np.all(terminal <= controller.q_upper+1e-9))
        self.assertNotIn("prediction_time_constant_identified", diagnostic)
        self.assertEqual(diagnostic["prediction_response_kind"], MIXED_RESPONSE_KIND)
        np.testing.assert_array_equal(diagnostic["measured_acceleration_enu"], state.acceleration_enu)
        np.testing.assert_array_equal(diagnostic["initial_prediction_acceleration_xy"], state.acceleration_enu[:2])
        self.assertFalse(controller.metadata()["response_identified"])
        self.assertNotIn("base_time_constant_s", controller.metadata())

    def test_default_horizon_is_still_a_75_variable_qp_and_gradient_is_correct(self):
        controller, state, target = self.make(config={"nodes": 15})
        problem = controller._problem(0., .02, state, target)
        self.assertEqual(problem["H"].shape, (75, 75))
        self.assertEqual(problem["jacobians"].shape, (15, 12, 75))
        z = np.linspace(-.01, .01, 75)
        gradient = problem["H"]@z+problem["g"]
        objective = lambda value: np.linalg.norm(problem["A"]@value-problem["b"])**2
        for index in (0, 14, 37, 74):
            delta = np.zeros(75)
            delta[index] = 1e-6
            numeric = (objective(z+delta)-objective(z-delta))/(2e-6)
            self.assertAlmostEqual(gradient[index], numeric, delta=2e-9)

    def test_legacy_commands_do_not_depend_on_new_optional_acceleration(self):
        model = example_model(elbow_offset_body_m=(0., .04, 0.))
        _, state, target = self.make(acceleration=None)
        outputs = []
        for measured in (state, replace(state, acceleration_enu=[8., -4., 3.])):
            controller = MpcController(model, GAINS, config={"nodes": 6})
            controller.reset(state.position, state.rotation, state.q)
            outputs.append(controller.step(0., .02, measured, replace(target, position=target.position+[.02, 0., 0.])))
            self.assertEqual(controller.metadata()["base_time_constant_s"], [.2, .2, .25])
        for key in ("base_velocity_cmd", "qd_cmd", "a_ref"):
            np.testing.assert_array_equal(outputs[0][key], outputs[1][key])
        self.assertEqual(outputs[0]["diagnostics"]["predicted_states"].shape, (6, 10))


if __name__ == "__main__":
    unittest.main()
