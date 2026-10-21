import copy
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest import mock

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("compare_x650_response_models", ROOT/"scripts/compare_x650_response_models.py")
comparison = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = comparison
SPEC.loader.exec_module(comparison)


def synthetic_run(suffix, wn=5., zeta=.6):
    times = np.arange(0., 14.0001, .004)
    command_times = np.arange(0., 14.0001, .02)
    commands = np.zeros((len(command_times), 3))
    velocities = np.zeros((len(times), 3))
    for axis in range(3):
        start = 2.+4*axis
        amplitude = .05 if axis == 2 else .08
        commands[(command_times >= start)&(command_times < start+1.), axis] = amplitude
        commands[(command_times >= start+2.)&(command_times < start+3.), axis] = -amplitude
        velocities[:, axis] = comparison.predict_second_order(wn, zeta, command_times,
            commands[:, axis], times, 0.)[:, 0]
    return {"folder": suffix, "identity": {"condition": "same"}, "command_csv_sha256": suffix+"c",
            "normalized_measurement_sha256": suffix+"m", "truth_times": times, "velocities": velocities,
            "command_times": command_times, "commands": commands, "joints": np.zeros((len(times), 4)),
            "artifacts": {suffix+".csv": suffix+"hash"}}


class ExactPredictionTests(unittest.TestCase):
    def test_transition_matches_matrix_exponential_at_all_damping_regimes(self):
        for wn in (.3, 5., 30.):
            for zeta in (.1, .7, 1., 1.+1e-8, 2., 5.):
                matrix = np.array([[0., 1.], [-wn**2, -2*zeta*wn]])
                for dt in (0., .00013, .02, .731, 60.):
                    actual = np.array(comparison.transition(wn, zeta, dt)).reshape(2, 2)
                    np.testing.assert_allclose(actual, expm(matrix*dt), rtol=2e-8, atol=5e-13)

    def test_irregular_original_events_match_continuous_solution(self):
        events = np.array([-.1, .173, .411, .412, .88, 1.143])
        commands = np.array([.1, -.05, .12, .12, 0., -.03])
        observations = np.array([0., .03, .173, .177, .405, .411, .512, .91, 1.143, 1.42])
        initial = np.array([.015, -.023])
        for wn, zeta in ((5., .6), (3., 1.), (7., 2.)):
            expected = [initial.copy()]
            current = initial.copy()
            start = observations[0]
            for finish in observations[1:]:
                boundaries = [*events[(events > start)&(events < finish)], finish]
                for boundary in boundaries:
                    u = commands[np.searchsorted(events, start, side="right")-1]
                    solution = solve_ivp(lambda t, x: [x[1], wn**2*(u-x[0])-2*zeta*wn*x[1]],
                                         (start, boundary), current, rtol=1e-11, atol=1e-13)
                    current, start = solution.y[:, -1], boundary
                expected.append(current.copy())
            actual = comparison.predict_second_order(wn, zeta, events, commands, observations, *initial)
            np.testing.assert_allclose(actual, expected, atol=2e-11, rtol=2e-9)

    def test_critical_step_and_initial_state(self):
        times = np.array([0., .001, .031, .2, .9, 2.])
        states = comparison.predict_second_order(4., 1., [0.], [1.], times, 0.)
        expected = 1.-(1.+4.*times)*np.exp(-4.*times)
        np.testing.assert_allclose(states[:, 0], expected, atol=2e-15)
        np.testing.assert_allclose(states[:, 1], 16.*times*np.exp(-4.*times), atol=2e-15)

    def test_invalid_inputs_rejected(self):
        with self.assertRaises(ValueError):
            comparison.predict_second_order(5., 0., [0.], [1.], [0., .1], 0.)
        with self.assertRaises(ValueError):
            comparison.predict_second_order(5., .6, [0., 0.], [1., 2.], [0., .1], 0.)


class DevelopmentComparisonTests(unittest.TestCase):
    def test_recovers_known_parameters_from_training(self):
        prepared = comparison.prepare_run(synthetic_run("train", wn=5., zeta=.6))
        params, diagnostics = comparison.fit_axis(prepared, 0)
        self.assertAlmostEqual(params.second_wn_rad_s, 5., places=5)
        self.assertAlmostEqual(params.second_zeta, .6, places=5)
        self.assertEqual(diagnostics["fit_data"], "training only")
        self.assertEqual(diagnostics["fit_window_s"], [1., 14.])

    def test_no_validation_parameter_fit_or_formal_acceptance(self):
        training, validation = synthetic_run("train"), synthetic_run("old_holdout")
        parameters = comparison.Parameters(.2, 5., .6)
        with mock.patch.object(comparison, "fit_axis", return_value=(parameters, {"fit_data": "training only"})) as fitted:
            report, prepared, predictions = comparison.compare_loaded_runs(training, validation)
        self.assertEqual(fitted.call_count, 3)
        self.assertTrue(all(call.args[0] is prepared["training"] for call in fitted.call_args_list))
        self.assertTrue(report["development_only"])
        self.assertFalse(report["eligible_as_stage_gate"])
        self.assertFalse(report["new_model_independent_validation"])
        self.assertNotIn("passed", report)
        # Changing old holdout affects only reported predictions/errors, never a parameter.
        changed = copy.deepcopy(validation)
        changed["velocities"][changed["truth_times"] > 2., 0] += .003
        with mock.patch.object(comparison, "fit_axis", return_value=(parameters, {})):
            changed_report, _, changed_predictions = comparison.compare_loaded_runs(training, changed)
        for axis in range(3):
            self.assertEqual(report["axes"][axis]["parameters"], changed_report["axes"][axis]["parameters"])
        np.testing.assert_array_equal(predictions[("old_holdout_development", "x")]["second_order"],
                                      changed_predictions[("old_holdout_development", "x")]["second_order"])
        self.assertNotEqual(report["axes"][0]["old_holdout_development"]["primary_1_14_s"]["second_order"]["rmse_m_s"],
                            changed_report["axes"][0]["old_holdout_development"]["primary_1_14_s"]["second_order"]["rmse_m_s"])

    def test_nonquiet_initialization_rejected_instead_of_fitting_acceleration(self):
        run = synthetic_run("train")
        run["velocities"][run["truth_times"] < 1., 0] = .02
        with self.assertRaisesRegex(ValueError, "initial acceleration assumption"):
            comparison.prepare_run(run)

    def test_prediction_does_not_reset_at_later_axis_window(self):
        run = synthetic_run("train")
        data = comparison.prepare_run(run)
        parameters = comparison.Parameters(.2, 5., .6)
        before = comparison.predictions(data, 1, parameters)
        data["velocities"][data["times"] >= 6., 1] += .02
        after = comparison.predictions(data, 1, parameters)
        np.testing.assert_array_equal(before["second_order"], after["second_order"])
        np.testing.assert_array_equal(before["first_order"], after["first_order"])


if __name__ == "__main__":
    unittest.main()
