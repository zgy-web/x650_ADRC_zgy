"""The fitted effective response must predict a held-out opposite pulse."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "comparison_identification", ROOT/"simulations/x650_control_comparison_v1/identification.py")
IDENT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(IDENT)


def synthetic_trace(excited=True):
    gain, tau = np.array([.9, 1.15, 1.05]), np.array([.7, .5, .25])
    velocity = np.zeros(3)
    rows = []
    for k in range(2250):
        t = .02*k
        command = np.zeros(3)
        if excited:
            for axis, start in enumerate((5., 18., 31.)):
                if start <= t < start+2.:
                    command[axis] = .1
                elif start+4. <= t < start+6.:
                    command[axis] = -.1
        row = dict(t=t, px4_stamp_us=1_000_000+k*20_000)
        for i, axis in enumerate("xyz"):
            row["v"+axis] = velocity[i]
            row["v"+axis+"_cmd"] = command[i]
        rows.append(row)
        # Independent small-step continuous integration; no use of fit code.
        for _ in range(20):
            velocity += .001*(gain*command-velocity)/tau
    return rows, gain, tau


class ResponseIdentificationTests(unittest.TestCase):
    def test_recovers_known_effective_model_without_truth_and_predicts_negative_pulse(self):
        rows, gain, tau = synthetic_trace()
        model = IDENT.fit_response(rows)
        self.assertTrue(model["identified"])
        np.testing.assert_allclose(model["gain"], gain, atol=.003)
        np.testing.assert_allclose(model["tau"], tau, atol=.003)
        self.assertLess(max(model["validation_rmse_m_s"]), .0001)
        self.assertEqual(model["delay_s"], [0., 0., 0.])
        self.assertEqual(model["bias_velocity"], [0., 0., 0.])
        self.assertEqual(model["fit_boundary_hit"], [False]*3)

    def test_unexcited_trace_is_rejected_instead_of_becoming_a_response_model(self):
        rows, _, _ = synthetic_trace(excited=False)
        with self.assertRaisesRegex(ValueError, "excitation"):
            IDENT.fit_response(rows)


if __name__ == "__main__":
    unittest.main()
