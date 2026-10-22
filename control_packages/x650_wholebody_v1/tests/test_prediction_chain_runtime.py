"""Synthetic numerical equivalence, not flight or timing acceptance evidence."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from x650_wholebody import prediction_chain as chain
from test_prediction_chain import CHAIN, RESPONSE, setup


def legacy_thrust_rotation(acceleration, jerk, yaw, yaw_rate, gravity):
    """Frozen pre-optimization oracle, deliberately using generic np.cross."""
    force = np.asarray(acceleration, float)+[0., 0., gravity]
    norm = np.linalg.norm(force, axis=1)
    if np.any(norm < gravity*.25) or np.any(force[:, 2] <= gravity*.25):
        raise ValueError("Thrust-direction prediction outside upright flight domain")
    z = force/norm[:, None]
    zd = (jerk-z*np.sum(z*jerk, axis=1)[:, None])/norm[:, None]
    xc = np.column_stack((np.cos(yaw), np.sin(yaw), np.zeros(len(yaw))))
    xcd = np.column_stack((-np.sin(yaw), np.cos(yaw), np.zeros(len(yaw))))*yaw_rate[:, None]
    cross = np.cross(z, xc)
    cross_norm = np.linalg.norm(cross, axis=1)
    if np.any(cross_norm < .25):
        raise ValueError("Thrust direction is singular against heading")
    y = cross/cross_norm[:, None]
    cd = np.cross(zd, xc)+np.cross(z, xcd)
    yd = (cd-y*np.sum(y*cd, axis=1)[:, None])/cross_norm[:, None]
    x, xd = np.cross(y, z), np.cross(yd, z)+np.cross(y, zd)
    return np.stack((x, y, z), axis=2), np.stack((xd, yd, zd), axis=2)


class PredictionChainRuntimeEquivalenceTests(unittest.TestCase):
    def test_rotation_and_derivative_equal_legacy_for_nonzero_heading_and_jerk(self):
        rng = np.random.default_rng(20261022)
        for count in (0, 1, 2, 15, 64):
            for dtype in (np.float32, np.float64):
                args = (rng.uniform(-.8, .8, (count, 3)).astype(dtype),
                        rng.uniform(-2., 2., (count, 3)).astype(dtype),
                        rng.uniform(-np.pi, np.pi, count),
                        rng.uniform(-.5, .5, count), 9.8)
                with self.subTest(count=count, dtype=dtype):
                    actual = chain.thrust_rotation(*args)
                    expected = legacy_thrust_rotation(*args)
                    for value, oracle in zip(actual, expected):
                        self.assertTrue(np.isfinite(value).all())
                        np.testing.assert_array_equal(value, oracle)

    def test_domain_rejection_matches_legacy(self):
        for acceleration in ([0., 0., -9.8], [0., 0., -8.], [100., 0., 0.]):
            args = (np.array([acceleration]), np.zeros((1, 3)), np.zeros(1), np.zeros(1), 9.8)
            with self.subTest(acceleration=acceleration):
                with self.assertRaises(ValueError) as expected:
                    legacy_thrust_rotation(*args)
                with self.assertRaises(ValueError) as actual:
                    chain.thrust_rotation(*args)
                self.assertEqual(str(actual.exception), str(expected.exception))

    def test_complete_output_and_all_jacobians_equal_legacy(self):
        model, _, state, _ = setup()
        rng = np.random.default_rng(20261023)
        state = replace(state,
                        rotation=Rotation.from_euler("xyz", [.04, -.05, .7]).as_matrix(),
                        acceleration_enu=[.12, -.18, .09], omega_body=[.03, -.02, .08])
        for count in (1, 2, 15, 31):
            for sample in range(8):
                times = np.linspace(.028, .028+.04*(count-1), count)
                states = rng.uniform(-.3, .3, (count, 12))
                states[:, 6:8] = rng.uniform(model.joint_lower, model.joint_upper, (count, 2))
                states[:, 8:10] = rng.uniform(-.2, .2, (count, 2))
                states[:, 10:12] = rng.uniform(-.8, .8, (count, 2))
                commands = rng.uniform(-.3, .3, (count, 5))
                context = chain.attitude_context(state, times, CHAIN)
                actual = chain.endpoint_outputs(states, commands, context, model,
                                                RESPONSE["parameters"], jacobians=True)
                # The independent old rotation implementation is used both to
                # build the context and in every nominal/FD geometry evaluation.
                with patch.object(chain, "thrust_rotation", legacy_thrust_rotation):
                    old_context = chain.attitude_context(state, times, CHAIN)
                    expected = chain.endpoint_outputs(states, commands, old_context, model,
                                                      RESPONSE["parameters"], jacobians=True)
                with self.subTest(count=count, sample=sample):
                    self.assertEqual(set(actual), set(expected))
                    for key in actual:
                        self.assertTrue(np.isfinite(actual[key]).all(), key)
                        np.testing.assert_array_equal(actual[key], expected[key], err_msg=key)
                    for key in context:
                        np.testing.assert_array_equal(context[key], old_context[key], err_msg=key)


if __name__ == "__main__":
    unittest.main()
