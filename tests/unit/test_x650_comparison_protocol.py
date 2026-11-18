"""Coordinate and experiment-boundary checks for the common comparison path."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "comparison_protocol", ROOT / "simulations/x650_control_comparison_v1/protocol.py")
P = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(P)


class ComparisonProtocolTests(unittest.TestCase):
    def test_enu_ned_known_directions_and_round_trip(self):
        np.testing.assert_array_equal(P.enu_ned([1, 0, 0]), [0, 1, 0])  # east
        np.testing.assert_array_equal(P.enu_ned([0, 1, 0]), [1, 0, 0])  # north
        np.testing.assert_array_equal(P.enu_ned([0, 0, 1]), [0, 0, -1])  # up
        value = np.array([.12, -.3, 1.5])
        np.testing.assert_array_equal(P.enu_ned(P.enu_ned(value)), value)

    def test_prestream_never_starts_a_disturbance_or_trajectory(self):
        for case in P.CASES:
            with self.subTest(case=case):
                offset, joints, force = P.reference(case, -1.)
                np.testing.assert_array_equal(offset, [0]*3)
                np.testing.assert_array_equal(joints, [0]*2)
                np.testing.assert_array_equal(force, [0]*3)

    def test_step_axes_and_exact_switch_boundaries(self):
        for axis, start in enumerate((5., 15., 25.)):
            expected = np.zeros(3)
            expected[axis] = .2
            for when in (start, start + 4.999):
                np.testing.assert_array_equal(P.reference("step", when)[0], expected)
            np.testing.assert_array_equal(P.reference("step", start-.001)[0], [0]*3)
            np.testing.assert_array_equal(P.reference("step", start+5.)[0], [0]*3)

    def test_arm_and_force_have_independent_finite_windows(self):
        np.testing.assert_array_equal(P.reference("arm", 5.)[1], [.15, .10])
        np.testing.assert_array_equal(P.reference("arm", 13.)[1], [0, 0])
        np.testing.assert_array_equal(P.reference("force", 9.999)[2], [0, 0, 0])
        np.testing.assert_array_equal(P.reference("force", 10.)[2], [1, 0, 0])
        np.testing.assert_array_equal(P.reference("force", 12.)[2], [0, 0, 0])
        np.testing.assert_array_equal(P.reference("arm", 10.)[2], [0, 0, 0])
        np.testing.assert_array_equal(P.reference("force", 10.)[1], [0, 0])

    def test_joint_reference_does_not_leak_mutable_state_between_calls(self):
        reference = P.reference("arm", 6.)
        reference[1][0] = 99.
        np.testing.assert_array_equal(P.reference("arm", 6.)[1], [.15, .10])

    def test_limiter_applies_simulation_dt_to_slew(self):
        previous = np.array([.1, -.1, .39])
        limited, changed = P.limit_command([1., -1., 1.], previous, .04)
        np.testing.assert_allclose(limited, [.14, -.14, .4])
        self.assertTrue(changed)
        limited, changed = P.limit_command(previous, previous, .04)
        np.testing.assert_array_equal(limited, previous)
        self.assertFalse(changed)

    def test_common_limits_hold_during_reversing_commands(self):
        previous = np.zeros(3)
        for k in range(250):
            dt = .02 if k % 2 else .04
            requested = np.array([10., -5., 3.]) * (-1 if k % 50 < 25 else 1)
            command, _ = P.limit_command(requested, previous, dt)
            self.assertLessEqual(np.max(np.abs(command)), .4 + 1e-12)
            self.assertLessEqual(np.max(np.abs(command-previous)), dt + 1e-12)
            previous = command

    def test_invalid_command_and_timestep_are_rejected(self):
        for dt in (0, -.01, float("nan"), .151):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                P.limit_command([0]*3, [0]*3, dt)
        for value in ([0, 0], [0, 0, float("nan")], [0, float("inf"), 0]):
            with self.subTest(command=value), self.assertRaises(ValueError):
                P.limit_command(value, [0]*3, .02)
        with self.assertRaises(ValueError):
            P.reference("missing", 0.)

    def test_identification_separates_positive_and_negative_pulses(self):
        # Off-axis position hold continues while the selected axis receives an
        # exogenous probe; its positive and held-out negative pulses are disjoint.
        position, target = np.array([.01, -.02, .03]), np.zeros(3)
        for axis, start in enumerate((5., 18., 31.)):
            values = [P.identification_command(start+offset, position, target)[axis]
                      for offset in (0., 2., 4., 6.)]
            np.testing.assert_array_equal(values, [.1, 0., -.1, 0.])
            command = P.identification_command(start+.5, position, target)
            for other in range(3):
                if other != axis:
                    self.assertAlmostEqual(command[other], -1.2*position[other])


if __name__ == "__main__":
    unittest.main()
