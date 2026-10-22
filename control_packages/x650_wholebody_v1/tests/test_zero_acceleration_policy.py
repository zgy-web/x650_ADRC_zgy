"""The zero-FF policy must reach the final PVA command, without changing velocity."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from x650_wholebody.adapter import ControllerAdapter
from x650_wholebody.api import EndpointTarget
from x650_wholebody.model import tip_world
from test_adapter import AllocatorStub, GAINS, state
from test_model import example_model


class ZeroAccelerationPolicyTests(unittest.TestCase):
    def test_nominal_and_arm_accelerations_are_suppressed_in_final_px4_setpoint(self):
        model, measured = example_model(elbow_offset_body_m=(0., .04, 0.)), state()
        target = EndpointTarget(tip_world(measured.position, measured.rotation, measured.q, model))
        outputs = []
        for policy in ("legacy", "zero"):
            wrapper = ControllerAdapter(AllocatorStub(model, a=(10., -8., 7.), qdd=(10000., -10000.)),
                                        model, GAINS, acceleration_feedforward_policy=policy)
            wrapper.reset(measured.position, measured.rotation, measured.q)
            wrapper.observe(replace(measured, stamp_s=.98, omega_body=[.1, -.2, .3]))
            command = wrapper.step(0., .02, measured, target)
            outputs.append(command)
            self.assertEqual(wrapper.metadata()["acceleration_feedforward_policy"], policy)
            self.assertEqual(command.diagnostics["acceleration_feedforward_policy"], policy)
            self.assertGreater(np.linalg.norm(command.diagnostics["nominal_acceleration_world"]), 1.)
            self.assertGreater(np.linalg.norm(command.diagnostics["arm_acceleration_world"]), .01)
            if policy == "zero":
                np.testing.assert_array_equal(command.acceleration_feedforward, np.zeros(3))
                np.testing.assert_array_equal(command.diagnostics["command_acceleration_world"], np.zeros(3))
                np.testing.assert_array_equal(command.px4_setpoint([0., 0., -2.], np.zeros(3)).acceleration, np.zeros(3))
                self.assertFalse(command.diagnostics["acceleration_feedforward_applied"])
                self.assertFalse(command.diagnostics["acceleration_saturated"])
                self.assertTrue(command.diagnostics["candidate_acceleration_saturated"])
        self.assertAlmostEqual(np.linalg.norm(outputs[0].acceleration_feedforward), 3.)
        for name in ("position_reference", "velocity_feedforward", "base_velocity_target", "joint_velocity"):
            np.testing.assert_array_equal(getattr(outputs[0], name), getattr(outputs[1], name))
        np.testing.assert_array_equal(outputs[0].px4_setpoint([0., 0., -2.], np.zeros(3)).velocity,
                                      outputs[1].px4_setpoint([0., 0., -2.], np.zeros(3)).velocity)

    def test_default_policy_remains_legacy_and_unknown_policy_fails(self):
        model = example_model()
        self.assertEqual(ControllerAdapter(AllocatorStub(model), model, GAINS).acceleration_feedforward_policy, "legacy")
        for policy in (None, False, "off", "ZERO"):
            with self.subTest(policy=policy), self.assertRaises(ValueError):
                ControllerAdapter(AllocatorStub(model), model, GAINS, acceleration_feedforward_policy=policy)


if __name__ == "__main__":
    unittest.main()
