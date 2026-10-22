"""Optional prediction-clock binding preserves old feedback documents."""
import unittest
from test_x650_mixed_estimated_feedback import acceleration_fixture
from simulations.x650_wholebody_mpc_v1.estimated_feedback import Px4EstimatedFeedback


class PredictionTimingFeedbackTests(unittest.TestCase):
    def test_old_state_does_not_gain_implicit_control_or_joint_times(self):
        node, reference = acceleration_fixture()
        adapter = Px4EstimatedFeedback(reference, node, require_acceleration=True)
        state, _, _ = adapter.read(node, 10., wall_now=100.)
        self.assertIsNone(state.control_time_s)
        self.assertIsNone(state.joint_stamp_s)
        self.assertNotIn("prediction_timing", adapter.document)

    def test_explicit_control_clock_and_original_joint_header_are_distinct(self):
        node, reference = acceleration_fixture()
        node.data["joints"].header.stamp.sec = 9
        node.data["joints"].header.stamp.nanosec = 988000000
        adapter = Px4EstimatedFeedback(reference, node, require_acceleration=True, require_prediction_timing=True)
        state, stamp, diagnostics = adapter.read(node, 10., wall_now=100.)
        self.assertEqual(state.control_time_s, 10.)
        self.assertEqual(state.joint_stamp_s, 9.988)
        self.assertEqual(state.joint_stamp_s, stamp)
        self.assertEqual(state.joint_stamp_s, diagnostics["joint_stamp_s"])
        self.assertEqual(state.stamp_s, 10.)

    def test_future_header_is_not_silently_backpropagated(self):
        node, reference = acceleration_fixture()
        node.data["joints"].header.stamp.nanosec = 1000000
        adapter = Px4EstimatedFeedback(reference, node, require_acceleration=True, require_prediction_timing=True)
        with self.assertRaisesRegex(RuntimeError, "future joint"):
            adapter.read(node, 10., wall_now=100.)


if __name__ == "__main__":
    unittest.main()
