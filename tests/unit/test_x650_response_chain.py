"""Physical-frame and causal timestamp checks for development chain diagnostics."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("chain", ROOT / "scripts/analyze_x650_response_chain.py")
chain = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(chain)


def quaternion_wxyz(matrix):
    x, y, z, w = Rotation.from_matrix(matrix).as_quat()
    return np.array([w, x, y, z])


class FrameTests(unittest.TestCase):
    def test_same_physical_attitude_matches_px4_and_native_with_yaw(self):
        for yaw in (0., 45., 90., 170.):
            native = Rotation.from_euler("ZYX", [yaw, 7., -3.], degrees=True).as_matrix()
            px4 = chain.NED_TO_ENU.T @ native @ chain.FRD_TO_FLU
            native_q, px4_q = quaternion_wxyz(native), quaternion_wxyz(px4)
            expected = np.rad2deg(np.arctan2(native[:2, 2], native[2, 2]))
            np.testing.assert_allclose(chain.projected_tilt_deg(native_q[None], px4_frd_ned=False)[0], expected, atol=1e-12)
            np.testing.assert_allclose(chain.projected_tilt_deg(px4_q[None], px4_frd_ned=True)[0], expected, atol=1e-12)

    def test_projected_direction_is_world_direction_and_quaternion_sign_invariant(self):
        q = quaternion_wxyz(Rotation.from_euler("Y", 6., degrees=True).as_matrix())
        values = chain.projected_tilt_deg(np.array([q, -q]), px4_frd_ned=False)
        np.testing.assert_allclose(values, [[6., 0.], [6., 0.]], atol=1e-12)

    def test_rejects_arbitrary_quaternion_normalization(self):
        with self.assertRaisesRegex(ValueError, "Nonunit"):
            chain.projected_tilt_deg(np.array([[2., 0., 0., 0.]]), px4_frd_ned=True)


class ClockTests(unittest.TestCase):
    def test_acceleration_integral_checks_velocity_without_removing_bias(self):
        times = np.array([0., .02, .05, .09])
        acceleration = np.tile([.4, -.2, 0.], (4, 1))
        velocity = np.array([.3, .1, -.05])+times[:, None]*acceleration
        streams = {"native_acceleration": {"times": times, "values": acceleration},
                   "native_velocity": {"times": times, "values": velocity}}
        result = chain.native_integration_check(streams)
        np.testing.assert_allclose(result["axis_rmse_m_s"], np.zeros(3), atol=1e-15)
        streams["native_acceleration"]["values"] = acceleration+np.array([.1, 0., 0.])
        result = chain.native_integration_check(streams)
        self.assertAlmostEqual(result["final_residual_m_s"][0], .009)

    def test_timestamp_sample_is_used_for_state_and_timestamp_for_target(self):
        class Item:
            name, multi_id = "test", 0
            data = {"timestamp": np.array([1020000., 1040000., 1060000., 1080000.]),
                    "timestamp_sample": np.array([1000000., 1020000., 1040000., 1060000.]), "v": np.arange(4.)}
        class Log:
            data_list = [Item()]
        state = chain.ulog_stream(Log(), "test", ["v"], [1.02, 1.06], state=True, max_gap=.03)
        target = chain.ulog_stream(Log(), "test", ["v"], [1.02, 1.06], state=False, max_gap=.03)
        self.assertEqual(state["values"][0, 0], 1.)
        self.assertEqual(target["values"][0, 0], 0.)
        self.assertEqual(state["timing"]["time_shift_s"], 0.)

    def test_missing_state_sample_timestamp_is_not_silently_replaced(self):
        class Item:
            name, multi_id = "test", 0
            data = {"timestamp": np.arange(4.)*20000., "v": np.arange(4.)}
        class Log:
            data_list = [Item()]
        with self.assertRaisesRegex(ValueError, "Missing"):
            chain.ulog_stream(Log(), "test", ["v"], [.02, .04], state=True, max_gap=.03)

    def test_sparse_and_conflicting_sample_streams_rejected(self):
        with self.assertRaisesRegex(ValueError, "sparse"):
            chain.validated_stream([0., .02, .20], [[0.], [1.], [2.]], [.01, .19], label="x", max_gap=.05, clock="sim_s")
        with self.assertRaisesRegex(ValueError, "Conflicting"):
            chain.validated_stream([0., .02, .02, .04], [[0.], [1.], [2.], [3.]], [.01, .03],
                                   label="x", max_gap=.05, clock="timestamp_sample")

    def test_first_crossing_is_causal_sample_interval_not_interpolated(self):
        stream = {"times": np.array([.98, 1.02, 1.06, 1.10]), "values": np.array([[0.], [.2], [.6], [1.]])}
        result = chain.first_sampled_crossing(stream, column=0, start=1., end=1.2, threshold=.5, direction=1)
        self.assertEqual(result["crossing_interval_t_s"], [1.02, 1.06])
        self.assertEqual(result["first_reached_t_s"], 1.06)
        self.assertAlmostEqual(result["elapsed_from_command_s"], .06)

    def test_preexisting_threshold_hit_does_not_claim_a_postcommand_crossing(self):
        stream = {"times": np.array([.98, 1.02, 1.06]), "values": np.array([[.8], [.8], [.9]])}
        result = chain.first_sampled_crossing(stream, column=0, start=1., end=1.2, threshold=.5, direction=1)
        self.assertTrue(result["already_reached_at_previous_sample"])
        self.assertIsNone(result["crossing_interval_t_s"])
        self.assertTrue(result["causal_onset_unresolved"])

    def test_crossing_interval_keeps_the_real_precommand_sample_time(self):
        stream = {"times": np.array([.98, 1.02, 1.06]), "values": np.array([[.2], [.8], [.9]])}
        result = chain.first_sampled_crossing(stream, column=0, start=1., end=1.2, threshold=.5, direction=1)
        self.assertEqual(result["crossing_interval_t_s"], [.98, 1.02])
        self.assertTrue(result["interval_straddles_command"])
        self.assertTrue(result["causal_onset_unresolved"])

    def test_event_stops_at_actual_command_departure_not_the_next_onset(self):
        command_t = np.array([1.98, 2.012, 2.512, 3.016, 3.20, 4.012])
        command_v = np.column_stack(([0., .08, .08, 0., 0., -.08], np.zeros((6, 2))))
        sampled_t = np.array([1.98, 2.02, 2.10, 2.20, 2.40, 2.90, 3.02, 3.40])
        native_v = np.column_stack(([0., .001, .002, .01, .02, .03, .06, .08], np.zeros((8, 2))))
        tilt = np.column_stack(([0., .1, .2, .15, .1, .05, .03, .02], np.zeros((8, 1))))
        streams = {
            "logical_velocity": {"times": command_t, "values": command_v},
            "internal_velocity": {"times": command_t, "values": command_v},
            "native_velocity": {"times": sampled_t, "values": native_v}}
        for name in ("attitude_target", "attitude_estimated", "attitude_native"):
            streams[name] = {"times": sampled_t, "values": tilt}
        result = chain.event_diagnostics(streams, 0, 2.)
        self.assertEqual(result["pulse_end_t_s"], 3.016)
        self.assertAlmostEqual(result["actual_pulse_hold_s"], 1.004)
        self.assertTrue(result["native_velocity_10_percent"]["observed"])
        self.assertFalse(result["native_velocity_50_percent"]["observed"])
        self.assertEqual(result["native_velocity_50_percent"]["search_end_t_s"], 3.016)


if __name__ == "__main__":
    unittest.main()
