"""Validate declared step timing/amplitudes, never flight-controller state."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from position_step_probe import (STATIC_HOVER_S, PROBE_DURATION_S, TOTAL_DURATION_S,
    world_offset, segment_at, probe_metadata)


class PositionStepProbeTests(unittest.TestCase):
    def test_six_independent_positive_and_negative_steps_with_six_second_windows(self):
        document = probe_metadata()
        self.assertEqual((STATIC_HOVER_S, PROBE_DURATION_S, TOTAL_DURATION_S), (20., 36., 56.))
        self.assertEqual(len(document["segments"]), 6)
        previous, previous_end = [0., 0., 0.], 20.
        expected_deltas = [[.15, 0., 0.], [-.15, 0., 0.], [0., .15, 0.],
                           [0., -.15, 0.], [0., 0., .10], [0., 0., -.10]]
        for segment, expected in zip(document["segments"], expected_deltas):
            self.assertEqual(segment["start_s"], previous_end)
            self.assertEqual(segment["end_s"]-segment["start_s"], 6.)
            self.assertEqual(segment["command_delta_world_enu_m"], expected)
            self.assertEqual(segment["previous_target_offset_world_enu_m"], previous)
            target = world_offset(segment["start_s"]+2.)
            self.assertEqual([a-b for a, b in zip(target, previous)], expected)
            self.assertEqual(sum(value != 0 for value in expected), 1)
            self.assertEqual(segment["signed_step_m"], expected[segment["axis_index"]])
            previous, previous_end = target, segment["end_s"]
        self.assertEqual(previous, [0., 0., 0.])
        self.assertEqual(previous_end, 56.)

    def test_exact_boundaries_have_no_missing_or_extra_step(self):
        self.assertIsNone(segment_at(0.))
        self.assertIsNone(segment_at(math.nextafter(20., 0.)))
        self.assertEqual(world_offset(19.999), [0., 0., 0.])
        for index in range(6):
            start = 20.+6.*index
            self.assertEqual(segment_at(start)["segment_index"], index)
            self.assertEqual(segment_at(math.nextafter(start+6., 0.))["segment_index"], index)
        self.assertIsNone(segment_at(56.))
        self.assertEqual(world_offset(56.), [0., 0., 0.])
        self.assertEqual(world_offset(1000.), [0., 0., 0.])

    def test_target_is_constant_within_segment_and_independent_of_call_order(self):
        for start in (20., 26., 32., 38., 44., 50.):
            expected = world_offset(start)
            for dt in (5.9, .02, 3., 0., 4., .1):
                self.assertEqual(world_offset(start+dt), expected)
        self.assertEqual(world_offset(20.), [.15, 0., 0.])

    def test_returned_values_cannot_mutate_the_protocol(self):
        point = world_offset(20.)
        point[0] = 1.
        segment = segment_at(26.)
        segment["target_offset_world_enu_m"][0] = 1.
        document = probe_metadata()
        document["segments"][0]["start_s"] = 0.
        document["analysis_contract"]["settling_position_error_m"] = .20
        self.assertEqual(world_offset(20.), [.15, 0., 0.])
        self.assertEqual(world_offset(26.), [0., 0., 0.])
        self.assertEqual(probe_metadata()["segments"][0]["start_s"], 20.)
        self.assertEqual(probe_metadata()["analysis_contract"]["settling_position_error_m"], .02)

    def test_analysis_contract_uses_absolute_thresholds_and_no_fitting(self):
        document = probe_metadata()
        analysis = document["analysis_contract"]
        self.assertEqual(analysis["rise_fraction"], [.10, .90])
        self.assertEqual(analysis["settling_position_error_m"], .02)
        self.assertEqual(analysis["settling_speed_m_s"], .03)
        self.assertEqual(analysis["settling_continuous_duration_s"], 2.)
        self.assertTrue(analysis["no_fitted_recentring"])
        self.assertIn("Adjacent declared targets", analysis["step_size_basis"])
        self.assertEqual(len(document["source_sha256"]), 64)

    def test_invalid_times_rejected_without_silently_restarting(self):
        for value in (-.01, float("nan"), float("inf"), None, True, "unknown"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                world_offset(value)


if __name__ == "__main__":
    unittest.main()
