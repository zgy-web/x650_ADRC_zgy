"""Counterexamples for failed-performance video labels and causal wrench replay.

No rendering backend, GPU, physics process or encoder is started by these tests.
"""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import render_x650_multirate_wrench as video


def audited_metrics():
    return {
        "sample_count": 6001,
        "position_rmse_m": 0.04235824924886268,
        "position_max_m": 0.1851721805797341,
        "axis_rmse_m": [0.042089038342305304, 0.002148369664531617,
                        0.004256599421281369],
        "duration_s": 23.999999999999993,
    }


def report_fixture():
    return {
        "kind": "x650_cpp_multirate_wrench_followup_report_v1",
        "timing_profile": "mpc25_dispatch50_v1",
        "complete": True,
        "integrity_and_execution_passed": True,
        "passed": False,
        "physical_acceptance": False,
        "recovery_performance_passed": True,
        "endpoint": audited_metrics(),
    }


class ReportBindingTests(unittest.TestCase):
    def test_complete_execution_keeps_failed_performance_and_original_values(self):
        report, metrics = report_fixture(), audited_metrics()
        before = deepcopy((report, metrics))
        video.check_report(report, metrics)
        self.assertEqual((report, metrics), before)
        self.assertFalse(report["passed"])
        self.assertFalse(report["physical_acceptance"])

    def test_recomputed_metrics_cannot_be_substituted(self):
        changes = {
            "sample_count": 6000,
            "position_rmse_m": 0.004,
            "position_max_m": 0.085,
            "axis_rmse_m": [0.042089038342305304, 0.002148369664531617, 0.01],
            "duration_s": 23.0,
        }
        for field, replacement in changes.items():
            metrics = audited_metrics()
            metrics[field] = replacement
            with self.subTest(field=field), self.assertRaises(ValueError):
                video.check_report(report_fixture(), metrics)

    def test_partial_or_wrong_audit_never_gets_complete_execution_label(self):
        for field, replacement in (
            ("complete", False),
            ("integrity_and_execution_passed", False),
            ("complete", 1),
            ("kind", "x650_cpp_fast_stage_certificate_v1"),
            ("timing_profile", "legacy"),
        ):
            report = report_fixture()
            report[field] = replacement
            with self.subTest(field=field, value=replacement), self.assertRaises(ValueError):
                video.check_report(report, audited_metrics())

    def test_acceptance_or_recovery_labels_cannot_be_reversed(self):
        for field, replacement in (
            ("passed", True),
            ("physical_acceptance", True),
            ("recovery_performance_passed", False),
            ("physical_acceptance", None),
        ):
            report = report_fixture()
            report[field] = replacement
            with self.subTest(field=field), self.assertRaises(ValueError):
                video.check_report(report, audited_metrics())

    def test_matching_metrics_below_peak_limit_cannot_be_called_this_failure(self):
        metrics = audited_metrics()
        metrics["position_max_m"] = 0.1
        report = report_fixture()
        report["endpoint"] = deepcopy(metrics)
        with self.assertRaises(ValueError):
            video.check_report(report, metrics)

    def test_nonfinite_metrics_are_rejected_even_when_report_has_same_nan(self):
        metrics = audited_metrics()
        metrics["position_rmse_m"] = float("nan")
        report = report_fixture()
        report["endpoint"] = deepcopy(metrics)
        with self.assertRaises(ValueError):
            video.check_report(report, metrics)


class WrenchAndTimelineTests(unittest.TestCase):
    def test_wrench_display_uses_latest_logged_sample_not_linear_interpolation(self):
        values = np.array([
            [6.996, 0., 0.],
            [7.000, 1.5, 0.],
            [7.604, 0., 0.],
            [13.000, -1.5, 0.],
            [13.604, 0., 0.],
            [20.504, 0., 0.04],
            [20.900, 0., 0.],
            [23.996, 0., 0.],
        ])
        before = values.copy()
        cases = (
            (0., (0., 0.)),
            (6.999, (0., 0.)),
            (7.000, (1.5, 0.)),
            (7.6039, (1.5, 0.)),
            (7.604, (0., 0.)),
            (12.9999, (0., 0.)),
            (13.000, (-1.5, 0.)),
            (20.500, (0., 0.)),
            (20.504, (0., 0.04)),
            (20.899, (0., 0.04)),
            (20.900, (0., 0.)),
            (24.000, (0., 0.)),
        )
        for now, expected in cases:
            with self.subTest(task_s=now):
                self.assertEqual(video.wrench_at(values, now), expected)
        np.testing.assert_array_equal(values, before)

    def test_one_times_timeline_is_24_seconds_with_separate_end_state(self):
        frames = video.visual.frame_times(24., expected_duration=24.)
        self.assertEqual(len(frames), 576)
        self.assertEqual(frames[0], 0.)
        self.assertEqual(frames[-1], 575 / 24)
        np.testing.assert_allclose(np.diff(frames), 1 / 24, rtol=0, atol=4e-15)
        self.assertEqual(len(frames) / video.visual.FPS, 24.)
        final = video.visual.frame_times(24., preview_time=24., expected_duration=24.)
        np.testing.assert_array_equal(final, [24.])


if __name__ == "__main__":
    unittest.main()
