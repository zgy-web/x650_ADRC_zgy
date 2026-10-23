"""Process cleanup must not abandon later owned groups on timeout/races."""
import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, Mock, patch
import uuid

import numpy as np

PATH = Path(__file__).resolve().parents[2] / "scripts/run_x650_ros2_structure_smoke.py"
SPEC = importlib.util.spec_from_file_location("structure_smoke", PATH)
smoke = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(smoke)


class CleanupTests(unittest.TestCase):
    def setUp(self):
        self.signals = SimpleNamespace(SIGINT=2, SIGTERM=15, SIGKILL=9)
        self.group_patch = patch.object(smoke, "group_has_live_members", return_value=False)
        self.group = self.group_patch.start()
        self.addCleanup(self.group_patch.stop)

    def test_already_exited_does_not_signal(self):
        proc = Mock(pid=123)
        proc.poll.return_value = 0
        with patch.object(smoke, "signal", self.signals), patch.object(smoke.os, "killpg", create=True) as kill:
            self.assertEqual(smoke.stop_owned_process(proc), [])
            kill.assert_not_called()

    def test_exit_race_is_reaped_without_hiding_cleanup(self):
        proc = Mock(pid=123)
        proc.poll.return_value = None
        proc.wait.return_value = 0
        with patch.object(smoke, "signal", self.signals), patch.object(smoke.os, "killpg", side_effect=ProcessLookupError, create=True):
            self.assertEqual(smoke.stop_owned_process(proc), [])
            proc.wait.assert_called_once_with(timeout=8.)

    def test_escalation_has_finite_kill_fallback(self):
        proc = Mock(pid=123)
        proc.poll.return_value = None
        proc.wait.side_effect = [subprocess.TimeoutExpired("owned", 8), subprocess.TimeoutExpired("owned", 5), 0]
        with patch.object(smoke, "signal", self.signals), patch.object(smoke.os, "killpg", create=True) as kill:
            self.assertEqual(smoke.stop_owned_process(proc), [])
            self.assertEqual([call.args for call in kill.call_args_list], [(123, 2), (123, 15), (123, 9)])

    def test_unreapable_group_returns_error_instead_of_escaping(self):
        proc = Mock(pid=123)
        proc.poll.return_value = None
        proc.wait.side_effect = subprocess.TimeoutExpired("owned", 1)
        with patch.object(smoke, "signal", self.signals), patch.object(smoke.os, "killpg", create=True):
            errors = smoke.stop_owned_process(proc)
            self.assertEqual(len(errors), 1)
            self.assertIn("SIGKILL", errors[0])

    def test_exited_cli_does_not_leave_its_live_node_behind(self):
        proc = Mock(pid=123)
        proc.poll.return_value = 0
        proc.wait.return_value = 0
        self.group.side_effect = [True, False, False]
        with patch.object(smoke, "signal", self.signals), patch.object(smoke.os, "killpg", create=True) as kill:
            self.assertEqual(smoke.stop_owned_process(proc), [])
            kill.assert_called_once_with(123, 2)


class ProcessGroupTests(unittest.TestCase):
    def test_linux_stat_handles_parenthesized_names_and_ignores_zombies(self):
        zombie, live = MagicMock(), MagicMock()
        zombie.name, live.name = "41", "42"
        zombie.__truediv__.return_value.read_text.return_value = "41 (old node) Z 1 123 123 0"
        live.__truediv__.return_value.read_text.return_value = "42 (node with ) space) S 1 123 123 0"
        with patch.object(smoke, "Path") as path:
            path.return_value.iterdir.return_value = [zombie]
            self.assertFalse(smoke.group_has_live_members(123))
            path.return_value.iterdir.return_value = [zombie, live]
            self.assertTrue(smoke.group_has_live_members(123))
            self.assertFalse(smoke.group_has_live_members(999))


class ReportTests(unittest.TestCase):
    def setUp(self):
        # Ordinary workspace mkdir retains Windows inherited permissions; the
        # test owns only this new directory and removes its immediate files.
        self.folder = Path(__file__).resolve().parent / ("_smoke_report_"+uuid.uuid4().hex)
        self.folder.mkdir()
        def cleanup():
            for path in self.folder.iterdir():
                path.unlink()
            self.folder.rmdir()
        self.addCleanup(cleanup)

    def test_ros_uint64_sequences_become_lossless_python_integers(self):
        original = np.array([0, 2**53+1, 2**64-1], dtype=np.uint64)
        saved = original.copy()
        counts = smoke.stream_counts_for_report(original)
        self.assertTrue(all(type(value) is int for value in counts))
        self.assertEqual(counts, [0, 9007199254740993, 18446744073709551615])
        np.testing.assert_array_equal(original, saved)
        path = self.folder/"report.json"
        smoke.write_json(path, {"expected_stream_counts": counts})
        self.assertEqual(json.loads(path.read_text())["expected_stream_counts"], counts)
        self.assertIn("18446744073709551615", path.read_text())

    def test_python_large_uint64_values_do_not_pass_through_float(self):
        values = (2**53+1, 2**63, 2**64-1)
        self.assertEqual(smoke.stream_counts_for_report(values), list(values))
        self.assertEqual(json.loads(json.dumps(smoke.stream_counts_for_report(values))), list(values))

    def test_invalid_sequence_shape_type_and_uint64_range_are_refused(self):
        for values in ([0, 1], [0, 1, 2, 3], [0, 1.5, 2], [0, "1", 2],
                       [0, True, 2], [0, np.bool_(True), 2], [-1, 0, 0], [0, 0, 2**64]):
            with self.subTest(values=values):
                with self.assertRaises((TypeError, ValueError)):
                    smoke.stream_counts_for_report(values)

    def test_unconverted_numpy_uint64_failure_leaves_no_partial_file(self):
        path = self.folder/"report.json"
        # This is the exact failure class from the healthy transport report:
        # list(ROS uint64[3]) preserves NumPy uint64 rather than Python int.
        report = {"complete": True, "expected_stream_counts": list(np.array([5, 5, 5], dtype=np.uint64))}
        with self.assertRaisesRegex(TypeError, "uint64"):
            smoke.write_json(path, report)
        self.assertFalse(path.exists())

    def test_nonfinite_and_unsupported_values_fail_before_opening_output(self):
        for value in ({"value": float("nan")}, {"value": float("inf")}, {"value": object()}):
            with self.subTest(value=value):
                path = self.folder/"report.json"
                with patch.object(smoke.Path, "open") as opened:
                    with self.assertRaises((TypeError, ValueError)):
                        smoke.write_json(path, value)
                    opened.assert_not_called()
                self.assertFalse(path.exists())

    def test_existing_failure_evidence_is_never_overwritten(self):
        path = self.folder/"report.json"
        original = b'{"complete":false,"error":"preserved evidence"}\n'
        path.write_bytes(original)
        with self.assertRaises(FileExistsError):
            smoke.write_json(path, {"complete": True})
        self.assertEqual(path.read_bytes(), original)
        with self.assertRaises(TypeError):
            smoke.write_json(path, {"value": np.uint64(1)})
        self.assertEqual(path.read_bytes(), original)

    def test_success_keeps_original_json_format_and_one_final_newline(self):
        path = self.folder/"report.json"
        value = {"complete": True, "nested": [1, 2**64-1, "关节"]}
        smoke.write_json(path, value)
        self.assertEqual(path.read_text(encoding="utf-8"), json.dumps(value, indent=2, allow_nan=False)+"\n")


if __name__ == "__main__":
    unittest.main()
