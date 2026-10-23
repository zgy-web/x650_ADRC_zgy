"""Process cleanup must not abandon later owned groups on timeout/races."""
import importlib.util
from pathlib import Path
import subprocess
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

PATH = Path(__file__).resolve().parents[2] / "scripts/run_x650_ros2_structure_smoke.py"
SPEC = importlib.util.spec_from_file_location("structure_smoke", PATH)
smoke = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(smoke)


class CleanupTests(unittest.TestCase):
    def setUp(self):
        self.signals = SimpleNamespace(SIGINT=2, SIGTERM=15, SIGKILL=9)

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


if __name__ == "__main__":
    unittest.main()
