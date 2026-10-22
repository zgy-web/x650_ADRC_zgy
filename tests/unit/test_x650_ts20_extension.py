"""Extension fault-recovery and unmodified capture-boundary contracts."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from unittest.mock import Mock
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_ts20_hold_v1.run_case import Ts20Extension
from simulations.x650_wholebody_mpc_v1 import run_case as shared


class Ts20ExtensionTests(unittest.TestCase):
    def test_recovery_retains_physical_feedback_after_actuator_source_failure(self):
        extension = Ts20Extension({}, {})
        def failed(*args):
            raise RuntimeError("missing/stale actuator log")
        extension.runtime = SimpleNamespace(measured_state=failed)
        physical = object()
        self.assertEqual(extension.after_state_read(SimpleNamespace(recovery=True), physical, 12., 12.), (physical, 12.))
        with self.assertRaisesRegex(RuntimeError, "actuator log"):
            extension.after_state_read(SimpleNamespace(recovery=False), physical, 12., 12.)

    def test_capture_keeps_original_complete_bytes_with_causal_end(self):
        extension = Ts20Extension({}, {})
        original = b"sim_s,q\n1.0,0.100000000\n1.004,0.200000000\n1.008,0.3\n1.012,0."
        saved = []
        def write(path, value):
            saved.append(value)
            return len(value)
        with patch.object(Path, "read_bytes", return_value=original), \
                patch.object(Path, "write_bytes", write), \
                patch("simulations.x650_ts20_hold_v1.run_case.task.sha", return_value="a" * 64):
            manifest = extension._freeze_csv(ROOT / "synthetic_live.csv", ROOT / "synthetic_frozen.csv", "sim_s", 1.004)
        self.assertEqual(saved, [b"sim_s,q\n1.0,0.100000000\n1.004,0.200000000\n"])
        self.assertEqual(manifest["samples"], 2)
        self.assertEqual(manifest["last_sim_s"], 1.004)

    def test_whole_compute_deadline_rejects_before_any_publisher_call(self):
        extension = Ts20Extension({"ts20_protocol": {"max_compute_s": .05}}, {})
        loop = shared.ControlLoop.__new__(shared.ControlLoop)
        loop.extension = extension
        loop.original_pump = lambda: None
        loop.upper_research = False
        loop.ctx = SimpleNamespace(node=SimpleNamespace(spin_once=lambda **kwargs: None))
        loop.recorder = SimpleNamespace(now=lambda: 10., phase="measure")
        loop.land_watchdog = SimpleNamespace(observe=lambda *args: None)
        loop.status_watchdog = None
        loop.control_now = lambda: 10.
        loop.next_control, loop.last, loop.strict = 0., None, True
        loop.read = lambda: (10., SimpleNamespace(q=np.zeros(2)), 10.)
        loop.cfg = {"control_period_s": .02, "stage": "hover_folded", "duration_s": 20.}
        loop.task_start, loop.controller, loop.send_reference, loop.recovery = None, None, False, False
        loop.execution_trace = SimpleNamespace(publish=Mock(), _failed=Mock())
        with patch.dict(sys.modules, {"pymavlink": SimpleNamespace(mavutil=object())}), \
                patch.object(shared, "callback_clock_snapshot", return_value={}), \
                patch("simulations.x650_ts20_hold_v1.run_case.time.perf_counter", side_effect=[0., .050001]):
            with self.assertRaisesRegex(RuntimeError, "before publication"):
                loop.pump()
        loop.execution_trace.publish.assert_not_called()
        loop.execution_trace._failed.assert_called_once()

    def test_recovery_skips_experimental_compute_deadline(self):
        extension = Ts20Extension({"ts20_protocol": {"max_compute_s": .05}}, {})
        result = extension.before_publish(SimpleNamespace(recovery=True), -100., 10., None)
        self.assertFalse(result["active"])


if __name__ == "__main__":
    unittest.main()
