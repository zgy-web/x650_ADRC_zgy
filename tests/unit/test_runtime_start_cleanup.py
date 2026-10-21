"""Failed startup cleans its own Popen, without trusting a replacement state."""
from contextlib import ExitStack
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

from amsim.runtime import process
from amsim import run_manager


class StartupCleanupTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.root = Path(self.stack.enter_context(tempfile.TemporaryDirectory()))
        self.state_dir = self.root / "state"
        self.state_file = self.state_dir / "runtime.json"
        self.results = self.root / "results"
        self.results.mkdir()
        for name, value in (("STATE_DIR",self.state_dir),("STATE_FILE",self.state_file),("PROJECT_ROOT",self.root)):
            self.stack.enter_context(patch.object(process,name,value))
        self.stack.enter_context(patch.object(process,"results_dir",return_value=self.results))
        self.stack.enter_context(patch.object(process,"status",return_value={"running":False}))
        self.proc = Mock(pid=4242, returncode=None)
        self.proc.poll.side_effect = lambda: self.proc.returncode
        def finish(timeout):
            self.proc.returncode = 0
            return 0
        self.proc.wait.side_effect = finish
        self.popen = self.stack.enter_context(patch.object(process.subprocess,"Popen",return_value=self.proc))
        self.stop = self.stack.enter_context(patch.object(process,"stop"))
        self.kill = self.stack.enter_context(patch.object(process.os,"kill"))
        # os.killpg is absent on native Windows, but must never be used here.
        self.killpg = self.stack.enter_context(patch.object(process.os,"killpg",create=True))

    def assert_owned_only(self):
        self.stop.assert_not_called()
        self.kill.assert_not_called()
        self.killpg.assert_not_called()

    def test_readiness_timeout_interrupts_and_reaps_owned_supervisor(self):
        with self.assertRaisesRegex(RuntimeError,"did not become ready"):
            process.start(wait_s=0)
        self.proc.send_signal.assert_called_once_with(signal.SIGINT)
        self.proc.wait.assert_called_once_with(timeout=15.)
        self.proc.terminate.assert_not_called()
        self.proc.kill.assert_not_called()
        self.assertFalse(self.state_file.exists())
        self.assertTrue(self.popen.call_args.kwargs["stdout"].closed)
        self.assert_owned_only()

    def test_state_write_failure_also_cleans_and_preserves_original_error(self):
        original = OSError("disk is full during state write")
        with patch.object(Path,"write_text",side_effect=original):
            with self.assertRaises(OSError) as caught:
                process.start(wait_s=0)
        self.assertIs(caught.exception,original)
        self.proc.send_signal.assert_called_once_with(signal.SIGINT)
        self.assert_owned_only()

    def test_replacement_state_never_selects_a_pid_to_signal_or_delete(self):
        replacement = {"pid":9999,"started_at":1.25}
        self.proc.send_signal.side_effect = lambda _: self.state_file.write_text(json.dumps(replacement))
        with self.assertRaisesRegex(RuntimeError,"did not become ready"):
            process.start(wait_s=0)
        self.assertEqual(json.loads(self.state_file.read_text()),replacement)
        self.proc.send_signal.assert_called_once_with(signal.SIGINT)
        self.assert_owned_only()

    def test_same_pid_with_different_start_identity_is_not_deleted(self):
        replacement = {"pid":self.proc.pid,"started_at":1.25}
        self.proc.send_signal.side_effect = lambda _: self.state_file.write_text(json.dumps(replacement))
        with self.assertRaisesRegex(RuntimeError,"did not become ready"):
            process.start(wait_s=0)
        self.assertEqual(json.loads(self.state_file.read_text()),replacement)
        self.assert_owned_only()

    def test_unrecognized_replacement_state_does_not_mask_timeout(self):
        self.proc.send_signal.side_effect = lambda _: self.state_file.write_text('[]')
        with self.assertRaisesRegex(RuntimeError,"did not become ready"):
            process.start(wait_s=0)
        self.assertEqual(self.state_file.read_text(),'[]')
        self.assert_owned_only()

    def test_cleanup_failure_preserves_start_error_and_running_state(self):
        self.proc.send_signal.side_effect = PermissionError("cannot interrupt")
        self.proc.terminate.side_effect = PermissionError("cannot terminate")
        self.proc.kill.side_effect = PermissionError("cannot kill")
        with self.assertRaisesRegex(RuntimeError,"did not become ready") as caught:
            process.start(wait_s=0)
        self.assertTrue(self.state_file.exists())
        if hasattr(caught.exception,"__notes__"):
            self.assertIn("Startup cleanup failed",caught.exception.__notes__[0])
        self.assert_owned_only()

    def test_keyboard_interrupt_still_cleans_just_created_process(self):
        with patch.object(process.time,"sleep",side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                process.start(wait_s=10)
        self.proc.send_signal.assert_called_once_with(signal.SIGINT)
        self.assertFalse(self.state_file.exists())
        self.assert_owned_only()

    def test_sigint_timeout_escalates_only_owned_popen(self):
        def wait(timeout):
            if self.proc.wait.call_count < 3:
                raise subprocess.TimeoutExpired("owned supervisor",timeout)
            self.proc.returncode = -9
            return -9
        self.proc.wait.side_effect = wait
        with self.assertRaisesRegex(RuntimeError,"did not become ready"):
            process.start(wait_s=0)
        self.proc.send_signal.assert_called_once_with(signal.SIGINT)
        self.proc.terminate.assert_called_once_with()
        self.proc.kill.assert_called_once_with()
        self.assertFalse(self.state_file.exists())
        self.assert_owned_only()

    def test_spawn_failure_closes_log_without_signalling(self):
        self.popen.side_effect = OSError("cannot spawn")
        with self.assertRaisesRegex(OSError,"cannot spawn"):
            process.start(wait_s=0)
        self.proc.send_signal.assert_not_called()
        self.assertTrue(self.popen.call_args.kwargs["stdout"].closed)
        self.assert_owned_only()

    def test_early_exit_does_not_signal_dead_process_and_retains_log_error(self):
        self.proc.returncode = 7
        with self.assertRaisesRegex(RuntimeError,"runtime exited with code 7"):
            process.start(wait_s=10)
        self.proc.send_signal.assert_not_called()
        self.assertFalse(self.state_file.exists())
        self.assert_owned_only()

    def test_success_keeps_runtime_and_existing_state_shape(self):
        folder = self.results / "runtime_success"
        def spawn(*args,**kwargs):
            folder.mkdir()
            (folder / "runtime.json").write_text('{}')
            return self.proc
        self.popen.side_effect = spawn
        state = process.start(wait_s=10,env_overrides={"AMSIM_MODEL_NAME":"example"})
        self.assertTrue(state["ready"])
        self.assertEqual(state["model"],"example")
        self.assertEqual(state["runtime_meta"],str(folder/"runtime.json"))
        self.assertEqual(json.loads(self.state_file.read_text()),state)
        self.proc.send_signal.assert_not_called()
        self.proc.wait.assert_not_called()
        self.assert_owned_only()


class RunManagerFailureTests(unittest.TestCase):
    def test_startup_error_is_not_masked_by_missing_results(self):
        scenario=types.SimpleNamespace(config_path=Path("unused.yaml"),config={},source="user",module="unused")
        with tempfile.TemporaryDirectory() as temporary, ExitStack() as stack:
            stack.enter_context(patch.object(run_manager,"get_scenario",return_value=scenario))
            stack.enter_context(patch.object(run_manager,"resolve_model",return_value=object()))
            stack.enter_context(patch.object(run_manager,"model_env",return_value={}))
            stack.enter_context(patch.object(run_manager,"runtime_status",return_value={"running":False}))
            stack.enter_context(patch.object(run_manager,"install_model"))
            stack.enter_context(patch.object(run_manager,"results_dir",return_value=Path(temporary)))
            stack.enter_context(patch.object(run_manager,"runtime_start",side_effect=RuntimeError("PX4 bridge discovery timed out")))
            stop=stack.enter_context(patch.object(run_manager,"runtime_stop"))
            with self.assertRaisesRegex(RuntimeError,"PX4 bridge discovery timed out") as caught:
                run_manager.run("test_case",headless=True)
            self.assertNotEqual(str(caught.exception),"Scenario did not produce a result directory.")
            # process.start now owns failed-launch cleanup; run_manager must not
            # guess which runtime to stop after start() raised.
            stop.assert_not_called()


if __name__=="__main__":
    unittest.main()
