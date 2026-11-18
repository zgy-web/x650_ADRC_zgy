"""Real subprocess cleanup checks scoped only to groups created by this test."""
from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import time
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "comparison_processes", ROOT/"simulations/x650_control_comparison_v1/processes.py")
PROCESSES = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROCESSES)


# The grandchild ignores SIGINT to exercise cleanup of the whole owned group
# even when its launcher exits first. Its SIGTERM disposition stays default.
CHILD = """
import signal, time
signal.signal(signal.SIGINT, signal.SIG_IGN)
print('ready', flush=True)
time.sleep(60)
"""
PARENT = """
import json, os, signal, subprocess, sys, time
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
child = subprocess.Popen([sys.executable, '-c', sys.argv[1]],
                         stdout=subprocess.PIPE, text=True)
assert child.stdout.readline().strip() == 'ready'
print(json.dumps({'child_pid': child.pid, 'child_pgid': os.getpgid(child.pid),
                  'parent_pgid': os.getpgrp()}), flush=True)
if sys.argv[2] == 'exit':
    sys.exit(0)
time.sleep(60)
"""


def stopped(pid):
    try:
        # A killed orphan can remain a zombie until its adopter reaps it.
        stat = Path(f"/proc/{pid}/stat").read_text()
    except FileNotFoundError:
        return True
    return stat[stat.rfind(')')+2:].split()[0] == 'Z'


def wait_stopped(pid, timeout=3.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if stopped(pid):
            return True
        time.sleep(.01)
    return stopped(pid)


@unittest.skipUnless(sys.platform.startswith('linux'), 'Requires Linux /proc and process groups')
class ProcessGroupCleanupTests(unittest.TestCase):
    def check_group(self, parent_exits):
        process = subprocess.Popen(
            [sys.executable, '-c', PARENT, CHILD, 'exit' if parent_exits else 'hold'],
            start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True,
        )
        child_pid = None
        try:
            readable, _, _ = select.select([process.stdout], [], [], 5.)
            self.assertTrue(readable, 'Owned helper did not report its grandchild')
            metadata = json.loads(process.stdout.readline())
            child_pid = metadata['child_pid']
            self.assertEqual(metadata['child_pgid'], process.pid)
            self.assertEqual(metadata['parent_pgid'], process.pid)
            self.assertFalse(stopped(child_pid))
            if parent_exits:
                self.assertEqual(process.wait(timeout=3.), 0)
                self.assertFalse(stopped(child_pid), 'Grandchild must outlive launcher for this regression')
            else:
                self.assertIsNone(process.poll())

            PROCESSES.stop_process_group(process)

            self.assertIsNotNone(process.poll())
            self.assertTrue(wait_stopped(child_pid), 'Owned grandchild survived group cleanup')
        finally:
            # This group was allocated by this exact Popen with a new session.
            # Never match process names or signal any other existing group.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=3.)
            if child_pid is not None:
                wait_stopped(child_pid)
            process.stdout.close()

    def test_partition_and_trial_identity_survive_detached_launcher(self):
        partition = f"comparison_audit_test_{os.getpid()}"
        def spawn(part, tagged=False, name="sleep"):
            env = dict(os.environ, GZ_PARTITION=part)
            env.pop(PROCESSES.TRIAL_ENV, None)
            if tagged:
                env[PROCESSES.TRIAL_ENV] = "test-token"
            worker = subprocess.Popen([name, "15"], executable="/bin/sleep", env=env,
                                      start_new_session=True)
            self.addCleanup(worker.wait)
            self.addCleanup(worker.terminate)
            return worker
        tagged = spawn(partition, True)
        legacy = spawn(partition, name="parameter_bridge")
        gz = spawn(partition, name="gz sim --verbose=1 -r -s /tmp/worlds/x650_portable_v1.sdf")
        foreign = spawn(partition + "_foreign", True)
        untagged = spawn(partition)
        snapshot = PROCESSES.process_snapshot(partition)
        self.assertFalse(snapshot["clean"])
        matched = {entry["pid"]: entry for entry in snapshot["processes"]}
        self.assertEqual(set(matched), {tagged.pid, legacy.pid, gz.pid})
        self.assertEqual(matched[gz.pid]["role"], "gazebo")
        self.assertEqual(matched[gz.pid]["executable"], "gz")
        self.assertEqual(matched[tagged.pid]["pgid"], tagged.pid)
        self.assertEqual(matched[legacy.pid]["role"], "parameter_bridge")
        gz.terminate()
        gz.wait()
        tagged.terminate()
        legacy.terminate()
        tagged.wait()
        legacy.wait()
        self.assertTrue(PROCESSES.wait_for_exit_snapshot(partition)["clean"])
        self.assertIsNone(foreign.poll())
        self.assertIsNone(untagged.poll())

    def test_dirty_first_trial_stops_batch_before_launching_the_second(self):
        partition = f"comparison_gate_test_{os.getpid()}"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            feature = root/"simulations"/"x650_control_comparison_v1"
            feature.mkdir(parents=True)
            (root/".amsim").mkdir()
            source = ROOT/"simulations"/feature.name
            for name in ("cli.py", "analysis.py", "processes.py", "protocol.py", "config.json", "response_model.json"):
                shutil.copy2(source/name, feature/name)
            stub = root/"amsim"
            stub.mkdir()
            (stub/"__init__.py").touch()
            (stub/"paths.py").write_text("import os\nfrom pathlib import Path\ndef results_dir(): return Path(os.environ['AMSIM_RESULTS_DIR'])\n")
            (stub/"run_manager.py").write_text(f"from pathlib import Path\ndef run(*a, **kw): Path({str(root/'launched')!r}).touch()\n")
            batch = root/"batch"
            env = dict(os.environ, GZ_PARTITION=partition, PYTHONPATH=str(root))
            helper = subprocess.Popen(["parameter_bridge", "30"], executable="/bin/sleep", env=env)
            try:
                result = subprocess.run([sys.executable, feature/"cli.py", "batch", "--development",
                    "--algorithms", "pid", "mpc", "--cases", "hover", "--repeats", "1", "--output", batch],
                    env=env, capture_output=True, text=True, timeout=25)
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertFalse((root/"launched").exists())
                manifest = json.loads((batch/"manifest.json").read_text())
                self.assertEqual(len(manifest["runs"]), 1)
                item = manifest["runs"][0]
                audit = json.loads(Path(item["process_audit_file"]).read_text())
                self.assertFalse(audit["launch_attempted"])
                self.assertFalse(item["process_audit_accepted"])
                self.assertIn(helper.pid, [proc["pid"] for proc in audit["before"]["processes"]])
                self.assertIsNone(helper.poll())
            finally:
                helper.terminate()
                helper.wait(timeout=2)

    def test_running_launcher_and_grandchild_are_both_stopped(self):
        self.check_group(parent_exits=False)

    def test_grandchild_is_stopped_after_launcher_already_exited(self):
        self.check_group(parent_exits=True)


if __name__ == '__main__':
    unittest.main()
