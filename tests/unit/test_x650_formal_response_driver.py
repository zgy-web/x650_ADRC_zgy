"""Supervisor failure boundaries with mocked children; never starts simulation."""
from contextlib import ExitStack
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("formal_driver_under_test", ROOT/"scripts/run_x650_formal_response.py")
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)


class FormalResponseDriverTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        (self.root/"runs").mkdir()
        self.output = self.root/"new_job"
        self.cfg = {"scenario":"x650_wholebody_mpc_v1", "stage":"response_probe",
                    "controller":"pid_ff", "response_probe_protocol":{"role":"train"}}
        self.task = self.root/"task.json"
        self.plan = self.root/"preregistration.json"
        self.task.write_text(json.dumps(self.cfg))
        self.plan.write_text('{"synthetic_test_plan":true}')
        self.python = self.root/"existing_python"
        self.python.write_text("synthetic interpreter placeholder; must never execute")
        self.original = self.root/"runs/x650_wholebody_mpc_v1_old"
        self.original.mkdir()
        (self.original/"control_trace.csv").write_text("original evidence must remain byte-identical\n")
        self.original_bytes = (self.original/"control_trace.csv").read_bytes()
        self.run = self.root/"runs/x650_wholebody_mpc_v1_new"
        self.launch = self.root/"runs/x650_wb_launch_new"
        self.verify = Mock()
        self.completed = Mock()
        self.preflight = Mock(return_value=(None,{"synthetic_preflight":True}))
        self.calls = []
        stack = ExitStack()
        self.addCleanup(stack.close)
        stack.enter_context(patch.object(driver,"ROOT",self.root))
        stack.enter_context(patch.object(driver,"validate_task_config",side_effect=lambda c:deepcopy(c)))
        stack.enter_context(patch.object(driver,"preflight",self.preflight))
        stack.enter_context(patch.object(driver,"require_completed_run",self.completed))
        stack.enter_context(patch.dict(sys.modules,{"identify_x650_response_protocol":SimpleNamespace(verify_plan=self.verify)}))

    def invoke(self, child, **kwargs):
        with patch.object(driver.subprocess,"run",side_effect=child) as process:
            result = driver.run_one(task_path=self.task,preregistration=self.plan,output=self.output,
                launch_python=self.python,analysis_python=self.python,**kwargs)
        return result, process

    def child(self, args, **kwargs):
        self.calls.append(args)
        if len(self.calls) == 1:
            self.run.mkdir()
            self.launch.mkdir()
            (self.run/"native_trace.csv").write_text("new independent raw evidence\n")
            (self.launch/"launch.json").write_text(json.dumps({"complete":True,"runtime_closed":True,
                "task":self.cfg,"run_directories":[str(self.run)]}))
        elif len(self.calls) == 2:
            self.assertIn("audit_x650_response_protocol.py",args[1])
            self.assertNotIn("PYTHONPATH",kwargs["env"])
            audit = self.output/"audit"
            audit.mkdir()
            (audit/"upper_stage_audit.json").write_text(json.dumps({"passed":True,
                "research_acceptance_verified":True,"source_run":self.run.name}))
        else:
            self.fail("Driver attempted a retry, fit, or second flight")
        return SimpleNamespace(returncode=0)

    def assert_original_preserved(self):
        self.assertEqual((self.original/"control_trace.csv").read_bytes(),self.original_bytes)

    def test_train_and_holdout_each_run_one_flight_then_one_audit_and_stop(self):
        for role in ("train","holdout"):
            with self.subTest(role=role):
                # A role is chosen by a separate invocation with separate paths.
                self.output = self.root/(role+"_job")
                self.run = self.root/("runs/x650_wholebody_mpc_v1_"+role)
                self.launch = self.root/("runs/x650_wb_launch_"+role)
                self.cfg["response_probe_protocol"]["role"] = role
                self.task.write_text(json.dumps(self.cfg))
                self.calls = []
                result, process = self.invoke(self.child)
                self.assertTrue(result["complete"])
                self.assertEqual(result["role"],role)
                self.assertEqual(process.call_count,2)
                self.assertEqual(driver.read_json(self.output/"task.json"),self.cfg)
                self.assertEqual(result["run_directories"],[str(self.run)])
                self.assert_original_preserved()

    def test_failed_flight_is_recorded_without_audit_retry_or_overwrite(self):
        def failed(args,**kwargs):
            self.child(args,**kwargs)
            return SimpleNamespace(returncode=2)
        with self.assertRaisesRegex(RuntimeError,"Flight failed"):
            self.invoke(failed)
        self.assertEqual(len(self.calls),1)
        status = driver.read_json(self.output/"job_status.json")
        self.assertFalse(status["complete"])
        self.assertEqual(status["phase"],"failed")
        self.assertEqual(status["run_directories"],[str(self.run)])
        self.assertFalse((self.output/"audit").exists())
        self.assert_original_preserved()

    def test_ambiguous_run_identity_stops_without_audit_or_second_flight(self):
        def ambiguous(args,**kwargs):
            result = self.child(args,**kwargs)
            (self.root/"runs/x650_wholebody_mpc_v1_unexpected").mkdir()
            return result
        with self.assertRaisesRegex(RuntimeError,"ambiguous"):
            self.invoke(ambiguous)
        self.assertEqual(len(self.calls),1)
        self.completed.assert_not_called()
        self.assertEqual(len(driver.read_json(self.output/"job_status.json")["run_directories"]),2)

    def test_failed_audit_exit_or_rejected_report_never_causes_another_flight(self):
        for failure in ("exit","report","wrong_run"):
            with self.subTest(failure=failure):
                self.output = self.root/(failure+"_job")
                self.run = self.root/("runs/x650_wholebody_mpc_v1_"+failure)
                self.launch = self.root/("runs/x650_wb_launch_"+failure)
                self.calls = []
                def rejected(args,**kwargs):
                    result = self.child(args,**kwargs)
                    if len(self.calls) == 2:
                        if failure == "exit":
                            return SimpleNamespace(returncode=2)
                        (self.output/"audit/upper_stage_audit.json").write_text(json.dumps({
                            "passed":failure == "wrong_run", "research_acceptance_verified":failure == "wrong_run",
                            "source_run":"another_run" if failure == "wrong_run" else self.run.name}))
                    return result
                with self.assertRaisesRegex(RuntimeError,"audit"):
                    self.invoke(rejected)
                self.assertEqual(len(self.calls),2)
                status = driver.read_json(self.output/"job_status.json")
                self.assertFalse(status["complete"])
                self.assertEqual(status["phase"],"failed")
                self.assertTrue((self.run/"native_trace.csv").exists())
                self.assert_original_preserved()

    def test_uncleared_restore_journal_stops_before_children_and_keeps_existing_evidence(self):
        (self.root/".amsim").mkdir()
        (self.root/".amsim/x650_diagnostic_restore_pending.json").write_text("{}")
        with self.assertRaisesRegex(RuntimeError,"pending restoration"):
            self.invoke(self.child)
        self.assertEqual(self.calls,[])
        self.preflight.assert_not_called()
        self.assertFalse(self.output.exists())
        self.assert_original_preserved()

    def test_existing_job_directory_is_never_reused(self):
        self.output.mkdir()
        marker = self.output/"job_status.json"
        marker.write_text("existing failed attempt\n")
        with self.assertRaisesRegex(ValueError,"new job directory"):
            self.invoke(self.child)
        self.assertEqual(marker.read_text(),"existing failed attempt\n")
        self.assertEqual(self.calls,[])
        self.verify.assert_not_called()

    def test_runtime_left_after_child_stops_before_audit(self):
        def runtime_left(args,**kwargs):
            result = self.child(args,**kwargs)
            (self.root/".amsim").mkdir()
            (self.root/".amsim/runtime.json").write_text("{}")
            return result
        with self.assertRaisesRegex(RuntimeError,"requires inspection"):
            self.invoke(runtime_left)
        self.assertEqual(len(self.calls),1)
        status = driver.read_json(self.output/"job_status.json")
        self.assertFalse(status["complete"])
        self.assertEqual(status["phase"],"failed")
        self.assertTrue((self.root/".amsim/runtime.json").exists())
        self.completed.assert_not_called()

    def test_missing_requested_pyulog_site_is_rejected_before_any_flight(self):
        with self.assertRaisesRegex(ValueError,"PyULog"):
            self.invoke(self.child,pyulog_site=self.root/"missing_package")
        self.assertEqual(self.calls,[])
        self.assertFalse(self.output.exists())
        self.preflight.assert_not_called()


if __name__ == "__main__":
    unittest.main()
