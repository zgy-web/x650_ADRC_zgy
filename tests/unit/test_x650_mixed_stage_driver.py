"""Mocked mixed-stage lifecycle: no Gazebo, PX4, native flight or certification."""
from contextlib import ExitStack
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('mixed_driver_under_test', ROOT/'scripts/run_x650_mixed_mpc_stage.py')
driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(driver)
import run_x650_precision_candidates as completion


class MixedStageDriverTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        (self.root/'runs').mkdir()
        self.output = self.root/'job'
        self.cfg = {'scenario': 'x650_wholebody_mpc_v1', 'stage': 'tip_hold', 'controller': 'mpc',
                    'mixed_mpc_protocol': {'synthetic_test_protocol': True}}
        self.task = self.root/'task.json'; self.task.write_text(json.dumps(self.cfg))
        self.python = self.root/'existing_python'; self.python.write_text('never execute this fixture')
        self.original = self.root/'runs/x650_wholebody_mpc_v1_original'
        self.original.mkdir(); (self.original/'raw.csv').write_text('original evidence\n')
        self.original_bytes = (self.original/'raw.csv').read_bytes()
        self.run = self.root/'runs/x650_wholebody_mpc_v1_new'
        self.launch = self.root/'runs/x650_wb_launch_new'
        self.binding = {'kind': 'synthetic accepted response binding', 'model_sha256': 'a'*64}
        self.load = Mock(side_effect=lambda cfg: {'binding': deepcopy(self.binding)})
        self.preflight = Mock(side_effect=lambda cfg: (None, {'accepted_response_binding': deepcopy(self.binding)}))
        self.calls = []
        stack = ExitStack(); self.addCleanup(stack.close)
        stack.enter_context(patch.object(driver, 'ROOT', self.root))
        stack.enter_context(patch.object(driver, 'validate_task_config', side_effect=lambda cfg: deepcopy(cfg)))
        stack.enter_context(patch.object(driver, 'preflight', self.preflight))
        stack.enter_context(patch.dict(sys.modules, {
            'research_condition': SimpleNamespace(is_mixed_mpc_research=lambda cfg: True),
            'mixed_mpc_protocol': SimpleNamespace(validate_mixed_mpc_protocol=lambda cfg: deepcopy(cfg)),
            'accepted_response_model': SimpleNamespace(load_accepted_response_model=self.load)}))

    def invoke(self, child=None, **kwargs):
        with patch.object(driver.subprocess, 'run', side_effect=child or self.child) as process:
            result = driver.run_one(task_path=self.task, output=self.output, launch_python=self.python,
                                    analysis_python=self.python, **kwargs)
        return result, process

    def child(self, args, **kwargs):
        self.calls.append(args)
        if len(self.calls) == 1:
            self.run.mkdir(); self.launch.mkdir()
            (self.run/'native_trace.csv').write_text('new raw evidence must remain\n')
            (self.launch/'launch.json').write_text(json.dumps({'complete': True, 'runtime_closed': True,
                'task': self.cfg, 'run_directories': [str(self.run)]}))
            metadata = {'task': self.cfg, 'complete': True, 'landed_and_disarmed': True,
                        'diagnostic_parameters_restored': True, 'diagnostic_restore_pending': False}
            (self.run/'controller_metadata.json').write_text(json.dumps(metadata))
            (self.run/'controller_task.json').write_text(json.dumps(self.cfg))
            parameters = {key: {'param_type': 9, 'raw_value_hex': struct.pack('<f', 1.).hex(), 'value': 1.}
                          for key in completion.load_parameter_evidence().CORE_CONTROL_PARAMETERS}
            for phase in ('before', 'after'):
                (self.run/f'px4_parameters_{phase}.json').write_text(json.dumps({'parameters': parameters}))
        elif len(self.calls) == 2:
            self.assertIn('audit_x650_mixed_stage.py', args[1])
            self.assertNotIn('PYTHONPATH', kwargs['env'])
            audit = self.output/'audit'; audit.mkdir()
            report = {'kind': 'x650_precision_mixed_stage_audit', 'schema_version': 3,
                'complete': True, 'passed': True, 'research_acceptance_verified': True,
                'full_window_ulog_verified': True, 'parameter_restore_verified': True,
                'upper_command_verified': True, 'feedback_contract_verified': True,
                'source_run': self.run.name, 'source_run_directory': str(self.run),
                'stage': self.cfg['stage'], 'controller': self.cfg['controller'],
                'accepted_response_binding': self.binding}
            (audit/'mixed_mpc_stage_audit.json').write_text(json.dumps(report))
        else:
            self.fail('Unexpected retry or automatic next flight')
        return SimpleNamespace(returncode=0)

    def assert_preserved(self):
        self.assertEqual((self.original/'raw.csv').read_bytes(), self.original_bytes)
        if self.run.exists(): self.assertTrue((self.run/'native_trace.csv').is_file())

    def test_pid_and_mpc_each_execute_exactly_one_flight_and_audit(self):
        for controller in ('pid_ff', 'mpc'):
            with self.subTest(controller=controller):
                self.cfg['controller'] = controller; self.task.write_text(json.dumps(self.cfg))
                self.output = self.root/(controller+'_job')
                self.run = self.root/('runs/x650_wholebody_mpc_v1_'+controller)
                self.launch = self.root/('runs/x650_wb_launch_'+controller)
                self.calls = []
                result, process = self.invoke()
                self.assertTrue(result['complete'])
                self.assertEqual(process.call_count, 2)
                self.assertEqual(result['accepted_response_binding'], self.binding)
                self.assertTrue(result['idle_after']['restore_journal_absent'])
                self.assert_preserved()

    def test_failed_flight_never_runs_audit_or_retry(self):
        def failed(args, **kwargs):
            self.child(args, **kwargs)
            return SimpleNamespace(returncode=2)
        with self.assertRaisesRegex(RuntimeError, 'Flight failed'): self.invoke(failed)
        self.assertEqual(len(self.calls), 1)
        status = driver.read_json(self.output/'job_status.json')
        self.assertFalse(status['complete'])
        self.assertEqual(status['run_directories'], [str(self.run)])
        self.assertFalse((self.output/'audit').exists())
        self.assert_preserved()

    def test_real_completion_guard_rejects_landing_restore_and_raw_parameter_changes(self):
        for fault in ('landing', 'restore', 'journal', 'parameter'):
            with self.subTest(fault=fault):
                self.output = self.root/(fault+'_job')
                self.run = self.root/('runs/x650_wholebody_mpc_v1_'+fault)
                self.launch = self.root/('runs/x650_wb_launch_'+fault)
                self.calls = []
                def failed(args, **kwargs):
                    result = self.child(args, **kwargs)
                    if fault == 'parameter':
                        path = self.run/'px4_parameters_after.json'; document = driver.read_json(path)
                        first = next(iter(document['parameters']))
                        document['parameters'][first].update(value=2., raw_value_hex=struct.pack('<f', 2.).hex())
                    else:
                        path = self.run/'controller_metadata.json'; document = driver.read_json(path)
                        document[{'landing': 'landed_and_disarmed', 'restore': 'diagnostic_parameters_restored',
                                  'journal': 'diagnostic_restore_pending'}[fault]] = fault == 'journal'
                    path.write_text(json.dumps(document))
                    return result
                with self.assertRaises(RuntimeError): self.invoke(failed)
                self.assertEqual(len(self.calls), 1)
                self.assertFalse((self.output/'audit').exists())
                self.assertFalse(driver.read_json(self.output/'job_status.json')['complete'])
                self.assert_preserved()

    def test_preflight_binding_change_stops_before_output_or_children(self):
        self.preflight.side_effect = None
        self.preflight.return_value = (None, {'accepted_response_binding': {'changed': True}})
        with self.assertRaisesRegex(RuntimeError, 'before launch'): self.invoke()
        self.assertEqual(self.calls, [])
        self.assertFalse(self.output.exists())
        self.assert_preserved()

    def test_binding_changes_after_flight_or_audit_stop_without_retry(self):
        for moment in ('flight', 'audit'):
            with self.subTest(moment=moment):
                self.output = self.root/(moment+'_job')
                self.run = self.root/('runs/x650_wholebody_mpc_v1_'+moment)
                self.launch = self.root/('runs/x650_wb_launch_'+moment)
                self.calls = []
                original = {'binding': deepcopy(self.binding)}
                changed = {'binding': {'changed': True}}
                self.load.side_effect = [original, changed] if moment == 'flight' else [original, original, changed]
                with self.assertRaisesRegex(RuntimeError, 'binding changed'): self.invoke()
                self.assertEqual(len(self.calls), 1 if moment == 'flight' else 2)
                self.assertFalse(driver.read_json(self.output/'job_status.json')['complete'])
                self.assert_preserved()

    def test_audit_exit_failure_or_inconsistent_report_never_completes(self):
        faults = ('exit', 'passed', 'complete', 'parameter_restore_verified', 'controller', 'source_run_directory')
        for fault in faults:
            with self.subTest(fault=fault):
                self.output = self.root/(fault+'_job')
                self.run = self.root/('runs/x650_wholebody_mpc_v1_'+fault)
                self.launch = self.root/('runs/x650_wb_launch_'+fault)
                self.calls = []
                def failed(args, **kwargs):
                    result = self.child(args, **kwargs)
                    if len(self.calls) == 2:
                        if fault == 'exit': return SimpleNamespace(returncode=2)
                        path = self.output/'audit/mixed_mpc_stage_audit.json'; report = driver.read_json(path)
                        report[fault] = 'wrong' if fault in ('controller', 'source_run_directory') else False
                        path.write_text(json.dumps(report))
                    return result
                with self.assertRaisesRegex(RuntimeError, 'audit'): self.invoke(failed)
                self.assertEqual(len(self.calls), 2)
                self.assertFalse(driver.read_json(self.output/'job_status.json')['complete'])
                self.assert_preserved()

    def test_pending_journal_and_existing_job_are_preserved_without_launch(self):
        (self.root/'.amsim').mkdir()
        (self.root/'.amsim/x650_diagnostic_restore_pending.json').write_text('{}')
        with self.assertRaisesRegex(RuntimeError, 'pending restoration'): self.invoke()
        self.assertFalse(self.output.exists()); self.assertEqual(self.calls, [])
        self.output.mkdir(); marker = self.output/'job_status.json'; marker.write_text('old failed job')
        with self.assertRaisesRegex(ValueError, 'new job directory'): self.invoke()
        self.assertEqual(marker.read_text(), 'old failed job')
        self.assert_preserved()

    def test_ambiguous_flight_identity_stops_before_audit(self):
        def ambiguous(args, **kwargs):
            result = self.child(args, **kwargs)
            (self.root/'runs/x650_wholebody_mpc_v1_unexpected').mkdir()
            return result
        with self.assertRaisesRegex(RuntimeError, 'ambiguous'): self.invoke(ambiguous)
        self.assertEqual(len(self.calls), 1)
        self.assertFalse((self.output/'audit').exists())
        self.assert_preserved()

    def test_runtime_left_after_child_requires_inspection_before_audit(self):
        def runtime_left(args, **kwargs):
            result = self.child(args, **kwargs)
            (self.root/'.amsim').mkdir()
            (self.root/'.amsim/runtime.json').write_text('{}')
            return result
        with self.assertRaisesRegex(RuntimeError, 'requires inspection'): self.invoke(runtime_left)
        self.assertEqual(len(self.calls), 1)
        self.assertFalse((self.output/'audit').exists())
        self.assertFalse(driver.read_json(self.output/'job_status.json')['complete'])
        self.assertTrue((self.root/'.amsim/runtime.json').exists())
        self.assert_preserved()


if __name__ == '__main__':
    unittest.main()
