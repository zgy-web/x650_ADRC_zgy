"""No-flight normal reference integration and unchanged hold defaults."""
from copy import deepcopy
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'src')]
from simulations.x650_cpp_live_v1 import run_case as live, launch as launcher
from simulations.x650_cpp_normal_v1 import task, run_case as normal


class NormalAdapterTests(unittest.TestCase):
    def setUp(self):
        self.hold = dict(stage='tip_hold', controller='mpc', duration_s=30., scenario='hold',
            model='model', disturbance_enabled=False, mpc_config={'nodes': 15, 'weight': 400.},
            ts20_inputs={'unchanged': True}, stage_evidence={'joint_probe': '/original/probe'})
        self.binding = {'path': '/original/task.json', 'sha256': 'a'*64}
        self.certificate = {'path': '/original/hold_certificate.json', 'sha256': 'b'*64}

    def config(self):
        with patch.object(task, 'baseline_config', return_value=deepcopy(self.hold)), \
                patch.object(task, 'bound_document', return_value={}):
            return task.build_task_config(self.binding, self.certificate)

    def test_task_only_changes_declared_normal_fields_and_rejects_retuning(self):
        cfg = self.config()
        self.assertEqual(cfg['trajectory_parameters'], dict(amplitude_x=.25, amplitude_z=.1,
            period_s=12., cycles=2, start_s=2., ramp_s=2.))
        self.assertEqual(cfg['duration_s'], 32.)
        for key in ('mpc_config', 'ts20_inputs', 'stage_evidence', 'disturbance_enabled'):
            self.assertEqual(cfg[key], self.hold[key])
        with patch.object(task, 'baseline_config', return_value=deepcopy(self.hold)), \
                patch.object(task, 'bound_document', return_value={}):
            self.assertEqual(task.validate_task_config(cfg), cfg)
            for mutate in (lambda x: x['mpc_config'].update(weight=401.),
                           lambda x: x['trajectory_parameters'].update(period_s=8.),
                           lambda x: x.update(stage='fast'), lambda x: x.update(duration_s=28.)):
                changed = deepcopy(cfg); mutate(changed)
                with self.assertRaises(ValueError): task.validate_task_config(changed)

    def test_real_file_identity_and_duplicate_keys_are_not_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'original.json'; path.write_text('{"value": 1}')
            binding = {'path': str(path), 'sha256': task.sha(path)}
            self.assertEqual(task.bound_document(binding), {'value': 1})
            path.write_text('{"value": 2}')
            with self.assertRaises(ValueError): task.bound_document(binding)
            path.write_text('{"value": 1, "value": 2}'); binding['sha256'] = task.sha(path)
            with self.assertRaisesRegex(ValueError, 'Duplicate'): task.bound_document(binding)

    def test_historical_certificate_defers_archived_source_validation_to_stage_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'certificate.json'
            original = {'software_qualification': {'verified_artifacts': [
                {'path': '/historical/replaced/worker_source.cpp', 'sha256': 'f'*64}]}}
            path.write_text(json.dumps(original))
            binding = {'path': str(path), 'sha256': task.sha(path)}
            self.assertEqual(task.bound_document(binding, recurse=False), original)
            with self.assertRaises(ValueError): task.bound_document(binding)

    def test_baseline_must_be_quiet_full_cpp_hold(self):
        for changed in (dict(self.hold, stage='normal'), dict(self.hold, duration_s=29.99),
                        dict(self.hold, numerical_backend={}), dict(self.hold, disturbance_enabled=True)):
            with patch.object(task, 'bound_document', return_value=changed), \
                    patch.object(task.hold, 'validate_task_config', side_effect=deepcopy):
                with self.assertRaises(ValueError): task.baseline_config(self.binding)

    def extension(self):
        cfg = self.config()
        with patch.object(task, 'baseline_config', return_value=deepcopy(self.hold)):
            item = normal.CppNormalExtension(cfg, {})
        return item, cfg

    def test_default_hold_is_strict_and_normal_does_not_mutate_it(self):
        hold = live.CppLiveExtension({}, {})
        hold.cpp_manifest = {'trajectory': {'kind': 'hold'}}
        hold.validate_trajectory_manifest()
        self.assertEqual(hold.source_trajectory_parameters(), {'trajectory_kind': 'hold'})
        item, cfg = self.extension()
        self.assertEqual(item.cfg, self.hold)
        item.cpp_manifest = {'trajectory': task.trajectory()}
        item.validate_trajectory_manifest()
        self.assertEqual(item.source_trajectory_parameters()['trajectory_kind'], 'figure8')
        self.assertEqual(item.normal_cfg, cfg)
        item.cpp_manifest['trajectory']['period_s'] = 8.
        with self.assertRaises(ValueError): item.validate_trajectory_manifest()
        hold.cpp_manifest['trajectory'] = task.trajectory()
        with self.assertRaises(ValueError): hold.validate_trajectory_manifest()

    def test_source_launch_passes_exact_normal_parameters_without_other_changes(self):
        item, _ = self.extension()
        item.session = 5; item.epoch = 1; item.task_sha = 'c'*64
        item.log_dir = Path('/original/logs'); item.out = Path('/original/output')
        item.cpp_manifest = {'config': {'base_position_gain': [.95, .95, 1.],
                                       'bus_period': .005, 'motion': [0.,0.,0.,0.,.01]}}
        item._ros = lambda package, executable, params: (package, executable, params)
        item._launch = Mock(); meta = {'isolated_execution': {}}
        item._prepare_additional_transport(None, None, meta)
        name, (package, executable, params) = item._launch.call_args.args
        self.assertEqual((name, package, executable), ('cpp_source', 'flyarm_state_cpp', 'state_source'))
        for key, value in task.source_parameters().items(): self.assertEqual(params[key], value)
        self.assertNotIn('duration_s', params)
        self.assertEqual(params['configuration_sha256'], item.task_sha)
        self.assertEqual(params['position_gains'], [.95, .95, 1.])
        self.assertFalse(meta['isolated_execution']['python_active_snapshot_publisher'])

    def test_actual_ground_prepared_allocator_and_recovery_methods_are_inherited(self):
        item, cfg = self.extension()
        self.assertIs(normal.CppNormalExtension.build_allocator, live.CppLiveExtension.build_allocator)
        for name in ('pump_actual', '_release', 'cleanup', '_bind_actual_source'):
            self.assertIs(getattr(normal.CppNormalExtension, name), getattr(live.CppLiveExtension, name))
        model = object(); loop = object(); pid = {'p': 1.}; trajectory = object()
        item.prepared_allocator = NS(step_count=0)
        item.prepared_allocator_claimed = False; item.prepared_configuration = deepcopy(cfg)
        item.prepared_model = model; item.prepared_pid = dict(pid); item.loop = loop
        result = item.build_allocator(cfg, model, pid, trajectory, loop)
        self.assertIs(result.trajectory, trajectory)
        with self.assertRaises(RuntimeError): item.build_allocator(cfg, model, pid, trajectory, loop)

    def test_activation_does_not_hash_or_rebind_and_only_attaches_metadata(self):
        item, cfg = self.extension(); meta = {}
        item.validate_activation_stage(cfg)
        with patch.object(live.CppLiveExtension, 'activate', return_value=15.) as activate, \
                patch.object(task, 'preflight', side_effect=AssertionError('airborne evidence IO')):
            self.assertEqual(item.activate(None, None, cfg, meta, None), 15.)
            activate.assert_called_once()
        self.assertEqual(meta['cpp_hold_certificate'], self.certificate)
        with self.assertRaises(ValueError): item.validate_activation_stage(dict(cfg, duration_s=31.))

    def test_normal_diagnostic_target_follows_reference_without_controller_work(self):
        import numpy as np
        item, _ = self.extension()
        target = np.array([1., 2., 3.])
        trajectory = NS(sample=Mock(return_value=(target, None, None)))
        loop = NS(task_start=40., trajectory=trajectory, recorder=NS(tip_target=None))
        with patch.object(live.CppLiveExtension, 'handle_active_pump', return_value=True) as parent:
            self.assertTrue(item.handle_active_pump(loop, 43.25))
            parent.assert_called_once_with(loop, 43.25)
        trajectory.sample.assert_called_once_with(3.25)
        np.testing.assert_array_equal(loop.recorder.tip_target, target)
        self.assertFalse(np.shares_memory(loop.recorder.tip_target, target))
        trajectory.sample.reset_mock()
        with patch.object(live.CppLiveExtension, 'handle_active_pump', return_value=False):
            self.assertFalse(item.handle_active_pump(loop, 44.))
        trajectory.sample.assert_not_called()
        with patch.object(live.CppLiveExtension, 'handle_active_pump', side_effect=RuntimeError('fault')):
            with self.assertRaisesRegex(RuntimeError, 'fault'): item.handle_active_pump(loop, 44.)
        trajectory.sample.assert_not_called()

    def test_normal_final_metadata_distinguishes_diagnostic_from_actual_target(self):
        item, _ = self.extension()
        meta, stage = {'upper_allocator_active': False}, {'upper_allocator_active': False}
        with patch.object(live.CppLiveExtension, 'finalize') as parent:
            item.finalize(meta, stage, '/original/output')
            parent.assert_called_once_with(meta, stage, '/original/output')
        for document in (meta, stage):
            self.assertTrue(document['upper_allocator_active'])
            self.assertFalse(document['tip_target_recording']['used_by_cpp_controller'])
            self.assertIn('not exact truth-row-time', document['tip_target_recording']['flight_samples_tip_target'])
            self.assertIn('ControlSnapshot', document['tip_target_recording']['authoritative_control_target'])
        self.assertIsNot(meta['tip_target_recording'], stage['tip_target_recording'])

    def test_scenario_uses_same_physical_model_and_protocol_protective_limits(self):
        from amsim.scenario_registry import get
        old, new = get('x650_cpp_live_v1'), get(task.SCENARIO)
        self.assertEqual(new.module, 'simulations/x650_cpp_normal_v1/run_case.py')
        for key in ('backend', 'model', 'vehicle', 'record', 'visualization'):
            self.assertEqual(old.config[key], new.config[key])
        hold = json.loads((ROOT/'configs/experiments/x650_cpp_live_v1.json').read_text())
        current = json.loads((ROOT/'configs/experiments/x650_cpp_normal_v1.json').read_text())
        for key in ('cpu_roles', 'worker_thread_roles', 'output_period_s', 'command_age_max_s',
                    'control_interval_max_s', 'source_wall_age_max_s', 'output_interval_max_s',
                    'consumption_policy', 'consumption_progress_budget_ms', 'core_compute_max_s'):
            self.assertEqual(current[key], hold[key])
        self.assertEqual(current['trajectory'], task.trajectory())
        self.assertFalse(current['fast_admitted'])

    def test_normal_values_match_the_previously_declared_trajectory(self):
        from simulations.x650_isolated_figure8_v1 import task as previous
        original = previous.protocol(); selected = task.trajectory()
        for left, right in [('amplitude_x', 'amplitude_x_m'), ('amplitude_z', 'amplitude_z_m'),
                            ('period_s', 'period_s'), ('cycles', 'cycles'), ('start_s', 'start_s'),
                            ('ramp_s', 'ramp_s'), ('duration_s', 'duration_s')]:
            self.assertEqual(selected[left], original[right])

    def test_extra_normal_sources_are_archived_and_cannot_escape_root(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); path = root/'simulations/x650_cpp_normal_v1/qualification.py'
            path.parent.mkdir(parents=True); path.write_text('qualification source')
            ignored = path.parent/'__pycache__/temporary.pyc'; ignored.parent.mkdir(); ignored.write_bytes(b'cache')
            with patch.object(launcher, 'ROOT', root), patch.object(launcher.q, 'sha', return_value='digest'):
                sources = launcher.source_hashes(('simulations/x650_cpp_normal_v1',))
                self.assertIn('simulations/x650_cpp_normal_v1/qualification.py', sources)
                self.assertFalse(any('__pycache__' in name for name in sources))
                with self.assertRaises(ValueError): launcher.source_hashes((str(root.parent),))

    def test_launch_preserves_interface_check_before_qualification_and_never_runs_in_dry_run(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            for name in ('task', 'startup', 'protocol', 'qualification'):
                (folder/(name+'.json')).write_text('{}')
            order = []
            qa = NS(verify=Mock(side_effect=lambda *a, **kw: order.append('qualification') or {'ready': True}))
            ta = NS(validate_task_config=lambda _: {'model': 'model'},
                    preflight=lambda _: (None, {}))
            with patch.object(launcher, 'ROOT', folder), \
                    patch.object(launcher, 'runtime_interfaces', side_effect=lambda: order.append('interfaces') or {'checked': 15}), \
                    patch.object(launcher, 'execution_resources', return_value={}), \
                    patch.object(launcher, 'runtime_hashes', return_value={}), \
                    patch('amsim.scenario_registry.get', return_value=NS(module='simulations/x650_cpp_normal_v1/run_case.py', config={'model': 'model'})), \
                    patch.object(launcher.subprocess, 'run', side_effect=AssertionError('dry-run must not start')), \
                    patch('sys.stdout', new_callable=io.StringIO):
                result = launcher.main(['--task', str(folder/'task.json'), '--startup-manifest', str(folder/'startup.json'),
                    '--startup-sha256', 'a'*64, '--qualification-manifest', str(folder/'qualification.json'),
                    '--protocol', str(folder/'protocol.json'), '--dry-run'], task_api=ta, qualification_api=qa,
                    scenario=task.SCENARIO)
            self.assertEqual(result, 0)
            self.assertEqual(order, ['interfaces', 'qualification'])


if __name__ == '__main__': unittest.main()
