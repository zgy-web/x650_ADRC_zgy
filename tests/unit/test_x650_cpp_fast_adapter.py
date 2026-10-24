"""Fast stage composition and inherited actual C++ execution; no ROS or flight."""
from copy import deepcopy
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'src')]
from simulations.x650_cpp_fast_v1 import task, run_case as fast
from simulations.x650_cpp_normal_v1 import run_case as normal, task as normal_task
from simulations.x650_cpp_live_v1 import run_case as live


class FastAdapterTests(unittest.TestCase):
    def setUp(self):
        self.hold = dict(stage='tip_hold', controller='mpc', duration_s=30., model='model',
            mpc_config={'nodes': 15, 'weight': 400.}, disturbance_enabled=False, ts20_inputs={'plant': 'same'})
        self.normal = dict(self.hold, stage='normal', trajectory='normal', scenario='normal', duration_s=32.,
            frozen_cpp_hold={'path': '/original/hold_task', 'sha256': 'a'*64},
            cpp_hold_certificate={'path': '/original/hold_certificate', 'sha256': 'b'*64})
        self.binding = {'path': '/original/normal_task.json', 'sha256': 'c'*64}
        self.cert = {'path': '/original/normal_certificate.json', 'sha256': 'd'*64}

    def config(self):
        with patch.object(task, 'normal_config', return_value=deepcopy(self.normal)), \
                patch.object(task, 'bound_document', return_value={}):
            return task.build_task_config(self.binding, self.cert)

    def extension(self):
        cfg = self.config()
        with patch.object(normal_task, 'baseline_config', return_value=deepcopy(self.hold)):
            return fast.CppFastExtension(cfg, {}), cfg

    def test_composition_changes_only_declared_trajectory_fields(self):
        cfg = self.config()
        changes = {'scenario', 'stage', 'trajectory', 'duration_s', 'trajectory_parameters',
                   'frozen_cpp_normal', 'cpp_normal_certificate', 'figure8_protocol'}
        self.assertEqual({k: v for k, v in cfg.items() if k not in changes},
                         {k: v for k, v in self.normal.items() if k not in changes})
        self.assertEqual(cfg['trajectory_parameters'], dict(amplitude_x=.25, amplitude_z=.1,
            period_s=8., cycles=2, start_s=2., ramp_s=2.))
        self.assertEqual(cfg['duration_s'], 24.)
        with patch.object(task, 'normal_config', return_value=deepcopy(self.normal)), \
                patch.object(task, 'bound_document', return_value={}):
            self.assertEqual(task.validate_task_config(cfg), cfg)
            for key, value in [('stage', 'normal'), ('duration_s', 32.), ('controller', 'pid_ff')]:
                changed = dict(cfg, **{key: value})
                with self.assertRaises(ValueError): task.validate_task_config(changed)
            changed = deepcopy(cfg); changed['mpc_config']['weight'] += 1
            with self.assertRaises(ValueError): task.validate_task_config(changed)

    def test_exact_source_and_worker_trajectory_and_normal_stays_strict(self):
        item, cfg = self.extension()
        self.assertEqual(item.cfg, self.hold)
        item.cpp_manifest = {'trajectory': task.trajectory()}
        item.validate_trajectory_manifest()
        self.assertEqual(item.source_trajectory_parameters()['period_s'], 8.)
        item.validate_activation_stage(cfg)
        with self.assertRaises(ValueError): item.validate_activation_stage(dict(cfg, stage='normal'))
        item.cpp_manifest['trajectory']['period_s'] = 12.
        with self.assertRaises(ValueError): item.validate_trajectory_manifest()
        old = normal.CppNormalExtension.__new__(normal.CppNormalExtension)
        old.cpp_manifest = {'trajectory': task.trajectory()}
        with self.assertRaises(ValueError): old.validate_trajectory_manifest()

    def test_async_transport_lifecycle_and_live_feedback_are_inherited(self):
        for name in ('build_allocator', 'pump_actual', '_release', 'cleanup', '_bind_actual_source',
                     'handle_active_pump', '_prepare_additional_transport'):
            self.assertIs(getattr(fast.CppFastExtension, name), getattr(normal.CppNormalExtension, name))
        item, cfg = self.extension(); meta = {}
        with patch.object(normal.CppNormalExtension, 'activate', return_value=10.) as parent, \
                patch.object(task, 'preflight', side_effect=AssertionError('airborne IO')):
            self.assertEqual(item.activate(None, None, cfg, meta, None), 10.)
            parent.assert_called_once()
        self.assertEqual(meta['cpp_normal_certificate'], self.cert)

    def test_normal_then_fast_gates_both_required(self):
        cfg = self.config()
        with patch.object(task, 'normal_config', return_value=self.normal), \
                patch.object(normal_task, 'validate_stage_gates', return_value={'cpp_complete_tip_hold': True}) as previous, \
                patch('simulations.x650_cpp_fast_v1.qualification.verify_normal_gate', return_value={'verified': True}) as gate:
            actual = task.validate_stage_gates(cfg, {'model_sha256': 'same'})
            previous.assert_called_once_with(self.normal, {'model_sha256': 'same'})
            gate.assert_called_once_with(self.cert, self.binding)
            self.assertTrue(actual['cpp_complete_normal_figure8']['verified'])

    def test_scenario_model_and_all_protective_limits_unchanged(self):
        from amsim.scenario_registry import get
        old, new = get('x650_cpp_normal_v1'), get(task.SCENARIO)
        for key in ('backend', 'model', 'vehicle', 'record', 'visualization'):
            self.assertEqual(old.config[key], new.config[key])
        prior = json.loads((ROOT/'configs/experiments/x650_cpp_normal_v1.json').read_text())
        current = json.loads((ROOT/'configs/experiments/x650_cpp_fast_v1.json').read_text())
        from simulations.x650_cpp_fast_v1.qualification import validate_protocol
        validate_protocol(current, prior)
        for key in ('output_period_s', 'command_age_max_s', 'consumption_progress_budget_ms', 'source_wall_age_max_s'):
            bad = deepcopy(current); bad[key] *= 1.1
            with self.assertRaises(ValueError): validate_protocol(bad, prior)

    def test_previously_declared_fast_amplitudes_period_and_duration(self):
        from simulations.x650_wholebody_mpc_v1.task import build_task_config
        previous = build_task_config('fast', 'mpc')
        self.assertEqual(task.trajectory()['duration_s'], previous['duration_s'])
        self.assertEqual({k: v for k, v in task.trajectory().items() if k not in {'kind', 'duration_s'}},
                         previous['trajectory_parameters'])


if __name__ == '__main__': unittest.main()
