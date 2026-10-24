"""No-flight contracts for C++ lifecycle integration; no ROS runtime required."""
import ast
import base64
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS, ModuleType
import sys
import unittest
from unittest.mock import Mock, patch
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def load():
    namespace = dict(np=np, Path=Path, time=time, deepcopy=deepcopy, json=json, base64=base64,
                     Ts20Extension=object, write=Mock())
    old = ROOT/'simulations/x650_isolated_live_v1/run_case.py'
    definition = next(n for n in ast.parse(old.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'LiveExtension')
    exec(compile(ast.Module(body=[definition], type_ignores=[]), str(old), 'exec'), namespace)
    path = ROOT/'simulations/x650_cpp_live_v1/run_case.py'
    definitions = [n for n in ast.parse(path.read_text()).body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name != 'scenario']
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


class CppLiveAdapterTests(unittest.TestCase):
    def setUp(self):
        self.ns = load()
        self.cls = self.ns['CppLiveExtension']

    def active(self):
        item = self.cls()
        item.active = True; item.failure = None; item.latest = {}
        item._activity_mark = Mock(); item._alive = Mock(); item._check_supervision = Mock(); item._release = Mock()
        return item

    def test_streaming_policy_only_changes_cpp_dispatcher_arguments(self):
        item = self.cls()
        item.protocol = {'execution_revision': 2, 'consumption_policy': 'latest_value_v1',
                         'consumption_progress_budget_ms': 40}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for executable in ('live_dispatcher', 'state_ingress'):
                path = root/'lib/flyarm_control_gateway'/executable
                path.parent.mkdir(parents=True, exist_ok=True); path.touch()
            item.prefix = lambda _: directory
            original = {'simulation_live': True}
            result = item._ros('flyarm_control_gateway', 'live_dispatcher', original)
            self.assertIn('consumption_policy:=latest_value_v1', result)
            self.assertIn('consumption_progress_budget_ms:=40', result)
            self.assertEqual(original, {'simulation_live': True})
            ingress = item._ros('flyarm_control_gateway', 'state_ingress', original)
            self.assertFalse(any('consumption_' in part for part in ingress))
            with self.assertRaisesRegex(ValueError, 'Conflicting'):
                item._ros('flyarm_control_gateway', 'live_dispatcher', {'consumption_policy': 'strict_per_tick'})

    def test_streaming_selection_cannot_silently_reuse_old_contract(self):
        valid = {'execution_revision': 2, 'consumption_policy': 'latest_value_v1',
                 'consumption_progress_budget_ms': 40}
        for key, bad in [('execution_revision', 1), ('consumption_policy', 'bounded_wait_v1'),
                         ('consumption_progress_budget_ms', 60), ('consumption_progress_budget_ms', 40.)]:
            with self.assertRaises(ValueError):
                self.ns['dispatcher_parameters'](dict(valid, **{key: bad}))

    def test_active_monitor_never_reads_or_constructs_python_control_state(self):
        item = self.active()
        loop = NS(recovery=False, read=Mock(side_effect=AssertionError('Python state read forbidden')))
        item.runtime = NS(context=Mock(side_effect=AssertionError('Python prediction context forbidden')))
        item.source_pub = NS(publish=Mock(side_effect=AssertionError('Python source publication forbidden')))
        self.assertTrue(item.handle_active_pump(loop, 1.))
        loop.read.assert_not_called(); item.runtime.context.assert_not_called(); item.source_pub.publish.assert_not_called()
        item._check_supervision.assert_called_once()

    def test_fault_and_recovery_preserve_original_release_contract(self):
        item = self.active(); loop = NS(recovery=False)
        item.latest['dispatch'] = NS(status=3, reason=2)
        with self.assertRaisesRegex(RuntimeError, 'reason 2'): item.handle_active_pump(loop, 1.)
        loop.recovery = True
        self.assertFalse(item.handle_active_pump(loop, 1.))
        item._release.assert_called_once_with(loop)

    def test_placeholder_has_bound_metadata_and_explicit_no_fallback(self):
        manifest = {'core_library': {'sha256': 'a'}, 'optimization_library': {'sha256': 'b'},
                    'predictor_library': {'sha256': 'c'}, 'config': {'nodes': 15}}
        allocator = self.ns['ExternalCppAllocator'](object(), {'MPC_XY_P': 1., 'MPC_Z_P': 2.}, manifest, 8)
        allocator.reset(np.zeros(3), np.eye(3), np.zeros(2))
        self.assertEqual(allocator.step_count, 0)
        with self.assertRaisesRegex(RuntimeError, 'forbidden'): allocator.step(0., .02, None)
        returned = allocator.metadata(); returned['configuration']['nodes'] = 1
        self.assertEqual(allocator.metadata()['configuration']['nodes'], 15)
        self.assertEqual(allocator.metadata()['kind'], 'cpp_full_pipeline_v1')
        with self.assertRaises(ValueError): allocator.reset(np.zeros(2), np.eye(3), np.zeros(2))

    def binding(self):
        item = self.cls(); item.session = 7; item.epoch = 1; item.task_sha = 'aa'*32
        item.cpp_manifest = {'config': {'base_position_gain': [1., 1., 2.], 'bus_period': .005, 'motion': [0,0,0,0,.01]}}
        item.msg = NS(SourceBinding=lambda: NS())
        reference = dict(ref_lat=1., ref_lon=2., ref_alt=3., ref_timestamp=4., xy_global=True, z_global=True,
                         xy_reset_counter=1, z_reset_counter=2, heading_reset_counter=3)
        extra = dict(vxy_reset_counter=4, vz_reset_counter=5, quat_reset_counter=6)
        document = dict(px4_reference_at_bind=reference, px4_source_contract_commit='f'*40,
                        frame_mode='raw_ev_world', target_frozen=True, fit_estimator_translation=False)
        loop = NS(fixed_reference=NS(document=document), feedback_adapter=NS(document={'extra_reset_counters_at_bind': extra}))
        item.binding_ack = None
        def publish(message):
            item._on_binding_ack(NS(session_id=7, epoch=1, configuration_sha256=message.configuration_sha256,
                binding_sequence=1, binding_issued_monotonic_ns=message.issued_monotonic_ns,
                observed_monotonic_ns=message.issued_monotonic_ns, accepted=True, rejection_code=0))
        item.binding_pub = NS(publish=Mock(side_effect=publish))
        ctx = NS(vehicle=NS(wait=lambda predicate, *_: self.assertTrue(predicate())))
        return item, ctx, loop

    def test_binding_uses_original_reference_and_exact_ack(self):
        item, ctx, loop = self.binding(); before = deepcopy(loop.fixed_reference.document)
        item._bind_actual_source(ctx, loop)
        message = item.binding_message
        self.assertEqual(message.ref_alt, 3.)
        self.assertEqual(message.ref_timestamp, 4)
        self.assertIs(type(message.ref_timestamp), int)
        self.assertEqual(message.quat_reset_counter, 6)
        self.assertEqual(message.frame_mode, 'raw_ev_world')
        self.assertEqual(loop.fixed_reference.document, before)
        self.assertEqual(item.binding_pub.publish.call_count, 1)

    def test_reference_unsigned_conversion_rejects_truncation_overflow_and_bool(self):
        convert = self.ns['binding_uint']
        self.assertEqual(convert(4740000., 64), 4740000)
        for value in (False, -.1, -1, .5, float('nan'), float('inf'), 2**64):
            with self.assertRaises(ValueError): convert(value, 64)
        with self.assertRaises(ValueError): convert(256, 8)

    def test_real_module_materialization_decodes_binary_and_rejects_feedback(self):
        # Import the actual module, including its own imports, instead of the
        # narrow AST harness used to avoid ROS in the lifecycle unit tests.
        from simulations.x650_cpp_live_v1 import run_case as actual
        modules = {name: ModuleType(name) for name in ('rclpy', 'rclpy.serialization', 'rosidl_runtime_py', 'rosidl_runtime_py.convert')}
        modules['rclpy.serialization'].deserialize_message = lambda raw, typ: NS(raw=raw, type=typ)
        modules['rosidl_runtime_py.convert'].message_to_ordereddict = lambda message: {'decoded': message.raw.decode()}
        with tempfile.TemporaryDirectory() as directory, patch.dict(sys.modules, modules):
            folder = Path(directory); (folder/'source_recording').mkdir()
            raw = folder/'source_recording/diagnostics.jsonl'
            event = {'sequence': 1, 'kind': 'ros', 'topic': 'events', 'cdr': {'base64': base64.b64encode(b'actual wire bytes').decode()}}
            raw.write_text(json.dumps(event)+'\n')
            (folder/'source_sender.json').write_text(json.dumps(dict(complete=True, dropped=0, attempts=1, sent=1)))
            sink = dict(rows=1, last_sequence=1, gaps=[], error=None, drained_and_fsynced=True, records_sha256=actual.sha(raw))
            (folder/'source_recording/manifest.json').write_text(json.dumps(sink))
            extension = actual.CppLiveExtension({}, {})
            extension.out = folder; extension.message_types = {'events': object}
            extension._materialize_source_recording()
            self.assertEqual(json.loads((folder/'events.json').read_text()), [{'decoded': 'actual wire bytes'}])
            event['kind'] = 'feedback'; raw.write_text(json.dumps(event)+'\n')
            sink['records_sha256'] = actual.sha(raw); (folder/'source_recording/manifest.json').write_text(json.dumps(sink))
            with self.assertRaisesRegex(RuntimeError, 'Python feedback is forbidden'):
                extension._materialize_source_recording()

    def test_wrong_ack_hash_or_original_issue_is_rejected_without_rebind(self):
        for field, value in [('configuration_sha256', [0]*32), ('binding_issued_monotonic_ns', 1),
                             ('accepted', False), ('rejection_code', 2)]:
            item, ctx, loop = self.binding(); original = item.binding_pub.publish.side_effect
            def mutate(message):
                original(message); setattr(item.binding_ack, field, value)
            item.binding_pub.publish.side_effect = mutate
            with self.assertRaisesRegex(RuntimeError, 'ACK'): item._bind_actual_source(ctx, loop)
            self.assertEqual(item.binding_pub.publish.call_count, 1)
        item, _, _ = self.binding(); item._on_binding_ack(NS(session_id=8, epoch=1))
        self.assertIsNone(item.binding_ack)

    def test_bind_finishes_before_existing_actual_send_handoff(self):
        item = self.cls(); order = []
        item._bind_actual_source = lambda *a: order.append('binding_ack')
        with patch.object(self.ns['LiveExtension'], 'activate', side_effect=lambda *a: order.append('actual_send_live_start')):
            item.activate(None, None, {}, {}, None)
        self.assertEqual(order, ['binding_ack', 'actual_send_live_start'])

    def test_worker_strict_thread_identity_and_disjoint_roles(self):
        rows = [{'tid': 4, 'starttime_ticks': 50, 'cpus': [87]}, {'tid': 5, 'starttime_ticks': 51, 'cpus': [89,91,97]}]
        ready = dict(pid=4, main_tid=4, stages=[{'threads': deepcopy(rows)}])
        roles = dict(executor_cpu=87, auxiliary_cpus=[89,91,97])
        verify = self.ns['verify_worker_threads']
        verify(rows, ready, 4, roles)
        for mutate in (lambda r:r[0]['cpus'].append(89), lambda r:r[1].update(starttime_ticks=52), lambda r:r.pop()):
            changed=deepcopy(rows);mutate(changed)
            with self.assertRaises(RuntimeError):verify(changed, ready, 4, roles)

    def test_new_source_joins_shutdown_without_changing_old_shutdown_order(self):
        old = self.ns['LiveExtension'](); new = self.cls()
        for item, expected in [(old, ['dispatcher','consumption','ingress','worker','recorder','source_recorder']),
                               (new, ['dispatcher','consumption','cpp_source','ingress','worker','recorder','source_recorder'])]:
            order=[];item.source_sender=None;item.logs=[]
            item._stop_process=order.append;item._close_supervision=Mock();item._write_placement_evidence=Mock()
            item.cleanup();self.assertEqual(order,expected)

    def test_ground_config_comparison_rejects_task_parameter_change(self):
        c = json.loads((ROOT/'docs/validation/isolated_live_20261024/execution_v17/quiet_task_cpp_v10.json').read_text())
        model = NS(joint_lower=[-1.,-1.], joint_upper=[1.,1.], lengths=[.2,.2], angle_offsets=[0.,0.],
                   mount_body=[0.,0.,-.1], elbow_offset_body_m=[0.,0.,0.])
        p = NS(J=1.,b=2.,Fc=3.,offset=4.,delay_s=.01,Fs=5.,delta=6.,vs=7.,kp=np.array([6.,6.]),kd=np.array([.25,.25]),
               bus_period_s=.005,max_step_s=.001,trust_radius_normalized=.02)
        prior={'parameters':{'x':{'wn_rad_s':1.,'zeta':.8},'y':{'wn_rad_s':2.,'zeta':.9},'z':{'tau_s':.3}}}
        self.ns['shared']=NS(json_value=lambda value:json.loads(json.dumps(value)))
        pid={'MPC_XY_P':.95,'MPC_Z_P':1.}
        expected=self.ns['expected_core_config'](c,model,pid,p,prior)
        self.assertEqual(len(expected),24)
        self.assertEqual(expected['weights'],[400.,4.,.25,.02,.01,.01])
        item=self.cls();item.cfg=c;item.predictor=p;item.provenance={'base_response_prior':{'path':'prior','sha256':'h'}}
        item.cpp_manifest={'config':expected,'core_library':{},'optimization_library':{},'predictor_library':{}}
        item.procs={'worker':NS(pid=99)}
        self.ns['sha']=lambda _: 'h';self.ns['task']=NS(read=lambda _:prior)
        loop=NS(model=model,pid=pid)
        self.assertEqual(item._prepare_allocator(loop).metadata()['worker_pid'],99)
        item.cpp_manifest['config']['weights'][0]+=1
        with self.assertRaisesRegex(RuntimeError,'configuration differs'):item._prepare_allocator(loop)


if __name__ == '__main__':unittest.main()
