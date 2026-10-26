"""Synthetic lifecycle checks; never invokes ROS, Gazebo or actuator output."""
from contextlib import contextmanager
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import uuid

from simulations.x650_cpp_robustness_v1 import disturbance as d

ROOT = Path(__file__).resolve().parents[2]


@contextmanager
def space():
    path = ROOT / ('.tmp_robust_disturbance_' + uuid.uuid4().hex)
    path.mkdir()
    try:
        yield path
    finally:
        for child in sorted(path.rglob('*'), key=lambda p: len(p.parts), reverse=True):
            if child.is_file(): child.unlink()
            elif child.is_dir(): child.rmdir()
        path.rmdir()


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


def runtime(out):
    directory = out / 'benchmark/whole_body_closed_loop_v1'
    directory.mkdir(parents=True)
    for name in ('libdisturbance.so', 'disturbance.cpp', 'disturbance_loader.py', 'gz_world_entity_binding.py'):
        (directory / name).write_bytes(('synthetic fixture: ' + name).encode())
    probe = out / 'generated/gz_world_entity_probe_build_v1_20261022/gz_world_entity_probe'
    probe.parent.mkdir(parents=True); probe.write_bytes(b'synthetic probe, never executed')
    put(directory / 'disturbance_build.json', {'passed': True, 'checks': 12,
        'source_sha256': d.identity(directory / 'disturbance.cpp')['sha256'],
        'library_sha256': d.identity(directory / 'libdisturbance.so')['sha256']})
    return directory, d.runtime_binding(out)


def ready():
    return {'state': 'READY_WAITING_START', 'ready': True, 'anchored': False,
            'complete': False, 'success': False, 'model': 'actual_uav_0',
            'link': 'base_link', 'duration_s': 24.}


class Base:
    def __init__(self, cfg):
        self.robust_cfg = cfg
        self.started = self.active = False
        self.events = []
    def on_loop_ready(self, ctx, loop, meta, out): self.events.append('super_ready')
    def activate(self, ctx, loop, cfg, meta, rotation):
        self.events.append('super_activate'); self.started = self.active = True
        return 100.
    def _release(self, loop, *, bounded_failed_recovery=False):
        self.events.append(('super_release', bounded_failed_recovery))
        self.active = False
    def finalize(self, meta, stage, out): self.events.append('super_finalize')
    def cleanup(self): self.events.append('super_cleanup')


class Extension(d.DisturbanceMixin, Base): pass


def cfg(binding=None, kind='external_wrench'):
    return {'model': 'nominal_model', 'duration_s': 24., 'robustness': {
        'kind': kind, 'wrench_runtime': binding, 'schedule': [
            {'start_s': 7., 'end_s': 7.6, 'force_world_n': [1.5, 0., 0.],
             'torque_world_nm': [0., 0., 0.]}]}}


class RuntimeProof(unittest.TestCase):
    def test_bound_build_proof_and_all_six_artifacts_then_tamper(self):
        with space() as out:
            directory, value = runtime(out)
            checked = d.verify_runtime_binding(value, plugin_dir=directory)
            self.assertEqual(set(checked), d.RUNTIME_KEYS)
            (directory / 'disturbance.cpp').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'artifact changed'):
                d.verify_runtime_binding(value, plugin_dir=directory)

    def test_mismatched_build_and_missing_hash_are_rejected(self):
        with space() as out:
            directory, value = runtime(out)
            build = json.loads((directory / 'disturbance_build.json').read_text())
            build['source_sha256'] = '0' * 64
            put(directory / 'disturbance_build.json', build)
            value['build'] = d.identity(directory / 'disturbance_build.json')
            with self.assertRaisesRegex(ValueError, 'build proof'):
                d.verify_runtime_binding(value, plugin_dir=directory)
            del value['probe']['sha256']
            with self.assertRaisesRegex(ValueError, 'SHA256'):
                d.verify_runtime_binding(value, plugin_dir=directory)


class AtomicStartMarker(unittest.TestCase):
    def test_physics_reader_cannot_see_open_empty_or_partial_staging_file(self):
        with space() as out:
            marker=out/'disturbance_start.txt';text='t0_sim_s=100.004\n'
            original_open,original_link=Path.open,d.os.link
            observations=[];closed=[]
            def observe(label):
                observations.append(label)
                self.assertFalse(marker.exists(),label)
            class Writer:
                def __init__(self,stream):self.stream=stream
                def __enter__(self):return self
                def write(self,value):
                    observe('before_write')
                    count=self.stream.write(value[:4])
                    self.stream.flush()
                    observe('partial_staging_bytes')
                    count+=self.stream.write(value[4:])
                    observe('complete_but_open_staging')
                    return count
                def __exit__(self,*args):
                    result=self.stream.__exit__(*args)
                    closed.append(self.stream.closed)
                    observe('closed_before_publication')
                    return result
            def open_observed(path,*args,**kwargs):
                stream=original_open(path,*args,**kwargs)
                if args and args[0]=='x' and path.name.endswith('.staging'):
                    observe('created_empty_staging')
                    return Writer(stream)
                return stream
            def publish(source,target):
                self.assertEqual(closed,[True])
                self.assertEqual(Path(source).read_text(),text)
                self.assertEqual(Path(target),marker)
                observe('immediately_before_atomic_link')
                original_link(source,target)
                self.assertEqual(marker.read_text(),text)
            with patch.object(Path,'open',open_observed),patch.object(d.os,'link',side_effect=publish):
                d._publish_start_marker(marker,text)
            self.assertIn('partial_staging_bytes',observations)
            self.assertEqual(marker.read_text(),text)
            self.assertEqual(list(out.iterdir()),[marker])

    def test_existing_final_marker_is_never_replaced_and_staging_is_removed(self):
        with space() as out:
            marker=out/'disturbance_start.txt';marker.write_bytes(b't0_sim_s=99\n')
            before=d.identity(marker)
            with self.assertRaises(FileExistsError):d._publish_start_marker(marker,'t0_sim_s=100\n')
            self.assertEqual(d.identity(marker),before)
            self.assertEqual(list(out.iterdir()),[marker])

    def test_failed_partial_write_never_exposes_final_name(self):
        with space() as out:
            marker=out/'disturbance_start.txt';original_open=Path.open
            class FailingWriter:
                def __init__(self,stream):self.stream=stream
                def __enter__(self):return self
                def write(self,value):
                    self.stream.write(value[:4]);self.stream.flush()
                    raise OSError('synthetic staging write failure')
                def __exit__(self,*args):return self.stream.__exit__(*args)
            def opened(path,*args,**kwargs):
                stream=original_open(path,*args,**kwargs)
                return FailingWriter(stream) if args and args[0]=='x' else stream
            with patch.object(Path,'open',opened),self.assertRaisesRegex(OSError,'staging write failure'):
                d._publish_start_marker(marker,'t0_sim_s=100\n')
            self.assertFalse(marker.exists());self.assertEqual(list(out.iterdir()),[])

    def test_exclusive_staging_collision_preserves_foreign_original_file(self):
        with space() as out:
            marker=out/'disturbance_start.txt'
            staging=out/'.disturbance_start.txt.fixed.staging';staging.write_bytes(b'original collision evidence')
            before=d.identity(staging)
            with patch.object(d.uuid,'uuid4',return_value=SimpleNamespace(hex='fixed')):
                with self.assertRaises(FileExistsError):d._publish_start_marker(marker,'t0_sim_s=100\n')
            self.assertFalse(marker.exists());self.assertEqual(d.identity(staging),before)


class Lifecycle(unittest.TestCase):
    def fixture(self, out):
        directory, binding = runtime(out)
        extension = Extension(cfg(binding))
        ctx = SimpleNamespace(node=SimpleNamespace(data={
            'status': SimpleNamespace(arming_state=1), 'land': SimpleNamespace(landed=True)}))
        loop = SimpleNamespace(pump=lambda: None, monitor_now=lambda: 100.004)
        put(out / '.amsim/runtime.json', {'ready': True, 'model': 'nominal_model',
            'runtime_meta': 'runtime_metadata.json'})
        put(out / 'runtime_metadata.json', {'instance': 'actual_uav_0'})
        load_calls, capture_calls = [], []
        def capture(root, instance, output, probe, pump):
            capture_calls.append((root, instance, output, probe))
            output.mkdir()
            put(output / 'entity_binding.json', {'synthetic': True})
            return output / 'entity_binding.json'
        def load(output, instance, schedule, duration, pump, *, binding):
            load_calls.append((output, instance, schedule, duration, binding))
            for name in ('disturbance_config.txt', 'disturbance_load.log', 'applied_wrench.csv'):
                (output / name).write_text('synthetic fixture\n')
            result = {'service_acknowledged': True, 'force_experiment_complete': False,
                      'library_sha256': extension.robust_cfg['robustness']['wrench_runtime']['library']['sha256']}
            put(output / 'disturbance_load_binding.json', result)
            put(output / 'applied_wrench.csv.status.json', ready())
            return result
        modules = [SimpleNamespace(capture_binding=capture), SimpleNamespace(load_disturbance=load)]
        return extension, ctx, loop, directory, modules, load_calls, capture_calls

    @contextmanager
    def prepared(self, out):
        extension, ctx, loop, directory, modules, loads, captures = self.fixture(out)
        original_verify = d.verify_runtime_binding
        def verify(value): return original_verify(value, plugin_dir=directory)
        with patch.object(d, 'ROOT', out), patch.object(d, 'verify_runtime_binding', side_effect=verify), \
                patch.object(d, '_module', side_effect=modules):
            meta = {}; extension.on_loop_ready(ctx, loop, meta, out)
        yield extension, ctx, loop, meta, loads, captures

    def test_ground_load_uses_fresh_actual_binding_and_same_start_origin(self):
        with space() as out, self.prepared(out) as (extension, ctx, loop, meta, loads, captures):
            self.assertEqual(extension.events, ['super_ready'])
            self.assertEqual(captures[0][1], 'actual_uav_0')
            self.assertEqual(loads[0][4], out / 'physical_disturbance/entity_identity/entity_binding.json')
            self.assertEqual(loads[0][2], extension.robust_cfg['robustness']['schedule'])
            self.assertTrue(meta['external_force_disturbance'])
            self.assertEqual(extension.activate(ctx, loop, extension.robust_cfg, meta, None), 100.)
            path = out / 'physical_disturbance/disturbance_start.txt'
            self.assertEqual(path.read_text(), 't0_sim_s=100\n')
            self.assertAlmostEqual(extension.wrench_arm_receipt['activation_lag_s'], .004)
            with self.assertRaises(FileExistsError):
                extension.activate(ctx, loop, extension.robust_cfg, meta, None)

    def test_late_activation_never_creates_start_marker(self):
        with space() as out, self.prepared(out) as (extension, ctx, loop, meta, _, __):
            loop.monitor_now = lambda: 100.051
            with self.assertRaisesRegex(RuntimeError, '50 ms'):
                extension.activate(ctx, loop, extension.robust_cfg, meta, None)
            self.assertFalse((extension.wrench_dir / 'disturbance_start.txt').exists())

    def test_normal_release_stop_after_original_then_complete_summary(self):
        with space() as out, self.prepared(out) as (extension, ctx, loop, meta, _, __):
            extension.activate(ctx, loop, extension.robust_cfg, meta, None)
            status = ready(); status.update(state='SUCCESS', anchored=True, complete=True,
                                           success=True, t0_sim_s=100.)
            put(extension.wrench_dir / 'applied_wrench.csv.status.json', status)
            extension._release(loop)
            self.assertEqual(extension.wrench_stop_receipt['reason'], 'output_release_finished')
            stage = dict(complete=True, passed=True, success=True)
            meta['complete'] = True
            extension.finalize(meta, stage, out)
            self.assertTrue(meta['robustness_external']['complete'])
            self.assertEqual(meta['robustness_external'], stage['robustness_external'])
            self.assertEqual(meta['robustness_external'], d.read(extension.wrench_dir / 'final_summary.json'))
            self.assertTrue(stage['passed'])

    def test_failure_stops_before_bounded_release_and_never_qualifies(self):
        with space() as out, self.prepared(out) as (extension, ctx, loop, meta, _, __):
            extension._release(loop, bounded_failed_recovery=True)
            self.assertEqual(extension.wrench_stop_receipt['reason'], 'bounded_failed_recovery_before_release')
            meta['complete'] = True
            stage = dict(complete=True, passed=True, success=True)
            extension.finalize(meta, stage, out)
            self.assertFalse(meta['complete']); self.assertFalse(stage['passed'])
            self.assertFalse(meta['robustness_external']['complete'])

    def test_non_wrench_case_is_pure_delegate_and_never_loads(self):
        extension = Extension(cfg(kind='mass_inertia_scale'))
        with patch.object(d, '_module') as load, patch.object(d, 'verify_runtime_binding') as verify:
            meta, stage = {}, {}
            extension.on_loop_ready(None, None, meta, None)
            self.assertEqual(extension.activate(None, None, extension.robust_cfg, meta, None), 100.)
            extension._release(None); extension.finalize(meta, stage, None); extension.cleanup()
            load.assert_not_called(); verify.assert_not_called()
            self.assertNotIn('robustness_external', meta)
            self.assertEqual(len(extension.events), 5)

    def test_armed_loading_is_refused_before_plugin_access(self):
        extension = Extension(cfg())
        ctx = SimpleNamespace(node=SimpleNamespace(data={
            'status': SimpleNamespace(arming_state=2), 'land': SimpleNamespace(landed=False)}))
        with patch.object(d, 'verify_runtime_binding') as verify:
            with self.assertRaisesRegex(RuntimeError, 'disarmed ground'):
                extension.on_loop_ready(ctx, None, {}, None)
            verify.assert_not_called()


if __name__ == '__main__': unittest.main()
