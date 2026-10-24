"""Ground-loaded, physics-clock external-wrench lifecycle for robustness runs.

This mixin neither publishes actuator commands nor reads a file in the ACTIVE
pump. The original Gazebo plugin owns the immutable native-step waveform.
Its enqueue ledger is evidence of the commanded ECS increment, not a force
sensor or independent proof that the dynamics engine consumed the wrench.
"""
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[2]
PLUGIN_DIR = ROOT / 'benchmark/whole_body_closed_loop_v1'
RUNTIME_KEYS = {'library', 'build', 'probe', 'source', 'loader', 'binding_source'}


def read(path):
    def invalid(value):
        raise ValueError('Nonfinite JSON: ' + value)
    return json.loads(Path(path).read_text(encoding='utf-8'), parse_constant=invalid)


def identity(path):
    path = Path(path).resolve(strict=True)
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'bytes': path.stat().st_size}


def _bound(binding):
    if (not isinstance(binding, dict) or set(binding) not in ({'path', 'sha256'}, {'path', 'sha256', 'bytes'})
            or not isinstance(binding.get('path'), str) or not Path(binding['path']).is_absolute()
            or not isinstance(binding.get('sha256'), str) or len(binding['sha256']) != 64
            or set(binding['sha256']) - set('0123456789abcdef')):
        raise ValueError('Absolute path and lowercase SHA256 artifact binding required')
    actual = identity(binding['path'])
    if any(actual[key] != value for key, value in binding.items()):
        raise ValueError('External-wrench runtime artifact changed: ' + binding['path'])
    return Path(actual['path'])


def runtime_binding(root=ROOT, probe_path=None):
    """Read-only builder helper; task inputs bind these bytes before flight."""
    root = Path(root).resolve(strict=True)
    directory = root / 'benchmark/whole_body_closed_loop_v1'
    probe = (Path(probe_path) if probe_path is not None else
             root / 'generated/gz_world_entity_probe_build_v1_20261022/gz_world_entity_probe')
    result = {name: identity(path) for name, path in {
        'library': directory / 'libdisturbance.so', 'build': directory / 'disturbance_build.json',
        'probe': probe, 'source': directory / 'disturbance.cpp',
        'loader': directory / 'disturbance_loader.py',
        'binding_source': directory / 'gz_world_entity_binding.py'}.items()}
    verify_runtime_binding(result, plugin_dir=directory)
    return result


def verify_runtime_binding(value, *, plugin_dir=PLUGIN_DIR):
    """Ground-only exact identities, including the original build provenance."""
    if not isinstance(value, dict) or set(value) != RUNTIME_KEYS:
        raise ValueError('Six explicitly bound external-wrench runtime artifacts required')
    paths = {key: _bound(binding) for key, binding in value.items()}
    expected = {'library': 'libdisturbance.so', 'build': 'disturbance_build.json',
                'source': 'disturbance.cpp', 'loader': 'disturbance_loader.py',
                'binding_source': 'gz_world_entity_binding.py'}
    directory = Path(plugin_dir).resolve(strict=True)
    if any(paths[key] != (directory / name).resolve(strict=True) for key, name in expected.items()):
        raise ValueError('External-wrench plugin runtime must use the bound original directory')
    build = read(paths['build'])
    if (build.get('passed') is not True or type(build.get('checks')) is not int or build['checks'] < 12
            or build.get('source_sha256') != value['source']['sha256']
            or build.get('library_sha256') != value['library']['sha256']):
        raise ValueError('External-wrench build proof differs from source/library')
    return paths


def _module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_new(path, value):
    with Path(path).open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


class DisturbanceMixin:
    """Place before CppFastExtension in the robustness extension's MRO.

    Expects self.robust_cfg['robustness'] with kind, schedule and the frozen
    wrench_runtime binding. For other cases all methods simply delegate.
    """

    def _external_wrench(self):
        return self.robust_cfg['robustness']['kind'] == 'external_wrench'

    def on_loop_ready(self, ctx, loop, meta, out):
        super().on_loop_ready(ctx, loop, meta, out)
        if not self._external_wrench():
            return
        if (ctx.node.data['status'].arming_state == 2 or not ctx.node.data['land'].landed
                or self.started or self.active):
            raise RuntimeError('External-wrench loading requires disarmed ground before LiveStart')
        frozen = deepcopy(self.robust_cfg['robustness']['wrench_runtime'])
        paths = verify_runtime_binding(frozen)
        self.wrench_dir = Path(out) / 'physical_disturbance'
        self.wrench_dir.mkdir(exist_ok=False)
        self.wrench_runtime = frozen
        self.wrench_arm_receipt = None
        self.wrench_stop_receipt = None
        self.wrench_meta = meta
        binding = _module('_x650_robust_wrench_identity', paths['binding_source'])
        loader = _module('_x650_robust_wrench_loader', paths['loader'])
        state = read(ROOT / '.amsim/runtime.json')
        metadata = Path(state['runtime_meta'])
        metadata = metadata if metadata.is_absolute() else ROOT / metadata
        actual = read(metadata)
        if state.get('ready') is not True or state.get('model') != self.robust_cfg['model']:
            raise RuntimeError('External-wrench runtime model differs from admitted task')
        self.wrench_instance = actual['instance']
        captured = binding.capture_binding(ROOT, self.wrench_instance,
            self.wrench_dir / 'entity_identity', paths['probe'], loop.pump)
        self.wrench_binding = identity(captured)
        self.wrench_load = loader.load_disturbance(self.wrench_dir, self.wrench_instance,
            deepcopy(self.robust_cfg['robustness']['schedule']), self.robust_cfg['duration_s'],
            loop.pump, binding=captured)
        if (not isinstance(self.wrench_load, dict)
                or self.wrench_load.get('service_acknowledged') is not True
                or self.wrench_load.get('force_experiment_complete') is not False
                or self.wrench_load.get('library_sha256') != frozen['library']['sha256']):
            raise RuntimeError('External-wrench load acknowledgement is incomplete or changed')
        status_path = self.wrench_dir / 'applied_wrench.csv.status.json'
        deadline = time.monotonic() + 5.
        while True:
            loop.pump()
            if status_path.is_file():
                status = read(status_path)
                if status.get('state') == 'FAILED' or status.get('complete'):
                    raise RuntimeError('External-wrench plugin failed before activation: ' + str(status))
                if status.get('ready') is True:
                    break
            if time.monotonic() >= deadline:
                raise TimeoutError('External-wrench physics-step READY was not observed on ground')
            time.sleep(.002)
        if (status.get('state') != 'READY_WAITING_START' or status.get('anchored') is not False
                or status.get('model') != self.wrench_instance or status.get('link') != 'base_link'
                or status.get('duration_s') != self.robust_cfg['duration_s']):
            raise RuntimeError('External-wrench READY identity/duration differs')
        verify_runtime_binding(frozen)
        self.wrench_ready = deepcopy(status)
        _write_new(self.wrench_dir / 'ground_ready.json', {
            'kind': 'x650_robust_wrench_ground_ready_v1', 'ready': status,
            'runtime': frozen, 'entity_binding': self.wrench_binding,
            'load_ack': self.wrench_load, 'disarmed_ground': True,
            'no_actuator_output_added': True})
        meta['external_force_disturbance'] = True

    def activate(self, ctx, loop, cfg, meta, rotation):
        start = super().activate(ctx, loop, cfg, meta, rotation)
        if not self._external_wrench():
            return start
        before = float(loop.monitor_now())
        if not math.isfinite(before) or not 0 <= before-start <= .05 + 1e-9:
            raise RuntimeError('External-wrench immutable activation already exceeds 50 ms sim budget')
        begin_ns = time.monotonic_ns()
        marker = self.wrench_dir / 'disturbance_start.txt'
        with marker.open('x', encoding='ascii', newline='\n') as stream:
            stream.write('t0_sim_s=' + format(float(start), '.17g') + '\n')
        end_ns = time.monotonic_ns()
        after = float(loop.monitor_now())
        receipt = {'kind': 'x650_robust_wrench_arm_receipt_v1', 'origin_s': float(start),
            'observed_before_sim_s': before, 'observed_after_sim_s': after,
            'begin_monotonic_ns': begin_ns, 'end_monotonic_ns': end_ns,
            'activation_lag_s': after-float(start), 'max_activation_lag_s': .05,
            'start_marker': identity(marker), 'same_origin_as_live_start': True,
            'clock_semantics': 'loop physical monitor maximum and immutable LiveStart origin; native plugin independently rejects late activation'}
        self.wrench_arm_receipt = receipt
        _write_new(self.wrench_dir / 'arm_receipt.json', receipt)
        if not math.isfinite(after) or not 0 <= after-start <= .05 + 1e-9:
            raise RuntimeError('External-wrench activation write exceeded 50 ms sim budget')
        meta['external_force_disturbance'] = True
        return start

    def _stop_wrench(self, reason):
        if not self._external_wrench() or not hasattr(self, 'wrench_dir'):
            return
        marker = self.wrench_dir / 'disturbance_stop.txt'
        if not marker.exists():
            with marker.open('x', encoding='ascii', newline='\n') as stream:
                stream.write('stop\n')
            self.wrench_stop_receipt = {'reason': reason, 'monotonic_ns': time.monotonic_ns(),
                'marker': identity(marker)}
            _write_new(self.wrench_dir / 'stop_receipt.json', self.wrench_stop_receipt)

    def _release(self, loop, *, bounded_failed_recovery=False):
        if bounded_failed_recovery:
            self._stop_wrench('bounded_failed_recovery_before_release')
        try:
            return super()._release(loop, bounded_failed_recovery=bounded_failed_recovery)
        finally:
            # On a normal full run the physics-clock plugin has already reached
            # SUCCESS. Writing after original release avoids aborting its last
            # native step merely because control and physics meet at t=duration.
            self._stop_wrench('output_release_finished')

    def finalize(self, meta, stage, out):
        try:
            super().finalize(meta, stage, out)
        finally:
            self._stop_wrench('finalize')
            if self._external_wrench() and hasattr(self, 'wrench_dir'):
                path = self.wrench_dir / 'applied_wrench.csv.status.json'
                status = read(path) if path.is_file() else None
                required = ['applied_wrench.csv', 'applied_wrench.csv.status.json',
                    'disturbance_config.txt', 'disturbance_load.log', 'disturbance_load_binding.json',
                    'disturbance_start.txt', 'arm_receipt.json', 'ground_ready.json',
                    'disturbance_stop.txt', 'stop_receipt.json']
                artifacts = {name: identity(self.wrench_dir / name) for name in required
                             if (self.wrench_dir / name).is_file()}
                complete = bool(status and status.get('state') == 'SUCCESS'
                    and status.get('ready') is True and status.get('anchored') is True
                    and status.get('complete') is True and status.get('success') is True
                    and len(artifacts) == len(required) and self.wrench_arm_receipt
                    and abs(status.get('t0_sim_s', -1)-self.wrench_arm_receipt['origin_s']) <= 1e-9)
                summary = {'kind': 'x650_robust_wrench_lifecycle_v1',
                    'directory': str(self.wrench_dir), 'complete': complete,
                    'ready': getattr(self, 'wrench_ready', None),
                    'runtime': getattr(self, 'wrench_runtime', None),
                    'entity_binding': getattr(self, 'wrench_binding', None),
                    'load_ack': getattr(self, 'wrench_load', None),
                    'arm_receipt': getattr(self, 'wrench_arm_receipt', None),
                    'stop_receipt': getattr(self, 'wrench_stop_receipt', None),
                    'status': status, 'artifacts': artifacts,
                    'measurement_semantics': 'Physics-step ECS wrench increment; independent full waveform and actual motion audit still required',
                    'actuator_ownership_changed': False}
                _write_new(self.wrench_dir / 'final_summary.json', summary)
                for document in (meta, stage):
                    document.update(external_force_disturbance=True, robustness_external=deepcopy(summary))
                if not complete:
                    meta['complete'] = False
                    meta['robustness_external_error'] = 'Incomplete or failed physical-wrench lifecycle'
                    stage.update(success=False, complete=False, passed=False)

    def cleanup(self):
        try:
            self._stop_wrench('cleanup')
        finally:
            super().cleanup()
