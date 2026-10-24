"""Actual-simulation C++ feedback/core adapter; Python owns lifecycle only.

The existing prestream, actual-send admission, physical-consumption and recovery
transactions remain unchanged. No Python optimizer or snapshot publisher exists
on this adapter's ACTIVE path. Launch/software/physical qualification is separate.
"""
from copy import deepcopy
import base64
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_isolated_live_v1.run_case import LiveExtension, Ts20Extension, task, shared, sha, write


def expected_core_config(cfg, model, pid, predictor, prior):
    """Ground-only explicit export, without constructing or stepping a controller."""
    c = cfg['mpc_config']
    scalar = {name: c[name] for name in ('nodes', 'max_iterations', 'feasibility_tolerance',
              'kkt_tolerance', 'arm_share', 'joint_position_gain')}
    scalar.update(node_dt=c['node_dt_s'], max_dt=c['max_dt_s'])
    scalar.update({name: list(c[name]) for name in ('velocity_limit', 'acceleration_limit', 'q_lower', 'q_upper')})
    parameters = prior['parameters']
    scalar.update(weights=[c[name+'_weight'] for name in ('tip_position', 'tip_velocity', 'base_position',
                'joint_posture', 'input_nominal', 'input_delta')],
        physical_lower=list(model.joint_lower), physical_upper=list(model.joint_upper),
        geometry=[*model.lengths, *model.angle_offsets, *model.mount_body, *model.elbow_offset_body_m],
        response=[parameters['x']['wn_rad_s'], parameters['x']['zeta'],
                  parameters['y']['wn_rad_s'], parameters['y']['zeta'], parameters['z']['tau_s']],
        base_position_gain=[pid['MPC_XY_P'], pid['MPC_XY_P'], pid['MPC_Z_P']],
        motion=[predictor.J, predictor.b, predictor.Fc, predictor.offset, predictor.delay_s,
                predictor.Fs, predictor.delta, predictor.vs], kp=list(predictor.kp), kd=list(predictor.kd),
        bus_period=predictor.bus_period_s, integration_step=predictor.max_step_s,
        trust_radius=predictor.trust_radius_normalized)
    return shared.json_value(scalar)


class ExternalCppAllocator:
    """Metadata/reset bridge required by shared lifecycle; execution is forbidden."""
    def __init__(self, model, pid, manifest, worker_pid):
        self.model = model
        self.base_position_gain = np.array([pid['MPC_XY_P'], pid['MPC_XY_P'], pid['MPC_Z_P']])
        self.step_count = 0
        self.trajectory = None
        self._metadata = {'kind': 'cpp_full_pipeline_v1', 'online_python_steps_allowed': False,
            'worker_pid': worker_pid, 'core_library': deepcopy(manifest['core_library']),
            'optimization_library': deepcopy(manifest['optimization_library']),
            'predictor_library': deepcopy(manifest['predictor_library']),
            'configuration': deepcopy(manifest['config']), 'acceleration_feedforward_policy': 'zero'}

    def reset(self, position, rotation, q):
        # The actual C++ reset is performed once by LiveStart. This validates
        # only the lifecycle wrapper's inputs and cannot advance native memory.
        if (np.asarray(position).shape != (3,) or np.asarray(rotation).shape != (3, 3)
                or np.asarray(q).shape != (2,) or not all(np.isfinite(v).all()
                    for v in (np.asarray(position), np.asarray(rotation), np.asarray(q)))):
            raise ValueError('Actual C++ handoff anchor is malformed')

    def step(self, *args, **kwargs):
        raise RuntimeError('Python controller execution is forbidden for cpp_full_pipeline_v1')

    def metadata(self):
        return deepcopy(self._metadata)


def thread_rows(pid):
    result = []
    for p in sorted((Path('/proc')/str(pid)/'task').iterdir(), key=lambda x: int(x.name)):
        stat = (p/'stat').read_text()
        # /proc comm may contain spaces and parentheses; fields after its final
        # closing parenthesis start at field 3, so starttime (22) is index 19.
        result.append({'tid': int(p.name), 'starttime_ticks': int(stat[stat.rfind(')')+2:].split()[19]),
                       'cpus': sorted(os.sched_getaffinity(int(p.name)))})
    return result


def verify_worker_threads(rows, ready, worker_pid, roles):
    if ready['pid'] != worker_pid or ready['main_tid'] != worker_pid:
        raise RuntimeError('C++ worker PID/main TID changed')
    original = ready['stages'][0]['threads']
    if {(r['tid'], r['starttime_ticks']) for r in rows} != {(r['tid'], r['starttime_ticks']) for r in original}:
        raise RuntimeError('C++ worker thread identities changed')
    for row in rows:
        expected = [roles['executor_cpu']] if row['tid'] == worker_pid else roles['auxiliary_cpus']
        if row['cpus'] != expected:
            raise RuntimeError('C++ worker executor/auxiliary mask changed')


def binding_uint(value, bits):
    # The original archived reference uses float-valued ref_timestamp. Convert
    # only exact integral values, never truncate or wrap a reference identity.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('Reference counter/timestamp must be an exact unsigned integer')
    if isinstance(value, float) and (not np.isfinite(value) or not value.is_integer()):
        raise ValueError('Reference counter/timestamp must be an exact unsigned integer')
    result = int(value)
    if not 0 <= result < (1 << bits):
        raise ValueError('Reference counter/timestamp exceeds wire range')
    return result


class CppLiveExtension(LiveExtension):
    def prepare(self, out):
        self.protocol = task.read(Path(os.environ['FLYARM_CPP_PROTOCOL']))
        self.resources = json.loads(os.environ['FLYARM_EXECUTION_RESOURCES'])
        if self.resources['roles'] != self.protocol['cpu_roles']:
            raise ValueError('C++ execution CPU plan differs from selected protocol')
        declaration = self.protocol['worker_thread_roles']
        if set(declaration) != {'executor', 'auxiliary'} or len(declaration['executor']) != 1:
            raise ValueError('C++ worker requires one explicit executor and auxiliary CPUs')
        executor, auxiliary = declaration['executor'][0], declaration['auxiliary']
        roles = {'executor_cpu': executor, 'auxiliary_cpus': auxiliary}
        if (isinstance(executor, bool) or not isinstance(executor, int) or executor < 0
                or executor in auxiliary or sorted(set(auxiliary)) != auxiliary or not auxiliary
                or any(isinstance(v, bool) or not isinstance(v, int) or v < 0 for v in auxiliary)
                or sorted([executor, *auxiliary]) != self.resources['roles']['worker']):
            raise ValueError('C++ worker thread roles do not partition its resource mask')
        self.worker_thread_roles = roles
        self.worker_affinity_ready = self.worker_affinity_handoff = self.affinity_handoff_observed = None
        for p in Path('/proc/self/task').iterdir():
            os.sched_setaffinity(int(p.name), set(self.resources['roles']['source']))
        from ament_index_python.packages import get_package_prefix
        from rclpy.serialization import serialize_message
        from flyarm_control_interfaces import msg
        self.msg, self.serialize, self.prefix = msg, serialize_message, get_package_prefix
        self.active = self.stopping = self.started = self.cleaned = False
        self.out = Path(out)/'isolated_live'; self.out.mkdir()
        self.session = int.from_bytes(os.urandom(4), 'little') & 0x7fffffff or 1
        self.epoch = 1
        self.task_path = Path(os.environ[task.CONFIG_ENV]); self.task_sha = sha(self.task_path)
        self.procs, self.logs, self.sources = {}, [], []
        self.latest, self.message_counts, self.evidence_sequence = {}, {}, 0
        self.source_sender = self.supervisor = None
        self.pump_times = []
        self.failure = None; self.last_source = -1.; self.source_sequence = 0
        self.binding_ack = None
        from simulations.x650_isolated_live_v1.source_activity import SourceActivityTrace, Phase
        self.source_activity = SourceActivityTrace(); self.activity_phase = Phase; self.activity_token = 0
        manifest_path = Path(os.environ['FLYARM_CPP_STARTUP_MANIFEST']).resolve()
        expected = os.environ['FLYARM_CPP_STARTUP_SHA256']
        if not manifest_path.is_absolute() or sha(manifest_path) != expected:
            raise ValueError('C++ startup manifest hash differs')
        self.cpp_manifest = task.read(manifest_path)
        binding = self.cpp_manifest['task_config']
        if (Path(binding['path']).resolve() != self.task_path.resolve() or binding['sha256'] != self.task_sha
                or self.cpp_manifest['kind'] != 'cpp_full_pipeline_v1'
                or self.cpp_manifest['trajectory'] != {'kind': 'hold'} or 'numerical_backend' in self.cfg):
            raise ValueError('New complete C++ startup/task selection differs')
        write(self.out/'execution_resources.json', self.resources)
        write(self.out/'cpp_startup_binding.json', {'path': str(manifest_path), 'sha256': expected,
              'protocol_path': os.environ['FLYARM_CPP_PROTOCOL'], 'protocol_sha256': sha(Path(os.environ['FLYARM_CPP_PROTOCOL']))})
        # Only the legacy lifecycle's low-rate events use this recorder. The C++
        # source and worker persist their own raw CDR and actual outputs.
        self.private = Path(tempfile.mkdtemp(prefix='flyarm-cpp-live-')); self.private.chmod(0o700)
        source_sock = self.private/'source.sock'
        self._launch('source_recorder', [sys.executable, str(ROOT/'scripts/x650_isolated_diagnostic_recorder.py'),
            '--socket', str(source_sock), '--output', str(self.out/'source_recording')])
        deadline = time.monotonic()+10
        while not source_sock.exists():
            self._alive()
            if time.monotonic() > deadline: raise TimeoutError('Lifecycle recorder not ready')
            time.sleep(.01)
        from flyarm_mpc_reference.diagnostic_channel import DiagnosticSender
        self.source_sender = DiagnosticSender(source_sock)
        self._launch('worker', self._ros('flyarm_mpc_cpp', 'isolated_worker', {
            'startup_manifest': manifest_path, 'startup_sha256': expected,
            'expected_session_id': self.session, 'expected_epoch': self.epoch, 'simulation_live': True,
            'output_dir': self.out, 'executor_cpu': executor, 'auxiliary_cpus': auxiliary}))
        deadline = time.monotonic()+60
        while not (self.out/'worker_ready').exists():
            self._alive()
            if time.monotonic() > deadline: raise TimeoutError('C++ worker not ready before flight')
            time.sleep(.01)
        startup = task.read(self.out/'worker_startup.json')
        if (startup['implementation'] != 'cpp_full_pipeline_v1' or startup['python_in_worker']
                or startup['startup_controller_steps'] != 0 or startup['session_id'] != self.session
                or startup['epoch'] != self.epoch or startup['configuration_sha256'] != self.task_sha
                or startup['startup_manifest'] != self.cpp_manifest):
            raise RuntimeError('Actual C++ worker initialization differs')
        self.worker_affinity_ready = task.read(self.out/'worker_affinity_ready.json')
        verify_worker_threads(thread_rows(self.procs['worker'].pid), self.worker_affinity_ready,
                              self.procs['worker'].pid, roles)

    def _create_source_publisher(self, ctx):
        # Deliberately no Python publisher for /source. C++ owns all selection,
        # coordinate conversion, actual-history assembly and source issue clocks.
        self.binding_pub = ctx.node.create_publisher(self.msg.SourceBinding, '/flyarm/isolated/v1/source_binding', 1)
        self.binding_sub = ctx.node.create_subscription(self.msg.SourceBindingAck,
            '/flyarm/isolated/v1/source_binding_ack', self._on_binding_ack, 1)

    def _on_binding_ack(self, message):
        if message.session_id == self.session and message.epoch == self.epoch:
            self.binding_ack = message

    def _prepare_allocator(self, loop):
        binding = self.provenance['base_response_prior']
        if sha(binding['path']) != binding['sha256']:
            raise RuntimeError('Frozen response prior changed before ground preparation')
        expected = expected_core_config(self.cfg, loop.model, loop.pid, self.predictor, task.read(binding['path']))
        if expected != self.cpp_manifest['config']:
            raise RuntimeError('C++ configuration differs from task/model/PX4 gains/TS20 predictor')
        return ExternalCppAllocator(loop.model, loop.pid, self.cpp_manifest, self.procs['worker'].pid)

    def _prepare_additional_transport(self, ctx, loop, meta):
        self._launch('cpp_source', self._ros('flyarm_state_cpp', 'state_source', {
            'expected_session_id': self.session, 'expected_epoch': self.epoch,
            'configuration_sha256': self.task_sha, 'simulation_live': True,
            'state_csv': self.log_dir/'ts20_state.csv', 'packet_csv': self.log_dir/'ts20_packets.csv',
            'ready_file': self.out/'cpp_source_ready', 'evidence_directory': self.out/'cpp_source',
            'position_gains': self.cpp_manifest['config']['base_position_gain'],
            'bus_period_s': self.cpp_manifest['config']['bus_period'],
            'delay_s': self.cpp_manifest['config']['motion'][4], 'trajectory_kind': 'hold'}))
        meta['isolated_execution'].update(kind='cpp_full_pipeline_v1', numerical_backend_unchanged=False,
            active_source='C++ direct PX4/clock/actual TS20 CSV', active_controller='C++ complete Core',
            python_role='prestream, original reference binding, supervision, recovery and evidence only',
            python_active_snapshot_publisher=False, python_active_optimizer=False,
            numerical_policy='frozen model/parameters; new complete C++ implementation separately qualified')

    def _transport_ready(self):
        return ((self.out/'worker_ready').exists() and (self.out/'cpp_source_ready').exists()
            and self.start_pub.get_subscription_count() >= 5
            and self.binding_pub.get_subscription_count() == 1
            and all((self.out/(name+'_ready')).exists() for name in ('ingress', 'dispatcher', 'consumption'))
            and self.message_counts.get('dispatch', 0) > 2)

    def _bind_actual_source(self, ctx, loop):
        document = loop.fixed_reference.document
        reference = document['px4_reference_at_bind']
        extra = loop.feedback_adapter.document['extra_reset_counters_at_bind']
        message = self.msg.SourceBinding()
        message.schema_version = 1; message.session_id = self.session; message.epoch = self.epoch
        message.configuration_sha256 = list(bytes.fromhex(self.task_sha)); message.binding_sequence = 1
        message.px4_source_contract_commit = document['px4_source_contract_commit']
        message.frame_mode = document['frame_mode']; message.target_frozen = document['target_frozen']
        message.fit_estimator_translation = document['fit_estimator_translation']
        for key in ('ref_lat', 'ref_lon', 'ref_alt', 'ref_timestamp', 'xy_global', 'z_global',
                    'xy_reset_counter', 'z_reset_counter', 'heading_reset_counter'):
            value = reference[key]
            if key == 'ref_timestamp': value = binding_uint(value, 64)
            elif key.endswith('_counter'): value = binding_uint(value, 8)
            setattr(message, key, value)
        for key in ('vxy_reset_counter', 'vz_reset_counter', 'quat_reset_counter'):
            setattr(message, key, binding_uint(extra[key], 8))
        message.position_gains = self.cpp_manifest['config']['base_position_gain']
        message.bus_period_s = self.cpp_manifest['config']['bus_period']
        message.delay_s = self.cpp_manifest['config']['motion'][4]
        message.issued_monotonic_ns = time.monotonic_ns()
        self.binding_pub.publish(message)
        # Original prestream remains the sole writer while waiting. A failed
        # one-shot binding aborts; it is never silently retried with a new origin.
        ctx.vehicle.wait(lambda: self.binding_ack is not None, 1., 'C++ original source binding ACK')
        ack = self.binding_ack
        if (ack.session_id != self.session or ack.epoch != self.epoch
                or list(ack.configuration_sha256) != list(message.configuration_sha256)
                or ack.binding_sequence != 1 or ack.binding_issued_monotonic_ns != message.issued_monotonic_ns
                or not ack.accepted or ack.rejection_code != 0
                or not message.issued_monotonic_ns <= ack.observed_monotonic_ns <= time.monotonic_ns()
                or ack.observed_monotonic_ns-message.issued_monotonic_ns > 25_000_000):
            raise RuntimeError('C++ source binding ACK mismatched or rejected')
        self.binding_message = message

    def activate(self, ctx, loop, cfg, meta, rotation):
        self._bind_actual_source(ctx, loop)
        return super().activate(ctx, loop, cfg, meta, rotation)

    def _verify_handoff_resources(self):
        observed = {'source': self._affinity(os.getpid())}
        begin, cpu_begin = time.monotonic_ns(), time.thread_time_ns()
        for name, process in self.procs.items():
            if name == 'worker':
                rows = thread_rows(process.pid)
                verify_worker_threads(rows, self.worker_affinity_ready, process.pid, self.worker_thread_roles)
                observed[name] = {str(row['tid']): row['cpus'] for row in rows}
                self.worker_affinity_handoff = {'kind': 'cpp_worker_thread_handoff_v1', 'worker_pid': process.pid,
                    'threads': rows, 'monotonic_ns': time.monotonic_ns()}
            else:
                observed[name] = self._affinity(process.pid)
        for name, threads in observed.items():
            if name == 'worker': continue
            if not threads or any(cpus != self.resources['roles'].get(name, self.resources['background']) for cpus in threads.values()):
                raise RuntimeError('Owned C++ process CPU isolation differs: '+name)
        self.worker_affinity_handoff.update(check_begin_ns=begin, check_end_ns=time.monotonic_ns(),
            check_thread_cpu_begin_ns=cpu_begin, check_thread_cpu_end_ns=time.thread_time_ns(),
            scope='current PID/TID/starttime/affinity at handoff; no whole-lifetime auxiliary-thread claim')
        self.affinity_handoff_observed = observed

    def handle_active_pump(self, loop, sim):
        if not self.active: return False
        if loop.recovery:
            self._release(loop); return False
        self._activity_mark('ALIVE'); self._alive()
        self._activity_mark('SUPERVISION_SECOND'); self._check_supervision()
        if self.failure: raise RuntimeError(self.failure)
        tick, event = self.latest.get('dispatch'), self.latest.get('events')
        if tick is not None and tick.status == 3:
            raise RuntimeError('C++ output latched fault reason '+str(tick.reason))
        if event is not None and not event.accepted:
            raise RuntimeError('C++ candidate gate rejected reason '+str(event.reason))
        return True

    def _runtime_process_names(self):
        return ('cpp_source', *super()._runtime_process_names())

    def _write_source_timing(self):
        # The original timing file means actual Python source issue. Do not
        # replace it with synthetic rows or an empty file bearing that meaning.
        write(self.out/'source_adapter.json', {'kind': 'cpp_raw_feedback_v1',
            'actual_evidence_directory': str(self.out/'cpp_source'),
            'python_source_samples': 0, 'physical_acceptance_pending': True})
        if hasattr(self, 'binding_message'):
            from rosidl_runtime_py.convert import message_to_ordereddict
            write(self.out/'source_binding.json', message_to_ordereddict(self.binding_message))
            write(self.out/'source_binding_ack.json', message_to_ordereddict(self.binding_ack))

    def _materialize_source_recording(self):
        """Reconcile lifecycle DDS events only; C++ has the raw source evidence."""
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        sender = task.read(self.out/'source_sender.json')
        sink = task.read(self.out/'source_recording/manifest.json')
        raw = self.out/'source_recording/diagnostics.jsonl'
        digest = sha(raw)
        if (not sender['complete'] or sender['dropped'] != 0
                or not sender['attempts'] == sender['sent'] == sink['rows'] == sink['last_sequence']
                or sink['gaps'] or sink['error'] is not None or not sink['drained_and_fsynced']
                or digest != sink['records_sha256']):
            raise RuntimeError('C++ lifecycle DDS evidence count/hash/drain mismatch')
        output = {name: [] for name in self.message_types}
        observed = 0
        with raw.open() as stream:
            for expected, line in enumerate(stream, 1):
                row = json.loads(line)
                if row['sequence'] != expected or row['kind'] != 'ros' or row['topic'] not in self.message_types:
                    raise RuntimeError('Unexpected or missing lifecycle event; Python feedback is forbidden')
                name = row['topic']
                message = deserialize_message(base64.b64decode(row['cdr']['base64']), self.message_types[name])
                output[name].append(message_to_ordereddict(message))
                observed = expected
        if observed != sink['rows']:
            raise RuntimeError('Lifecycle event file does not contain every acknowledged row')
        for name, values in output.items(): write(self.out/(name+'.json'), values)
        write(self.out/'source_recording_reconciled.json', {'complete': True, 'attempts': sender['attempts'],
            'dropped': 0, 'feedback_rows': 0, 'message_rows': {k: len(v) for k, v in output.items()},
            'records_sha256': digest, 'scope': 'lifecycle DDS events only; actual source CDR belongs to cpp_source'})


def scenario():
    if os.environ.get('X650_EXECUTION_ADAPTER') != 'cpp_full_pipeline_v1':
        raise ValueError('Explicit complete C++ simulation selection required')
    if os.environ.get('AMSIM_SCENARIO_NAME') != 'x650_cpp_live_v1':
        raise ValueError('Dedicated C++ actual simulation scenario required')
    cfg = task.selected_task_config()
    model, provenance = task.preflight(cfg)
    out = Path(os.environ['AMSIM_RUN_DIR']).resolve()
    extension = CppLiveExtension(cfg, provenance)
    try:
        extension.prepare(out)
        return shared.execute(cfg, model, provenance, out, extension=extension)
    finally:
        if hasattr(extension, 'procs'): extension.cleanup()


if __name__ == '__main__':
    scenario()
