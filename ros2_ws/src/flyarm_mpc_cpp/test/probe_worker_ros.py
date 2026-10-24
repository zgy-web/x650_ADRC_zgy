"""Private-domain software replay of original CDR through the actual C++ worker.

Only LiveStart and admitted ControlSnapshot are published. There is no ingress,
dispatcher, Gazebo, PX4 client, actuator publisher or physical acceptance here.
"""
from __future__ import annotations
import argparse
import base64
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def identity(path):
    path = Path(path).resolve()
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return dict(path=str(path), bytes=path.stat().st_size, sha256=digest.hexdigest())


def binding(path):
    value = identity(path)
    return {key: value[key] for key in ('path', 'sha256')}


def write(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)


def delta(a, b):
    if len(a) != len(b):
        raise ValueError('Proposal array shape changed')
    if not all(math.isfinite(float(v)) for v in [*a,*b]):
        raise ValueError('Proposal contains nonfinite value')
    return max((abs(float(x)-float(y)) for x, y in zip(a, b)), default=0.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('history', 'case-config', 'task', 'core-library', 'optimization-library', 'predictor-library', 'worker', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--core-source', type=Path, nargs='+', required=True)
    parser.add_argument('--required-prior-exit', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, default=221)
    parser.add_argument('--probe-cpu', type=int, default=94)
    args = parser.parse_args()
    if not 1 <= args.domain_id <= 232:
        raise ValueError('Explicit private DDS domain required')
    if args.required_prior_exit.read_text().strip() != '0':
        raise ValueError('Prior core qualification has not ended successfully; no concurrent replay')
    os.sched_setaffinity(0, {args.probe_cpu})
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    args.output.mkdir(parents=True, exist_ok=False)
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    import rclpy
    from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal, LiveStart

    recording = args.history / 'recording/diagnostics.jsonl'
    original_start = args.history / 'live_start.json'
    old_manifest = json.loads((args.history / 'recording/manifest.json').read_text())
    original_identity = identity(recording)
    if original_identity['sha256'] != old_manifest['records_sha256']:
        raise ValueError('Original raw recording SHA differs')
    rows = []
    for line in recording.open():
        row = json.loads(line)
        snapshot = deserialize_message(base64.b64decode(row['snapshot_cdr']['base64']), ControlSnapshot)
        proposal = deserialize_message(base64.b64decode(row['proposal_cdr']['base64']), ControlProposal)
        if snapshot.sequence != len(rows)+1 or proposal.snapshot_sequence != snapshot.sequence:
            raise ValueError('Original CDR sequence differs')
        rows.append((snapshot, proposal))
    if len(rows) != 1056 or len(rows) != old_manifest['rows']:
        raise ValueError('This probe explicitly requires the complete original 1056-row hold25 recording')
    start_values = json.loads(original_start.read_text())
    package = Path(__file__).resolve().parents[1]
    source_paths = [recording, original_start, args.case_config, *args.core_source,
                    *sorted(p for p in package.rglob('*') if p.is_file() and '__pycache__' not in p.parts)]
    source_paths = list(dict.fromkeys(str(p.resolve()) for p in source_paths))
    manifest = dict(schema_version=1, kind='cpp_full_pipeline_v1', simulation_only=True,
        task_config=binding(args.task), core_library=binding(args.core_library), optimization_library=binding(args.optimization_library),
        predictor_library={**binding(args.predictor_library), 'symbol':'ts20_prediction_batch'},
        source_artifacts=[binding(p) for p in source_paths],
        config=json.loads(args.case_config.read_text())['config'], trajectory=dict(kind='hold'))
    manifest_path = args.output / 'startup_manifest.json'
    write(manifest_path, manifest)
    report = dict(kind='cpp_worker_actual_ros_software_replay_v1', passed=False, complete=False,
        physical_acceptance=False, no_actuator_publishers=True,
        scope='Actual ROS C++ worker and original CDR only; no dispatcher/consumer/plant. Not hard-real-time proof.',
        original_recording=original_identity, original_live_start=identity(original_start),
        binary=identity(args.worker), core_library=identity(args.core_library),
        startup_manifest=identity(manifest_path), domain_id=args.domain_id, probe_cpu=args.probe_cpu,
        modified_snapshot_fields=['session_id','epoch','configuration_sha256','issued_monotonic_ns'],
        control_time_dt_and_actual_history_unchanged=True, cases=[])
    hash_bytes = list(bytes.fromhex(manifest['task_config']['sha256']))
    rclpy.init()
    node = rclpy.create_node('cpp_worker_private_software_probe')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    snapshots = node.create_publisher(ControlSnapshot, '/flyarm/isolated/v1/admitted/snapshot', qos)
    starts = node.create_publisher(LiveStart, '/flyarm/isolated/v1/live_start', 1)
    received = []
    node.create_subscription(ControlProposal, '/flyarm/isolated/v1/output/proposal', received.append, qos)
    child = None
    log = None

    def spin_until(condition, timeout, label):
        until = time.monotonic()+timeout
        while not condition():
            if time.monotonic() >= until:
                raise TimeoutError(label)
            if child is not None and child.poll() is not None:
                # Drain a final DDS proposal that may precede process exit.
                rclpy.spin_once(node, timeout_sec=.002)
                if condition():
                    break
                raise RuntimeError(f'{label}: worker exited {child.returncode}')
            rclpy.spin_once(node, timeout_sec=.001)

    try:
        for case_index, case_name in enumerate(('complete_history_sigint', 'invalid_hash_latched')):
            out = args.output / case_name
            out.mkdir()
            session = 240000000 + case_index
            received.clear()
            log = (out/'child.log').open('wb')
            command = ['taskset','-c','87,89,91,97',str(args.worker),'--ros-args',
                '-p',f'startup_manifest:={manifest_path}', '-p',f'startup_sha256:={identity(manifest_path)["sha256"]}',
                '-p',f'output_dir:={out}', '-p',f'expected_session_id:={session}', '-p','expected_epoch:=1',
                '-p','simulation_live:=true', '-p','executor_cpu:=87', '-p','auxiliary_cpus:=[89,91,97]']
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            spin_until(lambda: (out/'worker_ready').exists() and snapshots.get_subscription_count()==1
                       and starts.get_subscription_count()==1, 15., 'READY/DDS endpoint discovery')
            start = LiveStart()
            for field, value in start_values.items():
                setattr(start, field, value)
            start.session_id, start.epoch = session, 1
            start.configuration_sha256 = hash_bytes
            start.issued_monotonic_ns = time.monotonic_ns()
            starts.publish(start)
            until = time.monotonic()+.04
            while time.monotonic()<until:
                rclpy.spin_once(node, timeout_sec=.001)
            count = 1056 if case_index==0 else 3
            case = dict(name=case_name, rows=[], passed=False)
            report['cases'].append(case)
            due = time.monotonic()
            for index, (original, expected) in enumerate(rows[:count]):
                while time.monotonic() < due:
                    rclpy.spin_once(node, timeout_sec=min(.001, max(0.,due-time.monotonic())))
                snapshot = copy.deepcopy(original)
                snapshot.session_id, snapshot.epoch = session, 1
                snapshot.configuration_sha256 = hash_bytes
                invalid = case_index==1 and index==2
                if invalid:
                    snapshot.configuration_sha256[0] ^= 1
                snapshot.issued_monotonic_ns = time.monotonic_ns()
                snapshots.publish(snapshot)
                spin_until(lambda: len(received)>index, 1., f'proposal {index+1}')
                got = received[index]
                if got.sequence!=index+1 or got.snapshot_sequence!=index+1 or got.session_id!=session or got.epoch!=1 or list(got.configuration_sha256)!=hash_bytes:
                    raise ValueError('Actual ROS proposal identity/sequence changed')
                if invalid:
                    if got.status!=1 or got.accepted_optimum or any(got.base_velocity_target) or any(got.joint_velocity):
                        raise ValueError('Bad hash was not rejected with zero commands')
                    case['rows'].append(dict(sequence=index+1, status=int(got.status), intentionally_invalid=True))
                    valid = copy.deepcopy(original)
                    valid.session_id, valid.epoch = session, 1
                    valid.configuration_sha256 = hash_bytes
                    valid.issued_monotonic_ns = time.monotonic_ns()
                    snapshots.publish(valid)
                    child.wait(timeout=5.)
                    until=time.monotonic()+.05
                    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.001)
                    if child.returncode!=1 or len(received)!=count:
                        raise ValueError('Failure did not latch/exit; additional candidate appeared')
                else:
                    errors = {field:delta(getattr(got,field),getattr(expected,field)) for field in
                        ('base_velocity_target','joint_velocity','position_reference_enu','velocity_feedforward_enu','acceleration_feedforward_enu')}
                    if got.status!=0 or not got.accepted_optimum or not got.nonlinear_check_passed or got.subproblems!=expected.subproblems or max(errors.values())>1e-8:
                        raise ValueError(f'Original ROS proposal numerical/branch mismatch at {index+1}: {errors}, status={got.status}')
                    case['rows'].append(dict(sequence=index+1, status=int(got.status), absolute_errors=errors,
                        native_callback_ms=(got.compute_end_monotonic_ns-got.compute_start_monotonic_ns)/1e6))
                due += float(rows[min(index+1,len(rows)-1)][0].dt_s)
            if case_index==0:
                child.send_signal(signal.SIGINT)
                child.wait(timeout=8.)
                if child.returncode!=0:raise ValueError('Normal SIGINT worker did not finish successfully')
            final=json.loads((out/'worker_final.json').read_text())
            sender=json.loads((out/'diagnostic_sender.json').read_text())
            timing=json.loads((out/'worker_timing.json').read_text())
            affinity=json.loads((out/'worker_affinity_final.json').read_text())
            if final['requests']!=count or final['accepted']!=(count if case_index==0 else count-1) or final['faulted']!=(case_index==1):
                raise ValueError('Final worker counters/latch differ')
            if not sender['complete'] or sender['written']!=count or not timing['complete'] or len(timing['rows'])!=count:
                raise ValueError('Raw diagnostics or phase evidence incomplete')
            if [s['stage'] for s in affinity['stages']]!=['ready','live_start','final']:
                raise ValueError('Normal signal cleanup did not preserve strict live DDS thread stages')
            record_files=[]
            for index in range(count):
                path=out/'native_records'/f'{index+1:06d}.json'
                evidence=json.loads(path.read_text())
                if evidence['sequence']!=index+1 or evidence['diagnostic_mode']!='compact':raise ValueError('Call evidence identity/mode differs')
                for field in ('snapshot_cdr','proposal_cdr'):
                    if evidence[field]!=identity(evidence[field]['path']):raise ValueError('Raw CDR artifact identity differs')
                record_files.append(identity(path))
            case.update(passed=True, child_exit=child.returncode, final=final, recorder=sender,
                records=record_files, affinity=identity(out/'worker_affinity_final.json'), timing=identity(out/'worker_timing.json'))
            log.close();log=None;child=None
            print(json.dumps(dict(case=case_name, passed=True, rows=count)),flush=True)
        if manifest['source_artifacts'] != [binding(p) for p in source_paths]:
            raise ValueError('Original or bound sources changed during software replay')
        report.update(passed=True, complete=True)
    except Exception as error:
        report['error']=f'{type(error).__name__}: {error}'
        raise
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:child.wait(timeout=3.)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3.)
        if log is not None:log.close()
        node.destroy_node();rclpy.shutdown()
        write(args.output/'report.json',report)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
