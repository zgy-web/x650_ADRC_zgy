#!/usr/bin/env python3
"""Synthetic private-DDS check of the real ingress rejected-input evidence.

Starts only the explicitly hash-bound state_ingress binary. No Gazebo, PX4,
worker, dispatcher or physical actuator runs. The fixture checks CONTEXT
rejection, original source identity/timestamps, every decoded CDR field and a
separate wrong-session case. Only this script's own child PID is stopped.
"""
from __future__ import annotations
import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import traceback
import uuid

TOPIC = '/flyarm/isolated/v1/'


def require(value, message):
    if not value:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def plain(value):
    if hasattr(value, 'get_fields_and_field_types'):
        return {name: plain(getattr(value, name)) for name in value.get_fields_and_field_types()}
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if hasattr(value, 'tolist'):
        return plain(value.tolist())
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    if hasattr(value, 'item'):
        return value.item()
    return value


def check_evidence(metadata, decoded, original, first_control, event, cdr_bytes):
    """Compare all source fields, permitting only the two admission rewrites."""
    expected = deepcopy(plain(original))
    expected['sequence'] = int(event.snapshot_sequence)
    expected['dt_s'] = float(original.control_time_s) - first_control
    actual = plain(decoded)
    require(set(actual) == set(expected), 'Decoded CDR field set changed')
    different = [key for key in expected if actual[key] != expected[key]]
    require(not different, 'Rejected CDR fields differ: ' + ', '.join(different))
    require(metadata['kind'] == 'isolated_rejected_snapshot_v1' and metadata['complete'] is True,
            'Incomplete rejection evidence')
    require(metadata['snapshot_semantics'] ==
            'exact_gate_input_after_admission_sequence_and_actual_dt_assignment', 'Unknown input semantics')
    require(metadata['ros_type'] == 'flyarm_control_interfaces/msg/ControlSnapshot', 'Wrong CDR type')
    require(metadata['payload'] == 'rejected_snapshot.cdr' and metadata['bytes'] == cdr_bytes,
            'CDR size/name mismatch')
    require(metadata['forward'] is False and metadata['reason'] == int(event.reason) == 15,
            'Expected a nonforwarded CONTEXT failure')
    for name in ('session_id', 'epoch'):
        require(metadata[name] == int(getattr(original, name)) == int(getattr(event, name)),
                'Evidence/event/source identity differs: ' + name)
    require(metadata['sequence'] == int(event.snapshot_sequence) and
            metadata['event_sequence'] == int(event.sequence), 'Event sequence mismatch')
    require(metadata['source_sequence'] == int(original.sequence), 'Original source sequence was lost')
    require(metadata['configuration_sha256'] == bytes(original.configuration_sha256).hex(),
            'Configuration binding differs')
    issued = int(original.issued_monotonic_ns)
    require(metadata['source_issued_monotonic_ns'] == issued, 'Source issue clock was rewritten')
    require(issued <= metadata['source_received_monotonic_ns'] <= metadata['checked_monotonic_ns']
            <= metadata['gate_return_monotonic_ns'], 'Rejected input clocks are not causal')
    require(metadata['event_received_monotonic_ns'] == metadata['checked_monotonic_ns']
            == int(event.received_monotonic_ns), 'Gate check and event clocks differ')
    return {'all_decoded_fields_equal': True, 'field_count': len(expected),
            'allowed_rewrites': ['sequence', 'dt_s'], 'source_sequence': int(original.sequence),
            'admission_sequence': int(event.snapshot_sequence), 'actual_dt_s': expected['dt_s']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--binary-sha256', required=True)
    parser.add_argument('--domain-id', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-s', type=float, default=35.)
    args = parser.parse_args(argv)
    require(sys.platform.startswith('linux'), 'Sourced Linux ROS workspace required')
    require(1 <= args.domain_id <= 232, 'Explicit private nondefault DDS domain required')
    require(math.isfinite(args.timeout_s) and 15 <= args.timeout_s <= 120, 'Timeout must be 15..120 seconds')
    require(re.fullmatch('[0-9a-f]{64}', args.binary_sha256) is not None, 'Lowercase binary SHA256 required')
    require(args.binary.is_absolute(), 'Absolute state_ingress binary required')
    binary = args.binary.resolve(strict=True)
    require(binary.name == 'state_ingress' and os.access(binary, os.X_OK), 'Only state_ingress may be launched')
    require(sha(binary) == args.binary_sha256, 'Binary differs from supplied hash')
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    report = dict(kind='x650_rejected_snapshot_synthetic_probe_v1', passed=False, complete=False,
                  synthetic_software_fixture_only=True, physical_validation=False, flight_accepted=False,
                  hardware_qualified=False, eligible_as_stage_gate=False, domain_id=args.domain_id,
                  binary=dict(path=str(binary), sha256=args.binary_sha256), probe_sha256=sha(__file__), cases=[])
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1')
    deadline = time.monotonic() + args.timeout_s
    child = node = log = None; initialized = False
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.serialization import deserialize_message
        from flyarm_control_interfaces import msg
        rclpy.init(); initialized = True
        node = rclpy.create_node('rejected_snapshot_probe_' + uuid.uuid4().hex[:10])
        best = QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE)
        observed = {'admitted': [], 'accepted': [], 'status': [], 'events': []}
        identity = {'session': 0}
        subscriptions = []
        def receive(key, value):
            if int(value.session_id) == identity['session']:
                observed[key].append(value)
        for key, typ, topic in [('admitted', msg.ControlSnapshot, 'admitted/snapshot'),
                                ('accepted', msg.AcceptedCommand, 'accepted'),
                                ('status', msg.IngressStatus, 'source_status'), ('events', msg.GateEvent, 'event')]:
            # AcceptedCommand wraps its identity; it must never be produced in
            # either case, but retain any such message for the negative check.
            callback = (lambda value: observed['accepted'].append(value)) if key == 'accepted' else (
                lambda value, key=key: receive(key, value))
            subscriptions.append(node.create_subscription(typ, TOPIC + topic, callback, best))
        subscriptions.append(node.create_subscription(msg.GateEvent, TOPIC + 'event', lambda value: None, best))
        publishers = {key: node.create_publisher(typ, TOPIC + topic, qos) for key, typ, topic, qos in [
            ('source', msg.ControlSnapshot, 'source', best), ('start', msg.LiveStart, 'live_start', reliable),
            ('stop', msg.RuntimeStop, 'stop', reliable), ('ack', msg.ActuatorSend, 'actuator_send', best),
            ('consumed', msg.ExecutionReceipt, 'consumed', best)]}
        def pump(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                require(time.monotonic() < deadline, 'Global software probe timeout')
                if child is not None:
                    require(child.poll() is None, 'Ingress exited unexpectedly')
                rclpy.spin_once(node, timeout_sec=min(.0003, max(0., end-time.monotonic())))
        def until(predicate, label, seconds=.15):
            end = min(deadline, time.monotonic()+seconds)
            while not predicate():
                require(time.monotonic() < end, 'Timeout: ' + label)
                pump(.0003)
        pump(.15)
        require(len(node.get_node_names()) == 1, 'DDS domain is not empty; choose a private domain')
        for label, wrong_session in [('context_rejection', False), ('wrong_session_only', True)]:
            folder = out / label; folder.mkdir()
            for values in observed.values(): values.clear()
            session = int.from_bytes(os.urandom(4), 'little') & 0x3fffffff or 1
            identity['session'] = session; epoch = 1
            digest = hashlib.sha256(('synthetic rejected input ' + uuid.uuid4().hex).encode()).hexdigest()
            ready = folder / 'ingress_ready'
            command = [str(binary), '--ros-args', '-p', 'simulation_live:=true',
                       '-p', f'expected_session_id:={session}', '-p', f'expected_epoch:={epoch}',
                       '-p', f'configuration_sha256:={digest}', '-p', 'position_gains:=[0.95,0.95,1.0]',
                       '-p', 'task_origin_s:=10.0', '-p', f'ready_file:={ready}']
            log = (folder / 'ingress.log').open('wb')
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())
            row = dict(case=label, own_pid=child.pid, session_id=session, epoch=epoch, configuration_sha256=digest)
            report['cases'].append(row)
            until(lambda: ready.exists() and all(p.get_subscription_count() >= 1 for p in publishers.values()),
                  'DDS endpoints matched', 8.)
            start = msg.LiveStart(); start.session_id = session; start.epoch = epoch
            start.configuration_sha256 = list(bytes.fromhex(digest)); start.origin_s = 10.
            start.issued_monotonic_ns = time.monotonic_ns(); publishers['start'].publish(start); pump(.003)
            def snapshot(sequence, task_s):
                s = msg.ControlSnapshot(); s.schema_version = 1; s.session_id = session; s.epoch = epoch
                s.sequence = sequence; s.replay_only = False; s.configuration_sha256 = list(bytes.fromhex(digest))
                s.control_time_s = 10.+task_s; s.task_time_s = task_s; s.dt_s = .007
                s.state_time_s = s.control_time_s-.002; s.joint_time_s = s.control_time_s
                s.position_enu = [.1, -.2, 2.]; s.velocity_enu = [.01, -.02, .03]
                s.omega_body_flu = [.04, -.05, .06]; s.acceleration_enu = [.07, -.08, .09]
                s.rotation_row_major = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                s.q = [.1, .2]; s.dq = [.11, -.12]; s.target_position_enu = [.13, -.14, 1.6]
                s.target_velocity_enu = [.15, -.16, .17]; s.target_acceleration_enu = [.18, -.19, .20]
                s.position_gains = [.95, .95, 1.]; s.reference_time_s = task_s
                s.reference_r = [.21, -.22]; s.bus_origin_s = -.02; s.command_count = 2; s.packet_count = 3
                s.command_time_s = [-.015, -.005] + [0.]*62
                s.command_velocity = [.03, -.04, .05, -.06] + [0.]*124
                s.packet_send_time_s = [-.02, -.015, -.01] + [0.]*61
                for name, values in [('packet_q_ref', [.1, .2, .11, .21, .12, .22]),
                                     ('packet_dq_ref', [.03, -.04, .05, -.06, .07, -.08]),
                                     ('packet_tau_ff', [.001, -.002, .003, -.004, .005, -.006]),
                                     ('packet_kp', [6.]*6), ('packet_kd', [.25]*6)]:
                    setattr(s, name, values+[0.]*122)
                s.issued_monotonic_ns = time.monotonic_ns()
                return s
            first = snapshot(41, .016); publishers['source'].publish(first)
            until(lambda: observed['status'] and observed['status'][-1].source_sequence == 41, 'first state received')
            require(not observed['admitted'], 'Initial 16ms state unexpectedly admitted')
            rejected = snapshot(42, .020); rejected.reference_time_s -= .001
            if wrong_session: rejected.session_id += 1
            publishers['source'].publish(rejected)
            if wrong_session:
                until(lambda: observed['status'] and observed['status'][-1].faulted, 'wrong session rejected before gate')
                require(not observed['events'], 'Wrong-session source reached admission gate')
            else:
                until(lambda: observed['events'], 'CONTEXT event')
                event = observed['events'][0]
                require(not event.accepted and event.reason == 15, 'Fixture did not reach CONTEXT rejection')
                # A later foreign session must not replace the first evidence.
                foreign = snapshot(99, .024); foreign.session_id += 1
                publishers['source'].publish(foreign); pump(.002)
            require(not observed['admitted'] and not observed['accepted'], 'Rejected input was forwarded')
            require(not (folder/'rejected_snapshot.cdr').exists() and not (folder/'rejected_snapshot.json').exists(),
                    'Rejection evidence was written before shutdown')
            child.send_signal(signal.SIGINT); child.wait(timeout=3.); log.close(); log = None
            row['child_returncode'] = child.returncode; require(child.returncode == 0, 'Ingress did not exit cleanly')
            child = None
            if wrong_session:
                require(not (folder/'rejected_snapshot.cdr').exists() and not (folder/'rejected_snapshot.json').exists(),
                        'Foreign-session input contaminated retained evidence')
                row['no_foreign_session_evidence'] = True
            else:
                cdr = (folder/'rejected_snapshot.cdr').read_bytes()
                metadata = json.loads((folder/'rejected_snapshot.json').read_text())
                decoded = deserialize_message(cdr, msg.ControlSnapshot)
                row.update(check_evidence(metadata, decoded, rejected, 10., event, len(cdr)))
                row['payload_sha256'] = hashlib.sha256(cdr).hexdigest()
                (folder/'expected_source.json').write_text(json.dumps(plain(rejected), indent=2)+'\n')
                (folder/'decoded_gate_input.json').write_text(json.dumps(plain(decoded), indent=2)+'\n')
                (folder/'observed_event.json').write_text(json.dumps(plain(event), indent=2)+'\n')
            row['passed'] = True
        require(sha(binary) == args.binary_sha256, 'Binary changed during probe')
        report.update(passed=True, complete=True)
    except BaseException as exc:
        report.update(error=repr(exc), traceback=traceback.format_exc())
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try: child.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                child.terminate()
                try: child.wait(timeout=2.)
                except subprocess.TimeoutExpired: child.kill(); child.wait(timeout=2.)
        if log is not None: log.close()
        if node is not None: node.destroy_node()
        if initialized and rclpy.ok(): rclpy.shutdown()
    (out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({'passed': report['passed'], 'report': str(out/'report.json'), 'error': report.get('error')}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
