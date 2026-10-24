"""Offline audit of original C++ source CDR, selected CSV state, and worker input.

This is evidence verification, not an online controller or a flight certificate.
The unchanged Python feedback/runtime are independent numerical oracles; their
offline execution time has no 50 ms deployment deadline. CSV visibility at each
poll was not recorded by source v1. Accordingly this checks the recorded selected
prefix and actual causal contents, not whether a later-flushed row was readable.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'control_packages/x650_wholebody_v1'))
HEADER = struct.Struct('<IQQdI')
RAW_NAMES = {1: 'position', 2: 'attitude', 3: 'angular_velocity',
             4: 'estimator_flags', 5: 'clock'}
PREFIX_LIMITATION = ('The original source did not record per-poll CSV byte offsets. '
    'This audit reconstructs the prefix ending at the recorded selected joint row '
    'and last actual MIT packet. It proves original values and causal history, '
    'but does not prove that no newer complete row was file-visible at that poll.')


def require(condition, label):
    if not condition:
        raise ValueError(label)


def identity(path):
    path = Path(path).resolve()
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return {'path': str(path), 'bytes': path.stat().st_size, 'sha256': digest.hexdigest()}


def load_ros():
    from rclpy.serialization import deserialize_message
    from px4_msgs.msg import (VehicleLocalPosition, VehicleAttitude,
                             VehicleAngularVelocity, EstimatorStatusFlags)
    from rosgraph_msgs.msg import Clock
    from flyarm_control_interfaces.msg import (LiveStart, ControlSnapshot,
                                               RuntimeStop, SourceBinding, SourceBindingAck)
    classes = dict(enumerate((VehicleLocalPosition, VehicleAttitude,
        VehicleAngularVelocity, EstimatorStatusFlags, Clock, LiveStart,
        ControlSnapshot, RuntimeStop, SourceBinding, SourceBindingAck), start=1))
    return lambda kind, raw: deserialize_message(raw, classes[kind])


def wire_records(path, decode):
    """Read the bounded original wire container, never unpickle diagnostics."""
    with Path(path).open('rb') as stream:
        require(stream.read(8) == b'FSCdr001', 'Invalid C++ source CDR magic')
        prior = 0
        while True:
            raw_header = stream.read(HEADER.size)
            if not raw_header:
                break
            require(len(raw_header) == HEADER.size, 'Truncated source wire header')
            kind, wall, sequence, sim, count = HEADER.unpack(raw_header)
            require(1 <= kind <= 10 and 0 < count <= (1 << 20), 'Invalid source wire kind/size')
            require(wall >= prior and wall > 0, 'Source wire monotonic clock regressed')
            raw = stream.read(count)
            require(len(raw) == count, 'Truncated original source CDR')
            prior = wall
            yield SimpleNamespace(kind=kind, wall=wall, sequence=sequence, sim=sim,
                                  message=decode(kind, raw))


def numeric_rows(path):
    with Path(path).open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        require(reader.fieldnames and len(set(reader.fieldnames)) == len(reader.fieldnames),
                'Missing/duplicate CSV header')
        result = []
        for row in reader:
            require(None not in row and all(v is not None for v in row.values()), 'Malformed CSV row')
            parsed = {key: float(value) for key, value in row.items()}
            require(all(math.isfinite(v) for v in parsed.values()), 'Nonfinite CSV evidence')
            result.append(parsed)
    require(result, 'Empty physical CSV evidence')
    return result


class PrefixReader:
    """Offline visibility reconstruction; never used in the deployed pipeline."""
    def __init__(self, rows, key):
        self.rows, self.key = rows, key
        self.times = [row[key] for row in rows]
        require(all(a < b for a, b in zip(self.times, self.times[1:])), 'CSV time not increasing')
        self.index = 0
        self.end = 0

    def select_end(self, at):
        index = bisect.bisect_left(self.times, at)
        candidates = [i for i in (index - 1, index) if 0 <= i < len(self.times)]
        best = min(candidates, key=lambda i: abs(self.times[i] - at))
        require(abs(self.times[best] - at) <= 1e-10, 'Recorded prefix endpoint absent from actual CSV')
        require(best + 1 >= self.end, 'Recorded selected CSV prefix regressed')
        self.end = best + 1

    def read(self):
        result = self.rows[self.index:self.end]
        self.index = self.end
        return result


def close_values(actual, expected, label, maxima, tolerance=1e-12):
    actual, expected = np.asarray(actual), np.asarray(expected)
    require(actual.shape == expected.shape, label + ': shape differs')
    require(np.isfinite(actual).all() and np.isfinite(expected).all(), label + ': nonfinite value')
    error = float(np.max(np.abs(actual - expected))) if actual.size else 0.
    maxima[label] = max(maxima.get(label, 0.), error)
    require(error <= tolerance, label + ': differs by ' + repr(error))


def validate_binding(message, reference, extra, final):
    require(message.schema_version == 1 and message.binding_sequence == 1, 'Invalid binding schema/sequence')
    for key in ('px4_source_contract_commit', 'frame_mode', 'target_frozen', 'fit_estimator_translation'):
        require(getattr(message, key) == reference[key], 'Original binding differs: ' + key)
    require(reference['frame_mode'] == 'raw_ev_world' and reference['target_frozen'] is True
            and reference['fit_estimator_translation'] is False, 'Reference was fitted or not frozen raw EV')
    for key in ('ref_lat', 'ref_lon', 'ref_alt', 'ref_timestamp', 'xy_global', 'z_global',
                'xy_reset_counter', 'z_reset_counter', 'heading_reset_counter'):
        require(getattr(message, key) == reference['px4_reference_at_bind'][key],
                'Source did not bind original reference: ' + key)
    for key in ('vxy_reset_counter', 'vz_reset_counter', 'quat_reset_counter'):
        require(getattr(message, key) == extra[key], 'Original extra reset differs: ' + key)
    validate_identity(message, final)
    require(np.isfinite(message.position_gains).all() and math.isfinite(message.bus_period_s)
            and message.bus_period_s > 0 and math.isfinite(message.delay_s) and message.delay_s >= 0,
            'Invalid binding gains/bus/delay')


def validate_identity(message, final):
    require(message.session_id == final['session_id'] and message.epoch == final['epoch'],
            'Source session/epoch identity differs')
    if hasattr(message, 'configuration_sha256'):
        require(bytes(message.configuration_sha256).hex() == final['configuration_sha256'],
                'Source configuration hash differs')


def unused_history_is_zero(s):
    require(1 <= s.command_count <= 64 and 1 <= s.packet_count <= 64, 'Invalid history count')
    for name, count, width in [('command_time_s', s.command_count, 1),
            ('command_velocity', s.command_count, 2), ('packet_send_time_s', s.packet_count, 1),
            *[(name, s.packet_count, 2) for name in ('packet_q_ref', 'packet_dq_ref',
                                                   'packet_tau_ff', 'packet_kp', 'packet_kd')]]:
        require(all(value == 0. for value in getattr(s, name)[count * width:]),
                'Nonzero unused source history: ' + name)


def audit_source(evidence, state_csv, packet_csv, reference, extra, *, worker_directory=None,
                 expected_binding=None, decode=None, allow_failed_source=False):
    """Verify one finalized hold source. A failed source cannot produce passed=true.

    reference/extra must come from the original archived bind documents, not be
    synthesized from the later source request. This API also permits the explicit
    known synthetic reference of the software probe; that scope stays separate.
    """
    from simulations.x650_wholebody_mpc_v1.estimated_feedback import Px4EstimatedFeedback
    from x650_wholebody.ts20_runtime import Ts20Runtime
    evidence = Path(evidence)
    decode = decode or load_ros()
    final = json.loads((evidence / 'source_final.json').read_text())
    require(final['schema_version'] == 1 and final['source_kind'] == 'cpp_raw_px4_ts20_v1',
            'Unsupported source evidence schema')
    require(final['session_id'] > 0 and final['epoch'] > 0 and len(final['configuration_sha256']) == 64,
            'Invalid final source identity')
    require(final['hardware_qualified'] is False, 'Unexpected hardware qualification claim')
    if expected_binding is not None:
        require(final['configuration_sha256'] == expected_binding['configuration_sha256'],
                'Source configuration differs from separately bound worker startup')
    require(final['ready'] and final['bound'] and final['started'], 'Source did not complete actual handoff')
    rows = numeric_rows(state_csv)
    packets = numeric_rows(packet_csv)
    state_reader, packet_reader = PrefixReader(rows, 'sim_s'), PrefixReader(packets, 'send_s')
    with (evidence / 'source_selections.csv').open(newline='') as stream:
        selections = list(csv.DictReader(stream))
    require(len(selections) == final['snapshots'] and selections, 'Selection/source count differs or empty')
    selected = {int(row['sequence']): row for row in selections}
    require(list(selected) == list(range(1, len(selections) + 1)), 'Selection sequences not exact and unique')
    node = SimpleNamespace(data={}, received={}, received_sim={}, callback_sequences={},
        estimator_feedback_enabled=True, estimator_feedback_error=None, estimator_clock_s=math.nan)
    feedback = runtime = binding = live = ack = None
    source_by_issue = {}
    record_count = 0
    maxima, timing = {}, []
    snapshots = bindings = acks = starts = stops = 0
    previous_control = -math.inf
    previous_generation = None
    raw_counts = {name: 0 for name in RAW_NAMES.values()}
    for record in wire_records(evidence / 'source_wire.cdr', decode):
        record_count += 1
        message = record.message
        if record.kind in RAW_NAMES:
            name = RAW_NAMES[record.kind]
            raw_counts[name] += 1
            require(record.sequence == raw_counts[name], 'Original raw callback sequence has a gap: ' + name)
            if name == 'clock':
                require(message.clock.sec >= 0 and 0 <= message.clock.nanosec <= 999999999,
                        'Invalid original clock fields')
                now = message.clock.sec + message.clock.nanosec * 1e-9
                require(not math.isfinite(node.estimator_clock_s) or now >= node.estimator_clock_s - 1e-9,
                        'Original raw clock regressed')
                node.estimator_clock_s = now
            require((math.isnan(record.sim) and math.isnan(node.estimator_clock_s))
                    or record.sim == node.estimator_clock_s, 'Raw callback receipt clock differs')
            node.data[name] = message
            node.received[name] = record.wall * 1e-9
            node.received_sim[name] = record.sim
            node.callback_sequences[name] = record.sequence
            continue
        validate_identity(message, final)
        require(math.isfinite(record.sim) and record.sim == node.estimator_clock_s,
                'Handoff/selection differs from held original clock')
        if record.kind == 9:
            bindings += 1
            require(bindings == 1 and not starts, 'Repeated or post-LiveStart source binding')
            validate_binding(message, reference, extra, final)
            require(record.sequence == message.binding_sequence, 'Binding record sequence differs')
            if expected_binding is not None:
                for name in ('position_gains', 'bus_period_s', 'delay_s'):
                    require(np.array_equal(getattr(message, name), expected_binding[name]),
                            'Original binding differs from frozen startup: ' + name)
            require(0 < message.issued_monotonic_ns <= record.wall
                    <= message.issued_monotonic_ns + 25_000_000, 'Original binding exceeded issue-age gate')
            require(Px4EstimatedFeedback._reset_snapshot(node.data) == extra,
                    'Actual raw resets differ at original source binding')
            binding = message
            feedback = Px4EstimatedFeedback(reference, node, require_acceleration=True,
                require_prediction_timing=True, joint_source_kind='ts20_physical_q_endpoint_dq_v1')
            runtime = Ts20Runtime(state_csv, packet_csv,
                SimpleNamespace(bus_period_s=message.bus_period_s, delay_s=message.delay_s))
            runtime.states, runtime.packets = state_reader, packet_reader
        elif record.kind == 10:
            acks += 1
            require(binding is not None and acks == 1 and not starts and message.accepted
                    and message.rejection_code == 0 and message.binding_sequence == record.sequence == 1,
                    'Missing, rejected, repeated or late source binding ACK')
            require(message.binding_issued_monotonic_ns == binding.issued_monotonic_ns
                    and binding.issued_monotonic_ns <= message.observed_monotonic_ns <= record.wall
                    and message.observed_monotonic_ns - binding.issued_monotonic_ns <= 25_000_000,
                    'Binding ACK identity/age differs')
            ack = message
        elif record.kind == 6:
            starts += 1
            require(starts == 1 and ack is not None and snapshots == 0, 'Invalid LiveStart order/count')
            require(0 < message.issued_monotonic_ns <= record.wall
                    <= message.issued_monotonic_ns + 25_000_000, 'LiveStart exceeded issue-age gate')
            require(math.isfinite(message.origin_s) and message.origin_s >= 0
                    and abs(message.ref_alt - binding.ref_alt) <= 1e-6, 'LiveStart original origin differs')
            live = message
            previous_control = live.origin_s
        elif record.kind == 8:
            stops += 1
        elif record.kind == 7:
            require(live is not None and stops == 0, 'Snapshot before handoff or after stop')
            snapshots += 1
            s = message
            require(s.schema_version == 1 and not s.replay_only and s.sequence == snapshots
                    and record.sequence == snapshots, 'Source snapshot identity/order differs')
            require(s.issued_monotonic_ns not in source_by_issue, 'Repeated source issue identity')
            require(s.control_time_s == record.sim and s.control_time_s > previous_control,
                    'Source snapshot clock did not strictly advance')
            require(s.task_time_s == s.control_time_s - live.origin_s and s.dt_s == .02,
                    'Source task time/default dt differs')
            unused_history_is_zero(s)
            trace = selected[s.sequence]
            require(int(trace['issued_ns']) == s.issued_monotonic_ns
                    and int(trace['select_end_ns']) == record.wall
                    and 0 < s.issued_monotonic_ns <= record.wall <= int(trace['publish_end_ns']),
                    'Source issue/select/publish clock linkage differs')
            for field, value in [('control_s', s.control_time_s), ('joint_s', s.joint_time_s),
                                  ('state_s', s.state_time_s)]:
                require(float(trace[field]) == value, 'Source trace differs: ' + field)
            require(int(trace['command_count']) == s.command_count and int(trace['packet_count']) == s.packet_count,
                    'Source trace context counts differ')
            generation = int(trace['generation'])
            require(generation > 0 and (previous_generation is None or generation == previous_generation + 1),
                    'Source observation generation repeated or skipped')
            previous_generation = generation
            for name, field in [('position', 'position_sequence'), ('attitude', 'attitude_sequence'),
                ('angular_velocity', 'angular_sequence'), ('estimator_flags', 'flags_sequence'), ('clock', 'clock_sequence')]:
                require(int(trace[field]) == node.callback_sequences[name], 'Selected raw sequence differs: ' + name)
                require(node.received[name] <= s.issued_monotonic_ns * 1e-9,
                        'Snapshot used a callback observed after source issue: ' + name)
            state_reader.select_end(s.joint_time_s)
            packet_reader.select_end(s.packet_send_time_s[s.packet_count - 1] + live.origin_s)
            observation = runtime.select_joint_observation(s.control_time_s)
            state, stamp, _ = feedback.read(node, s.control_time_s,
                wall_now=s.issued_monotonic_ns * 1e-9, joint_observation=observation)
            state, _ = runtime.measured_state(state, observation=observation)
            context = runtime.context(state, s.task_time_s)
            expected = dict(position_enu=state.position, velocity_enu=state.velocity,
                acceleration_enu=state.acceleration_enu, rotation_row_major=state.rotation.reshape(-1),
                omega_body_flu=state.omega_body, state_time_s=state.stamp_s, joint_time_s=stamp,
                q=state.q, dq=state.dq, reference_r=context['reference_state']['r'],
                reference_time_s=context['reference_state']['time_s'], bus_origin_s=context['bus_origin_s'],
                position_gains=binding.position_gains, target_position_enu=live.target_position_enu,
                target_velocity_enu=[0., 0., 0.], target_acceleration_enu=[0., 0., 0.])
            for name, expected_value in expected.items():
                close_values(getattr(s, name), expected_value, name, maxima)
            expected_commands = [[when, *value] for when, value in context['command_history']]
            actual_commands = [[s.command_time_s[i], *s.command_velocity[i * 2:i * 2 + 2]]
                               for i in range(s.command_count)]
            close_values(actual_commands, expected_commands, 'actual_consumed_commands', maxima)
            actual_packets, expected_packets = [], []
            names = ('q_ref', 'dq_ref', 'tau_ff', 'kp', 'kd')
            for i in range(s.packet_count):
                actual_packets.append([s.packet_send_time_s[i], *[v for name in names
                    for v in getattr(s, 'packet_' + name)[i * 2:i * 2 + 2]]])
            for p in context['packet_history']:
                expected_packets.append([p['send_time_s'], *[v for name in names for v in p[name]]])
            close_values(actual_packets, expected_packets, 'actual_mit_packets', maxima)
            source_by_issue[s.issued_monotonic_ns] = s
            previous_control = s.control_time_s
            timing.append((int(trace['publish_end_ns']) - s.issued_monotonic_ns) / 1e6)
    require(record_count == final['wire_records'] and snapshots == final['snapshots'],
            'Original source wire/final count differs')
    require(bindings == acks == starts == 1 and snapshots > 0, 'Incomplete source handoff/evidence')
    require(state_reader.end <= final['state_rows_read'] <= len(rows)
            and packet_reader.end <= final['packet_rows_read'] <= len(packets),
            'Source final CSV row counts inconsistent with selected prefixes/full originals')
    worker_count = 0
    workers = []
    worker_evidence = []
    if worker_directory is not None:
        worker_directory = Path(worker_directory)
        workers = sorted(worker_directory.glob('*.snapshot.cdr'))
        require(workers, 'No original worker input CDR')
        for name in ('diagnostic_sender.json', 'worker_timing.json', 'worker_live_start.cdr'):
            require((worker_directory.parent / name).is_file(), 'Missing complete worker evidence: ' + name)
            worker_evidence.append(worker_directory.parent / name)
        sender = json.loads(worker_evidence[0].read_text())
        require(sender['complete'] is True and sender['failed'] is False and sender['dropped'] == 0
                and sender['offered'] == sender['written'] == len(workers),
                'Worker CDR files do not cover complete original recorder output')
        phases = json.loads(worker_evidence[1].read_text())
        require(phases['complete'] is True and phases['dropped'] == 0 and len(phases['rows']) == len(workers),
                'Worker phase rows do not cover complete original input records')
        require(len(set(phases['columns'])) == len(phases['columns']), 'Duplicate worker phase columns')
        phase_rows = [dict(zip(phases['columns'], row)) for row in phases['rows']]
        require(all(len(row) == len(phases['columns']) for row in phases['rows']), 'Incomplete worker phase row')
        worker_live = decode(6, worker_evidence[2].read_bytes())
        for name in live.get_fields_and_field_types():
            require(np.array_equal(getattr(live, name), getattr(worker_live, name)),
                    'Worker received a different actual LiveStart: ' + name)
        previous = live.origin_s
        used = set()
        for worker_count, path in enumerate(workers, start=1):
            message = decode(7, path.read_bytes())
            phase = phase_rows[worker_count - 1]
            require(phase['sequence'] == worker_count and phase['snapshot_issued_ns'] == message.issued_monotonic_ns
                    and phase['callback_entry_ns'] >= message.issued_monotonic_ns and phase['record_offered'] == 1,
                    'Worker phase/input issue identity differs')
            record_path = path.with_name(path.name.removesuffix('.snapshot.cdr') + '.json')
            record = json.loads(record_path.read_text())
            expected_identity = record['snapshot_cdr']
            actual_identity = identity(path)
            require(record['sequence'] == worker_count and all(expected_identity[key] == actual_identity[key]
                    for key in ('bytes', 'sha256')), 'Worker original CDR digest/sequence differs from call record')
            worker_evidence.append(record_path)
            validate_identity(message, final)
            require(message.sequence == worker_count and message.issued_monotonic_ns not in used,
                    'Worker admission sequence repeated or has a gap')
            require(message.issued_monotonic_ns in source_by_issue, 'Worker received input absent from original source')
            source = source_by_issue[message.issued_monotonic_ns]
            for name in message.get_fields_and_field_types():
                if name not in ('sequence', 'dt_s'):
                    require(np.array_equal(getattr(source, name), getattr(message, name)),
                            'Worker input mutated source field: ' + name)
            require(message.control_time_s > previous and message.dt_s == message.control_time_s - previous,
                    'Worker admitted dt is not actual control-clock delta')
            previous = message.control_time_s
            used.add(message.issued_monotonic_ns)
    clean = final['failure'] == '' and final['stopped'] is True and stops > 0
    result = dict(kind='cpp_raw_source_evidence_audit_v1', record_integrity_passed=True,
        passed=clean, clean_source_stop=clean, complete_physical_hold_qualified=False,
        hardware_qualified=False, source_failure=final['failure'], original_wire_records=record_count,
        source_snapshots=snapshots, original_worker_snapshots_verified=worker_count,
        worker_link_checked=worker_directory is not None, raw_callback_counts=raw_counts,
        maximum_absolute_error=maxima, issue_to_publish_ms=dict(min=min(timing), max=max(timing)),
        trajectory_scope='hold only; nonzero trajectory velocity/acceleration rejected',
        csv_visibility_limitation=PREFIX_LIMITATION,
        offline_oracle_timing='Not a deployment deadline; no GC/deadline policy changed',
        scope='Original source selection/frames/actual-history/worker-input evidence; not full physical hold or safety certification',
        artifacts=[identity(p) for p in (evidence / 'source_final.json', evidence / 'source_wire.cdr',
            evidence / 'source_selections.csv', Path(state_csv), Path(packet_csv), Path(__file__),
            ROOT / 'simulations/x650_wholebody_mpc_v1/estimated_feedback.py',
            ROOT / 'control_packages/x650_wholebody_v1/x650_wholebody/ts20_runtime.py')],
        worker_input_identities=[identity(p) for p in workers],
        worker_recording_identities=[identity(p) for p in worker_evidence])
    if not clean and not allow_failed_source:
        raise ValueError('Source evidence values verified, but source did not stop cleanly: ' + final['failure'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-failed-source', action='store_true',
                        help='Postmortem only; never changes passed=false for failed source')
    args = parser.parse_args()
    require(not args.output.exists(), 'Refusing to overwrite an audit report')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    reference_path, feedback_path = args.run / 'hover_reference.json', args.run / 'estimated_feedback.json'
    startup_path = args.run / 'isolated_live/worker_startup.json'
    try:
        startup = json.loads(startup_path.read_text())
        cfg = startup['startup_manifest']['config']
        expected = dict(configuration_sha256=startup['configuration_sha256'],
                        position_gains=cfg['base_position_gain'], bus_period_s=cfg['bus_period'],
                        delay_s=cfg['motion'][4])
        report = audit_source(args.run / 'isolated_live/cpp_source', args.run / 'ts20_state.csv',
            args.run / 'ts20_packets.csv', json.loads(reference_path.read_text()),
            json.loads(feedback_path.read_text())['extra_reset_counters_at_bind'],
            worker_directory=args.run / 'isolated_live/native_records', expected_binding=expected,
            allow_failed_source=args.allow_failed_source)
        report['original_binding_documents'] = [identity(reference_path), identity(feedback_path), identity(startup_path)]
    except Exception as error:
        report = dict(kind='cpp_raw_source_evidence_audit_v1', passed=False,
                      record_integrity_passed=False, error=repr(error), run=str(args.run.resolve()),
                      complete_physical_hold_qualified=False, hardware_qualified=False)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({key: report[key] for key in ('passed', 'record_integrity_passed')
                      if key in report} | {'report': str(args.output)}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
