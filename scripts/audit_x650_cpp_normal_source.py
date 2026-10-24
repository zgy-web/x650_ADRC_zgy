"""Normal-reference extension of the preserved hold source auditor.

Only audit_source is copied to inject the independent dynamic target. All raw
CDR, selection, frame, reset, actual TS20 history and worker-link helpers are
imported unchanged from the original audit. The old hash-bound file is untouched.
"""
from pathlib import Path
from types import SimpleNamespace
import csv
import json
import math
import numpy as np
from audit_x650_cpp_source import (ROOT, PREFIX_LIMITATION, RAW_NAMES, PrefixReader,
    close_values, identity, load_ros, numeric_rows, require, unused_history_is_zero,
    validate_binding, validate_identity, wire_records)
from simulations.x650_cpp_normal_v1.qualification import figure8, normal_trajectory

def audit_source(evidence, state_csv, packet_csv, reference, extra, *, worker_directory=None,
                 expected_binding=None, decode=None, allow_failed_source=False, trajectory=None):
    """Verify one finalized ordinary-figure-eight source. A failed source cannot produce passed=true.

    reference/extra must come from the original archived bind documents, not be
    synthesized from the later source request. This API also permits the explicit
    known synthetic reference of the software probe; that scope stays separate.
    """
    from simulations.x650_wholebody_mpc_v1.estimated_feedback import Px4EstimatedFeedback
    from x650_wholebody.ts20_runtime import Ts20Runtime
    require(trajectory == normal_trajectory(), 'Exact normal source trajectory required')
    target_sample = None
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
            target_sample = figure8(live.target_position_enu).sample
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
            target_p, target_v, target_a = target_sample(s.task_time_s)
            expected = dict(position_enu=state.position, velocity_enu=state.velocity,
                acceleration_enu=state.acceleration_enu, rotation_row_major=state.rotation.reshape(-1),
                omega_body_flu=state.omega_body, state_time_s=state.stamp_s, joint_time_s=stamp,
                q=state.q, dq=state.dq, reference_r=context['reference_state']['r'],
                reference_time_s=context['reference_state']['time_s'], bus_origin_s=context['bus_origin_s'],
                position_gains=binding.position_gains, target_position_enu=target_p,
                target_velocity_enu=target_v, target_acceleration_enu=target_a)
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
        trajectory_scope='Frozen C2 normal: original world LiveStart centre, same original source time; no phase/position fitting',
        trajectory=trajectory,
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
