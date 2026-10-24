"""Read-only consumption audit for periodic output with asynchronous feedback.

Anonymous Float64 commands are mapped by a per-joint receive counter ONLY under
the declared single-writer, ordered, exactly-once bridge contract. A counter is
the latest callback sampled by PreUpdate, not a motor/bus acknowledgement.
Complete physics rows distinguish adoption from replacement before adoption;
missing ROS observations alone cannot make that distinction. This audit does
not certify the complete flight or infer a PX4 per-setpoint acknowledgement.
"""
from __future__ import annotations

import argparse
import base64
import bisect
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path


def require(value, message):
    if not value:
        raise ValueError(message)


def uint(value):
    require(not isinstance(value, bool), 'Boolean counter/identity')
    # CSV counters are integral decimal strings, never rounded through float.
    if isinstance(value, str):
        require(value.isdecimal(), 'Invalid decimal counter/identity')
        value = int(value)
    require(isinstance(value, int) and 0 <= value < 2**64, 'Invalid uint64 identity')
    return value


def finite(value):
    result = float(value)
    require(math.isfinite(result), 'Nonfinite evidence value')
    return result


def close(a, b, tolerance=1e-9):
    return abs(finite(a)-finite(b)) <= tolerance


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def artifact(path):
    p = Path(path).resolve()
    data = p.read_bytes()
    return {'path': str(p), 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}


def physics_rows(source):
    """Require every physics interval; a gap must never become 'superseded'."""
    result = []
    for index, raw in enumerate(source):
        r = {'index': index, 'sim': finite(raw['sim_s']),
             'begin': finite(raw['interval_begin_s']), 'dt': finite(raw['step_dt_s']),
             'counter': [uint(raw[f'upper_sequence{j}']) for j in (1, 2)],
             'u': [finite(raw[f'u{j}']) for j in (1, 2)],
             'receipt': [finite(raw[f'receipt{j}_s']) for j in (1, 2)]}
        require(0 < r['dt'] <= .02 and close(r['sim']-r['begin'], r['dt']),
                'Invalid physical interval duration')
        if result:
            previous = result[-1]
            require(r['sim'] > previous['sim'] and close(r['begin'], previous['sim']),
                    'Missing/overlapping physics row; cannot infer supersession')
            for j in range(2):
                require(r['counter'][j] >= previous['counter'][j], 'Counter regressed')
                if r['counter'][j] == previous['counter'][j]:
                    require(r['u'][j] == previous['u'][j] and
                            r['receipt'][j] == previous['receipt'][j],
                            'Unchanged counter has changed value or receipt')
                else:
                    require(close(r['receipt'][j], r['begin']),
                            'Adoption receipt does not match PreUpdate boundary')
        require(all(t <= r['begin']+1e-9 for t in r['receipt']), 'Future adoption receipt')
        result.append(r)
    require(result, 'Empty physics evidence')
    return result


def audit_consumption(sends, receipts, source, start, release, publications):
    """Audit immutable materialized messages against all native CSV rows.

    Caller must separately reconcile materialized messages with their original
    CDR/count/hash evidence (the CLI below does this). No deadline is enlarged.
    The result certifies adoption accounting, NOT every send being applied.
    """
    rows = physics_rows(source)
    times = [r['sim'] for r in rows]

    def at_time(stamp):
        stamp = finite(stamp)
        position = bisect.bisect_left(times, stamp-1e-9)
        require(position < len(rows) and close(rows[position]['sim'], stamp),
                'Observation has no exact original physics row')
        return rows[position]

    session = (uint(start['session_id']), uint(start['epoch']))
    require(all(session), 'Missing live identity')
    start_ns = uint(start['issued_monotonic_ns'])
    require(sends, 'Empty send ledger')
    dispatches = [uint(s['dispatch_sequence']) for s in sends]
    require(dispatches == sorted(set(dispatches)), 'Duplicate/reordered send identity')
    send_map = dict(zip(dispatches, sends))
    send_times = [uint(s['sent_monotonic_ns']) for s in sends]
    require(send_times[0] >= start_ns and all(a < b for a, b in zip(send_times, send_times[1:])),
            'Send wall chronology invalid')
    counters = [[uint(v) for v in s['expected_counter']] for s in sends]
    require(all(len(c) == 2 and min(c) > 0 for c in counters), 'Invalid expected counter pair')
    base = [v-1 for v in counters[0]]
    baseline = at_time(sends[0]['sim_s'])
    require(baseline['counter'] == base, 'Initial send baseline differs from original physics observation')
    for i, s in enumerate(sends):
        require((uint(s['session_id']), uint(s['epoch'])) == session, 'Send session/epoch differs')
        require(len(s['joint_velocity']) == 2, 'Invalid joint command shape')
        [finite(v) for v in s['joint_velocity']]
        require(counters[i] == [v+i+1 for v in base], 'Send counter gap/duplicate or another writer')
        if i:
            require(finite(s['sim_s']) >= finite(sends[i-1]['sim_s']), 'Send observation clock regressed')
    zeros = [s for s in sends if s['stop_zero']]
    require(len(zeros) == 1 and zeros[0] is sends[-1] and
            all(finite(v) == 0 for v in zeros[0]['joint_velocity']), 'Stop must be one final zero')

    receipt_ids = [uint(r['dispatch_sequence']) for r in receipts]
    require(len(receipt_ids) == len(set(receipt_ids)) and set(receipt_ids) <= set(dispatches),
            'Duplicate or unknown receipt identity')
    receipt_map = dict(zip(receipt_ids, receipts))
    zero = zeros[0]
    require(zero['dispatch_sequence'] in receipt_map, 'Final zero has no actual observation receipt')
    final_receipt = receipt_map[zero['dispatch_sequence']]
    end_row = at_time(final_receipt['observation_sim_s'])
    require(end_row['index'] >= baseline['index'], 'Final observation predates baseline')
    window = rows[baseline['index']:end_row['index']+1]
    # Map every observed adoption, including intermediate rows omitted by the
    # ROS latest-value reader. Counter jumps are retained, never interpolated.
    events = [{}, {}]
    pair_first = {}
    for r in window:
        pair_first.setdefault(tuple(r['counter']), r)
        for j in range(2):
            count = r['counter'][j]
            require(base[j] <= count <= counters[-1][j], 'Unknown in-epoch counter/other writer')
            if count > base[j]:
                s = sends[count-base[j]-1]
                require(close(r['u'][j], s['joint_velocity'][j], 1e-12),
                        'Adopted velocity differs from mapped send')
                events[j].setdefault(count, r)
    event_counters = [sorted(e) for e in events]
    outcomes = []
    counts = [Counter(), Counter()]
    lag = []
    for i, s in enumerate(sends):
        outcome = {'dispatch_sequence': s['dispatch_sequence'], 'command_sequence': s['command_sequence'],
                   'stop_zero': s['stop_zero'], 'joints': []}
        for j in range(2):
            expected = counters[i][j]
            adopted = events[j].get(expected)
            if adopted is not None:
                delay = adopted['receipt'][j]-finite(s['sim_s'])
                require(-.008 <= delay <= .05, 'Actual adoption outside existing simulation-clock bound')
                lag.append(delay)
                value = {'status': 'adopted', 'counter': expected, 'physics_row': adopted['index'],
                         'adopted_sim_s': adopted['receipt'][j], 'first_record_sim_s': adopted['sim'],
                         'source_observation_to_adoption_s': delay}
            else:
                pos = bisect.bisect_right(event_counters[j], expected)
                require(pos < len(event_counters[j]), 'Unresolved send: no adoption or later counter')
                following = events[j][event_counters[j][pos]]
                require(following['index'] > baseline['index'], 'No complete preceding interval for supersession')
                previous = rows[following['index']-1]
                require(previous['counter'][j] < expected < following['counter'][j],
                        'Supersession is not proven by consecutive physical intervals')
                value = {'status': 'superseded_before_physics_adoption', 'counter': expected,
                         'preceding_physics_row': previous['index'], 'following_physics_row': following['index'],
                         'following_counter': following['counter'][j],
                         'following_adopted_sim_s': following['receipt'][j]}
            counts[j][value['status']] += 1
            outcome['joints'].append(value)
        pair = pair_first.get(tuple(counters[i]))
        outcome['both_joint_values_observed_together'] = pair is not None
        outcome['same_physics_boundary_adoption'] = (pair is not None and
            close(pair['receipt'][0], pair['receipt'][1]))
        outcome['receipt_observed'] = s['dispatch_sequence'] in receipt_map
        outcomes.append(outcome)

    for r in receipts:
        s = send_map[r['dispatch_sequence']]
        require((uint(r['session_id']), uint(r['epoch'])) == session and
                uint(r['command_sequence']) == uint(s['command_sequence']), 'Receipt identity mismatch')
        require(r['joint_consumed'] is True and r['failed'] is False and
                r['pva_send_returned'] is (not s['stop_zero']), 'Receipt semantics invalid')
        observed = at_time(r['observation_sim_s'])
        require([uint(v) for v in r['consumed_counter']] == list(s['expected_counter']) == observed['counter'],
                'Receipt does not represent one physical row with both exact counters')
        require(len(r['consumed_sim_s']) == 2 and all(close(r['consumed_sim_s'][j], observed['receipt'][j])
                    and close(s['joint_velocity'][j], observed['u'][j], 1e-12) for j in range(2)),
                'Receipt differs from original adopted velocity/time')
        require(uint(r['observed_monotonic_ns']) >= uint(s['sent_monotonic_ns']), 'Receipt wall clock precedes send')
        require(observed['index'] <= end_row['index'], 'Receipt follows final zero observation')
    require(all(x['status'] == 'adopted' for x in outcomes[-1]['joints']), 'Zero was not adopted by both joints')
    require(release['zero_consumed'] is True and release['writer_exited'] is True,
            'Zero adoption/exclusive writer release unverified')
    release_ns = uint(release['wall_monotonic_ns'])
    require(release_ns >= uint(final_receipt['observed_monotonic_ns']), 'Ownership released before zero observation')
    for e in publications:
        if 'publish_before' in e:
            wall = e['publish_before'].get('wall_monotonic_s')
            require(wall is not None, 'Legacy publication lacks wall-clock evidence')
            require(not start_ns <= finite(wall)*1e9 < release_ns, 'Overlapping legacy and isolated output owners')
    return {'passed': True, 'kind': 'streaming_consumption_accounting_v1',
            'scope': 'physics-boundary adoption accounting; not motor ACK, bus-application or flight qualification',
            'complete_physical_hold_qualified': False,
            'mapping_assumptions': ['exclusive writer during epoch', 'ordered exactly-once per-joint bridge delivery',
                                    'initial legacy publications drained before the frozen send baseline'],
            'mapping_limit': 'Float64 has no wire identity; assumptions are not proven by counters alone.',
            'full_physics_rows': len(rows), 'epoch_physics_rows': len(window), 'initial_counter': base,
            'send_count': len(sends), 'observed_receipt_count': len(receipts),
            'joint_counts': [dict(c) for c in counts],
            'both_joint_values_observed_together': sum(x['both_joint_values_observed_together'] for x in outcomes),
            'same_physics_boundary_adoptions': sum(x['same_physics_boundary_adoption'] for x in outcomes),
            'all_sends_applied': all(c['adopted'] == len(sends) for c in counts),
            'zero_consumed': True, 'no_overlapping_legacy_publications': True,
            'adoption_lag_sim_s': {'min': min(lag), 'max': max(lag)},
            'lag_clock_note': 'Plugin adoption minus latest plugin observation at send; not wall transit latency.',
            'sends': outcomes}


def audit_async_ledger(ledger, baseline, sends, receipts, source, start, release):
    """Reconcile the writer's bounded online ledger with independent physics.

    Status 2 records the *later* observed value that proved progress, not the
    skipped command's adoption. This check retains the fixed 40 ms unconfirmed
    progress budget separately from the 25 ms actual-state observation age.
    """
    require(ledger['schema'] == 'latest_value_ledger_v1' and ledger['complete'] is True and
            ledger['overflow'] is False and ledger['consumption_policy'] == 'latest_value_v1' and
            ledger['consumption_progress_budget_ms'] == 40 and ledger['capacity'] == 16 and
            ledger['have_epoch_baseline'] is True, 'Streaming ledger incomplete/wrong policy/overflow')
    rows = physics_rows(source)
    times = [r['sim'] for r in rows]

    def row_at(t):
        pos = bisect.bisect_left(times, finite(t)-1e-9)
        require(pos < len(rows) and close(rows[pos]['sim'], t), 'Ledger observation absent in physics')
        return rows[pos]

    require((baseline['session_id'], baseline['epoch']) == (start['session_id'], start['epoch']),
            'Initial actuator CDR identity differs')
    initial = row_at(baseline['sim_s'])
    require(initial['counter'] == list(baseline['consumed_counter']) == ledger['initial_counter'] ==
            [v-1 for v in sends[0]['expected_counter']] and close(baseline['sim_s'], sends[0]['sim_s']),
            'Initial actuator CDR does not establish send counter baseline')
    require(all(close(baseline['consumed_velocity'][j], initial['u'][j], 1e-12) and
                close(baseline['consumed_sim_s'][j], initial['receipt'][j]) for j in range(2)),
            'Initial actuator CDR differs from actual native state')
    observed = uint(baseline['observed_monotonic_ns'])
    received = uint(ledger['baseline_received_ns'])
    require(uint(start['issued_monotonic_ns']) <= observed <= received <= sends[0]['sent_monotonic_ns'] and
            sends[0]['sent_monotonic_ns']-observed <= 25_000_000, 'Initial actuator state stale or predates epoch')
    entries = ledger['rows']
    require([e['dispatch_sequence'] for e in entries] == [s['dispatch_sequence'] for s in sends],
            'Online ledger does not cover every actual send exactly once')
    receipt_map = {r['dispatch_sequence']: r for r in receipts}
    progress_ns = []
    stop_ns = []
    for e, s in zip(entries, sends):
        require(e['command_sequence'] == s['command_sequence'] and e['sent_ns'] == s['sent_monotonic_ns'] and
                e['expected_counter'] == list(s['expected_counter']) and e['stop_zero'] is s['stop_zero'] and
                close(e['sim_s'], s['sim_s']) and all(close(e['velocity'][j], s['joint_velocity'][j], 1e-12)
                    for j in range(2)), 'Online ledger entry differs from actual send CDR')
        for key in ('joint_status','consumed_sim_s','observation_sim_s','observed_ns','received_ns'):
            require(len(e[key]) == 2, 'Invalid online ledger field shape')
        for j in range(2):
            status = e['joint_status'][j]
            require(status in (1,2), 'Online ledger joint remains unconfirmed')
            raw = row_at(e['observation_sim_s'][j])
            require((raw['counter'][j] == e['expected_counter'][j] if status == 1 else
                     raw['counter'][j] > e['expected_counter'][j]), 'Online ledger status contradicts actual counter')
            require(close(raw['receipt'][j], e['consumed_sim_s'][j]),
                    'Online ledger progress timestamp differs from original physics row')
            obs, recv = uint(e['observed_ns'][j]), uint(e['received_ns'][j])
            require(s['sent_monotonic_ns'] <= recv <= release['wall_monotonic_ns'] and
                    0 < obs <= recv and recv-obs <= 25_000_000,
                    'Online ledger state freshness/receipt chronology invalid')
            delay = recv-s['sent_monotonic_ns']
            if not s['stop_zero']:
                require(delay <= 40_000_000, 'Per-joint unconfirmed progress exceeded fixed 40 ms budget')
                progress_ns.append(delay)
            else:
                stop_ns.append(delay)
        require(e['paired'] is (s['dispatch_sequence'] in receipt_map),
                'Online paired observation and original receipt set differ')
        if e['paired']:
            r = receipt_map[s['dispatch_sequence']]
            require(e['joint_status'] == [1,1] and close(r['observation_sim_s'], max(e['observation_sim_s'])) and
                    all(close(r['consumed_sim_s'][j], e['consumed_sim_s'][j]) for j in range(2)) and
                    uint(r['observed_monotonic_ns']) >= max(e['received_ns']),
                    'Paired receipt differs from the online observation that generated it')
    return {'passed': True, 'original_baseline_cdr_verified': True, 'rows': len(entries),
            'progress_budget_ms': 40, 'actual_state_age_limit_ms': 25,
            'per_joint_progress_observation_ms': {'min': min(progress_ns)/1e6 if progress_ns else None,
                                                'max': max(progress_ns)/1e6 if progress_ns else None},
            'stop_progress_observation_ms': [v/1e6 for v in stop_ns],
            'stop_limit_scope': 'Existing release timeout is separate; the tracking progress watchdog stops after STOP.',
            'ignored_old_states': uint(ledger['ignored_old_states']),
            'scope': 'online observation/progress bookkeeping; status 2 is not an adoption acknowledgement'}


def original_messages(live):
    """Re-read original CDR; derived JSON alone cannot certify the ledger."""
    from flyarm_control_interfaces import msg
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    sender = read(live/'source_sender.json')
    sink = read(live/'source_recording/manifest.json')
    raw = live/'source_recording/diagnostics.jsonl'
    require(sender['complete'] and sender['dropped'] == 0 and
            sender['attempts'] == sender['sent'] == sink['rows'] == sink['last_sequence'] and
            not sink['gaps'] and sink['error'] is None and sink['drained_and_fsynced'] and
            artifact(raw)['sha256'] == sink['records_sha256'], 'Original CDR recorder incomplete/corrupt')
    classes = {'dispatch': msg.DispatchTick, 'sends': msg.ActuatorSend,
               'receipts': msg.ExecutionReceipt, 'events': msg.GateEvent}
    output = {key: [] for key in classes}
    count = 0
    with raw.open(encoding='utf-8') as stream:
        for count, line in enumerate(stream, 1):
            row = json.loads(line)
            require(row['sequence'] == count and row['kind'] == 'ros' and row['topic'] in classes,
                    'Missing/unexpected original lifecycle CDR')
            name = row['topic']
            payload = base64.b64decode(row['cdr']['base64'], validate=True)
            output[name].append(message_to_ordereddict(deserialize_message(payload, classes[name])))
    require(count == sink['rows'], 'Original CDR row count differs')
    for key, values in output.items():
        require(read(live/(key+'.json')) == values, 'Derived view differs from original CDR: '+key)
    require([t['sequence'] for t in output['dispatch']] == list(range(1,len(output['dispatch'])+1)),
            'Original dispatch DDS observations missing or reordered')
    valid = [t for t in output['dispatch'] if t['output_valid']]
    require([t['sequence'] for t in valid] == [s['dispatch_sequence'] for s in output['sends'] if not s['stop_zero']],
            'Valid output ticks and send ledger differ')
    return output


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    require(not args.output.exists(), 'Refusing to overwrite evidence')
    live = args.run/'isolated_live'
    report = {'passed': False, 'kind': 'streaming_consumption_accounting_v1',
              'complete_physical_hold_qualified': False}
    paths = [live/'source_sender.json', live/'source_recording/manifest.json',
             live/'source_recording/diagnostics.jsonl', live/'live_start.json', live/'lifecycle.json',
             live/'actuator_epoch_state.cdr', live/'actuator_ledger.json',
             args.run/'ts20_state.csv', args.run/'control_execution_trace.jsonl']
    try:
        messages = original_messages(live)
        with (args.run/'ts20_state.csv').open(newline='', encoding='utf-8') as stream:
            source = list(csv.DictReader(stream))
        publications = [json.loads(line) for line in (args.run/'control_execution_trace.jsonl').read_text().splitlines() if line.strip()]
        report = audit_consumption(messages['sends'], messages['receipts'], source,
            read(live/'live_start.json'), read(live/'lifecycle.json')['release'], publications)
        from flyarm_control_interfaces.msg import ActuatorState
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        baseline = message_to_ordereddict(deserialize_message((live/'actuator_epoch_state.cdr').read_bytes(),ActuatorState))
        report['online_ledger'] = audit_async_ledger(read(live/'actuator_ledger.json'), baseline,
            messages['sends'], messages['receipts'], source,
            read(live/'live_start.json'), read(live/'lifecycle.json')['release'])
        report['original_cdr_reconciled'] = True
    except Exception as exc:
        report['passed'] = False
        report['error'] = str(exc)
    report['inputs'] = [artifact(path) for path in paths if path.is_file()]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'passed': report['passed'], 'report': str(args.output), 'error': report.get('error')}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
