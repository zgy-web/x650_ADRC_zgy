"""Read-only hold22 physical-consumption and publication review; no ROS node."""
import csv
from collections import Counter
import hashlib
import json
from pathlib import Path

ROOT = Path('/root/autodl-tmp/safe-aerial-manipulator-testbed/runs/x650_isolated_live_v1_20261024_120535_11859')
sources = {}


def raw(name):
    path = ROOT / name
    data = path.read_bytes()
    sources[name] = {'source_path': str(path), 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
    return data


def read(name):
    return json.loads(raw(name))


live = lambda name: read('isolated_live/' + name + '.json')
sends, receipts, events, start = [live(n) for n in ['sends', 'receipts', 'events', 'live_start']]
lifecycle = live('lifecycle')
fault = next(e for e in events if not e['accepted'])
fault_ns = fault['received_monotonic_ns']
by_send = {s['dispatch_sequence']: s for s in sends}
by_receipt = {r['dispatch_sequence']: r for r in receipts}
assert len(by_send) == len(sends) == len(by_receipt) == len(receipts)
assert by_send.keys() == by_receipt.keys()
state_rows = list(csv.DictReader(raw('ts20_state.csv').decode().splitlines()))
states = [{}, {}]
for row in state_rows:
    for j in range(2):
        states[j].setdefault(int(row[f'upper_sequence{j+1}']), row)
lag = []
for i, send in enumerate(sends):
    receipt = by_receipt[send['dispatch_sequence']]
    assert all(send[k] == receipt[k] == start[k] for k in ['session_id', 'epoch'])
    assert send['command_sequence'] == receipt['command_sequence']
    assert receipt['joint_consumed'] and not receipt['failed']
    assert receipt['pva_send_returned'] is (not send['stop_zero'])
    assert receipt['consumed_counter'] == send['expected_counter']
    if i:
        assert send['expected_counter'] == [c+1 for c in sends[i-1]['expected_counter']]
        if not send['stop_zero']:
            assert by_receipt[sends[i-1]['dispatch_sequence']]['observed_monotonic_ns'] <= send['sent_monotonic_ns']
    for j in range(2):
        row = states[j][send['expected_counter'][j]]
        assert abs(float(row[f'u{j+1}']) - send['joint_velocity'][j]) <= 1e-12
        stamp = float(row[f'receipt{j+1}_s'])
        assert abs(stamp - receipt['consumed_sim_s'][j]) <= 1e-9
        assert -.008 <= stamp - send['sim_s'] <= .05
        lag.append(stamp - send['sim_s'])
zeros = [s for s in sends if s['stop_zero']]
assert len(zeros) == 1 and zeros[0] is sends[-1] and zeros[0]['joint_velocity'] == [0., 0.]
assert all(s['sent_monotonic_ns'] < fault_ns for s in sends if not s['stop_zero'])
release = lifecycle['release']
assert release['zero_consumed'] and release['writer_exited']
trace = [json.loads(line) for line in raw('control_execution_trace.jsonl').splitlines()]
publications = [row for row in trace if 'publish_before' in row]
assert not any(start['issued_monotonic_ns'] <= row['publish_before'].get('wall_monotonic_s', -1)*1e9 < release['wall_monotonic_ns'] for row in publications)
ticks = live('dispatch')
worker = [json.loads(line) for line in raw('isolated_live/recording/diagnostics.jsonl').splitlines()]
assert all(row['diagnostic']['accepted_optimum'] and row['diagnostic']['ts20_nonlinear_check_passed'] and not row['error'] for row in worker)
report = {
    'kind': 'hold22_physical_consumption_review_v1', 'derived_analysis': True,
    'full_hold_pass': False, 'recovery_checks_passed': True,
    'first_rejected_gate_event': fault,
    'dispatch_status_counts': dict(Counter(r['status'] for r in ticks)),
    'first_dispatch_fault': next((r for r in ticks if r['status'] == 3), None),
    'final_dispatch': ticks[-1],
    'all_sends': len(sends), 'normal_sends': len(sends)-1,
    'normal_repeats': sum(r['repeated'] for r in sends if not r['stop_zero']),
    'all_receipts': len(receipts), 'individual_joint_commands': 2*len(sends),
    'per_joint_counter_range': [sends[0]['expected_counter'][0], sends[-1]['expected_counter'][0]],
    'exact_send_receipt_identity_and_raw_joint_counter_value_time_matches': True,
    'normal_send_waited_for_previous_actual_receipt': True,
    'normal_output_after_gate_rejection': False,
    'delivery_lag_sim_s': lag, 'stop_zero_send': zeros[0],
    'stop_zero_receipt': by_receipt[zeros[0]['dispatch_sequence']],
    'stop_zero_send_after_gate_rejection_ms': (zeros[0]['sent_monotonic_ns']-fault_ns)/1e6,
    'stop_zero_receipt_after_gate_rejection_ms': (by_receipt[zeros[0]['dispatch_sequence']]['observed_monotonic_ns']-fault_ns)/1e6,
    'release': release, 'no_legacy_publication_overlap': True,
    'legacy_publication_records_checked': len(publications),
    'worker_diagnostics': [{'sequence': r['sequence'], 'solve_ms': r['diagnostic']['solve_ms'],
                            'accepted_optimum': r['diagnostic']['accepted_optimum'],
                            'timing': r['diagnostic']['ts20_timing_observation']}
                           for r in worker],
    'source_files': sources,
    'scope': 'Original plugin consumption and command ownership for this failed attempt; not a full-hold or hardware qualification.'}
print(json.dumps(report, indent=2, allow_nan=False))
