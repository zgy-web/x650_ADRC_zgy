#!/usr/bin/env python3
"""Supplement existing MPC audits with actual event-model node stopping checks.

This reads the Full diagnostic JSON and original snapshot/proposal CDR already
bound by each passed audit. It neither solves a QP nor runs a predictor/physics.
An accepted prediction horizon is not an executed trajectory. Actual first-send
commits and the explicitly discarded unsent tail are therefore reported apart.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time


def require(value, reason):
    if not value:
        raise ValueError(reason)


def identity(path, data=None):
    path = Path(path).resolve(strict=True)
    data = path.read_bytes() if data is None else data
    return dict(path=str(path), bytes=len(data), sha256=hashlib.sha256(data).hexdigest())


def vector(value, count, name):
    require(isinstance(value, (list, tuple)) and len(value) == count, name + ' shape')
    result = [float(x) for x in value]
    require(all(math.isfinite(x) for x in result), name + ' finite')
    return result


def close(a, b, tolerance, label):
    require(len(a) == len(b) and all(abs(x-y) <= tolerance for x, y in zip(a, b)), label)


def stopping_check(config, source_age, states):
    """Same stated engineering node reserve; does not certify true deceleration."""
    nodes = config['nodes']
    require(type(nodes) is int and nodes > 0 and len(states) == nodes*12, 'Node state dimensions')
    require(math.isfinite(source_age) and 0 <= source_age <= .025+1e-10, 'Original joint freshness')
    velocity = vector(config['velocity_limit'], 5, 'velocity_limit')
    acceleration = vector(config['acceleration_limit'], 5, 'acceleration_limit')
    lower, upper = (vector(config[name], 2, name) for name in ('q_lower', 'q_upper'))
    require(all(v > 0 for v in acceleration[3:]), 'Positive reserve denominators')
    reserve = [velocity[3+j]/(2*acceleration[3+j])+config['max_dt']+config['command_hold_s']+source_age for j in range(2)]
    worst = None
    for node in range(nodes):
        for joint in range(2):
            q, dq = states[node*12+6+joint], states[node*12+8+joint]
            protected = q+reserve[joint]*dq
            margin = min(protected-lower[joint], upper[joint]-protected)
            value = dict(node_zero_based=node, joint_one_based=joint+1, q_rad=q,
                dq_rad_s=dq, reserve_s=reserve[joint], protected_q_rad=protected,
                lower_rad=lower[joint], upper_rad=upper[joint], margin_rad=margin)
            if worst is None or margin < worst['margin_rad']:
                worst = value
    tolerance = config['feasibility_tolerance']
    violation = max(0., -worst['margin_rad'])
    return dict(passed=violation <= tolerance, violation_rad=violation,
        minimum_margin_rad=worst['margin_rad'], reserve_s=reserve, worst_node=worst)


def audit(report_path):
    from rclpy.serialization import deserialize_message
    from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal

    report_path = Path(report_path).resolve(strict=True)
    report_bytes = report_path.read_bytes()
    original = json.loads(report_bytes)
    require(original['kind'] == 'x650_upper_closed_loop_audit_v1'
        and original['backend'] == 'mpc_callable' and original['passed'] is True
        and original['complete'] is True, 'Expected complete original passed callable MPC audit')
    bound = {row['path']: row for row in original['artifact_identities']}
    require(len(bound) == len(original['artifact_identities']), 'Duplicate bound artifacts')
    bound[original['startup_manifest']['path']] = original['startup_manifest']
    verified = {}

    def read_bound(path, decode_json=True):
        path = Path(path).resolve(strict=True)
        key = str(path)
        require(key in bound, 'Original audit did not bind ' + key)
        data = path.read_bytes()
        actual = identity(path, data)
        require(actual == bound[key], 'Original audited bytes changed: ' + key)
        verified[key] = actual
        return json.loads(data) if decode_json else data

    run = Path(original['run_directory']).resolve(strict=True)
    live = run/'isolated_live'
    startup = read_bound(live/'worker_startup.json')
    manifest = read_bound(original['startup_manifest']['path'])
    require(startup['startup_manifest'] == manifest, 'Loaded startup manifest differs')
    require(startup['implementation'] == 'cpp_callable_mpc_v1'
        and startup['backend'] == 'mpc_callable' and startup['diagnostic_mode'] == 'full', 'Worker identity')
    config = manifest['config']
    require(config['nodes'] == 15 and config['command_hold_s'] == .16
        and config['max_dt'] == .1 and config['node_dt'] == .04, 'Expected original selected profile')
    final = read_bound(live/'worker_final.json')
    ledger = read_bound(live/'worker_commit_timing.json')
    require(ledger['complete'] is True and ledger['requires_consumer_ack'] is False,
        'Incomplete first-send commit ledger')
    columns = ledger['columns']
    require(len(columns) == len(set(columns)), 'Duplicate ledger fields')
    rows = [dict(zip(columns, row)) for row in ledger['rows']]
    require(all(len(row) == len(columns) for row in ledger['rows']), 'Ledger row width')
    committed = [row['sequence'] for row in rows]
    require(committed == list(range(1, len(rows)+1))
        and all(row['committed_memory_steps'] == row['sequence'] for row in rows), 'Actual-send prefix')
    files = sorted((live/'native_records').glob('*.json'))
    require(final['faulted'] is False and final['last_error'] == ''
        and final['requests'] == final['accepted'] == len(files)
        and final['controller_steps'] == final['actual_send_commits'] == len(rows)
        and final['committed_send_sequence'] == len(rows)
        and final['pending_send_sequence'] == 0, 'Worker actual commit/final count')
    require(len(files)-len(rows) in (0, 1), 'Unexpected unsent candidate count')
    discarded = len(files) if len(files) > len(rows) else 0
    require(final['discarded_unsent_sequence'] == ledger['discarded_unsent_sequence'] == discarded,
        'Unsent tail must be explicitly discarded')
    results = []
    for sequence, path in enumerate(files, 1):
        record = read_bound(path)
        require(record['kind'] == 'cpp_full_pipeline_call_v1'
            and record['sequence'] == sequence and record['backend'] == 'mpc_callable'
            and record['implementation'] == 'cpp_callable_mpc_v1'
            and record['diagnostic_mode'] == 'full' and record['full_matrix_packet_capture'] is True
            and record['valid_proposal_published'] is True and record['error'] == ''
            and record['memory_before_step_count'] == sequence-1
            and record['selected_next_step_count'] == sequence, 'Original proposal/commit history')
        raw_snapshot = read_bound(record['snapshot_cdr']['path'], False)
        raw_proposal = read_bound(record['proposal_cdr']['path'], False)
        require(record['snapshot_cdr'] == verified[record['snapshot_cdr']['path']]
            and record['proposal_cdr'] == verified[record['proposal_cdr']['path']], 'Record CDR binding')
        snapshot = deserialize_message(raw_snapshot, ControlSnapshot)
        proposal = deserialize_message(raw_proposal, ControlProposal)
        require(snapshot.sequence == proposal.sequence == proposal.snapshot_sequence == sequence
            and snapshot.session_id == proposal.session_id == startup['session_id']
            and snapshot.epoch == proposal.epoch == startup['epoch']
            and bytes(snapshot.configuration_sha256) == bytes(proposal.configuration_sha256)
            and bytes(snapshot.configuration_sha256).hex() == startup['configuration_sha256']
            and proposal.status == 0 and proposal.accepted_optimum and proposal.nonlinear_check_passed
            and not proposal.replay_only, 'Exact original message identity')
        result = record['result']
        d = result['diagnostics']
        require(d['accepted'] is True and d['nonlinear_passed'] is True
            and d['actual_bounds'] is True and d['previous_plan_fallback'] is False,
            'Expected actually accepted original native candidate')
        n = config['nodes']
        h = vector(d['horizon'], n, 'horizon')
        close(h, [snapshot.dt_s]+[config['node_dt']]*(n-1), 1e-12, 'Original horizon')
        predicted = vector(d['predicted_states'], n*12, 'predicted_states')
        queries = vector(d['query_times'], len(d['query_times']), 'query_times')
        require(all(b > a for a, b in zip(queries, queries[1:])), 'Unique sorted query times')
        actual = vector(d['candidate_states'], len(queries)*4, 'candidate_states')
        at = snapshot.task_time_s
        for node, interval in enumerate(h):
            at += interval
            found = [index for index, time_s in enumerate(queries) if abs(time_s-at) <= 1e-9]
            require(len(found) == 1, 'Original event node absent or ambiguous')
            close(predicted[node*12+6:node*12+10], actual[found[0]*4:found[0]*4+4],
                1e-13, 'Actual event q/dq differs from diagnostic node states')
        age = snapshot.control_time_s-snapshot.joint_time_s
        value = stopping_check(config, age, predicted)
        value.update(sequence=sequence, task_time_s=snapshot.task_time_s,
            source_age_s=age, actually_sent_first_command=sequence <= len(rows),
            original_recovery_triggered=d['recovery_triggered'],
            diagnostic=verified[str(path)], snapshot=record['snapshot_cdr'], proposal=record['proposal_cdr'])
        results.append(value)

    def summary(values):
        require(values, 'No candidate evidence')
        worst = min(values, key=lambda x: x['minimum_margin_rad'])
        return dict(candidates=len(values), passed=sum(v['passed'] for v in values),
            failed=sum(not v['passed'] for v in values),
            max_stopping_violation_rad=max(v['violation_rad'] for v in values),
            minimum_stopping_margin_rad=worst['minimum_margin_rad'],
            worst_sequence=worst['sequence'], worst_task_time_s=worst['task_time_s'],
            worst_node=worst['worst_node'],
            task_time_coverage_s=[values[0]['task_time_s'], values[-1]['task_time_s']],
            recovery_candidates=sum(v['original_recovery_triggered'] for v in values))

    require(identity(report_path) == identity(report_path, report_bytes), 'Source report changed')
    return dict(case_id=original['case_id'], run_directory=str(run),
        original_report=identity(report_path, report_bytes), original_passed=original['passed'],
        config=config, config_sha256=original['config_sha256'],
        all_accepted_candidates=summary(results),
        actually_sent_first_command_candidates=summary([v for v in results if v['actually_sent_first_command']]),
        discarded_unsent_sequences=[v['sequence'] for v in results if not v['actually_sent_first_command']],
        verified_source_artifacts=len(verified),
        metadata_artifacts=[v for p, v in verified.items() if '/native_records/' not in p],
        results=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(len(args.report) == 3 and len(set(args.report)) == 3, 'Exactly hold/normal/fast original MPC reports')
    require(not args.output.exists(), 'Refuse to overwrite supplemental output')
    started = time.time()
    cases = [audit(path) for path in args.report]
    require(len({x['case_id'] for x in cases}) == 3, 'Distinct original cases')
    report = dict(kind='x650_recorded_actual_stopping_supplement_v1', complete=True,
        script=identity(__file__), created_unix=time.time(), elapsed_s=time.time()-started,
        scope='Read-only recomputation on exact original Full event-predictor nodes and original CDR source age; no optimizer, predictor or physical rerun; no changes to previous acceptance reports.',
        limitation='This is an engineering node stopping reserve, not a proof of physical guaranteed braking or continuous-time invariance. Future predicted nodes were not all executed. First-send receipt is distinct from actuator consumption.',
        formula='r_j=v_limit[j]/(2*a_limit[j])+max_dt+command_hold_s+(control_time-joint_time); q_lower<=actual_q+r_j*actual_dq<=q_upper with original feasibility_tolerance',
        cases=cases)
    args.output.mkdir(parents=True)
    target = args.output/'report.json'
    target.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(report=identity(target), cases=[dict(case_id=x['case_id'],
        all_accepted_candidates=x['all_accepted_candidates'],
        actually_sent_first_command_candidates=x['actually_sent_first_command_candidates'],
        discarded_unsent_sequences=x['discarded_unsent_sequences']) for x in cases]), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
