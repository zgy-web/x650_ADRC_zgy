#!/usr/bin/env python3
"""Read-only C++ hold postmortem; an incomplete prefix is never acceptance.

No ROS, optimizer, replay, actuator or simulation is invoked. The error metric
uses the original same-time Gazebo native tip and the frozen LiveStart target.
All original JSON/CDR files, the physical trace, and this script are hash-bound.
"""
from pathlib import Path
import argparse
import collections
import csv
import hashlib
import json
import math


def require(ok, text):
    if not ok:
        raise ValueError(text)


def read(path):
    def invalid(value):
        raise ValueError('Nonfinite JSON: ' + value)
    return json.loads(Path(path).read_text(encoding='utf-8'), parse_constant=invalid)


def identity(path):
    path = Path(path).resolve(strict=True)
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return {'path': str(path), 'sha256': digest.hexdigest(), 'bytes': path.stat().st_size}


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


def physical_prefix(path, origin, end, target, anchor, requested):
    require(len(target) == len(anchor) == 3 and all(map(finite, [origin, end, requested, *target, *anchor])),
            'Finite frozen origin, target and anchor required')
    require(end > origin and requested > 0, 'A positive actually controlled interval is required')
    previous = None
    integral = [0., 0., 0.]
    count = invalid = 0
    peak = tilt_peak = body_peak = max_gap = 0.
    q_max, dq_max = [0., 0.], [0., 0.]
    with Path(path).open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            t = float(row['sim_s'])
            require(math.isfinite(t), 'Nonfinite physical timestamp')
            if not origin-1e-9 <= t <= end+1e-9:
                continue
            invalid += any(row[k] != '1' for k in ('valid_base', 'valid_joint_state', 'valid_tip'))
            tip = [float(row['tip_'+a]) for a in 'xyz']
            position = [float(row['base_p'+a]) for a in 'xyz']
            quat = [float(row['base_q'+a]) for a in 'wxyz']
            q = [float(row['arm_joint_'+str(j+1)+'_q']) for j in range(2)]
            dq = [float(row['arm_joint_'+str(j+1)+'_qd']) for j in range(2)]
            require(all(map(finite, tip+position+quat+q+dq)), 'Nonfinite physical state')
            norm = sum(x*x for x in quat)
            require(norm > 1e-18, 'Invalid physical quaternion')
            error2 = [(tip[j]-target[j])**2 for j in range(3)]
            peak = max(peak, math.sqrt(sum(error2)))
            tilt_peak = max(tilt_peak, math.degrees(math.acos(max(-1., min(1., 1.-2.*(quat[1]**2+quat[2]**2)/norm)))))
            body_peak = max(body_peak, math.sqrt(sum((position[j]-anchor[j])**2 for j in range(3))))
            for j in range(2):
                q_max[j] = max(q_max[j], abs(q[j]))
                dq_max[j] = max(dq_max[j], abs(dq[j]))
            if previous is None:
                first = t
            else:
                dt = t-previous[0]
                require(dt > 0, 'Physical samples regressed/repeated')
                max_gap = max(max_gap, dt)
                for j in range(3):
                    integral[j] += .5*(previous[1][j]+error2[j])*dt
            previous = t, error2
            count += 1
    require(count > 1, 'Physical prefix is empty')
    duration = previous[0]-first
    return {'scope': 'Descriptive prefix through last normal actual send; not a complete-task acceptance',
            'source': 'original native_trace.csv independent physical tip', 'same_time': True,
            'no_phase_alignment': True, 'requested_duration_s': requested,
            'window_sim_s': [first, previous[0]], 'duration_s': duration,
            'covers_requested_duration': duration >= requested-.008,
            'sample_count': count, 'invalid_samples': invalid, 'max_gap_s': max_gap,
            'target_enu_m': target, 'endpoint_rmse_m': math.sqrt(sum(integral)/duration),
            'endpoint_peak_m': peak, 'endpoint_axis_rmse_m': [math.sqrt(v/duration) for v in integral],
            'tilt_peak_deg': tilt_peak, 'base_anchor_displacement_peak_m': body_peak,
            'joint_abs_peak_rad': q_max, 'joint_velocity_abs_peak_rad_s': dq_max}


def numerical_records(folder):
    result = {'records': 0, 'sequences_contiguous': True, 'all_accepted': True,
              'all_nonlinear': True, 'all_actual_bounds': True, 'any_fallback': False,
              'max_primal': 0., 'max_kkt': 0., 'max_compute_ms': 0.,
              'max_q_linearization_error': 0., 'max_dq_linearization_error': 0.,
              'attempts': collections.Counter(), 'solver_modes': collections.Counter(),
              'max_abs_command': [0.]*5, 'errors': [], 'tail': []}
    keys = {'max_primal': 'primal_violation', 'max_kkt': 'kkt_stationarity',
            'max_compute_ms': 'compute_ms', 'max_q_linearization_error': 'q_linearization_error',
            'max_dq_linearization_error': 'dq_linearization_error'}
    for sequence, path in enumerate(sorted(Path(folder).glob('*.json')), 1):
        value = read(path)
        require(value.get('kind') == 'cpp_full_pipeline_call_v1', 'Unknown native record format')
        result['records'] += 1
        result['sequences_contiguous'] &= value['sequence'] == sequence
        for key in ('snapshot_cdr', 'proposal_cdr'):
            require(identity(value[key]['path']) == value[key], 'Original native CDR identity changed')
        if value.get('error') or 'result' not in value:
            result['errors'].append({'sequence': value['sequence'], 'error': value.get('error', 'missing result')})
            result['all_accepted'] = False
            continue
        output = value['result']
        d = output['diagnostics']
        result['all_accepted'] &= d['accepted'] is True
        result['all_nonlinear'] &= d['nonlinear_passed'] is True
        result['all_actual_bounds'] &= d['actual_bounds'] is True
        result['any_fallback'] |= d['previous_plan_fallback'] is not False
        for summary, field in keys.items():
            require(finite(d[field]) and d[field] >= 0, 'Invalid native numerical diagnostic: '+field)
            result[summary] = max(result[summary], d[field])
        result['attempts'][d['subproblems']] += 1
        result['solver_modes'][d['solver_mode']] += 1
        command = output['command']
        require(len(command) == 5 and all(map(finite, command)), 'Invalid native command')
        for j in range(5):
            result['max_abs_command'][j] = max(result['max_abs_command'][j], abs(command[j]))
        result['tail'].append({'sequence': value['sequence'], **{k: d[k] for k in (
            'compute_ms', 'primal_violation', 'kkt_stationarity', 'solver_mode', 'active_constraints', 'subproblems', 'objective')}})
        result['tail'] = result['tail'][-5:]
    require(result['records'] > 0, 'No actual native calls recorded')
    result['scope'] = 'Owned outputs of actual native calls; no extra solve or independent QP reconstruction'
    return result


def report(run):
    run = Path(run).resolve(strict=True)
    live = run/'isolated_live'
    # Hash before analysis and recheck afterward: never silently bind changed
    # bytes after reporting values computed from an earlier version.
    files = sorted(set(run.rglob('*.json')) | set(run.rglob('*.cdr')) |
                   {run/'native_trace.csv', Path(__file__).resolve()})
    artifacts = [identity(p) for p in files]
    meta, task = read(run/'controller_metadata.json'), read(run/'controller_task.json')
    start, lifecycle = read(live/'live_start.json'), read(live/'lifecycle.json')
    sends, receipts, events = (read(live/n) for n in ('sends.json', 'receipts.json', 'events.json'))
    normal = [s for s in sends if not s['stop_zero']]
    require(normal, 'No actual normal command was sent')
    native = physical_prefix(run/'native_trace.csv', start['origin_s'], normal[-1]['sim_s'],
                             start['target_position_enu'], start['anchor_position_enu'], task['duration_s'])
    manifest = read(run/'native_trace_manifest.json')
    # A failed task deliberately marks the formal window incomplete. Preserve
    # that fact; only the unchanged physical rows underpin this prefix report.
    require(manifest['sha256'] == identity(run/'native_trace.csv')['sha256'],
            'Independent physical recorder hash mismatch')
    result = {'kind': 'cpp_live_descriptive_prefix_postmortem_v1', 'complete': True,
              'physical_acceptance': False, 'certificate': False, 'run_dir': str(run),
              'reported_failure': meta.get('error'), 'numerical': numerical_records(live/'native_records'),
              'native_prefix': native, 'native_manifest_complete': manifest['complete'],
              'worker_final': read(live/'worker_final.json'),
              'diagnostic_sender': read(live/'diagnostic_sender.json'),
              'transport_counts': {'gate_events': len(events), 'gate_accepted': sum(e['accepted'] for e in events),
                  'normal_sends': len(normal), 'repeated_normal_sends': sum(s['repeated'] for s in normal),
                  'stop_sends': len(sends)-len(normal), 'receipts': len(receipts),
                  'all_consumed': all(v['joint_consumed'] and not v['failed'] for v in receipts)},
              'recovery': {'landed_and_disarmed': meta['landed_and_disarmed'],
                  'parameters_restored': meta['diagnostic_parameters_restored'],
                  'restore_pending': meta['diagnostic_restore_pending'], 'release': lifecycle['release'],
                  'child_returncodes': {k: v['returncode'] for k, v in lifecycle['processes'].items()}},
              'limitations': ['The prefix ends at the last normal actual send, not at the requested task end.',
                  'This report does not certify full-window ULog configuration or independently recompute each QP.',
                  'Native acceptance flags describe recorded core output; original full qualification remains separate.'],
              'artifacts': artifacts}
    require(all(identity(a['path']) == a for a in artifacts), 'Original evidence changed during analysis')
    result['original_artifacts_unchanged'] = True
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists(), 'Postmortem output must be a new directory')
    value = report(args.run_dir)
    args.output.mkdir(parents=True)
    path = args.output/'report.json'
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({'report': identity(path), 'physical_acceptance': False,
                      'numerical_records': value['numerical']['records'],
                      'native_prefix': value['native_prefix']}, indent=2))


if __name__ == '__main__':
    main()
