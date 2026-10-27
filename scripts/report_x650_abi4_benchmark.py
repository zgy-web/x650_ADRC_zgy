#!/usr/bin/env python3
"""Verify all ABI 4 paired chunks, retain failed checks, export compact evidence.

Runs only on the workstation after both target processes complete. Timings
come from Core clocks; SSH wall time is never used as solver time.
"""
from __future__ import annotations
import argparse
from collections import Counter
import csv
import io
import json
from pathlib import Path

from compare_x650_paired_core_benchmark import (compare, comparison, identity,
    load_json, require, verify_identity)
from prepare_x650_orin_benchmark import atomic_write, json_bytes
from report_x650_paired_core_benchmark import environment


def rows(result, phase):
    return [result[phase]] if phase == 'cold_first_call' else result[phase]


def make_report(root, prep, output):
    manifest_path = prep/'preparation.json'
    manifest = load_json(manifest_path)
    protocol_path = prep/'protocol.json'
    verify_identity(protocol_path, manifest['protocol'], 'Frozen protocol')
    protocol = load_json(protocol_path)
    require(protocol['expected_core_abi'] == 4 and protocol['expected_optimization_abi'] == 2,
        'Wrong protocol ABI')
    output.mkdir(parents=True, exist_ok=False)
    group_pairs, group_cold, group_warm, reports = {}, {}, {}, []
    errors, all_phase_errors, signatures = [], [], {}
    strict_keys, strict_cases = set(), set()
    maximum_cross, maximum_historical = {}, {'server': {}, 'nx': {}}
    returned_counts = Counter()
    native_invalid_counts = Counter()
    for entry in manifest['inputs']:
        path = prep/Path(entry['path']).name
        verify_identity(path, entry, 'Frozen input')
        bundle = load_json(path)
        require(bundle['provenance']['comparison_protocol']['protocol_sha256'] == manifest['protocol']['sha256'],
            'Input is not bound to this frozen protocol')
        require(bundle['run']['cpu'] == protocol['run']['cpu']
            and bundle['run']['warmup_rounds'] == protocol['run']['warmup_rounds']
            and bundle['run']['measured_rounds'] == entry['measured_rounds'], 'Run settings changed')
        group = entry['group']
        folders = {s: root/'formal_v1'/s/path.stem for s in ('server', 'nx')}
        results = {s: load_json(f/'native_stdout.json') for s, f in folders.items()}
        report = compare(path, folders['server'], folders['nx'], manifest_path,
            root/'server_build_report.json', root/'nx_build_report.json', prep/'code_only_sources.tar.gz',
            'recorded-derived-v2', 'abi4_mpc25_dispatch50')
        report_path = output/(path.stem+'_comparison.json')
        atomic_write(report_path, json_bytes(report))
        reports.append({**identity(report_path), 'group': group, 'pass': report['pass'],
            'cross_target_mismatches': len(report['cross_target_mismatches']),
            'historical_mismatches': len(report['historical_output_mismatches']),
            'strict_failure_count': report['strict_failure_count'],
            'relationship_failures': len(report['derived_acceleration_relationship_failures'])})
        for item in report['strict_failures']:
            strict_keys.add((group, item['phase'], item['case_id'], item['round']))
            strict_cases.add(item['case_id'])
        for field, error in report['cross_target_max_absolute_errors'].items():
            maximum_cross[field] = max(maximum_cross.get(field, 0.), error)
        for side, fields in report['historical_max_absolute_errors'].items():
            for field, error in fields.items():
                maximum_historical[side][field] = max(maximum_historical[side].get(field, 0.), error)
        for key in ('cross_target_mismatches', 'historical_output_mismatches', 'derived_acceleration_relationship_failures'):
            for failure in report[key]:
                errors.append({'kind': key, 'group': group, 'chunk': path.stem, **failure})
        for phase, collection in [('samples', group_pairs), ('cold_first_call', group_cold), ('warmup_samples', group_warm)]:
            left, right = (rows(results[s], phase) for s in ('server', 'nx'))
            collection.setdefault(group, []).extend(zip(left, right))
            for side in ('server', 'nx'):
                for sample in rows(results[side], phase):
                    returned_counts[side] += int(sample['returned'])
                    native_invalid_counts[side] += int(not sample['valid'])
                    if not sample['returned'] or sample['exceeded_50ms_wall']:
                        all_phase_errors.append({'side': side, 'group': group, 'phase': phase,
                            'case_id': bundle['cases'][sample['case_index']]['id'], 'round': sample['round'],
                            'returned': sample['returned'], 'error': sample['error'],
                            'wall_ns': sample['core_step_wall_ns']})
        for side, result in results.items():
            count = signatures.setdefault(group, {}).setdefault(side, Counter())
            for sample in result['samples']:
                d = sample['diagnostics']
                signature = f"QP={d['subproblems']};rows={d['qp_rows']};variables={d['qp_columns']};interval_rows={d['interval_rows']};hold_passed={d['hold_passed']}"
                count[signature] += 1
        print(json.dumps({'chunk':path.stem,'pass':report['pass'],
            'cross_target_mismatches':reports[-1]['cross_target_mismatches'],
            'historical_mismatches':reports[-1]['historical_mismatches']}), flush=True)
    ordinary = [pair for name, pairs in group_pairs.items() if not name.endswith('_heavy') for pair in pairs]
    heavy = [pair for name, pairs in group_pairs.items() if name.endswith('_heavy') for pair in pairs]
    require(len(ordinary) == 24200 and len(heavy) == 2000, 'Missing or duplicated planned measured samples')
    unique = {}
    for failure in errors:
        key = (failure['kind'], failure['group'], failure['case_id'], failure.get('side', 'paired'))
        row = unique.setdefault(key, {'kind': key[0], 'group': key[1], 'case_id': key[2],
            'side': key[3], 'failed_phase_calls': 0, 'max_absolute_errors': {}})
        row['failed_phase_calls'] += 1
        for name, item in failure.get('field_errors', {}).items():
            row['max_absolute_errors'][name] = max(row['max_absolute_errors'].get(name, 0.), item['maximum_absolute_error'])
    summary = {'kind': 'x650_abi4_paired_summary_v1', 'complete': True,
        'pass': all(r['pass'] for r in reports), 'execution_and_50ms_deadline_pass': not all_phase_errors,
        'numerical_policy_pass': not errors, 'protocol': manifest['protocol'],
        'preparation': identity(manifest_path), 'source_package': manifest['source_package'],
        'cases': {'unique_real_recorded_states': 2420, 'measured_calls_per_host': len(ordinary)+len(heavy),
            'ordinary_measured_calls_per_host': len(ordinary), 'extra_heavy_calls_per_host': len(heavy),
            'all_phase_returned_calls_per_host': dict(returned_counts), 'all_phase_invalid_calls_per_host': dict(native_invalid_counts)},
        'all_recorded': comparison(ordinary), 'heavy': comparison(heavy),
        'groups': {g: {'measured': comparison(pairs), 'cold': comparison(group_cold[g]),
            'warmup': comparison(group_warm[g]), 'branch_counts': signatures[g],
            'historical_physical_audit': manifest['groups'][g.removesuffix('_heavy')]['physical_audit']}
            for g, pairs in group_pairs.items()},
        'comparisons': reports, 'unique_failed_numerical_cases': list(unique.values()),
        'cross_target_max_absolute_errors': maximum_cross,
        'historical_max_absolute_errors': maximum_historical,
        'strict_uniform_extra_check': {'passed': not strict_keys, 'unique_inputs': len(strict_cases),
            'all_phase_logical_pairs_failed': len(strict_keys),
            'measured_logical_pairs_failed': sum(key[1] == 'samples' for key in strict_keys),
            'all_failure_records_retained_in_comparison_json': True},
        'all_phase_execution_failures': all_phase_errors,
        'environment': {s: environment(load_json(root/(s+'_before.json')), load_json(root/(s+'_after.json')))
            for s in ('server', 'nx')},
        'nx_code_only_audit': load_json(root/'nx_code_only_final_audit.json'),
        'timing_excludes': ['SSH and network', 'JSON decode and hashes', 'Memory restore',
            'reference comparison', 'result encoding and disk writes', 'ROS and actuator transport'],
        'interpretation_limits': ['Original failed physical wrench experiment remains failed.',
            'Independent restored snapshots; not a new flight, PX4-in-loop or real hardware control test.',
            'No CUDA kernels are used; no CUDA speedup is claimed.',
            'CPU 2 and MAXN describe the observed setup, not hard real-time worst-case.',
            'Heavy cohort repeats four actual two-QP cases; it does not cover all possible three-QP or interval-repair states.']}
    atomic_write(output/'summary.json', json_bytes(summary))
    csv_rows = []
    for row in unique.values():
        for field, error in row['max_absolute_errors'].items():
            csv_rows.append({k: row[k] for k in ('kind', 'group', 'case_id', 'side', 'failed_phase_calls')} |
                {'field': field, 'max_absolute_error': error})
    stream = io.StringIO(newline='')
    writer = csv.DictWriter(stream, fieldnames=['kind','group','case_id','side','failed_phase_calls','field','max_absolute_error'])
    writer.writeheader();writer.writerows(csv_rows)
    atomic_write(output/'numerical_failures.csv', stream.getvalue().encode('utf-8'))
    return summary


def render(summary, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    names = list(summary['groups'])
    labels = {'hold':'Endpoint hold', 'wrench':'Disturbed figure-eight',
        'wrench_heavy':'Two-QP repeated', 'mass085':'Mass/inertia -15%', 'mass115':'Mass/inertia +15%'}
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.4), gridspec_kw={'width_ratios':[1.35,1]})
    colors = {'server':'#347ab7', 'nx':'#d97717'}
    y = np.arange(len(names))
    for offset, side in ((-.17, 'server'), (.17, 'nx')):
        values = [summary['groups'][g]['measured']['wall'][side] for g in names]
        axes[0].barh(y+offset, [v['p50'] for v in values], height=.3, color=colors[side], label=side.upper()+' median')
        axes[0].scatter([v['p99'] for v in values], y+offset, marker='|', color='black', s=110, zorder=5,
            label='p99' if side=='server' else None)
        axes[0].scatter([v['max'] for v in values], y+offset, marker='x', color=colors[side], s=28,
            label=side.upper()+' max')
        for pos, value in zip(y+offset, values):
            axes[0].text(value['max']+.25, pos, f"{value['max']:.2f}", va='center', fontsize=8)
    axes[0].set_yticks(y, [labels[n] for n in names]);axes[0].invert_yaxis()
    axes[0].set_xlim(0, max(summary['groups'][g]['measured']['wall'][side]['max']
        for g in names for side in ('server', 'nx'))*1.12)
    axes[0].set_xlabel('Core::step wall time [ms]');axes[0].grid(axis='x', alpha=.2)
    axes[0].legend(fontsize=8, loc='lower right')
    maxima = [summary['all_recorded']['wall']['nx']['max'], summary['heavy']['wall']['nx']['max'], 40, 50]
    axes[1].barh(['NX: full corpus max','NX: two-QP max','MPC nominal period','Core deadline'], maxima,
        color=[colors['nx'], '#a95d12', '#578768', '#797f85'], height=.55)
    for i,v in enumerate(maxima): axes[1].text(v+.7, i, f'{v:.2f} ms', va='center', fontsize=9)
    axes[1].set_xlim(0, 60);axes[1].invert_yaxis();axes[1].set_xlabel('Time [ms]')
    axes[1].grid(axis='x', alpha=.2)
    fig.suptitle('ABI 4 | same-input CPU benchmark: server vs Orin NX', fontsize=15)
    caption = '26,200 measured calls per host. All slow/failed checks retained. Numerical equivalence: '+('PASS' if summary['numerical_policy_pass'] else 'NOT ALL PASS')
    fig.text(.5,.018,caption,ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.04,1,.94])
    fig.savefig(output/'abi4_timing.png',dpi=180)
    fig.savefig(output/'abi4_timing.pdf')
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--preparation', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plot-only', action='store_true')
    args = parser.parse_args(argv)
    summary = load_json(args.output/'summary.json') if args.plot_only else make_report(args.root, args.preparation, args.output)
    render(summary, args.output)
    print(json.dumps({k:summary[k] for k in ('complete','pass','execution_and_50ms_deadline_pass','numerical_policy_pass')}))
    return 0 if summary['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
