#!/usr/bin/env python3
"""Read-only common event-model audit of all selected standard solver results.

Successful and unsuccessful numerical solutions are both checked when a complete
finite physical plan exists. A diagnostic event pass cannot turn a failed solve
into an admitted candidate. No optimizer, ROS or physics is started here.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import time

BACKENDS = {
    'ipopt': ('ipopt', 'ipopt_reference'),
    'acados_sqp': ('sqp', 'acados_sqp'),
    'acados_rti': ('rti', 'acados_sqp_rti'),
}
COMPACT_FIELDS = ('backend', 'raw_status', 'solver_success', 'solve_elapsed_ms',
    'objective', 'numerically_feasible', 'first_order_kkt_passed', 'residuals',
    'multiple_shooting_dynamics_residual_inf', 'rollout_disagreement_inf', 'details')


def require(value, reason):
    if not value:
        raise ValueError(reason)


def canonical_hash(value):
    data = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return hashlib.sha256(data).hexdigest()


def identity(path):
    path = Path(path).resolve(strict=True)
    data = path.read_bytes()
    return dict(path=str(path), bytes=len(data), sha256=hashlib.sha256(data).hexdigest())


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'),
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON token: '+value)))


def write(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def finite_vector(value, length):
    return isinstance(value, list) and len(value) == length and all(
        type(item) in (int, float) and math.isfinite(item) for item in value)


def physical_plan(value, nodes=15):
    """Require both solver serializations to agree; never reshape an incomplete plan."""
    plan, flat = value.get('plan'), value.get('plan_flat')
    if not (isinstance(plan, list) and len(plan) == nodes
            and all(finite_vector(row, 5) for row in plan)
            and finite_vector(flat, nodes*5)):
        return None
    return flat if flat == [item for row in plan for item in row] else None


def load_bound_report(report_path):
    report_path = Path(report_path).resolve(strict=True)
    report_binding = identity(report_path)
    report = read(report_path)
    require(report.get('kind') == 'x650_standard_nmpc_offline_qualification_v1'
        and report.get('complete') is True, 'Qualification is incomplete; wait for its final report')
    require(report.get('selected_count') == 16 and report.get('mode') == 'all',
        'Expected all 16 preselected cases with all three backends')
    require(all(report.get(name) is False for name in
        ('physical_acceptance', 'online_acceptance', 'accepted_for_publication')),
        'Offline source must not already claim publication acceptance')
    manifest_path = report_path.parent/'delivery_manifest.json'
    manifest_binding = identity(manifest_path)
    manifest = read(manifest_path)
    require(manifest.get('kind') == 'x650_standard_nmpc_offline_manifest_v1', 'Wrong delivery manifest')
    delivered = {item['path']: item for item in manifest['files']}
    require(len(delivered) == len(manifest['files']) and delivered.get(str(report_path)) == report_binding,
        'Qualification report bytes differ from final delivery manifest')
    verified = {str(report_path): report_binding, str(manifest_path): manifest_binding}

    def bound_read(path):
        actual = identity(path)
        require(delivered.get(actual['path']) == actual, 'Result not bound by final delivery manifest: '+str(path))
        verified[actual['path']] = actual
        return read(path)

    config = None
    selected = []
    seen = set()
    for binding in report['selections']:
        require(identity(binding['path']) == binding, 'Predeclared selection SHA/bytes changed')
        verified[binding['path']] = binding
        selection = read(binding['path'])
        require(selection.get('kind') == 'standard_nmpc_bound_fixture_selection_v1', 'Unexpected selection schema')
        require(canonical_hash(selection['config']) == report['configuration_sha256'], 'Selected configuration differs')
        if config is None:
            config = selection['config']
        require(selection['config'] == config, 'Selections use different physical configuration')
        for wrapped in selection['cases']:
            source_index = wrapped['source_index']
            require(type(source_index) is int and source_index >= 0, 'Invalid original source index')
            key = (wrapped['source_sha256'], source_index)
            require(key not in seen, 'Duplicated original case in selections')
            seen.add(key)
            case = wrapped['case']
            require(all(name in case for name in ('id', 'input', 'memory', 'oracle')), 'Incomplete original selected case')
            selected.append(dict(index=len(selected), selection=binding, source=wrapped['source'],
                source_sha256=wrapped['source_sha256'], source_index=source_index,
                case=case, case_sha256=canonical_hash(case)))
    require(len(selected) == 16 and config['nodes'] == 15, 'Fixed selection count/node count changed')
    for binding in report['source_artifacts']:
        require(identity(binding['path']) == binding, 'Qualification implementation changed after recorded run')
        verified[binding['path']] = binding
    rows = {}
    for row in report['cases']:
        index = row.get('index')
        require(type(index) is int and 0 <= index < 16 and index not in rows, 'Duplicate/invalid report case index')
        source = selected[index]
        for name in ('selection', 'source', 'source_sha256', 'source_index', 'case_sha256'):
            require(row.get(name) == source[name], 'Case provenance differs at '+str(index)+': '+name)
        require(row.get('case_id') == source['case']['id'], 'Original case ID differs')
        rows[index] = row
    return report_path, report, config, selected, rows, delivered, verified, bound_read


def collect_candidates(report_path, rows, selected, delivered, bound_read):
    records, inputs = [], []
    for source in selected:
        index, case = source['index'], source['case']
        qualified = rows.get(index)
        for backend, (row_key, expected_backend) in BACKENDS.items():
            record = dict(id=f'case_{index:03d}__{backend}', index=index,
                case_id=case['id'], task_time_s=case['input']['t'], backend=backend,
                selection=source['selection'], source=source['source'],
                source_sha256=source['source_sha256'], source_index=source['source_index'],
                case_sha256=source['case_sha256'], status='missing_qualification_case',
                numerical_result_available=False, finite_complete_plan=False,
                event_checked=False, solver_numerical_passed=False, common_event_passed=False,
                numerical_and_common_passed=False, accepted_for_publication=False)
            records.append(record)
            if qualified is None:
                continue
            record['source_case_qualification_passed'] = qualified['passed']
            record['source_case_error'] = qualified.get('error')
            path = report_path.parent/f'case_{index:03d}'/(backend+'.json')
            if not path.exists():
                require(str(path) not in delivered and row_key not in qualified,
                    'Qualification claims a result whose bound file is missing')
                record['status'] = 'missing_solver_result'
                continue
            result = bound_read(path)
            require(result.get('kind') == 'x650_standard_nmpc_solver_result_v1'
                and result.get('backend') == expected_backend, 'Wrong solver result/backend')
            require(result['graph_sha256'] == qualified['graph_sha256']
                and result['binding_sha256'] == qualified['binding_sha256'], 'Solver graph/source binding differs')
            numeric = bound_read(qualified['numeric_case']['path'])
            require(identity(qualified['numeric_case']['path']) == qualified['numeric_case'], 'Numeric case identity differs')
            require(numeric.get('kind') == 'x650_standard_nmpc_numeric_case_v1'
                and numeric.get('N') == 15 and numeric.get('nu') == 5
                and numeric['config_sha256'] == canonical_hash(read(source['selection']['path'])['config'])
                and numeric['source_identity'] == {name: source[name] for name in
                    ('source', 'source_sha256', 'source_index', 'case_sha256')},
                'Exported numerical problem does not match the selected physical source')
            require(numeric['graph_sha256'] == result['graph_sha256']
                and numeric['binding_sha256'] == result['binding_sha256'], 'Numeric case and solver binding differ')
            compact = {name: result[name] for name in COMPACT_FIELDS}
            if row_key in qualified:
                require(compact == qualified[row_key], 'Full solver result differs from reported numerical checks')
            for name in ('solver_success', 'numerically_feasible', 'first_order_kkt_passed'):
                require(type(result[name]) is bool, 'Non-boolean numerical acceptance flag')
            require(result.get('accepted_for_publication') is False
                and result.get('native_event_validation_required') is True, 'Source omitted native event qualification')
            record.update(numerical_result_available=True, solver_result=identity(path),
                numeric_case=qualified['numeric_case'], numerical=compact,
                source_deadline=result.get('deadline'),
                source_rejection_reasons=result.get('rejection_reasons'),
                solver_numerical_passed=all(result[name] for name in
                    ('solver_success', 'numerically_feasible', 'first_order_kkt_passed')))
            plan = physical_plan(result)
            if plan is None:
                record['status'] = 'invalid_or_inconsistent_physical_plan'
                continue
            record.update(status='awaiting_common_event_check', finite_complete_plan=True,
                plan_sha256=canonical_hash(plan))
            # Exact selected original Memory and StepInput; no history invented
            # from this numerical candidate, previous result or warm-start plan.
            inputs.append(dict(id=record['id'], memory=case['memory'], input=case['input'], plan=plan))
    require(len(records) == 48 and len({v['id'] for v in records}) == 48, 'Expected all 48 distinct slots')
    return records, inputs


def merge_event(records, inputs, event):
    require(event.get('kind') == 'x650_standard_common_event_replay_v1'
        and event['count'] == len(inputs) and len(event['records']) == len(inputs), 'Native event count/schema differs')
    expected = [row['id'] for row in inputs]
    require([row['id'] for row in event['records']] == expected, 'Native event results missing/reordered/reused')
    indexed = {row['id']: row for row in records}
    for source, checked in zip(inputs, event['records']):
        require(type(checked['passed']) is bool, 'Invalid native event pass flag')
        require(checked['plan'] == source['plan'] and checked['task_time_s'] == source['input']['t']
            and checked['dt_s'] == source['input']['dt']
            and checked['committed_step_count'] == source['memory']['step_count'], 'Native input identity changed')
        if checked['passed']:
            require(all(checked[name] is True for name in ('input_valid', 'command_bounds_passed',
                'reference_bounds_passed', 'native_checked', 'native_bounds_passed', 'stopping_passed',
                'pending_guard_passed', 'hold_passed')), 'Native pass omitted a common hard check')
        row = indexed[source['id']]
        row.update(event_checked=True, event=checked, common_event_passed=checked['passed'],
            numerical_and_common_passed=row['solver_numerical_passed'] and checked['passed'],
            status='checked_common_pass' if checked['passed'] else 'checked_common_rejection')
    require(event['accepted_count'] == sum(r['passed'] for r in event['records']), 'Native accepted count differs')


def runtime_identity(executable):
    result = subprocess.run(['ldd', str(executable)], text=True, capture_output=True, check=True)
    libraries = [identity(path) for path in sorted(set(re.findall(r'=>\s+(/\S+)', result.stdout)))]
    require(any(Path(x['path']).name == 'libx650_mpc_core.so' for x in libraries)
        and any(Path(x['path']).name == 'libx650_standard_event_predictor.so' for x in libraries),
        'Native executable is not linked to the common event implementation')
    return dict(executable=identity(executable), libraries=libraries, ldd_stdout=result.stdout)


def summarize(records):
    summaries = {}
    for backend in BACKENDS:
        rows = [v for v in records if v['backend'] == backend]
        summaries[backend] = dict(expected=16, available=sum(v['numerical_result_available'] for v in rows),
            finite_plans=sum(v['finite_complete_plan'] for v in rows),
            event_checked=sum(v['event_checked'] for v in rows),
            native_propagated=sum(v.get('event', {}).get('native_checked', False) for v in rows),
            solver_numerical_passed=sum(v['solver_numerical_passed'] for v in rows),
            common_event_passed=sum(v['common_event_passed'] for v in rows),
            numerical_and_common_passed=sum(v['numerical_and_common_passed'] for v in rows),
            statuses=dict(Counter(v['status'] for v in rows)),
            common_rejection_reasons=dict(Counter(v['event']['reason'] for v in rows
                if v['event_checked'] and not v['common_event_passed'])))
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--validator', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists(), 'Use a new output directory; never replace earlier evidence')
    report_path, original, config, selected, rows, delivered, verified, bound_read = load_bound_report(args.report)
    records, inputs = collect_candidates(report_path, rows, selected, delivered, bound_read)
    runtime, script = runtime_identity(args.validator), identity(__file__)
    args.output.mkdir(parents=True)
    generated = []
    if inputs:
        native_input = args.output/'event_input.json'
        native_output = args.output/'event_output.json'
        write(native_input, dict(config=config, cases=inputs))
        process = subprocess.run([str(args.validator.resolve()), str(native_input.resolve()),
            str(native_output.resolve())], text=True, capture_output=True)
        (args.output/'event_replay.log').write_text(process.stdout+process.stderr, encoding='utf-8')
        require(process.returncode == 0, 'Native event replay failed; log retained in new output directory')
        event = read(native_output)
        require(event['input_sha256'] == identity(native_input)['sha256'], 'Native input SHA differs')
        merge_event(records, inputs, event)
        generated += [identity(native_input), identity(native_output), identity(args.output/'event_replay.log')]
    for binding in list(verified.values())+[runtime['executable'], *runtime['libraries'], script]:
        require(identity(binding['path']) == binding, 'Source/runtime changed during audit: '+binding['path'])
    output = dict(kind='x650_standard_candidates_event_audit_v1', complete=True,
        created_unix=time.time(), original_report=verified[str(report_path)],
        source_qualification_passed=original['passed'], source_configuration_sha256=original['configuration_sha256'],
        expected_case_count=16, expected_result_count=48, audited_candidates=len(inputs),
        all_slots_have_finite_plans=len(inputs)==48,
        all_candidates_pass_numerical_and_common=all(v['numerical_and_common_passed'] for v in records),
        summary=summarize(records), records=records, runtime=runtime, script=script,
        source_artifacts=list(verified.values()), generated_artifacts=generated,
        online_acceptance=False, accepted_for_publication=False, physical_acceptance=False,
        executed_optimizer=False, new_physical_experiment=False, original_reports_modified=False,
        scope='Exact original selected Memory/input + each numerical backend physical plan, independently repropagated through the same common encoded event model. No result reused, missing/invalid plans explicit. Numerical failure remains failure even if event checks pass.',
        limitation='Passing this offline model check is not live admission, guaranteed physical braking, recursive feasibility or global NLP optimality. Original issue deadline and actual transport have not been tested.')
    write(args.output/'report.json', output)
    print(json.dumps(dict(report=identity(args.output/'report.json'), audited_candidates=len(inputs),
        summary=output['summary'], online_acceptance=False), indent=2))


if __name__ == '__main__':
    main()
