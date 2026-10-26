"""Admit the proven ABI-3 software into the unchanged physical wrench case.

This is a ground-only evidence check. The previous fast flight is historical
physical preparation, and the fixed-input regression and actual ROS worker
probe remain software evidence. A fresh full physical audit is still required.
"""
from pathlib import Path

from simulations.x650_cpp_live_v1 import qualification as hold
from simulations.x650_cpp_robustness_v1 import qualification as robust, task
from simulations.x650_cpp_recovery_v1 import qualification as recovery

Evidence, read, sha, identity = hold.Evidence, hold.read, hold.sha, hold.identity
require, same_artifact, finite_bounded = hold.require, hold.same_artifact, hold.finite_bounded
archive_integrity = robust.archive_integrity
fast_trajectory, figure8 = robust.fast_trajectory, robust.figure8
verify_runtime_interfaces = hold.verify_runtime_interfaces
validate_protocol = recovery.validate_protocol

EXPECTED_LIBRARIES = {
    'core': 'd38fd4e7956ec49b6e558fb74093bff82c1e97d460a7e35c0ce26a8277dbd173',
    'optimization': '5188f6f6af9b5c07f9508d756dc20d09552f597bc2f0bd09ed9e341cf54c3ff6',
    'predictor': '9ff66d6db9afb3eb4c24ca6c0c3fcfc9453c6ab4384af134ec22b6f5c38701d8',
}
EXECUTION_SOURCES = recovery.EXECUTION_SOURCES | {
    'control_packages/x650_wholebody_v1/native/canonical_interval.hpp',
    'control_packages/x650_wholebody_v1/native/joint_safety.hpp',
    'control_packages/x650_wholebody_v1/native/optimization.cpp',
}


def check_numeric_summary(summary):
    require(summary.get('kind') == 'x650_interval_hard_constraint_software_v1'
        and summary.get('passed') is True and summary.get('closed_loop') is False
        and summary.get('actuator_io') is False and summary.get('hardware_qualified') is False
        and summary.get('slack_variables') == 0 and summary.get('inner_velocity_limits') == [.42, .32]
        and summary.get('expected_libraries') == EXPECTED_LIBRARIES
        and summary.get('calls') == summary.get('passed_calls') == 429
        and finite_bounded(summary.get('core_ms_max'), 50.),
        'Complete fixed-original ABI-3 interval software summary required')


def verify_regression(report, startup, evidence):
    require(report.get('kind') == 'x650_interval_fixed_original_regression_v1'
        and report.get('complete') is True and report.get('passed') is True
        and report.get('closed_loop') is False and report.get('actuator_io') is False
        and report.get('delay_injection') is False and report.get('expected_libraries') == EXPECTED_LIBRARIES
        and report.get('sequences') == list(range(1, 430)) and report.get('passed_cases') == 429
        and len(report.get('rows', [])) == 429, 'Complete 429 fixed-original interval regression required')
    original = evidence.document(report['original_report'])
    require(original.get('complete') is True and original.get('all_original_outcomes_reproduced') is True
        and original.get('memory_reconstructed_in_cpp') is True and original.get('history_from_proposals') is False
        and original.get('row_count') == 429 and original.get('step_count_after') == 428
        and original.get('stopped_at_original_rejection') is True
        and len(original.get('rows', [])) == 429, 'Actual original Memory prefix reconstruction required')
    for sequence, (row, source) in enumerate(zip(report['rows'], original['rows']), 1):
        require(row.get('sequence') == source.get('sequence') == sequence and row.get('passed') is True
            and row.get('returncode') == 0 and row.get('returned') is True
            and finite_bounded(row.get('core_ms'), 50.)
            and finite_bounded(row.get('canonical_maximum_error'), 1e-13),
            'Fixed-original call missing, rejected, late or canonical repropagation failed')
        fixture = evidence.document(source['fixture'])
        raw = evidence.document(row['raw'])
        require(fixture['config'] == startup['config'] and fixture['memory']['step_count'] == sequence-1
            and raw.get('kind') == 'x650_cpp_recovery_raw_replay_v1' and raw.get('core_abi') == 3
            and raw.get('complete') is True and raw.get('returned') is True
            and raw.get('input_sha256') == source['fixture']['sha256']
            and raw.get('memory_before') == fixture['memory'] and raw.get('memory_commit_valid') is True
            and raw['memory_after']['step_count'] == sequence
            and all(raw['loaded_code'][k]['sha256'] == digest for k, digest in EXPECTED_LIBRARIES.items()),
            'Fixed-original input, configuration, complete Memory or loaded code differs')
        d = raw['diagnostics']
        require(all(d.get(k) is True for k in ('accepted', 'actual_bounds', 'nonlinear_passed'))
            and d.get('previous_plan_fallback') is False and finite_bounded(d.get('compute_ms'), 50.)
            and type(d.get('subproblems')) is int and 1 <= d['subproblems'] <= 3
            and d.get('qp_columns') == 5*startup['config']['nodes'], 'Fixed call did not pass original native/solver gates')
        folder = Path(row['raw']['path']).parent
        audit = evidence.document(identity(folder/'audit.json'))
        canonical = evidence.document(identity(folder/'canonical_proof.json'))
        require(audit.get('kind') == 'x650_interval_constraint_independent_audit_v1'
            and audit.get('passed') is True and audit.get('complete') is True
            and audit.get('controller_call_accepted') is True and audit.get('memory_commit_verified') is True
            and audit.get('input_sha256') == source['fixture']['sha256']
            and audit.get('physical_validation') is False and audit.get('canonical_grid_verified') is True
            and audit.get('hard_margin_preserved') == dict(q_padding=.0003, dq_padding=.03, tolerance=2e-7)
            and audit.get('qp', {}).get('no_slack_variables') is True,
            'Independent interval arithmetic audit incomplete or altered margins')
        require(canonical.get('kind') == 'x650_interval_canonical_repropagation_v1'
            and canonical.get('passed') is True
            and canonical.get('input_sha256') == source['fixture']['sha256']
            and canonical.get('raw_sha256') == row['raw']['sha256']
            and canonical.get('predictor_sha256') == EXPECTED_LIBRARIES['predictor']
            and finite_bounded(canonical.get('maximum_absolute_error'), 1e-13)
            and canonical.get('substeps') == d['candidate_substeps']
            and recovery.numerical_equal(canonical.get('envelope'), d['candidate_envelope'], tolerance=1e-13)
            and recovery.numerical_equal(canonical.get('canonical_query_times'), d['query_times'], tolerance=1e-12)
            and recovery.numerical_equal(canonical.get('canonical_boundary_times'), d['candidate_boundary_times'], tolerance=1e-12),
            'Independent unchanged-grid native propagation is not bound to this call')
    last = report['rows'][-1]
    require(last.get('interval_rows') == 39 and last.get('subproblems') == 2,
            'Original 429 interval recovery coverage differs')
    return dict(verified=True, fixed_original_calls=429, original_memory_reconstructed=True,
                recovered_sequence=429, physical_acceptance=False)


def verify_worker(probe, startup, evidence, binaries, worker):
    """Actual worker uses a fresh sequential native oracle, not original Memory."""
    require(probe.get('kind') == 'x650_cpp_interval_actual_worker_probe_v1'
        and probe.get('core_abi') == 3 and probe.get('complete') is True and probe.get('passed') is True
        and probe.get('no_actuator_publishers') is True and probe.get('physical_acceptance') is False
        and probe.get('physical_validation') is False and probe.get('counterfactual_closed_loop_claimed') is False
        and probe.get('injected_delay') is False and probe.get('test_sequence_complete') is True
        and probe.get('rows_count') == 429 and probe.get('child_exit') == 0,
        'Actual ABI-3 worker with fresh sequential native oracle required')
    require(same_artifact(probe['worker'], worker)
        and binaries.get(str(Path(worker['path']).resolve())) == worker['sha256'],
        'Installed ABI-3 worker differs from actual tested binary')
    tested = evidence.document(probe['startup'])
    require(tested['config'] == startup['config'] and tested['trajectory'] == startup['trajectory']
        and recovery.identical_task_copy(tested['task_config'], startup['task_config'], evidence),
        'Actual worker tested a different task or mathematical configuration')
    actual_startup = evidence.document(probe['worker_startup'])
    require(actual_startup.get('core_abi') == 3 and actual_startup.get('diagnostic_mode') == 'compact'
        and actual_startup.get('python_in_worker') is False and actual_startup.get('startup_controller_steps') == 0
        and actual_startup.get('startup_manifest') == tested, 'Actual ABI-3 process startup/ownership differs')
    for role, key in [('core', 'core_library'), ('optimization', 'optimization_library'), ('predictor', 'predictor_library')]:
        require(same_artifact(probe['libraries'][role], startup[key])
            and same_artifact(tested[key], startup[key])
            and str(Path(startup[key]['path']).resolve()) == actual_startup['loaded_'+role+'_path'],
            'Actual worker loaded a different library: '+role)
    oracle, recording = evidence.document(probe['oracle']), evidence.document(probe['recording'])
    require(oracle.get('kind') == 'x650_cpp_interval_sequence_oracle_v1'
        and oracle.get('core_abi') == 3 and oracle.get('complete') is True and oracle.get('passed') is True
        and oracle.get('row_count') == oracle.get('step_count_after') == 429
        and oracle.get('memory_sequentially_reconstructed_in_cpp') is True
        and oracle.get('original_proposals_used_as_output_oracle') is False
        and oracle.get('recorded_states_and_actual_history_preserved') is True
        and oracle.get('physical_validation') is False and oracle.get('counterfactual_closed_loop_claimed') is False
        and same_artifact(oracle['recording'], probe['recording'])
        and len(oracle.get('rows', [])) == 429
        and all(oracle['loaded_code'][k]['sha256'] == v for k, v in EXPECTED_LIBRARIES.items()),
        'Fresh sequential native oracle is missing, mixes versions or uses original proposals')
    require(recording.get('kind') == 'x650_original_cpp_recording_export_v1'
        and recording.get('originals_verified_unchanged') is True
        and recording.get('config') == startup['config'] and recording.get('trajectory') == startup['trajectory']
        and len(recording.get('rows', [])) == 429, 'Actual worker did not use the original recorded states/history')
    previous = None
    for sequence, (binding, source) in enumerate(zip(oracle['rows'], recording['rows']), 1):
        call = evidence.document(binding)
        require(binding.get('sequence') == source.get('sequence') == call.get('sequence') == sequence
            and call.get('kind') == 'x650_cpp_interval_sequence_oracle_call_v1'
            and call.get('returned') is True and same_artifact(call['source_row'], source)
            and call['memory_before']['step_count'] == sequence-1 and call['memory_after']['step_count'] == sequence
            and (previous is None or previous == call['memory_before']),
            'Native oracle memory was not advanced from its own preceding accepted output')
        previous = call['memory_after']
    final, recorder = probe['final'], probe['recorder']
    require(final.get('requests') == final.get('accepted') == 429 and final.get('faulted') is False
        and recorder.get('complete') is True and recorder.get('dropped') == 0
        and recorder.get('written') == recorder.get('offered') == 429
        and len(probe.get('records', [])) == len(probe.get('wire_crosscheck', [])) == len(probe.get('rows', [])) == 429,
        'Actual worker did not finish or retain the complete original input prefix')
    for sequence, (row, wire) in enumerate(zip(probe['rows'], probe['wire_crosscheck']), 1):
        require(row.get('sequence') == wire.get('sequence') == sequence and row.get('passed') is True
            and finite_bounded(row.get('native_callback_ms'), 50.)
            and wire.get('snapshot_all_fields_exact') is True and wire.get('proposal_all_fields_exact') is True,
            'Actual ABI-3 worker raw wire identity or original deadline failed')
        for key in ('absolute_errors', 'full_native_output_absolute_errors', 'diagnostic_absolute_errors'):
            if key == 'diagnostic_absolute_errors':
                require(finite_bounded(row.get('diagnostic_absolute_error'), 1e-8),
                        'Actual ABI-3 worker diagnostic differs from fresh native oracle')
                continue
            require(row.get(key) and all(finite_bounded(v, 1e-8) for v in row[key].values()),
                    'Actual ABI-3 worker differs from sequential native oracle: '+key)
    return dict(verified=True, actual_worker=worker, sequential_native_oracle=True,
                original_snapshot_calls=429, physical_acceptance=False)


def verify_software(value, startup, evidence, binaries):
    require(value.get('kind') == 'x650_interval_flight_software_acceptance_v1'
        and value.get('complete') is True and value.get('passed') is True
        and value.get('core_abi') == 3 and value.get('physical_acceptance') is False,
        'Explicit ABI-3 flight software acceptance binding required')
    summary = evidence.document(value['summary'])
    check_numeric_summary(summary)
    code = value['code']
    require({k: v['sha256'] for k, v in code['libraries'].items()} == EXPECTED_LIBRARIES,
            'ABI-3 library version has no matching software qualification')
    for role, key in [('core', 'core_library'), ('optimization', 'optimization_library'), ('predictor', 'predictor_library')]:
        evidence.artifact(code['libraries'][role])
        require(same_artifact(code['libraries'][role], startup[key]), 'ABI-3 startup library differs: '+role)
    installed = {str(evidence.artifact(v)): v['sha256'] for v in startup['source_artifacts']}
    names = set()
    for item in code['sources']:
        path = evidence.artifact(item)
        require(item['relative_path'] not in names, 'Repeated ABI-3 source receipt')
        names.add(item['relative_path'])
        if item['relative_path'] in EXECUTION_SOURCES:
            require(installed.get(str(path)) == item['sha256'], 'Startup omits tested ABI-3 source: '+item['relative_path'])
    require(EXECUTION_SOURCES <= names, 'ABI-3 interval/quantization execution source proof incomplete')
    numerical = verify_regression(evidence.document(value['regression']), startup, evidence)
    worker = verify_worker(evidence.document(value['ros_worker_probe']), startup, evidence, binaries, code['worker'])
    return dict(verified=True, core_abi=3, initial_trust_radius=.02, slack_variables=0,
        inner_velocity_limits=[.42, .32], original_compute_limit_ms=50., numerical=numerical, worker=worker,
        physical_acceptance=False, scope='Software admission only; fresh 24 s physical acceptance remains required')


def verify(manifest_path, *, task_path, startup_path, startup_sha256, protocol_path, binaries):
    evidence, manifest = Evidence(), read(manifest_path)
    require(set(manifest) == {'kind', 'simulation_only', 'task_config', 'startup_manifest', 'protocol',
        'interval_software_acceptance', 'original_robust_qualification'}
        and manifest['kind'] == 'x650_cpp_interval_qualification_v1' and manifest['simulation_only'] is True,
        'Explicit interval ABI-3 qualification manifest required')
    for key, path in [('task_config', task_path), ('startup_manifest', startup_path), ('protocol', protocol_path)]:
        require(same_artifact(manifest[key], identity(path)), 'Selected interval artifact differs: '+key)
    require(manifest['startup_manifest']['sha256'] == startup_sha256, 'Selected ABI-3 startup hash differs')
    cfg = task.validate_task_config(read(evidence.artifact(manifest['task_config'])))
    require(cfg['robustness']['case_id'] == 'wrench_fast' and cfg['scenario'] == 'x650_cpp_robust_wrench_v1',
            'Unchanged original 24 s wrench task required')
    original = evidence.document(manifest['original_robust_qualification'])
    require(original.get('kind') == 'x650_cpp_robustness_qualification_v1'
        and recovery.identical_task_copy(original['task_config'], manifest['task_config'], evidence),
        'Original physical wrench task differs')
    prior = robust.verify_fast_gate(cfg['cpp_fast_certificate'], cfg['frozen_cpp_fast'])
    old_startup = read(evidence.artifact(original['startup_manifest']))
    startup = evidence.document(manifest['startup_manifest'])
    ignored = {'task_config', 'source_artifacts', 'core_library', 'optimization_library'}
    require({k: v for k, v in startup.items() if k not in ignored}
        == {k: v for k, v in old_startup.items() if k not in ignored}
        and same_artifact(startup['task_config'], manifest['task_config'])
        and startup['config'] == prior['startup']['config'] and startup['trajectory'] == fast_trajectory(),
        'Interval changed task, reference, configuration, predictor or startup contract')
    protocol = read(evidence.artifact(manifest['protocol']))
    old_protocol = read(evidence.artifact(original['protocol']))
    require(old_protocol == task.protocol(cfg), 'Original wrench waveform or protocol changed')
    validate_protocol(protocol, old_protocol)
    for path, digest in prior['runtime_binaries'].items():
        if Path(path).name in {'isolated_worker', 'libx650_mpc_core.so', 'libx650_optimization.so'}:
            continue
        require(binaries.get(path) == digest, 'Interval changed unqualified feedback/transport/predictor binary: '+path)
    software = verify_software(evidence.document(manifest['interval_software_acceptance']), startup, evidence, binaries)
    _, physical = task.physical_condition(cfg)
    return dict(kind='x650_cpp_interval_admission_v1', ready=True, physical_acceptance=False,
        qualification_manifest=identity(manifest_path), physical_condition=physical,
        interval_software_proof=software, original_fast_certificate=prior['certificate'],
        artifacts_checked=len(evidence.checked), verified_artifacts=list(evidence.checked.values()),
        historical_scope='Nominal physical preparation only; ABI-3 Core/QP/worker independently proven',
        scope='Original physical waveform, trajectory, PID, timing and consumption gates; only declared CPU placement rebound; no delay injection')
