"""Bind the ABI-2 recovery software to a fresh, unchanged wrench experiment.

The archived fast flight is historical physical preparation. It does not certify
the replacement Core or worker. Those bytes require their own raw regression and
actual ROS worker evidence; the new flight still needs independent acceptance.
"""
from pathlib import Path
import math

from simulations.x650_cpp_live_v1 import qualification as hold
from simulations.x650_cpp_robustness_v1 import qualification as robust, task

Evidence, read, sha, identity = hold.Evidence, hold.read, hold.sha, hold.identity
require, same_artifact, finite_bounded = hold.require, hold.same_artifact, hold.finite_bounded
archive_integrity = robust.archive_integrity
fast_trajectory, figure8 = robust.fast_trajectory, robust.figure8
verify_runtime_interfaces = hold.verify_runtime_interfaces

PROTOCOL_RESOURCE_KEYS = {'cpu_roles', 'cpu_scope', 'worker_thread_roles'}
EXECUTION_SOURCES = {
    'control_packages/x650_wholebody_v1/native/mpc_core.cpp',
    'control_packages/x650_wholebody_v1/native/mpc_core.hpp',
    'ros2_ws/src/flyarm_mpc_cpp/src/isolated_worker.cpp',
    'ros2_ws/src/flyarm_mpc_cpp/include/flyarm_mpc_cpp/diagnostics_json.hpp',
}


def numerical_equal(left, right, *, tolerance=1e-8, ignored=frozenset()):
    """Compare independent deterministic calls, excluding only named timers."""
    if isinstance(left, dict) and isinstance(right, dict):
        a, b = set(left)-ignored, set(right)-ignored
        return a == b and all(numerical_equal(left[k], right[k], tolerance=tolerance, ignored=ignored) for k in a)
    if isinstance(left, list) and isinstance(right, list):
        return len(left) == len(right) and all(numerical_equal(a, b, tolerance=tolerance, ignored=ignored)
                                              for a, b in zip(left, right))
    if type(left) in (int, float) and type(right) in (int, float):
        return math.isfinite(left) and math.isfinite(right) and abs(left-right) <= tolerance
    return type(left) is type(right) and left == right


def identical_task_copy(left, right, evidence):
    """A launch-owned task may move paths, but its exact bytes may not change."""
    a, b = evidence.artifact(left), evidence.artifact(right)
    return left['sha256'] == right['sha256'] and a.read_bytes() == b.read_bytes()


def compare_recovered_calls(probe, regression, fixture, expected_libraries, config):
    require(fixture['config'] == config, 'Recovered fixture changed controller configuration')
    for raw in (probe, regression):
        require(raw.get('kind') == 'x650_cpp_recovery_raw_replay_v1' and raw.get('complete') is True
            and raw.get('core_abi') == 2 and raw.get('returned') is True
            and raw.get('memory_commit_valid') is True and raw['memory_before'] == fixture['memory']
            and all(raw['loaded_code'][k]['sha256'] == v for k, v in expected_libraries.items()),
            'Recovered raw call memory/version/library differs')
        d = raw['diagnostics']
        require(d.get('accepted') is True and d.get('nonlinear_passed') is True
            and d.get('actual_bounds') is True and d.get('previous_plan_fallback') is False
            and d.get('recovery_triggered') is True and d.get('recovery_trust_radius') == .02
            and finite_bounded(d.get('compute_ms'), 50.)
            and finite_bounded(raw.get('core_step_wall_ms'), 50.), 'Recovered raw call gate failed')
    require(probe['input_sha256'] == regression['input_sha256']
        and numerical_equal(probe['output'], regression['output'])
        and numerical_equal(probe['memory_before'], regression['memory_before'])
        and numerical_equal(probe['memory_after'], regression['memory_after'])
        and numerical_equal(probe['diagnostics'], regression['diagnostics'], ignored={'compute_ms', 'elapsed_ms'}),
        'Independent recovered calls differ in input/output/memory/branch')


def validate_protocol(protocol, original):
    """Only CPU placement is rebound; every numerical/time/consumption gate stays."""
    require({k: v for k, v in protocol.items() if k not in PROTOCOL_RESOURCE_KEYS}
        == {k: v for k, v in original.items() if k not in PROTOCOL_RESOURCE_KEYS},
        'Recovery changed physical, numerical, timing or consumption protocol')
    roles, prior_roles = protocol.get('cpu_roles', {}), original.get('cpu_roles', {})
    require(set(roles) == set(prior_roles) and 'worker' in roles,
            'Recovery CPU roles differ')
    seen = set()
    for name, values in roles.items():
        require(isinstance(values, list) and len(values) == len(prior_roles[name])
            and values == sorted(set(values))
            and all(type(v) is int and v >= 0 for v in values)
            and not seen.intersection(values), 'Invalid or overlapping recovery CPU role: '+name)
        seen.update(values)
    threads = protocol.get('worker_thread_roles', {})
    require(set(threads) == {'executor', 'auxiliary'} and len(threads['executor']) == 1
        and len(threads['auxiliary']) == len(roles['worker'])-1
        and all(isinstance(v, list) and v == sorted(set(v))
            and all(type(c) is int and c >= 0 for c in v) for v in threads.values())
        and not set(threads['executor']).intersection(threads['auxiliary'])
        and sorted(threads['executor']+threads['auxiliary']) == roles['worker'],
        'Recovery worker thread roles do not partition its CPU mask')
    require(isinstance(protocol.get('cpu_scope'), str) and protocol['cpu_scope'].strip(),
            'Explicit recovery CPU placement scope required')


def verify_software(summary, startup, evidence, binaries):
    require(summary.get('kind') == 'x650_trusted_centre_recovery_software_acceptance_v1'
        and summary.get('complete') is True and summary.get('passed') is True
        and summary.get('physical_validation') is False
        and summary.get('new_closed_loop_trajectory_claimed') is False
        and summary.get('hardware_qualified') is False and summary.get('injected_delay') is False,
        'Explicit complete recovery software acceptance required')
    expected = dict(core_abi=2, initial_trust_radius=.02, formal_recovery_trust_radius=.02,
        maximum_control_qp_attempts=3, maximum_auxiliary_projection_qp_attempts=1,
        maximum_centres=2, original_deadline_ms=50.)
    require(all(summary.get(k) == v for k, v in expected.items()),
            'Recovery ABI, trust radius, search budget or original deadline differs')
    code = summary['code']
    for role, key in [('core', 'core_library'), ('optimization', 'optimization_library'),
                      ('predictor', 'predictor_library')]:
        require(same_artifact(code['libraries'][role], startup[key]), 'Recovery software library differs: '+role)
        evidence.artifact(code['libraries'][role])
    worker = evidence.artifact(code['worker'])
    require(binaries.get(str(worker)) == code['worker']['sha256'],
            'Installed recovery worker differs from actual tested worker')
    installed_sources = {str(evidence.artifact(v)): v['sha256'] for v in startup['source_artifacts']}
    proven_sources = {}
    for item in code['sources']:
        path = evidence.artifact(item)
        require(item['relative_path'] not in proven_sources, 'Repeated software source identity')
        proven_sources[item['relative_path']] = item
        # Summary includes tooling and tests as well as execution sources. The
        # four ABI-sensitive execution files must also be bound by the worker.
        if item['relative_path'] in EXECUTION_SOURCES:
            require(installed_sources.get(str(path)) == item['sha256'],
                    'Current startup omits proven ABI-2 execution source: '+item['relative_path'])
    require(EXECUTION_SOURCES <= proven_sources.keys(), 'Recovery execution source proof incomplete')
    reconstruction = evidence.document(summary['original_prefix_reconstruction'])
    require(reconstruction.get('kind') == 'x650_cpp_original_sequence_reconstruction_v1'
        and all(reconstruction.get(k) is True for k in ('complete', 'all_original_outcomes_reproduced',
            'stopped_at_original_rejection', 'memory_reconstructed_in_cpp'))
        and reconstruction.get('history_from_proposals') is False
        and reconstruction.get('counterfactual_closed_loop_claimed') is False
        and reconstruction.get('physical_validation_claimed') is False
        and reconstruction.get('row_count') == 379 and reconstruction.get('step_count_after') == 378,
        'Original raw failed prefix reconstruction incomplete')
    regression = evidence.document(summary['regression'])
    require(regression.get('kind') == 'x650_cpp_recovery_recording_regression_v1'
        and all(regression.get(k) is True for k in ('complete', 'passed',
            'replay_code_and_original_report_unchanged', 'original_source_and_fixtures_unchanged'))
        and regression.get('closed_loop_or_physical_validation_claimed') is False
        and regression.get('injected_delay') is False
        and regression.get('configuration', {}).get('value') == startup['config']
        and regression.get('expected_case_count') == regression.get('passed_cases') == 380
        and regression.get('failed_cases') == 0 and len(regression.get('rows', [])) == 380
        and same_artifact(regression['original_reconstruction'], summary['original_prefix_reconstruction']),
        'Complete normal/recovery/rejection regression required')
    expected_libraries = {k: v['sha256'] for k, v in code['libraries'].items()}
    require(regression['expected_new_libraries'] == expected_libraries,
            'Regression did not load the selected recovery libraries')
    expected_cases = [(i, 'unchanged') for i in range(1, 379)]+[(379, 'recovered'), ('extra_rejected', 'rejected')]
    for row, (sequence, expectation) in zip(regression['rows'], expected_cases):
        require(row.get('sequence') == sequence and row.get('expectation') == expectation
            and row.get('passed') is True, 'Recovery regression call missing, reordered or failed')
        audit = evidence.document(row['audit'])
        require(audit.get('kind') == 'x650_cpp_recovery_independent_replay_audit_v1'
            and audit.get('complete') is True and audit.get('passed') is True
            and audit.get('expectation') == expectation
            and audit.get('physical_validation_claimed') is False,
            'Independent recovery raw audit did not pass')
        require(same_artifact(audit['artifacts']['input'], row['fixture'])
            and audit.get('input_sha256') == row['fixture']['sha256'], 'Raw audit input differs from regression call')
        for version, libraries in [('old', regression['expected_old_libraries']), ('new', expected_libraries)]:
            raw = evidence.document(row[version]['raw'])
            require(raw.get('kind') == 'x650_cpp_recovery_raw_replay_v1'
                and raw.get('complete') is True
                and all(raw.get('loaded_code', {}).get(role, {}).get('sha256') == digest
                    for role, digest in libraries.items())
                and same_artifact(audit['artifacts'][version], row[version]['raw']),
                'Raw replay did not load the declared version/library: '+version)
    probe = evidence.document(summary['ros_worker_probe'])
    require(probe.get('kind') == 'x650_cpp_recovery_actual_worker_probe_v1'
        and all(probe.get(k) is True for k in ('complete', 'passed', 'test_sequence_complete', 'no_actuator_publishers'))
        and probe.get('physical_validation') is False and probe.get('physical_acceptance') is False
        and probe.get('counterfactual_closed_loop_claimed') is False and probe.get('injected_delay') is False
        and probe.get('rows_count') == 379 and probe.get('child_exit') == 0
        and same_artifact(probe['worker'], code['worker']), 'Actual ABI-2 ROS worker proof incomplete')
    require(probe['recording']['sha256'] == reconstruction['recording_sha256'],
        'ROS worker did not replay the same original prefix')
    recovered_row = regression['rows'][378]
    fixture = evidence.document(recovered_row['fixture'])
    raw_probe, raw_regression = (evidence.document(item) for item in
                               (probe['recovered_report'], recovered_row['new']['raw']))
    require(raw_probe['input_sha256'] == recovered_row['fixture']['sha256'],
            'Actual worker recovered raw proof used a different fixture')
    compare_recovered_calls(raw_probe, raw_regression, fixture, expected_libraries, startup['config'])
    tested = evidence.document(probe['startup'])
    require(tested['config'] == startup['config'] and tested['trajectory'] == startup['trajectory']
        and identical_task_copy(tested['task_config'], startup['task_config'], evidence),
        'Worker tested a different task/model')
    for role, key in [('core', 'core_library'), ('optimization', 'optimization_library'),
                      ('predictor', 'predictor_library')]:
        require(same_artifact(probe['libraries'][role], startup[key])
            and same_artifact(tested[key], startup[key]), 'Worker tested different library: '+role)
    final, recorder = probe['final'], probe['recorder']
    require(final.get('requests') == final.get('accepted') == 379 and final.get('faulted') is False
        and recorder.get('complete') is True and recorder.get('dropped') == 0
        and recorder.get('written') == recorder.get('offered') == 379
        and len(probe.get('records', [])) == len(probe.get('wire_crosscheck', [])) == len(probe.get('rows', [])) == 379,
        'Recovery worker completion, raw records or delivery incomplete')
    for sequence, (row, wire) in enumerate(zip(probe['rows'], probe['wire_crosscheck']), 1):
        require(row.get('sequence') == wire.get('sequence') == sequence and row.get('passed') is True
            and finite_bounded(row.get('native_callback_ms'), 50.)
            and wire.get('snapshot_all_fields_exact') is True and wire.get('proposal_all_fields_exact') is True,
            'Recovery worker input/output identity or original deadline failed')
        require(row.get('recovery_triggered') is (sequence == 379), 'ROS prefix recovery coverage differs')
        for key in ('absolute_errors', 'full_native_output_absolute_errors', 'diagnostic_absolute_errors'):
            require(row.get(key) and all(finite_bounded(v, 1e-8) for v in row[key].values()),
                    'Recovery worker raw/command/prediction discrepancy: '+key)
    return dict(verified=True, core_abi=2, core_library=startup['core_library'], worker=code['worker'],
        regression_calls=380, original_ros_calls=379, original_deadline_ms=50., trust_radius=.02,
        physical_acceptance=False, scope='Software only; fresh physical lifecycle/trajectory audit required')


def verify(manifest_path, *, task_path, startup_path, startup_sha256, protocol_path, binaries):
    evidence, manifest = Evidence(), read(manifest_path)
    required = {'kind', 'simulation_only', 'task_config', 'startup_manifest', 'protocol',
                'recovery_software_acceptance', 'original_robust_qualification'}
    require(set(manifest) == required and manifest['kind'] == 'x650_cpp_recovery_qualification_v1'
        and manifest['simulation_only'] is True, 'Explicit ABI-2 recovery qualification required')
    for key, path in [('task_config', task_path), ('startup_manifest', startup_path), ('protocol', protocol_path)]:
        require(same_artifact(manifest[key], identity(path)), 'Selected recovery artifact differs: '+key)
    require(manifest['startup_manifest']['sha256'] == startup_sha256, 'Selected startup SHA differs')
    cfg = task.validate_task_config(read(evidence.artifact(manifest['task_config'])))
    require(cfg['robustness']['case_id'] == 'wrench_fast'
        and cfg['scenario'] == 'x650_cpp_robust_wrench_v1', 'Original unchanged wrench case required')
    original = evidence.document(manifest['original_robust_qualification'])
    require(original.get('kind') == 'x650_cpp_robustness_qualification_v1'
        and original.get('simulation_only') is True
        and identical_task_copy(original['task_config'], manifest['task_config'], evidence),
        'Original robustness task/qualification identity differs')
    prior = robust.verify_fast_gate(cfg['cpp_fast_certificate'], cfg['frozen_cpp_fast'])
    # The original source identities belong to the archived old executable.
    # Do not try to validate historical source paths against the replaced tree.
    # verify_fast_gate above validates the old source/executable archive bytes.
    old_startup = read(evidence.artifact(original['startup_manifest']))
    startup = evidence.document(manifest['startup_manifest'])
    ignored = {'task_config', 'source_artifacts', 'core_library'}
    require({k: v for k, v in startup.items() if k not in ignored}
        == {k: v for k, v in old_startup.items() if k not in ignored},
        'Recovery changed configuration, reference, QP, TS20 or startup contract')
    require(same_artifact(startup['task_config'], manifest['task_config'])
        and startup['trajectory'] == fast_trajectory()
        and startup.get('kind') == 'cpp_full_pipeline_v1' and startup.get('simulation_only') is True,
        'Recovery startup task/reference differs')
    require(startup['config'] == prior['startup']['config']
        and startup['trajectory'] == prior['startup']['trajectory'], 'Recovery changed accepted numerical configuration')
    for key in ('optimization_library', 'predictor_library'):
        require(same_artifact(startup[key], prior['startup'][key]), 'Recovery changed nominal library: '+key)
    require(not same_artifact(startup['core_library'], prior['startup']['core_library']),
            'Recovery must explicitly bind the separately proven replacement Core')
    protocol = read(evidence.artifact(manifest['protocol']))
    original_protocol = read(evidence.artifact(original['protocol']))
    require(original_protocol == task.protocol(cfg), 'Original disturbance protocol differs')
    validate_protocol(protocol, original_protocol)
    for path, digest in prior['runtime_binaries'].items():
        if Path(path).name in {'isolated_worker', 'libx650_mpc_core.so'}:
            continue
        require(binaries.get(path) == digest, 'Recovery changed unqualified transport/source/QP/TS20 binary: '+path)
    software = verify_software(evidence.document(manifest['recovery_software_acceptance']), startup, evidence, binaries)
    _, physical = task.physical_condition(cfg)
    return dict(kind='x650_cpp_recovery_admission_v1', ready=True, physical_acceptance=False,
        qualification_manifest=identity(manifest_path), physical_condition=physical,
        recovery_software_proof=software, original_fast_certificate=prior['certificate'],
        artifacts_checked=len(evidence.checked), verified_artifacts=list(evidence.checked.values()),
        historical_scope='Archived nominal fast preparation only; ABI-2 Core/worker separately proven; fresh flight required',
        scope='Original fast wrench waveform, nominal physics, PID, MPC values and timing gates; CPU placement rebound; no delay injection')
