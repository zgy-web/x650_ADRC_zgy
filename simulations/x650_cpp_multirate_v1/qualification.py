"""Fail-closed admission for a new timing experiment, never an old certificate."""
from pathlib import Path

from simulations.x650_cpp_live_v1 import qualification as legacy
from simulations.x650_cpp_robustness_v1 import qualification as robust, task as old_task
from simulations.x650_cpp_recovery_v1 import qualification as recovery
from simulations.x650_cpp_interval_v1 import qualification as interval
from . import task, profile

Evidence, read, sha, identity = legacy.Evidence, legacy.read, legacy.sha, legacy.identity
require, same_artifact, finite_bounded = legacy.require, legacy.same_artifact, legacy.finite_bounded
archive_integrity = robust.archive_integrity
verify_runtime_interfaces = legacy.verify_runtime_interfaces
RESOURCE_KEYS = recovery.PROTOCOL_RESOURCE_KEYS
EXECUTION_SOURCES = interval.EXECUTION_SOURCES | {
    'ros2_ws/src/flyarm_control_gateway/include/flyarm_control_gateway/timing_profile.hpp',
    'ros2_ws/src/flyarm_control_gateway/include/flyarm_control_gateway/admission_cadence.hpp',
    'ros2_ws/src/flyarm_control_gateway/include/flyarm_control_gateway/shadow_gate.hpp',
    'ros2_ws/src/flyarm_control_gateway/include/flyarm_control_gateway/periodic_gate.hpp',
    'ros2_ws/src/flyarm_control_gateway/src/state_ingress.cpp',
    'ros2_ws/src/flyarm_control_gateway/src/periodic_dispatcher.cpp',
    'ros2_ws/src/flyarm_mpc_cpp/include/flyarm_mpc_cpp/config_io.hpp',
    'ros2_ws/src/flyarm_mpc_cpp/include/flyarm_mpc_cpp/worker_contract.hpp',
    'ros2_ws/src/flyarm_mpc_cpp/include/flyarm_mpc_cpp/trajectory_plan.hpp',
}
LIBRARY_FIELDS = {'core':'core_library', 'optimization':'optimization_library', 'predictor':'predictor_library'}


def validate_protocol(value, cfg):
    expected = task.protocol(cfg)
    # CPU placement is the only non-timing customization. All timing values,
    # trajectories, physical limits, ownership and progress policy are exact.
    require({k:v for k,v in value.items() if k not in RESOURCE_KEYS}
        == {k:v for k,v in expected.items() if k not in RESOURCE_KEYS},
        'Undeclared multirate protocol or physical/timing protection change')
    recovery.validate_protocol(value, expected)
    profile.validate(value['effective_timing_profile'])


def verify_test_report(binding, *, kind, required, evidence, libraries=None, binaries=None):
    report = evidence.document(binding)
    require(report.get('kind') == kind and report.get('complete') is True and report.get('passed') is True
        and report.get('physical_acceptance') is False and report.get('timing_profile') == profile.NAME,
        'Explicit new-profile software test report required: '+kind)
    profile.validate(report['effective_profile'])
    if libraries is not None:
        require(report.get('libraries') == libraries, 'Native tests used different library identities')
    tests = report.get('tests', [])
    require(tests and len({v['name'] for v in tests}) == len(tests)
        and required <= {v['name'] for v in tests}, 'Necessary profile test cases missing')
    for row in tests:
        evidence.artifact(row['binary'])
        require(row.get('passed') is True and type(row.get('returncode')) is int and row['returncode'] == 0,
            'Profile software test failed: '+row['name'])
        if 'result' in row:
            evidence.artifact(row['result'])
    if binaries is not None:
        require(report.get('runtime_binaries') == binaries, 'Gateway tests do not bind selected runtime binaries')
    return dict(verified=True, report=identity(binding['path']), tests=[v['name'] for v in tests])


def verify_recording_derivation(recording, startup, evidence):
    """Verify decimation/relabeling, never merely trust a preservation flag."""
    require(recording.get('kind') == 'x650_cpp_multirate_derived_recording_v1'
        and recording.get('original_states_and_actual_history_preserved') is True
        and recording.get('physical_validation') is False
        and recording.get('counterfactual_closed_loop_claimed') is False
        and recording.get('timing_profile') == profile.NAME,
        'Explicit software-only low-rate recording derivation required')
    original = evidence.document(recording['original_recording'])
    require(original.get('originals_verified_unchanged') is True, 'Original source recording is unverified')
    original_rows = {v['sequence']:v for v in original['rows']}
    previous = 0.
    ignored = {'sequence','configuration_sha256','dt_s'}
    digest = list(bytes.fromhex(startup['task_config']['sha256']))
    for sequence, binding in enumerate(recording['rows'], 1):
        row = evidence.document(binding)
        index = row.get('original_sequence')
        require(index in original_rows and same_artifact(row['source_row'], original_rows[index]),
            'Derived input lacks its immutable original source row')
        source = evidence.document(original_rows[index])
        snapshot, actual = row['snapshot'], source['snapshot']
        require({k:v for k,v in snapshot.items() if k not in ignored}
            == {k:v for k,v in actual.items() if k not in ignored}
            and snapshot['sequence'] == sequence and snapshot['configuration_sha256'] == digest,
            'Derived replay altered measured state, reference, real history or source timestamps')
        interval = snapshot['task_time_s']-previous
        require(.04-1e-9 <= interval <= .1+1e-10 and abs(snapshot['dt_s']-interval) <= 1e-12,
            'Derived replay fabricated its measured low-rate dt')
        previous = snapshot['task_time_s']
    return dict(verified=True, calls=len(recording['rows']), physical_acceptance=False,
        original_recording=recording['original_recording'])


def verify_worker(probe, startup, evidence, binaries, code):
    count = probe.get('rows_count')
    require(probe.get('kind') == 'x650_cpp_multirate_actual_worker_probe_v1'
        and probe.get('core_abi') == 4 and probe.get('complete') is True and probe.get('passed') is True
        and probe.get('test_sequence_complete') is True and probe.get('timing_profile') == profile.NAME
        and type(count) is int and count >= 20 and probe.get('no_actuator_publishers') is True
        and probe.get('physical_acceptance') is False,
        'New-profile actual ROS worker probe required; replay is not physical acceptance')
    profile.validate(probe['effective_profile'])
    require(same_artifact(probe['worker'], code['worker'])
        and binaries.get(str(Path(code['worker']['path']).resolve())) == code['worker']['sha256'],
        'Selected actual worker differs from profile probe')
    tested = evidence.document(probe['startup'])
    require(tested['config'] == startup['config'] and tested['trajectory'] == startup['trajectory']
        and recovery.identical_task_copy(tested['task_config'], startup['task_config'], evidence),
        'Actual worker tested a different profile, task, model or trajectory')
    actual = evidence.document(probe['worker_startup'])
    require(actual.get('core_abi') == 4 and actual.get('diagnostic_mode') == 'compact'
        and actual.get('python_in_worker') is False and actual.get('startup_controller_steps') == 0
        and actual.get('startup_manifest') == tested and actual.get('timing_profile') == profile.NAME,
        'Actual ABI-4 worker startup or timing selection differs')
    profile.validate(actual['effective_profile'])
    for role, key in LIBRARY_FIELDS.items():
        require(same_artifact(probe['libraries'][role], startup[key])
            and same_artifact(tested[key], startup[key])
            and str(Path(startup[key]['path']).resolve()) == actual['loaded_'+role+'_path'],
            'Actual worker library differs: '+role)
    oracle = evidence.document(probe['oracle'])
    require(oracle.get('kind') == 'x650_cpp_multirate_sequence_oracle_v1'
        and oracle.get('core_abi') == 4 and oracle.get('complete') is True and oracle.get('passed') is True
        and oracle.get('row_count') == oracle.get('step_count_after') == count
        and oracle.get('memory_sequentially_reconstructed_in_cpp') is True
        and oracle.get('original_proposals_used_as_output_oracle') is False
        and oracle.get('recorded_states_and_actual_history_preserved') is True
        and oracle.get('physical_validation') is False and oracle.get('counterfactual_closed_loop_claimed') is False
        and same_artifact(oracle['recording'], probe['recording']),
        'New native oracle must advance its own Memory using preserved actual state/history')
    recording = evidence.document(probe['recording'])
    require(recording.get('config') == startup['config'] and recording.get('trajectory') == startup['trajectory']
        and len(recording.get('rows', [])) == count
        and recording.get('originals_verified_unchanged') is True,
        'Derived multirate replay did not preserve its original recording')
    derivation = verify_recording_derivation(recording, startup, evidence)
    rows, wires = probe.get('rows', []), probe.get('wire_crosscheck', [])
    require(len(rows) == len(wires) == len(probe.get('records', [])) == len(oracle.get('rows', [])) == count,
        'Incomplete profile actual worker/oracle/wire evidence')
    previous = None
    for sequence, (binding, source, row, wire) in enumerate(zip(oracle['rows'], recording['rows'], rows, wires), 1):
        call = evidence.document(binding)
        require(binding.get('sequence') == source.get('sequence') == call.get('sequence') == row.get('sequence')
            == wire.get('sequence') == sequence and call.get('returned') is True
            and same_artifact(call['source_row'], source)
            and call['memory_before']['step_count'] == sequence-1 and call['memory_after']['step_count'] == sequence
            and (previous is None or previous == call['memory_before'])
            and row.get('passed') is True and finite_bounded(row.get('native_callback_ms'), 50.)
            and wire.get('snapshot_all_fields_exact') is True and wire.get('proposal_all_fields_exact') is True,
            'Sequential actual worker identity, Memory, CDR or 50 ms budget failed')
        previous = call['memory_after']
        for field in ('absolute_errors', 'full_native_output_absolute_errors'):
            require(row.get(field) and all(finite_bounded(v, 1e-8) for v in row[field].values()),
                'Actual worker numerical output differs: '+field)
        require(finite_bounded(row.get('diagnostic_absolute_error'), 1e-8), 'Actual worker diagnostics differ')
    require(probe['final'].get('requests') == probe['final'].get('accepted') == count
        and probe['final'].get('faulted') is False and probe['recorder'].get('complete') is True
        and probe['recorder'].get('dropped') == 0
        and probe['recorder'].get('written') == probe['recorder'].get('offered') == count,
        'Actual worker did not finish or retain its complete evidence')
    return dict(verified=True, calls=count, core_abi=4, physical_acceptance=False, input_derivation=derivation,
        scope='New sequential software replay and actual ROS process only')


def verify_software(value, startup, evidence, binaries):
    require(value.get('kind') == 'x650_cpp_multirate_software_acceptance_v1'
        and value.get('core_abi') == 4 and value.get('complete') is True and value.get('passed') is True
        and value.get('physical_acceptance') is False and value.get('timing_profile') == profile.NAME,
        'Explicit ABI-4 multirate software acceptance required')
    profile.validate(value['effective_profile'])
    code = value['code']
    require(code.get('runtime_binaries') == binaries, 'Multirate runtime identities differ from tested binaries')
    for role, key in LIBRARY_FIELDS.items():
        path = evidence.artifact(code['libraries'][role])
        require(same_artifact(code['libraries'][role], startup[key])
            and binaries.get(str(path)) == code['libraries'][role]['sha256'], 'Startup/library proof differs: '+role)
    require(code['libraries']['predictor']['sha256'] == interval.EXPECTED_LIBRARIES['predictor'],
        'The new timing experiment may not silently replace TS20 dynamics')
    installed = {str(evidence.artifact(v)):v['sha256'] for v in startup['source_artifacts']}
    names = set()
    for item in code['sources']:
        path = evidence.artifact(item)
        name = item['relative_path']
        require(name not in names, 'Duplicate tested source identity')
        names.add(name)
        if name in EXECUTION_SOURCES:
            require(installed.get(str(path)) == item['sha256'], 'Startup omits tested source: '+name)
    require(EXECUTION_SOURCES <= names, 'Native/gateway/worker timing source proof incomplete')
    native = verify_test_report(value['native_tests'], kind='x650_cpp_multirate_native_acceptance_v1',
        required={'legacy_default_equivalence','extended_dt_hold_guard'}, evidence=evidence, libraries=code['libraries'])
    gateway = verify_test_report(value['gateway_tests'], kind='x650_cpp_multirate_gateway_acceptance_v1',
        required={'multirate_cadence','lease_repetition','timeout_and_stale_source'}, evidence=evidence, binaries=binaries)
    worker = verify_worker(evidence.document(value['ros_worker_probe']), startup, evidence, binaries, code)
    return dict(verified=True, core_abi=4, native=native, gateway=gateway, worker=worker,
        timing_profile=profile.NAME, effective_profile=profile.effective(), physical_acceptance=False)


def verify_hold_acceptance(binding, cfg, *, binaries=None, startup=None, evidence=None):
    evidence = Evidence() if evidence is None else evidence
    cert = read(evidence.artifact(binding))
    require(cert.get('kind') == 'x650_cpp_multirate_tip_hold_certificate_v1'
        and all(cert.get(k) is True for k in ('passed','complete','certificate','physical_acceptance',
            'full_window_ulog_verified','parameter_restore_verified','native_ground_verified','actuator_log_verified'))
        and cert.get('timing_profile') == profile.NAME and cert.get('hardware_qualified') is False,
        'A fresh same-profile complete physical hold is required before wrench')
    profile.validate(cert['effective_profile'])
    hold_cfg = task.validate_task_config(read(evidence.artifact(cert['task_config'])))
    require(hold_cfg['multirate']['case_id'] == 'tip_hold' and hold_cfg['duration_s'] == 30.
        and task.baseline_config(hold_cfg) == task.baseline_config(cfg)
        and hold_cfg['mpc_config'] == cfg['mpc_config'], 'Hold certificate belongs to different physical/controller parameters')
    window = cert['window_sim_s']
    require(len(window) == 2 and abs(window[1]-window[0]-30.) <= 1e-9
        and finite_bounded(cert['endpoint']['position_rmse_m'], .05)
        and finite_bounded(cert['endpoint']['position_max_m'], .1)
        and finite_bounded(cert['tilt_peak_deg'], 8.), 'Full original hold duration or endpoint/tilt envelope failed')
    artifacts = cert.get('artifact_identities', [])
    require(artifacts and len({v['path'] for v in artifacts}) == len(artifacts), 'Hold lacks original hashed evidence')
    for item in artifacts:
        evidence.artifact(item)
    require(cert.get('artifacts') == {v['path']:v['sha256'] for v in artifacts}, 'Hold artifact index differs')
    if binaries is not None:
        require(cert.get('runtime_binaries_sha256') == binaries, 'Wrench runtime differs from the newly accepted hold')
    if startup is not None:
        hold_startup = read(evidence.artifact(cert['startup_manifest']))
        require(hold_startup['config'] == startup['config']
            and all(same_artifact(hold_startup[k], startup[k]) for k in LIBRARY_FIELDS.values()),
            'Wrench native backend differs from accepted new hold')
    return dict(verified=True, certificate=identity(binding['path']), physical_acceptance=True,
        timing_profile=profile.NAME, scope='New 30 s hold only; wrench still needs its own physical audit')


def verify(manifest_path, *, task_path, startup_path, startup_sha256, protocol_path, binaries):
    evidence, manifest = Evidence(), read(manifest_path)
    require(set(manifest) == {'kind','simulation_only','task_config','startup_manifest','protocol',
        'multirate_software_acceptance','historical_qualification','prior_multirate_hold_acceptance'}
        and manifest['kind'] == 'x650_cpp_multirate_qualification_v1' and manifest['simulation_only'] is True,
        'Explicit new multirate admission manifest required')
    for key, path in [('task_config',task_path),('startup_manifest',startup_path),('protocol',protocol_path)]:
        require(same_artifact(manifest[key], identity(path)), 'Selected multirate artifact differs: '+key)
    require(manifest['startup_manifest']['sha256'] == startup_sha256, 'Selected startup hash differs')
    cfg = task.validate_task_config(read(evidence.artifact(manifest['task_config'])))
    startup = evidence.document(manifest['startup_manifest'])
    require(startup.get('kind') == 'cpp_full_pipeline_v1' and startup.get('simulation_only') is True
        and startup.get('timing_profile') == profile.NAME
        and same_artifact(startup['task_config'], manifest['task_config'])
        and startup['trajectory'] == task.trajectory(cfg), 'New startup/reference/task differs')
    historical = read(evidence.artifact(manifest['historical_qualification']))
    require(historical.get('kind') == 'x650_cpp_robustness_qualification_v1', 'Original physical preparation binding required')
    old_cfg = old_task.validate_task_config(read(evidence.artifact(historical['task_config'])))
    prior = robust.verify_fast_gate(old_cfg['cpp_fast_certificate'], old_cfg['frozen_cpp_fast'])
    require(task.baseline_config(cfg) == old_task.baseline_config(old_cfg['frozen_cpp_hold']),
        'New task changed the historically prepared physical/PID baseline')
    if cfg['multirate']['case_id'] == 'wrench_fast':
        require(task.original(cfg) == old_cfg, 'Original three-pulse waveform or fast trajectory changed')
    expected = dict(prior['startup']['config'], max_dt=.1, command_hold_s=.16)
    require(startup['config'] == expected, 'New timing changed undeclared numerical/physical parameters')
    validate_protocol(read(evidence.artifact(manifest['protocol'])), cfg)
    # Only native Core/QP, worker, ingress and dispatcher are replaced. Raw
    # feedback conversion, actual-consumption source and TS20 dynamics retain
    # their separately proven identities.
    replacements = {'isolated_worker','state_ingress','live_dispatcher','libx650_mpc_core.so','libx650_optimization.so'}
    for path, digest in prior['runtime_binaries'].items():
        if Path(path).name not in replacements:
            copies = [value for name,value in binaries.items() if Path(name).name == Path(path).name]
            require(copies == [digest], 'Unqualified feedback/consumption/predictor change: '+path)
    software = verify_software(evidence.document(manifest['multirate_software_acceptance']), startup, evidence, binaries)
    prior_hold = manifest['prior_multirate_hold_acceptance']
    require(prior_hold == cfg['multirate']['prior_tip_hold_certificate'], 'Task and admission hold gate differ')
    if cfg['multirate']['case_id'] == 'tip_hold':
        require(prior_hold is None, 'Initial new hold may not inherit physical acceptance')
        hold_proof = None
    else:
        hold_proof = verify_hold_acceptance(prior_hold, cfg, binaries=binaries, startup=startup, evidence=evidence)
    _, physical = task.physical_condition(cfg)
    return dict(kind='x650_cpp_multirate_admission_v1', ready=True, physical_acceptance=False,
        case_id=cfg['multirate']['case_id'], timing_profile=profile.NAME, effective_profile=profile.effective(),
        qualification_manifest=identity(manifest_path), physical_condition=physical,
        software_qualification=software, prior_multirate_hold=hold_proof,
        historical_scope='Original nominal plant/PID preparation only; old timing/flight qualification not transferred',
        artifacts_checked=len(evidence.checked), verified_artifacts=list(evidence.checked.values()),
        scope='New multi-rate simulation only; unchanged physical safety/performance limits; no delay injection or hardware claim')
