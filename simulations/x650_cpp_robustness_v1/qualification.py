"""Reuse exact fast controller evidence while binding each distinct physical case."""
from pathlib import Path
from simulations.x650_cpp_fast_v1 import qualification as fast
from simulations.x650_cpp_live_v1 import qualification as hold
from . import task

read, sha, identity = fast.read, fast.sha, fast.identity
require, Evidence, archive_integrity = fast.require, fast.Evidence, fast.archive_integrity
fast_trajectory, figure8 = fast.fast_trajectory, fast.figure8


def verify_fast_gate(binding, baseline_binding):
    evidence = Evidence()
    cert = read(evidence.artifact(binding)); baseline = task.fast_config(baseline_binding)
    require(cert.get('kind') == 'x650_cpp_fast_stage_certificate_v1'
        and all(cert.get(k) is True for k in ('passed','complete','certificate','physical_acceptance',
            'full_window_ulog_verified','parameter_restore_verified','native_ground_verified','actuator_log_verified')),
        'Complete original fast physical certificate required')
    require(len(cert['window_sim_s']) == 2 and abs(cert['window_sim_s'][1]-cert['window_sim_s'][0]-24.) < 1e-9,
            'Complete 24 s prior fast required')
    actual = {str(evidence.artifact(a)): a['sha256'] for a in cert['artifact_identities']}
    require(actual == cert['artifacts'] and len(actual) == len(cert['artifact_identities']),
            'Original fast artifact index differs')
    launch, run = Path(cert['launch_directory']), Path(cert['run_directory'])
    record = read(launch/'launch.json'); meta = read(run/'controller_metadata.json')
    require(all(record.get(k) is True for k in ('complete','runtime_closed','sources_unchanged',
        'runtime_binaries_unchanged','launch_inputs_unchanged')) and record.get('exit_code') == 0
        and record.get('execution_scenario') == 'x650_cpp_fast_v1'
        and record.get('run_directories') == [str(run.resolve())], 'Historical fast lifecycle differs')
    require(baseline == read(launch/'task.json') == meta['task'] == record['task']
        and all(meta.get(k) is True for k in ('complete','landed_and_disarmed','diagnostic_parameters_restored')),
        'Historical fast task/recovery differs')
    sources, binaries = archive_integrity(launch, record)
    return dict(verified=True, certificate=identity(Path(binding['path'])), baseline_task=identity(Path(baseline_binding['path'])),
        launch_directory=str(launch), run_directory=str(run), startup=read(launch/'startup_manifest.json'),
        protocol=read(launch/'protocol.json'), runtime_binaries=binaries, original_artifacts=len(actual),
        historical_scope='Nominal physical history only; does not qualify a changed robustness plant')


def verify(manifest_path, *, task_path, startup_path, startup_sha256, protocol_path, binaries):
    evidence = Evidence(); manifest = read(manifest_path)
    required = {'kind','simulation_only','task_config','startup_manifest','protocol','nominal_qualification'}
    require(set(manifest) == required and manifest['kind'] == 'x650_cpp_robustness_qualification_v1'
        and manifest['simulation_only'] is True, 'Explicit robustness qualification required')
    for key, path in [('task_config',task_path),('startup_manifest',startup_path),('protocol',protocol_path)]:
        require(hold.same_artifact(manifest[key],identity(path)), 'Selected robustness identity differs: '+key)
    require(manifest['startup_manifest']['sha256'] == startup_sha256, 'Selected startup hash differs')
    cfg = task.validate_task_config(read(evidence.artifact(manifest['task_config'])))
    prior = verify_fast_gate(cfg['cpp_fast_certificate'], cfg['frozen_cpp_fast'])
    startup = evidence.document(manifest['startup_manifest'])
    require({k:v for k,v in startup.items() if k != 'task_config'}
        == {k:v for k,v in prior['startup'].items() if k != 'task_config'},
        'Robustness changed the accepted worker, mathematics, trajectory or numerical libraries')
    require(hold.same_artifact(startup['task_config'],manifest['task_config']), 'Actual startup task differs')
    nominal_manifest = evidence.document(manifest['nominal_qualification'])
    nominal_proof = fast.verify(manifest['nominal_qualification']['path'],
        task_path=Path(nominal_manifest['task_config']['path']),
        startup_path=Path(nominal_manifest['startup_manifest']['path']),
        startup_sha256=nominal_manifest['startup_manifest']['sha256'],
        protocol_path=Path(nominal_manifest['protocol']['path']), binaries=binaries)
    require(nominal_manifest['task_config']['sha256'] == cfg['frozen_cpp_fast']['sha256'],
            'Qualified nominal controller is not the accepted fast task')
    require(read(evidence.artifact(manifest['protocol'])) == task.protocol(cfg),
            'Robustness schedule, physical scale or original execution protection changed')
    _, physical = task.physical_condition(cfg)
    return dict(ready=True, kind='x650_cpp_robustness_admission_v1', case_id=cfg['robustness']['case_id'],
        qualification_manifest=identity(manifest_path), artifacts_checked=len(evidence.checked),
        physical_condition=physical, nominal_software_proof=nominal_proof,
        original_fast_certificate=prior['certificate'], physical_acceptance=False,
        scope='Exact same binary/config/reference; task identity rebound to declared new physical test; no delay injection')


verify_runtime_interfaces = hold.verify_runtime_interfaces
