"""Read-only admission of new mismatch plants with an unchanged 25/50 Hz core.

The previous complete nominal hold proves this controller/runtime identity,
not acceptance of either changed plant. Each task has its own software probe,
current physical model checks, actual ground checks and subsequent flight audit.
"""
from pathlib import Path

from simulations.x650_cpp_multirate_v1 import qualification as accepted, task as accepted_task, profile
from simulations.x650_cpp_robustness_v1 import task as old_task
from . import task

Evidence, read, sha, identity = accepted.Evidence, accepted.read, accepted.sha, accepted.identity
require, same_artifact, finite_bounded = accepted.require, accepted.same_artifact, accepted.finite_bounded
archive_integrity = accepted.archive_integrity
verify_runtime_interfaces = accepted.verify_runtime_interfaces
RESOURCE_KEYS, LIBRARY_FIELDS = accepted.RESOURCE_KEYS, accepted.LIBRARY_FIELDS
EXECUTION_SOURCES = accepted.EXECUTION_SOURCES | {
    'simulations/x650_cpp_multirate_robustness_v1/task.py',
    'simulations/x650_cpp_multirate_robustness_v1/qualification.py',
    'simulations/x650_cpp_multirate_robustness_v1/run_case.py',
    'simulations/x650_cpp_multirate_robustness_v1/launch.py',
}


def validate_protocol(value, cfg):
    expected = task.protocol(cfg)
    require({k:v for k,v in value.items() if k not in RESOURCE_KEYS}
        == {k:v for k,v in expected.items() if k not in RESOURCE_KEYS},
        'Undeclared rigid-mismatch protocol, timing, protection or reference change')
    accepted.recovery.validate_protocol(value, expected)
    profile.validate(value['effective_timing_profile'])


def verify_software(value, startup, evidence, binaries):
    result = accepted.verify_software(value, startup, evidence, binaries)
    installed = {str(evidence.artifact(v)): v['sha256'] for v in startup['source_artifacts']}
    tested = {row['relative_path']: row for row in value['code']['sources']}
    require(EXECUTION_SOURCES <= set(tested), 'New rigid-mismatch source evidence incomplete')
    for name in EXECUTION_SOURCES:
        row = tested[name]
        path = evidence.artifact(row)
        require(installed.get(str(path)) == row['sha256'], 'Startup omits new tested source: '+name)
    return result


def verify_hold_acceptance(binding, cfg, *, binaries=None, startup=None, evidence=None):
    """Validate the full nominal certificate without passing a mass task to its API."""
    evidence = Evidence() if evidence is None else evidence
    cert = read(evidence.artifact(binding))
    require(cert.get('kind') == 'x650_cpp_multirate_tip_hold_certificate_v1'
        and all(cert.get(k) is True for k in ('passed','complete','certificate','physical_acceptance',
            'full_window_ulog_verified','parameter_restore_verified','native_ground_verified','actuator_log_verified'))
        and cert.get('timing_profile') == profile.NAME and cert.get('hardware_qualified') is False,
        'Complete nominal same-profile hold certificate required before changed-plant trial')
    profile.validate(cert['effective_profile'])
    hold_cfg = accepted_task.validate_task_config(read(evidence.artifact(cert['task_config'])))
    require(hold_cfg['multirate']['case_id'] == 'tip_hold' and hold_cfg['duration_s'] == 30.
        and accepted_task.baseline_config(hold_cfg) == task.baseline_config(cfg)
        and hold_cfg['mpc_config'] == cfg['mpc_config'],
        'Changed-plant trial altered the accepted nominal controller parameters')
    window = cert['window_sim_s']
    require(len(window) == 2 and abs(window[1]-window[0]-30.) <= 1e-9
        and finite_bounded(cert['endpoint']['position_rmse_m'], .05)
        and finite_bounded(cert['endpoint']['position_max_m'], .1)
        and finite_bounded(cert['tilt_peak_deg'], 8.), 'Original hold duration or endpoint/tilt envelope failed')
    artifacts = cert.get('artifact_identities', [])
    require(artifacts and len({v['path'] for v in artifacts}) == len(artifacts), 'Hold lacks original hashed evidence')
    for item in artifacts:
        evidence.artifact(item)
    require(cert.get('artifacts') == {v['path']:v['sha256'] for v in artifacts}, 'Hold artifact index differs')
    if binaries is not None:
        require(cert.get('runtime_binaries_sha256') == binaries, 'Changed-plant trial altered accepted runtime binaries')
    if startup is not None:
        hold_startup = read(evidence.artifact(cert['startup_manifest']))
        require(hold_startup['config'] == startup['config']
            and all(same_artifact(hold_startup[k], startup[k]) for k in LIBRARY_FIELDS.values()),
            'Changed-plant trial altered accepted numerical configuration or library identity')
    return dict(verified=True, certificate=identity(binding['path']), physical_acceptance=True,
        changed_plant_acceptance=False, timing_profile=profile.NAME,
        scope='Complete nominal 30 s hold only; changed plant needs its own new full physical audit')


def verify(manifest_path, *, task_path, startup_path, startup_sha256, protocol_path, binaries):
    evidence, manifest = Evidence(), read(manifest_path)
    require(set(manifest) == {'kind','simulation_only','task_config','startup_manifest','protocol',
        'multirate_software_acceptance','historical_qualification','prior_multirate_hold_acceptance'}
        and manifest['kind'] == 'x650_cpp_multirate_robustness_qualification_v1'
        and manifest['simulation_only'] is True, 'Explicit new rigid-mismatch admission manifest required')
    for key, path in [('task_config',task_path),('startup_manifest',startup_path),('protocol',protocol_path)]:
        require(same_artifact(manifest[key], identity(path)), 'Selected rigid-mismatch input differs: '+key)
    require(manifest['startup_manifest']['sha256'] == startup_sha256, 'Selected startup hash differs')
    cfg = task.validate_task_config(read(evidence.artifact(manifest['task_config'])))
    startup = evidence.document(manifest['startup_manifest'])
    require(startup.get('kind') == 'cpp_full_pipeline_v1' and startup.get('simulation_only') is True
        and startup.get('timing_profile') == profile.NAME
        and same_artifact(startup['task_config'], manifest['task_config'])
        and startup['trajectory'] == task.trajectory(cfg), 'New startup/task/fast reference differs')
    historical = read(evidence.artifact(manifest['historical_qualification']))
    require(historical.get('kind') == 'x650_cpp_robustness_qualification_v1',
        'Original rigid-mismatch input provenance required')
    old_cfg = old_task.validate_task_config(read(evidence.artifact(historical['task_config'])))
    require(task.original(cfg) == old_cfg, 'Original rigid-mismatch definition or fast reference changed')
    validate_protocol(read(evidence.artifact(manifest['protocol'])), cfg)
    prior_hold = manifest['prior_multirate_hold_acceptance']
    require(prior_hold == cfg['multirate']['prior_tip_hold_certificate'], 'Task and admission hold gate differ')
    hold_proof = verify_hold_acceptance(prior_hold, cfg, binaries=binaries, startup=startup, evidence=evidence)
    software = verify_software(evidence.document(manifest['multirate_software_acceptance']), startup, evidence, binaries)
    _, physical = task.physical_condition(cfg)
    return dict(kind='x650_cpp_multirate_robustness_admission_v1', ready=True, physical_acceptance=False,
        case_id=cfg['multirate']['case_id'], timing_profile=profile.NAME, effective_profile=profile.effective(),
        qualification_manifest=identity(manifest_path), physical_condition=physical,
        software_qualification=software, prior_multirate_hold=hold_proof,
        historical_scope='Original mismatch inputs only; no old mismatch timing or flight result inherited',
        artifacts_checked=len(evidence.checked), verified_artifacts=list(evidence.checked.values()),
        scope='Same accepted controller/runtime and timing; new .85/1.15 physical plant; no wrench, delay injection or hardware claim')
