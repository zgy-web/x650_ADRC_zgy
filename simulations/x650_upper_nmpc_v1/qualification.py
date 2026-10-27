"""Bound software admission and fresh-stage sequencing; no inherited flight result."""
from pathlib import Path
import hashlib
import json

from simulations.x650_cpp_live_v1.qualification import (
    read, sha, identity, require, verify_runtime_interfaces)
from simulations.x650_cpp_robustness_v1.qualification import archive_integrity
from simulations.x650_cpp_recovery_v1.qualification import PROTOCOL_RESOURCE_KEYS, validate_protocol as resource_protocol
from . import task

RESOURCE_KEYS = PROTOCOL_RESOURCE_KEYS


def config_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def document(binding):
    require(isinstance(binding, dict) and {'path', 'sha256'} <= set(binding), 'Missing explicit artifact identity')
    path = Path(binding['path']).resolve(strict=True)
    require(path.is_file() and sha(path) == binding['sha256'], 'Bound artifact changed: '+str(path))
    if 'bytes' in binding:
        require(path.stat().st_size == binding['bytes'], 'Bound artifact length changed')
    return read(path)


def validate_protocol(value, cfg):
    expected = task.protocol(cfg)
    require({k:v for k,v in value.items() if k not in RESOURCE_KEYS}
            == {k:v for k,v in expected.items() if k not in RESOURCE_KEYS},
            'Unspecified upper comparison protocol change')
    resource_protocol(value, expected)


def verify_prior_stage(binding, cfg, provenance):
    value = document(binding)
    selection = cfg['upper_controller']
    previous = {'normal': 'hold', 'fast': 'normal'}.get(selection['case_id'])
    require(previous is not None and value.get('kind') == 'x650_upper_closed_loop_audit_v1'
            and all(value.get(k) is True for k in ('passed','complete','execution_passed','performance_passed'))
            and value.get('backend') == selection['backend'] and value.get('case_id') == previous,
            'Fresh same-backend preceding-stage physical acceptance required')
    require(value['model_sha256'] == provenance['model_sha256'], 'Preceding physical model changed')
    launch = document(value['launch'])
    require(launch.get('complete') is True and launch['task_sha256'] == value['task_sha256'],
            'Preceding acceptance lacks its complete original launch')
    old_cfg = launch['task']
    require(old_cfg['mpc_config'] == cfg['mpc_config'] and old_cfg['model'] == cfg['model'],
            'Preceding controller parameters/model differ')
    startup = document(launch['startup_manifest'])
    require(config_sha256(startup['config']) == value['config_sha256'], 'Preceding core configuration hash differs')
    for item in value.get('artifact_identities', []):
        path = Path(item['path'])
        require(path.is_file() and sha(path) == item['sha256'], 'Preceding original evidence changed')
    return dict(verified=True, kind=value['kind'], backend=selection['backend'], prior_case=previous,
                report=identity(binding['path']), physical_acceptance_inherited=False)


def verify(manifest_path, *, task_path, startup_path, startup_sha256, protocol_path, binaries, options_path=None):
    manifest = read(manifest_path)
    require(manifest.get('kind') == 'x650_upper_qualification_v1' and manifest.get('simulation_only') is True,
            'Explicit new upper-controller software admission required')
    for name, path in [('task_config',task_path),('startup_manifest',startup_path),('protocol',protocol_path)]:
        document(manifest[name])
        require(sha(path) == manifest[name]['sha256'], 'Selected launch input differs: '+name)
    require(sha(startup_path) == startup_sha256, 'Startup request hash differs')
    cfg = task.validate_task_config(read(task_path))
    protocol, startup = read(protocol_path), read(startup_path)
    validate_protocol(protocol, cfg)
    options = document(manifest['options'])
    if options_path is not None:
        require(sha(options_path) == manifest['options']['sha256'], 'Selected options differ')
    require(options['backend'] == cfg['upper_controller']['backend']
        and options['task_config_sha256'] == startup['task_config']['sha256'] == sha(task_path)
        and startup['trajectory'] == task.trajectory(cfg), 'Backend/task/reference binding differs')
    require(options['options'] == dict(max_proposals=4,absolute_decrease=1e-10,relative_decrease=1e-7,
        proposal_budget_ms=40,total_budget_ms=50), 'Upper comparison numerical policy changed')
    for item in options['source_artifacts'] + startup['source_artifacts']:
        require(sha(item['path']) == item['sha256'], 'Bound execution source changed')
    require(binaries == manifest['runtime_binaries'], 'Qualified runtime binaries changed')
    worker = options['worker_executable']
    require(binaries.get(str(Path(worker['path']).resolve())) == worker['sha256'], 'Selected worker not in actual runtime')
    software = document(manifest['software'])
    require(software.get('kind') == 'x650_upper_software_acceptance_v1'
        and software.get('passed') is True and software.get('complete') is True
        and software.get('physical_acceptance') is False and software['runtime_binaries'] == binaries,
        'New software tests must bind these exact executables')
    test_names = set()
    for suite in software['suites']:
        report = document(suite)
        require(report.get('passed') is True and report.get('tests'), 'Native/ROS software suite failed or empty')
        for result in report['tests']:
            test_names.add(result['name'])
            require(result['returncode'] == 0, 'Software test failed: '+result['name'])
            for name in ('binary','log'):
                item = result[name]
                require(sha(item['path']) == item['sha256'], 'Software test evidence changed')
    require(len(software['suites']) >= 2, 'Native and ROS suites both required')
    require({'nmpc_offline_transaction','nmpc_online_admission','nmpc_nonlinear_objective'} <= test_names,
            'Missing nonlinear transaction, online deadline or independent objective test')
    probe = document(software['worker_probe'])
    require(probe.get('kind') == 'x650_upper_worker_probe_v1' and probe.get('passed') is True
        and probe.get('complete') is True and probe.get('physical_acceptance') is False
        and probe.get('no_actuator_publishers') is True
        and probe['backend'] == options['backend']
        and probe['worker']['sha256'] == worker['sha256'], 'Actual selected worker software probe failed')
    cases = {row['name']:row for row in probe['cases']}
    require({'healthy','stale_issue','unsafe_state'} <= set(cases)
        and all(row['passed'] is True for row in cases.values()), 'Missing healthy/deadline/unsafe-state rejection evidence')
    require(probe.get('artifact_identities'), 'Worker probe has no original evidence identities')
    for item in probe['artifact_identities']:
        require(sha(item['path']) == item['sha256'], 'Worker probe evidence changed')
    # Probe references may be the hold reference; trajectory derivation is
    # independently checked by task/source tests, never presented as new flight.
    proof_options = document(probe['options'])
    proof_startup = document(probe['startup'])
    require(proof_options['options'] == options['options'] and proof_options['backend'] == options['backend'],
            'Probe used different numerical options')
    require(proof_startup['config'] == startup['config']
        and all(proof_startup[k] == startup[k] for k in ('core_library','optimization_library','predictor_library')),
        'Probe used a different prediction model or numerical library')
    _, provenance = task.preflight(cfg)
    return dict(kind='x650_upper_admission_v1', ready=True, physical_acceptance=False,
        backend=options['backend'], case_id=cfg['upper_controller']['case_id'],
        qualification_manifest=identity(manifest_path), software=manifest['software'],
        stage_gates=task.validate_stage_gates(cfg, provenance))
