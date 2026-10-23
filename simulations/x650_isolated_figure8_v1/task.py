"""Normal trajectory only, admitted by a complete actual isolated hold audit."""
from copy import deepcopy
import os
from pathlib import Path
from simulations.x650_ts20_hold_v1 import task as hold
from simulations.x650_ts20_figure8_v1 import task as previous

ROOT = hold.ROOT
CONFIG_ENV = hold.CONFIG_ENV
SCENARIO = DAMPED_SCENARIO = 'x650_isolated_figure8_v1'
sha, read, canonical_hash = hold.sha, hold.read, hold.canonical_hash
bound_document = previous.bound_document


def baseline_sha256():
    """The reviewed execution revision selects its accepted complete-hold task."""
    value=read(ROOT/'configs/experiments/x650_isolated_live_v1.json')['frozen_hold_task_sha256']
    if not isinstance(value,str) or len(value)!=64 or set(value)-set('0123456789abcdef'):
        raise ValueError('Declared execution baseline SHA256 is invalid')
    return value


def baseline_config(binding):
    return hold.validate_task_config(bound_document(binding,baseline_sha256()))


def protocol():
    return {'kind': 'x650_isolated_normal_figure8_protocol_v1', 'stages': ['normal'],
            'baseline_task_sha256': baseline_sha256(),
            'amplitude_x_m': .25, 'amplitude_z_m': .10, 'cycles': 2,
            'start_s': 2., 'ramp_s': 2., 'period_s': 12., 'duration_s': 32.,
            'tail_hold_s': 4., 'full_window_required': True,
            'target_alignment': 'same world frame and timestamp; no fitted translation or phase',
            'controller_parameters': 'exact accepted hold task; no retuning',
            'gate': 'new complete actual isolated hold with unchanged numerical backend and execution protocol',
            'fast_admitted': False}


def build_task_config(baseline_binding, live_hold_binding):
    cfg = baseline_config(baseline_binding)
    p = protocol()
    cfg.update(scenario=SCENARIO, stage='normal', trajectory='normal', duration_s=p['duration_s'],
        trajectory_parameters={'amplitude_x': p['amplitude_x_m'], 'amplitude_z': p['amplitude_z_m'],
            'period_s': p['period_s'], 'cycles': p['cycles'], 'start_s': p['start_s'], 'ramp_s': p['ramp_s']},
        frozen_hold=deepcopy(baseline_binding), isolated_live_hold=deepcopy(live_hold_binding),
        figure8_protocol=p)
    return cfg


def validate_task_config(value):
    if not isinstance(value, dict) or value.get('stage') != 'normal':
        raise ValueError('Only normal figure eight is admitted; fast remains a later stage')
    expected = build_task_config(value.get('frozen_hold'), value.get('isolated_live_hold'))
    if canonical_hash(value) != canonical_hash(expected):
        raise ValueError('Frozen controller, numerical backend or normal trajectory changed')
    binding = value['isolated_live_hold']
    if not isinstance(binding, dict) or set(binding) != {'path', 'sha256'}:
        raise ValueError('Explicit reviewed live hold path and SHA256 required')
    return deepcopy(value)


def validate_live_hold(cfg):
    """Verify complete original evidence, not just a user-editable passed flag."""
    binding = cfg['isolated_live_hold']
    cert = bound_document(binding)
    if (cert.get('kind') != 'x650_isolated_live_stage_certificate_v1'
            or cert.get('stage') != 'tip_hold'
            or any(cert.get(k) is not True for k in ('passed', 'complete', 'full_window_ulog_verified',
                'parameter_restore_verified', 'native_ground_verified', 'actuator_log_verified'))):
        raise ValueError('New complete actual isolated endpoint hold certificate required')
    execution = read(ROOT/'configs/experiments/x650_isolated_live_v1.json')
    source = cert.get('source_binding', {})
    baseline=baseline_sha256()
    if (source.get('verified') is not True or source.get('task_sha256') != baseline
            or source.get('native_library_sha256') != execution['frozen_numerical_library_sha256']):
        raise ValueError('Hold certificate belongs to another frozen numerical configuration')
    window = cert.get('window_sim_s', [])
    if len(window) != 2 or abs(window[1]-window[0]-execution['tip_hold_duration_s']) > 1e-9:
        raise ValueError('Hold certificate does not cover the complete declared window')
    if not cert.get('artifacts'):
        raise ValueError('Hold certificate has no original evidence bindings')
    artifacts = {}
    for file, expected in cert['artifacts'].items():
        p = Path(file)
        p = p if p.is_absolute() else Path(binding['path']).parent/p
        if not p.is_file() or sha(p) != expected:
            raise ValueError('Hold evidence changed or missing: '+str(p))
        artifacts[str(p.resolve())] = expected
    launch_dir, run_dir = Path(cert['launch_directory']), Path(cert['source_run_directory'])
    for p in (launch_dir/'launch.json', launch_dir/'task.json', launch_dir/'source_manifest.json',
              launch_dir/'source_manifest_after.json', launch_dir/'sources.zip', run_dir/'controller_metadata.json'):
        if str(p.resolve()) not in artifacts:
            raise ValueError('Hold certificate omits essential evidence: '+str(p))
    launch = read(launch_dir/'launch.json')
    if (launch.get('protocol') != execution or launch.get('complete') is not True
            or launch.get('task_sha256') != baseline
            or sha(launch_dir/'task.json') != baseline
            or str(run_dir) not in launch.get('run_directories', [])
            or any(launch.get(k) is not True for k in ('runtime_closed','sources_unchanged','runtime_binaries_unchanged'))):
        raise ValueError('Hold execution protocol or completed launch identity differs')
    # Changes to the explicit trajectory entry/factory are archived by this
    # new launch. All mathematical sources and low-level transport stay fixed.
    before = read(launch_dir/'source_manifest.json')
    if before != read(launch_dir/'source_manifest_after.json'):
        raise ValueError('Hold source identity changed during its run')
    prefixes = ('control_packages/x650_wholebody_v1/', 'ros2_ws/src/flyarm_control_gateway/',
                'ros2_ws/src/flyarm_control_interfaces/')
    critical = [name for name in before if name.startswith(prefixes)]
    critical += ['ros2_ws/src/flyarm_mpc_reference/flyarm_mpc_reference/'+name
                 for name in ('factory.py','engine.py','isolated_node.py','diagnostic_channel.py')]
    if not critical or any(name not in before or sha(ROOT/name) != before[name] for name in critical):
        raise ValueError('Numerical, typed adapter or transport source changed since the hold')
    for p, expected in launch['runtime_binaries_sha256'].items():
        if not Path(p).is_file() or sha(p) != expected:
            raise ValueError('Execution binary changed since hold: '+p)
    return {**binding, 'stage': 'tip_hold', 'verified': True,
            'original_evidence_files': len(artifacts), 'frozen_source_files': len(critical),
            'execution_protocol_sha256': canonical_hash(execution)}


def selected_task_config():
    return validate_task_config(read(os.environ[CONFIG_ENV]))


def condition(cfg, **kwargs):
    return hold.condition(baseline_config(cfg['frozen_hold']), **kwargs)


def validate_stage_gates(cfg, provenance):
    gates = hold.validate_stage_gates(baseline_config(cfg['frozen_hold']), provenance)
    gates['isolated_live_tip_hold'] = validate_live_hold(cfg)
    return gates


def preflight(cfg, *, check_airframe=True):
    cfg = validate_task_config(cfg)
    model, provenance = hold.preflight(baseline_config(cfg['frozen_hold']), check_airframe=check_airframe)
    from amsim.scenario_registry import get
    descriptor = get(SCENARIO)
    if descriptor.source != 'user' or descriptor.module != 'simulations/x650_isolated_figure8_v1/run_case.py':
        raise ValueError('Dedicated normal isolated trajectory scenario required')
    provenance.update(stage='normal', frozen_hold=cfg['frozen_hold'],
                      figure8_protocol_sha256=canonical_hash(protocol()))
    provenance['stage_gates'] = validate_stage_gates(cfg, provenance)
    return model, provenance
