"""Derive a new timing experiment without inheriting a flight certificate.

Hold derives from the exact original quiet hold task; wrench derives from the
exact original three-pulse fast task. All changed fields are rebuilt explicitly.
"""
from copy import deepcopy
import os
from pathlib import Path

from simulations.x650_cpp_normal_v1 import task as normal
from simulations.x650_cpp_robustness_v1 import task as robust
from simulations.x650_ts20_hold_v1 import task as hold
from . import profile

ROOT, CONFIG_ENV = hold.ROOT, hold.CONFIG_ENV
read, sha, canonical_hash = normal.read, normal.sha, normal.canonical_hash
bound_document = normal.bound_document
SCENARIOS = {'tip_hold': 'x650_cpp_multirate_hold_v1', 'wrench_fast': 'x650_cpp_multirate_wrench_v1'}


def original_config(case_id, binding):
    value = bound_document(binding, recurse=False)
    if case_id == 'tip_hold':
        cfg = hold.validate_task_config(value)
        if (cfg['stage'] != 'tip_hold' or cfg['controller'] != 'mpc' or cfg['duration_s'] != 30.
                or cfg['disturbance_enabled'] or 'numerical_backend' in cfg):
            raise ValueError('Exact original quiet 30 s complete-C++ hold task required')
        return cfg
    if case_id == 'wrench_fast':
        cfg = robust.validate_task_config(value)
        if cfg['robustness']['case_id'] != 'wrench_fast':
            raise ValueError('Original three-pulse wrench task required')
        return cfg
    raise ValueError('Only separately admitted tip_hold and wrench_fast are enabled')


def build_task_config(case_id, original_task, *, hold_certificate=None):
    cfg = original_config(case_id, original_task)
    if case_id == 'tip_hold' and hold_certificate is not None:
        raise ValueError('New hold does not borrow a physical acceptance certificate')
    if case_id == 'wrench_fast':
        bound_document(hold_certificate, recurse=False)
    cfg['scenario'] = SCENARIOS[case_id]
    cfg['timing_profile'] = profile.NAME
    cfg['mpc_config'].update(max_dt_s=.1, command_hold_s=.16)
    cfg['ts20_protocol'].update(kind='x650_ts20_multirate_conditional_protocol_v1',
        control_period_s=.04, max_control_interval_s=.1, max_compute_s=.05,
        command_hold_s=.16, timing_profile=profile.NAME)
    cfg['multirate'] = dict(kind='x650_cpp_multirate_task_derivation_v1', case_id=case_id,
        timing_profile=profile.NAME, original_task=deepcopy(original_task),
        prior_tip_hold_certificate=deepcopy(hold_certificate), physical_acceptance_inherited=False,
        hardware_qualified=False)
    return cfg


def validate_task_config(value):
    if not isinstance(value, dict) or not isinstance(value.get('multirate'), dict):
        raise ValueError('Explicit multirate task derivation required')
    m = value['multirate']
    expected = build_task_config(m.get('case_id'), m.get('original_task'),
        hold_certificate=m.get('prior_tip_hold_certificate'))
    if canonical_hash(value) != canonical_hash(expected):
        raise ValueError('Undeclared multirate model, PID, reference, limits or waveform change')
    return deepcopy(value)


def selected_task_config():
    return validate_task_config(read(os.environ[CONFIG_ENV]))


def original(cfg):
    return original_config(cfg['multirate']['case_id'], cfg['multirate']['original_task'])


def baseline_config(cfg):
    source = original(cfg)
    return source if cfg['multirate']['case_id'] == 'tip_hold' else robust.baseline_config(source['frozen_cpp_hold'])


def trajectory(cfg):
    return {'kind': 'hold'} if cfg['multirate']['case_id'] == 'tip_hold' else robust.trajectory()


def protocol(cfg):
    cfg = validate_task_config(cfg)
    source = original(cfg)
    value = (read(ROOT/'configs/experiments/x650_cpp_live_v1.json')
        if cfg['multirate']['case_id'] == 'tip_hold' else robust.protocol(source))
    value.update(kind='x650_cpp_multirate_protocol_v1', timing_profile=profile.NAME,
        effective_timing_profile=profile.effective(), nominal_solve_period_s=.04,
        output_period_s=.02, command_age_max_s=.16, control_interval_max_s=.1,
        source_wall_age_max_s=.025, core_compute_max_s=.05, output_interval_max_s=.04,
        duration_s=cfg['duration_s'], trajectory=trajectory(cfg), delay_tests_enabled=False,
        required_prior_stage=('new_multirate_software' if cfg['multirate']['case_id'] == 'tip_hold'
            else 'complete_same_multirate_tip_hold'),
        next_solve_commit='nominal 40 ms admission; latest causal state; matching actual writer send; measured dt; no fabricated timestamps',
        qualification_scope='New experimental timing; historical physical certificates are preparation only')
    return value


def physical_condition(cfg, *, check_airframe=True):
    source = original(cfg)
    if cfg['multirate']['case_id'] == 'wrench_fast':
        return robust.physical_condition(source, check_airframe=check_airframe)
    model, provenance = hold.preflight(source, check_airframe=check_airframe)
    return model, dict(verified=True, scale=1., model_name=model.model_name,
        model_sha256=model.sha256, total_mass_kg=model.total_mass_kg,
        nominal_total_mass_kg=model.total_mass_kg,
        source_urdf_sha256=provenance['source_urdf_sha256'])


def validate_stage_gates(cfg, provenance):
    result = dict(historical_preparation_task=deepcopy(cfg['multirate']['original_task']),
        historical_scope='Original plant/PID preparation only; no timing or flight acceptance inherited',
        new_timing_profile=profile.NAME, new_actual_ground_required=True)
    if cfg['multirate']['case_id'] == 'wrench_fast':
        from .qualification import verify_hold_acceptance
        result['new_multirate_tip_hold'] = verify_hold_acceptance(
            cfg['multirate']['prior_tip_hold_certificate'], cfg)
    return result


def preflight(cfg, *, check_airframe=True):
    cfg = validate_task_config(cfg)
    source = original(cfg)
    api = hold if cfg['multirate']['case_id'] == 'tip_hold' else robust
    model, provenance = api.preflight(source, check_airframe=check_airframe)
    from amsim.scenario_registry import get
    descriptor = get(cfg['scenario'])
    if (descriptor.source != 'user' or descriptor.module != 'simulations/'+cfg['scenario']+'/run_case.py'
            or descriptor.config['model'] != cfg['model']):
        raise ValueError('Dedicated multirate scenario/model registration required')
    provenance = deepcopy(provenance)
    provenance.update(stage=cfg['stage'], timing_profile=profile.NAME,
        effective_timing_profile=profile.effective(), multirate_derivation=deepcopy(cfg['multirate']))
    # Motor/model and PID identities are retained; this new condition describes
    # the changed upper timing and cannot masquerade as its old certificate.
    provenance['ts20_operating_condition'].update(kind='x650_ts20_multirate_condition_v1',
        timing_profile=profile.NAME, effective_timing_profile=profile.effective(),
        historical_physical_preparation_only=True)
    provenance['ts20_operating_condition_sha256'] = canonical_hash(provenance['ts20_operating_condition'])
    provenance['stage_gates'] = validate_stage_gates(cfg, provenance)
    for path in sorted(Path(__file__).parent.glob('*.py')):
        provenance['source_files_sha256'][path.relative_to(ROOT).as_posix()] = sha(path)
    return model, provenance
