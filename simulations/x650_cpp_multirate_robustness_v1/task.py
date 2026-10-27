"""Derive two physical mismatch trials without changing controller parameters.

The original .85/1.15 mass/inertia definitions remain immutable. The same
accepted multirate timing profile is added; no old mismatch acceptance is
inherited and every new flight still requires independent physical acceptance.
"""
from copy import deepcopy
import os
from pathlib import Path

from simulations.x650_cpp_robustness_v1 import task as robust
from simulations.x650_cpp_multirate_v1 import profile

ROOT, CONFIG_ENV = robust.ROOT, robust.CONFIG_ENV
read, sha, canonical_hash = robust.read, robust.sha, robust.canonical_hash
bound_document = robust.bound_document
SCENARIOS = {
    'mass085_fast': 'x650_cpp_multirate_mass085_v1',
    'mass115_fast': 'x650_cpp_multirate_mass115_v1',
}
SCALES = {'mass085_fast': .85, 'mass115_fast': 1.15}


def original_config(case_id, binding):
    if case_id not in SCENARIOS:
        raise ValueError('Only the two declared .85/1.15 rigid mismatch trials are enabled')
    cfg = robust.validate_task_config(bound_document(binding, recurse=False))
    c = cfg['robustness']
    if (c['case_id'] != case_id or c['kind'] != 'uniform_rigid_mismatch'
            or c['mass_inertia_scale'] != SCALES[case_id] or c['schedule'] != []
            or c['wrench_runtime'] is not None or cfg['disturbance_enabled'] is not False
            or cfg['duration_s'] != 24.):
        raise ValueError('Exact original 24 s rigid mismatch without wrench required')
    return cfg


def build_task_config(case_id, original_task, *, hold_certificate):
    cfg = original_config(case_id, original_task)
    bound_document(hold_certificate, recurse=False)
    cfg['scenario'] = SCENARIOS[case_id]
    cfg['timing_profile'] = profile.NAME
    cfg['mpc_config'].update(max_dt_s=.1, command_hold_s=.16)
    cfg['ts20_protocol'].update(kind='x650_ts20_multirate_conditional_protocol_v1',
        control_period_s=.04, max_control_interval_s=.1, max_compute_s=.05,
        command_hold_s=.16, timing_profile=profile.NAME)
    cfg['multirate'] = dict(kind='x650_cpp_multirate_robustness_task_derivation_v1',
        case_id=case_id, timing_profile=profile.NAME, original_task=deepcopy(original_task),
        prior_tip_hold_certificate=deepcopy(hold_certificate),
        physical_acceptance_inherited=False, hardware_qualified=False)
    return cfg


def validate_task_config(value):
    if not isinstance(value, dict) or not isinstance(value.get('multirate'), dict):
        raise ValueError('Explicit new multirate rigid-mismatch derivation required')
    m = value['multirate']
    expected = build_task_config(m.get('case_id'), m.get('original_task'),
        hold_certificate=m.get('prior_tip_hold_certificate'))
    if canonical_hash(value) != canonical_hash(expected):
        raise ValueError('Undeclared controller, timing, physical mismatch or reference change')
    return deepcopy(value)


def selected_task_config():
    return validate_task_config(read(os.environ[CONFIG_ENV]))


def original(cfg):
    return original_config(cfg['multirate']['case_id'], cfg['multirate']['original_task'])


def baseline_config(cfg):
    return robust.baseline_config(original(cfg)['frozen_cpp_hold'])


def trajectory(cfg):
    return robust.trajectory()


def protocol(cfg):
    cfg = validate_task_config(cfg)
    value = robust.protocol(original(cfg))
    value.update(kind='x650_cpp_multirate_robustness_protocol_v1', timing_profile=profile.NAME,
        effective_timing_profile=profile.effective(), nominal_solve_period_s=.04,
        output_period_s=.02, command_age_max_s=.16, control_interval_max_s=.1,
        source_wall_age_max_s=.025, core_compute_max_s=.05, output_interval_max_s=.04,
        duration_s=cfg['duration_s'], trajectory=trajectory(cfg), delay_tests_enabled=False,
        required_prior_stage='complete_same_multirate_tip_hold',
        next_solve_commit='nominal 40 ms admission; latest causal state; matching actual writer send; measured dt; no fabricated timestamps',
        qualification_scope='Accepted controller and timing; newly admitted changed physical plant; no old mismatch result transferred')
    return value


def physical_condition(cfg, *, check_airframe=True):
    return robust.physical_condition(original(cfg), check_airframe=check_airframe)


def validate_stage_gates(cfg, provenance):
    from .qualification import verify_hold_acceptance
    return dict(historical_preparation_task=deepcopy(cfg['multirate']['original_task']),
        historical_scope='Original mismatch definition and nominal preparation only; no changed-plant flight acceptance inherited',
        new_timing_profile=profile.NAME, new_actual_ground_required=True,
        new_multirate_tip_hold=verify_hold_acceptance(cfg['multirate']['prior_tip_hold_certificate'], cfg))


def preflight(cfg, *, check_airframe=True):
    cfg = validate_task_config(cfg)
    model, provenance = robust.preflight(original(cfg), check_airframe=check_airframe)
    from amsim.scenario_registry import get
    descriptor = get(cfg['scenario'])
    if (descriptor.source != 'user' or descriptor.module != 'simulations/'+cfg['scenario']+'/run_case.py'
            or descriptor.config['model'] != model.model_name):
        raise ValueError('Independent multirate rigid-mismatch scenario/model registration required')
    provenance = deepcopy(provenance)
    provenance.update(stage=cfg['stage'], timing_profile=profile.NAME,
        effective_timing_profile=profile.effective(), multirate_derivation=deepcopy(cfg['multirate']))
    provenance['ts20_operating_condition'].update(kind='x650_ts20_multirate_rigid_mismatch_condition_v1',
        timing_profile=profile.NAME, effective_timing_profile=profile.effective(),
        historical_physical_preparation_only=True, changed_plant_acceptance_inherited=False)
    provenance['ts20_operating_condition_sha256'] = canonical_hash(provenance['ts20_operating_condition'])
    provenance['stage_gates'] = validate_stage_gates(cfg, provenance)
    for path in sorted(Path(__file__).parent.glob('*.py')):
        provenance['source_files_sha256'][path.relative_to(ROOT).as_posix()] = sha(path)
    return model, provenance
