"""Nominal controller identity is separate from each actual robustness plant."""
from copy import deepcopy
import os
from pathlib import Path
import math
from simulations.x650_cpp_fast_v1 import task as fast
from simulations.x650_cpp_fast_v1 import qualification as prior

ROOT, CONFIG_ENV = fast.ROOT, fast.CONFIG_ENV
read, sha, canonical_hash = fast.read, fast.sha, fast.canonical_hash
bound_document, baseline_config = fast.bound_document, fast.baseline_config
CASES = {
    'wrench_fast': ('x650_cpp_robust_wrench_v1', 'external_wrench', 1.),
    'mass085_fast': ('x650_cpp_robust_mass085_v1', 'uniform_rigid_mismatch', .85),
    'mass115_fast': ('x650_cpp_robust_mass115_v1', 'uniform_rigid_mismatch', 1.15),
}


def schedule():
    return [dict(start_s=7., end_s=7.6, force_world_n=[1.5, 0., 0.], torque_world_nm=[0., 0., 0.]),
            dict(start_s=13., end_s=13.6, force_world_n=[-1.5, 0., 0.], torque_world_nm=[0., 0., 0.]),
            dict(start_s=20.5, end_s=20.9, force_world_n=[0., 0., 0.], torque_world_nm=[0., .04, 0.])]


def trajectory():
    return fast.trajectory()


def fast_config(binding):
    return fast.validate_task_config(bound_document(binding, recurse=False))


def build_task_config(case_id, fast_binding, certificate_binding, *, physical_binding=None,
                      pid_binding=None, disturbance_binding=None):
    if case_id not in CASES:
        raise ValueError('An explicitly reviewed robustness case is required')
    scenario, kind, scale = CASES[case_id]
    cfg = fast_config(fast_binding)
    bound_document(certificate_binding, recurse=False)
    nominal = deepcopy(cfg)
    if kind == 'uniform_rigid_mismatch':
        report = bound_document(physical_binding, recurse=False)
        profile = bound_document(pid_binding, recurse=False)
        original = read(cfg['pid_profile'])
        if (report.get('kind') != 'x650_uniform_rigid_mismatch_v1' or report.get('scale') != scale
                or profile.get('model_sha256') != report['model_sha256']
                or {k:v for k,v in profile.items() if k != 'model_sha256'}
                   != {k:v for k,v in original.items() if k != 'model_sha256'}
                or disturbance_binding is not None):
            raise ValueError('Physical mismatch or unchanged PID value binding differs')
        cfg['model'] = report['model_name']
        cfg['pid_profile'] = pid_binding['path']
        cfg['ts20_inputs']['plant_manifest'] = str(Path(physical_binding['path']).with_name('plant_manifest.json'))
    elif physical_binding is not None or pid_binding is not None or not isinstance(disturbance_binding, dict):
        raise ValueError('External force uses the exact nominal plant and an explicit runtime binding')
    robustness = dict(case_id=case_id, kind=kind, mass_inertia_scale=scale,
        physical_manifest=deepcopy(physical_binding), pid_profile_binding=deepcopy(pid_binding),
        wrench_runtime=deepcopy(disturbance_binding), schedule=schedule() if kind == 'external_wrench' else [],
        recovery_threshold_m=.01, recovery_hold_s=1.)
    cfg.update(scenario=scenario, disturbance_enabled=(kind == 'external_wrench'),
        frozen_cpp_fast=deepcopy(fast_binding), cpp_fast_certificate=deepcopy(certificate_binding),
        robustness=robustness)
    # The reference and all controller fields remain exactly the accepted fast case.
    assert cfg['mpc_config'] == nominal['mpc_config'] and cfg['trajectory_parameters'] == nominal['trajectory_parameters']
    return cfg


def validate_task_config(value):
    if not isinstance(value, dict) or not isinstance(value.get('robustness'), dict):
        raise ValueError('Explicit robustness task required')
    c = value['robustness']
    expected = build_task_config(c.get('case_id'), value.get('frozen_cpp_fast'), value.get('cpp_fast_certificate'),
        physical_binding=c.get('physical_manifest'), pid_binding=c.get('pid_profile_binding'),
        disturbance_binding=c.get('wrench_runtime'))
    if canonical_hash(value) != canonical_hash(expected):
        raise ValueError('Undeclared controller, physical, trajectory or disturbance change')
    return deepcopy(value)


def selected_task_config():
    return validate_task_config(read(os.environ[CONFIG_ENV]))


def protocol(cfg):
    original = read(ROOT/'configs/experiments/x650_cpp_fast_v1.json')
    original.update(kind='x650_cpp_robustness_protocol_v1',
        required_prior_stage='complete_cpp_fast_figure8', robustness=deepcopy(cfg['robustness']),
        delay_tests_enabled=False)
    return original


def physical_condition(cfg, *, check_airframe=True):
    from amsim.modeling.manager import resolve, ensure_flight_ready, validate_px4_airframe
    nominal = fast_config(cfg['frozen_cpp_fast'])
    model = resolve(cfg['model']); ensure_flight_ready(model)
    if model.px4_sys_autostart != 4996 or model.joints != ['arm_joint_1', 'arm_joint_2']:
        raise ValueError('Actual robustness vehicle/airframe differs')
    if check_airframe and sha(validate_px4_airframe(model)) != sha(cfg['airframe_source']):
        raise ValueError('Original PX4 airframe values changed')
    c = cfg['robustness']
    if c['kind'] == 'uniform_rigid_mismatch':
        from verify_x650_robustness_model import verify_model
        result = verify_model(c['physical_manifest']['path'], expected_scale=c['mass_inertia_scale'])
        if (result['model_sha256'] != model.sha256 or result['model_name'] != model.model_name
                or not math.isclose(result['total_mass_kg'], model.total_mass_kg, abs_tol=1e-12)):
            raise ValueError('Loaded physical mismatch differs from its declared report')
    else:
        if cfg['model'] != nominal['model']:
            raise ValueError('Force experiment must retain the nominal physical model')
        from .disturbance import verify_runtime_binding
        verify_runtime_binding(c['wrench_runtime'])
        result = dict(verified=True, scale=1., model_name=model.model_name, model_sha256=model.sha256,
            total_mass_kg=model.total_mass_kg, nominal_total_mass_kg=model.total_mass_kg)
    return model, result


def validate_stage_gates(cfg, provenance):
    from .qualification import verify_fast_gate
    return dict(nominal_complete_fast=verify_fast_gate(cfg['cpp_fast_certificate'], cfg['frozen_cpp_fast']),
        historical_scope='Nominal preparation/control only; no prior flight certificate is transferred to a changed plant',
        new_actual_ground_required=True)


def preflight(cfg, *, check_airframe=True):
    cfg = validate_task_config(cfg)
    nominal = fast_config(cfg['frozen_cpp_fast'])
    _, provenance = fast.preflight(nominal, check_airframe=check_airframe)
    model, physical = physical_condition(cfg, check_airframe=check_airframe)
    from amsim.scenario_registry import get
    descriptor = get(cfg['scenario'])
    if (descriptor.source != 'user' or descriptor.module != 'simulations/'+cfg['scenario']+'/run_case.py'
            or descriptor.config['model'] != model.model_name):
        raise ValueError('Independent robustness scenario/model registration required')
    from research_condition import condition_for_task, frozen_research_pid
    condition = condition_for_task(cfg)
    _, _, pid_hash = frozen_research_pid(cfg)
    if condition['model_sha256'] != model.sha256 or pid_hash != provenance['pid_parameters_sha256']:
        raise ValueError('Actual plant/PID parameter operating condition differs')
    provenance.update(model_name=model.model_name, model_sha256=model.sha256,
        total_mass_kg=model.total_mass_kg, stage='fast', physical_robustness=physical,
        nominal_controller_task=deepcopy(cfg['frozen_cpp_fast']),
        operating_condition=condition, operating_condition_sha256=canonical_hash(condition))
    provenance['ts20_operating_condition'] = dict(kind='x650_cpp_robustness_condition_v1',
        actual_model_sha256=model.sha256, nominal_controller_model_sha256=read(nominal['pid_profile'])['model_sha256'],
        physical_robustness=physical, nominal_actuator_parameters_unchanged=True,
        loaded_hardware_response_qualified=False)
    provenance['ts20_operating_condition_sha256'] = canonical_hash(provenance['ts20_operating_condition'])
    provenance['stage_gates'] = validate_stage_gates(cfg, provenance)
    for p in sorted(Path(__file__).parent.glob('*.py')):
        provenance['source_files_sha256'][p.relative_to(ROOT).as_posix()] = sha(p)
    return model, provenance
