"""Exact fast trajectory composed from the complete accepted normal C++ case."""
from copy import deepcopy
import os

from simulations.x650_cpp_normal_v1 import task as normal

ROOT, CONFIG_ENV = normal.ROOT, normal.CONFIG_ENV
SCENARIO = DAMPED_SCENARIO = 'x650_cpp_fast_v1'
sha, read, canonical_hash = normal.sha, normal.read, normal.canonical_hash
bound_document, baseline_config = normal.bound_document, normal.baseline_config


def trajectory():
    return {'kind': 'fast_figure8', 'amplitude_x': .25, 'amplitude_z': .10,
            'period_s': 8., 'cycles': 2, 'start_s': 2., 'ramp_s': 2., 'duration_s': 24.}


def source_parameters():
    return {'trajectory_kind': 'figure8', **{k: v for k, v in trajectory().items()
            if k not in {'kind', 'duration_s'}}}


def protocol():
    return {'kind': 'x650_cpp_fast_protocol_v1', 'stages': ['fast'],
            'trajectory': trajectory(), 'tail_hold_s': 4., 'required_prior_stage': 'complete_cpp_normal_figure8',
            'target_alignment': 'same world frame and timestamp; no fitted translation or phase',
            'controller_parameters': 'exact accepted C++ normal task; no retuning'}


def normal_config(binding):
    # Validate the original normal task by its original hold and certificate
    # bindings; historical executable identities live in the launch archives.
    return normal.validate_task_config(bound_document(binding, recurse=False))


def build_task_config(normal_binding, certificate_binding):
    cfg = normal_config(normal_binding)
    bound_document(certificate_binding, recurse=False)
    cfg.update(scenario=SCENARIO, stage='fast', trajectory='fast', duration_s=24.,
        trajectory_parameters={k: v for k, v in trajectory().items() if k not in {'kind', 'duration_s'}},
        frozen_cpp_normal=deepcopy(normal_binding), cpp_normal_certificate=deepcopy(certificate_binding),
        figure8_protocol=protocol())
    return cfg


def validate_task_config(value):
    if not isinstance(value, dict) or value.get('stage') != 'fast':
        raise ValueError('Only the separately admitted fast figure eight is permitted')
    expected = build_task_config(value.get('frozen_cpp_normal'), value.get('cpp_normal_certificate'))
    if canonical_hash(value) != canonical_hash(expected):
        raise ValueError('Fast trajectory, original controller or physical parameters changed')
    return deepcopy(value)


def selected_task_config():
    return validate_task_config(read(os.environ[CONFIG_ENV]))


def condition(cfg, **kwargs):
    return normal.condition(normal_config(cfg['frozen_cpp_normal']), **kwargs)


def validate_stage_gates(cfg, provenance):
    from .qualification import verify_normal_gate
    gates = normal.validate_stage_gates(normal_config(cfg['frozen_cpp_normal']), provenance)
    gates['cpp_complete_normal_figure8'] = verify_normal_gate(
        cfg['cpp_normal_certificate'], cfg['frozen_cpp_normal'])
    return gates


def preflight(cfg, *, check_airframe=True):
    cfg = validate_task_config(cfg)
    model, provenance = normal.preflight(normal_config(cfg['frozen_cpp_normal']), check_airframe=check_airframe)
    from amsim.scenario_registry import get
    descriptor = get(SCENARIO)
    if (descriptor.source != 'user' or descriptor.module != 'simulations/x650_cpp_fast_v1/run_case.py'
            or descriptor.config['model'] != cfg['model']):
        raise ValueError('Dedicated fast C++ scenario/model registration required')
    provenance.update(stage='fast', frozen_cpp_normal=cfg['frozen_cpp_normal'],
                      figure8_protocol_sha256=canonical_hash(protocol()))
    provenance['stage_gates'] = validate_stage_gates(cfg, provenance)
    return model, provenance
