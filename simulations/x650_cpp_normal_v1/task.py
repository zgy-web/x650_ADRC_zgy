"""Exact normal task composition from an accepted full C++ hold, without retuning."""
from copy import deepcopy
import os
from pathlib import Path

from simulations.x650_ts20_hold_v1 import task as hold
from simulations.x650_cpp_live_v1 import qualification as evidence

ROOT = hold.ROOT
CONFIG_ENV = hold.CONFIG_ENV
SCENARIO = DAMPED_SCENARIO = 'x650_cpp_normal_v1'
sha, read, canonical_hash = hold.sha, evidence.read, hold.canonical_hash


def trajectory():
    return {'kind': 'normal_figure8', 'amplitude_x': .25, 'amplitude_z': .10,
            'period_s': 12., 'cycles': 2, 'start_s': 2., 'ramp_s': 2., 'duration_s': 32.}


def source_parameters():
    return {'trajectory_kind': 'figure8', **{k: v for k, v in trajectory().items()
            if k not in {'kind', 'duration_s'}}}


def protocol():
    return {'kind': 'x650_cpp_normal_protocol_v1', 'stages': ['normal'],
            'trajectory': trajectory(), 'tail_hold_s': 4., 'fast_admitted': False,
            'target_alignment': 'same world frame and timestamp; no fitted translation or phase',
            'controller_parameters': 'exact accepted C++ hold task; no retuning'}


def bound_document(binding, *, recurse=True):
    if (not isinstance(binding, dict) or set(binding) != {'path', 'sha256'}
            or not isinstance(binding['sha256'], str) or len(binding['sha256']) != 64
            or set(binding['sha256']) - set('0123456789abcdef')):
        raise ValueError('Explicit path and lowercase SHA256 binding required')
    checked = evidence.Evidence()
    return checked.document(binding) if recurse else read(checked.artifact(binding))


def baseline_config(binding):
    cfg = hold.validate_task_config(bound_document(binding))
    if (cfg['stage'] != 'tip_hold' or cfg['controller'] != 'mpc' or cfg['duration_s'] != 30.
            or cfg['disturbance_enabled'] or 'numerical_backend' in cfg):
        raise ValueError('Accepted quiet complete-C++ 30 s MPC hold task required')
    return cfg


def build_task_config(baseline_binding, certificate_binding):
    cfg = baseline_config(baseline_binding)
    # Validate identity shape/bytes now. Full acceptance is checked in preflight.
    # Historical source paths inside the hold's software proof may now point
    # to the explicitly qualified dynamic worker. verify_hold_gate validates
    # those historical bytes from its immutable archives, not current sources.
    bound_document(certificate_binding, recurse=False)
    cfg.update(scenario=SCENARIO, stage='normal', trajectory='normal', duration_s=32.,
        trajectory_parameters={k: v for k, v in trajectory().items() if k not in {'kind', 'duration_s'}},
        frozen_cpp_hold=deepcopy(baseline_binding), cpp_hold_certificate=deepcopy(certificate_binding),
        figure8_protocol=protocol())
    return cfg


def validate_task_config(value):
    if not isinstance(value, dict) or value.get('stage') != 'normal':
        raise ValueError('Only normal figure eight is admitted; fast requires a later stage')
    expected = build_task_config(value.get('frozen_cpp_hold'), value.get('cpp_hold_certificate'))
    if canonical_hash(value) != canonical_hash(expected):
        raise ValueError('Normal trajectory, original controller or physical parameters changed')
    return deepcopy(value)


def selected_task_config():
    return validate_task_config(read(os.environ[CONFIG_ENV]))


def condition(cfg, **kwargs):
    return hold.condition(baseline_config(cfg['frozen_cpp_hold']), **kwargs)


def validate_stage_gates(cfg, provenance):
    from .qualification import verify_hold_gate
    gates = hold.validate_stage_gates(baseline_config(cfg['frozen_cpp_hold']), provenance)
    gates['cpp_complete_tip_hold'] = verify_hold_gate(cfg['cpp_hold_certificate'], cfg['frozen_cpp_hold'])
    return gates


def preflight(cfg, *, check_airframe=True):
    cfg = validate_task_config(cfg)
    model, provenance = hold.preflight(baseline_config(cfg['frozen_cpp_hold']), check_airframe=check_airframe)
    from amsim.scenario_registry import get
    descriptor = get(SCENARIO)
    if (descriptor.source != 'user' or descriptor.module != 'simulations/x650_cpp_normal_v1/run_case.py'
            or descriptor.config['model'] != cfg['model']):
        raise ValueError('Dedicated normal C++ scenario/model registration required')
    provenance.update(stage='normal', frozen_cpp_hold=cfg['frozen_cpp_hold'],
                      figure8_protocol_sha256=canonical_hash(protocol()))
    provenance['stage_gates'] = validate_stage_gates(cfg, provenance)
    return model, provenance
