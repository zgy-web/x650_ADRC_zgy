"""Offline raw-input/independent-oracle export for the owning C++ controller.

No simulator or actuator I/O. The reference has no timing qualification role:
its deadline is extended only in this process, explicitly recorded, and restored
after each call. No production Python source or C++ deadline is modified.
"""
from __future__ import annotations
import argparse
from contextlib import contextmanager
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'scripts'), str(ROOT / 'control_packages/x650_wholebody_v1')]
from profile_x650_ts20_matrix_fixture import plain, file_identity


def configuration(c):
    p = c.actuator_predictor
    model = c.model
    response = c.response_model['parameters']
    return plain(dict(nodes=c.nodes, max_iterations=c.max_iterations,
        node_dt=c.node_dt, max_dt=c.max_dt, feasibility_tolerance=c.feasibility_tolerance,
        kkt_tolerance=c.kkt_tolerance, arm_share=c.arm_share,
        joint_position_gain=c.joint_position_gain,
        weights=[c.weights[k] for k in ('tip_position', 'tip_velocity', 'base_position',
                                      'joint_posture', 'input_nominal', 'input_delta')],
        velocity_limit=c.velocity_limit, acceleration_limit=c.acceleration_limit,
        q_lower=c.q_lower, q_upper=c.q_upper,
        physical_lower=model.joint_lower, physical_upper=model.joint_upper,
        geometry=[*model.lengths, *model.angle_offsets, *model.mount_body, *model.elbow_offset_body_m],
        response=[response['x']['wn_rad_s'], response['x']['zeta'],
                  response['y']['wn_rad_s'], response['y']['zeta'], response['z']['tau_s']],
        base_position_gain=c.base_position_gain,
        motion=[p.J, p.b, p.Fc, p.offset, p.delay_s, p.Fs, p.delta, p.vs],
        kp=p.kp, kd=p.kd, bus_period=p.bus_period_s,
        integration_step=p.max_step_s, trust_radius=p.trust_radius_normalized))


def memory(c):
    return plain(dict(initialized=c.initialized, history_primed=c._history_primed,
        has_last_t=c.last_t is not None, has_stamp=c._previous_stamp is not None,
        has_origin=c._chain_clock_origin is not None,
        anchor=dict(position=c.p0, rotation=c.R0.ravel(), q=c.q0),
        previous_command=c.previous_command,
        last_t=c.last_t or 0., previous_stamp=c._previous_stamp or 0.,
        origin=c._chain_clock_origin or 0., plan_t=c._plan_t or 0., step_count=c.step_count,
        plan=[] if c._plan is None else c._plan.ravel(),
        plan_h=[] if c._plan_h is None else c._plan_h,
        joint_history=[dict(time=when, velocity=u) for when, u in c._joint_history]))


def target(value):
    return plain({key: getattr(value, key) for key in ('position', 'velocity', 'acceleration')})


def raw_input(c, s, goal, t, dt):
    context = c.context_provider(s, t)
    h = np.full(c.nodes, c.node_dt); h[0] = dt
    state = {k: getattr(s, k) for k in ('position', 'velocity', 'omega_body', 'q', 'dq')}
    state.update(rotation=np.asarray(s.rotation).ravel(), acceleration=s.acceleration_enu,
                 stamp=s.stamp_s, control_time=s.control_time_s, joint_stamp=s.joint_stamp_s)
    return plain(dict(t=t, dt=dt, state=state, target=target(goal),
        preview=[target(c._target(t+x, goal, x)) for x in np.cumsum(h)],
        context=dict(reference_time=context['reference_state']['time_s'],
            reference=context['reference_state']['r'], bus_origin=context['bus_origin_s'],
            commands=[dict(time=when, velocity=u) for when, u in context['command_history']],
            packets=[dict(send_time=p['send_time_s'], **{k:p[k] for k in
                ('q_ref', 'dq_ref', 'tau_ff', 'kp', 'kd')}) for p in context['packet_history']])))


@contextmanager
def offline_math_reference():
    from x650_wholebody.ts20_mpc import RELINEARIZATION_POLICY
    original = RELINEARIZATION_POLICY['total_deadline_ms']
    if original != 50.:
        raise ValueError('Production reference deadline is unexpectedly different')
    RELINEARIZATION_POLICY['total_deadline_ms'] = 3_600_000.
    try:
        yield
    finally:
        RELINEARIZATION_POLICY['total_deadline_ms'] = original


def case(c, p, s, goal, *, t=0., dt=.02, label=None):
    from packed_candidate_capture import CandidateExecutionCapture
    item = dict(schema_version=1, label=label, config=configuration(c), memory=memory(c),
                input=raw_input(c, s, goal, t, dt))
    problems = []
    previous = c.__dict__.get('_problem')
    original = c._problem
    def observed(*args):
        result = original(*args)
        problems.append({key: result[key].copy() for key in
                         ('A','b','H','g','C','lo','hi')})
        return result
    c._problem = observed
    capture = CandidateExecutionCapture(p)
    begin = time.perf_counter()
    try:
        with offline_math_reference(), capture:
            result = c.step(t, dt, s, goal)
    finally:
        if previous is None:
            c.__dict__.pop('_problem', None)
        else:
            c._problem = previous
    d = result['diagnostics']
    item['oracle'] = plain(dict(accepted=d['accepted_optimum'], nonlinear_passed=d['ts20_nonlinear_check_passed'],
        command=np.r_[result['base_velocity_cmd'], result['qd_cmd']],
        plan=np.asarray(d['predicted_commands']).ravel(),
        predicted_states=np.asarray(d['predicted_states']).ravel(),
        predicted_tip=np.asarray(d['predicted_tip_positions']).ravel(),
        objective=d['objective'], subproblems=len(d['ts20_relinearization_attempts']),
        attempts=d['ts20_relinearization_attempts'], candidate_executions=capture.records,
        problems=problems, memory_after=memory(c),
        reference_wall_ms=(time.perf_counter()-begin)*1000,
        scope='independent Python math oracle only; timing does not qualify production',
        production_deadline_ms=50., offline_reference_deadline_ms=3_600_000.))
    return item


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--matrix-script', type=Path, required=True)
    ap.add_argument('--matrix-sha256', required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args(argv)
    from validate_x650_native_numerics import frozen_constructor, source_receipt
    a.output.mkdir(parents=True, exist_ok=False)
    build = frozen_constructor(a.matrix_script, a.matrix_sha256)
    specs = build()
    if len(specs) != 30:
        raise ValueError('Frozen matrix must contain exactly 30 cases')
    before = source_receipt()
    report = dict(kind='cpp_owning_core_raw_input_oracle_v1', no_flight=True,
        original_fixture=file_identity(a.matrix_script), exporter=file_identity(__file__),
        no_production_source_modified=True, online_deadline_unchanged_ms=50.,
        reference_timing_is_not_a_gate=True, rows=[], complete=False)
    for index, spec in enumerate(specs):
        c, p, s, goal = build(spec)
        value = case(c, p, s, goal, label=dict(index=index, fixture=spec))
        path = a.output / f'case_{index:03d}.json'
        path.write_text(json.dumps(value, separators=(',', ':'), allow_nan=False), encoding='utf-8')
        report['rows'].append(file_identity(path))
        print(json.dumps(dict(exported=index+1, count=len(specs))), flush=True)
    report.update(complete=True, source_files_unchanged=source_receipt()==before)
    (a.output/'manifest.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    return 0 if report['source_files_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
