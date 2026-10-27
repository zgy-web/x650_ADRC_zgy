#!/usr/bin/env python3
"""Independent physical audit of new upper NMPC / callable-MPC stages.

Full diagnostics are checked as Full; no old record is relabelled or patched.
Execution integrity and physical performance have separate outcomes. The raw
PX4, source, actual-send, consumption and landing evidence remain authoritative.
This is simulation acceptance, not a hardware or hard-real-time certificate.
"""
from pathlib import Path
import argparse
import json
import math
import sys
import traceback
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'src'), str(ROOT/'scripts'), str(ROOT/'control_packages/x650_wholebody_v1')]
import audit_x650_cpp_multirate as base
from audit_x650_cpp_multirate import (profile, identity, require, read, rows,
    check_timing_selection, check_hold_guard, check_runtime_recoveries,
    check_runtime_intervals, check_actual_send_barrier, check_ticks,
    check_recovery, upper_commands, check_handoff_bridge)
from audit_x650_cpp_hold import same, summary, numeric_array
from audit_x650_cpp_multirate_followup import observation_value_view
from simulations.x650_upper_nmpc_v1 import qualification, task

IMPLEMENTATIONS = {'nmpc': 'cpp_upper_nmpc_v1', 'mpc_callable': 'cpp_callable_mpc_v1'}
DURATIONS = {'hold': 30., 'normal': 32., 'fast': 24.}
FROZEN_BASE_SHA256 = 'd2a83b5e40abb6013784233000f258bd8d750131ca5eca06b1c01c5040ff5a2d'
FROZEN_FOLLOWUP_SHA256 = '69b4ed49238c4517f06607e83587848fa4def2b395bd971fc4dd47b41c51849a'


def check_case(cfg, protocol):
    selection = cfg['upper_controller']
    backend, case = selection['backend'], selection['case_id']
    require(backend in IMPLEMENTATIONS and case in DURATIONS
        and cfg['duration_s'] == protocol['duration_s'] == DURATIONS[case]
        and protocol['backend'] == backend and protocol['case_id'] == case
        and cfg['disturbance_enabled'] is False,
        'Only the declared complete quiet hold/normal/fast paired stages are admitted')
    require(protocol['trajectory'] == task.trajectory(cfg)
        and cfg['timing_profile'] == profile.NAME
        and protocol['effective_timing_profile'] == profile.effective(),
        'Declared reference or accepted timing profile changed')
    limits = cfg['ts20_protocol']
    require(limits['tip_rms_max_m'] == .05 and limits['tip_peak_max_m'] == .1
        and limits['tilt_max_deg'] == 8., 'Original physical performance limits changed')
    return backend, case


def check_full_qp(d, config):
    """Read actual full matrices; independently check hard rows and KKT.

    The assembler can omit identically zero feasible rows, so locate original
    trust rows before the five first-command rows and appended interval cuts.
    No row is invented and the dimension remains 5*N (no slack variable).
    """
    from scipy.optimize import nnls
    n, m = d['qp_columns'], d['qp_rows']
    require(type(n) is int and n == 5*config['nodes'] and type(m) is int,
            'Full QP dimensions or no-slack policy changed')
    cuts = d['interval_rows']; original_rows = m-cuts
    require(type(cuts) is int and 0 <= cuts <= 512
        and 9*config['nodes']+5 <= original_rows <= 15*config['nodes']+5,
        'Full QP original/cut row count differs')
    H = numeric_array(d['H'], n*n, 'H').reshape(n,n)
    g = numeric_array(d['g'], n, 'g')
    C = numeric_array(d['C'], m*n, 'C').reshape(m,n)
    lo, hi = (numeric_array(d[key], m, key) for key in ('lower','upper'))
    z = numeric_array(d['solution'], n, 'solution')
    require(np.all(lo <= hi) and np.max(abs(H-H.T)) <= 1e-8, 'Full QP bounds/Hessian invalid')
    np.linalg.cholesky(H)
    primal = max(0., float((lo-C@z).max()), float((C@z-hi).max()), float((abs(z)-1).max()))
    require(primal <= config['feasibility_tolerance'], 'Full hard QP row or input bound violated')
    plan = numeric_array(d['plan'], n, 'plan')
    require(same(plan, z*np.tile(config['velocity_limit'], config['nodes'])),
            'Physical plan differs from actual normalized QP solution')
    for i in range(config['nodes']):
        for j in range(2):
            row = original_rows-5-2*config['nodes']+2*i+j
            require(np.array_equal(C[row], np.eye(n)[5*i+3+j])
                and abs(hi[row]-lo[row]-.04) <= 1e-12,
                'Original normalized .02 trust width or coordinate changed')
    normals = np.vstack((C,-C,np.eye(n),-np.eye(n)))
    bounds = np.r_[hi,-lo,np.ones(2*n)]
    active = bounds-normals@z < 1e-6
    gradient = H@z+g
    multipliers = (nnls(normals[active].T,-gradient,
        maxiter=max(1000,30*int(active.sum())))[0] if active.any() else np.empty(0))
    residual = float(abs(gradient+normals[active].T@multipliers).max())
    require(residual <= config['kkt_tolerance'], 'Independent full-QP KKT residual exceeded')
    # A/b are the captured weighted residual objective, not a synthetic matrix.
    b = np.asarray(d['b'], dtype=float)
    require(b.ndim == 1 and len(b)>0 and np.isfinite(b).all(), 'Missing full residual vector')
    numeric_array(d['A'], len(b)*n, 'A')
    require(d['nominal_packets'] and d['candidate_packets'] and d['query_times'],
            'Full actual propagation/packet evidence missing')
    return dict(verified=True, rows=m, columns=n, no_slack_variables=True,
        primal_violation=primal, kkt_stationarity_independent=residual)


def objective_value(value):
    components = numeric_array(value['components'], 6, 'nonlinear cost components')
    total = value['total']
    require(type(total) in (int,float) and np.isfinite(total) and total >= 0
        and np.all(components>=0) and math.isclose(total, math.fsum(components), rel_tol=1e-12, abs_tol=1e-12),
        'Nonlinear objective component/total mismatch')
    return float(total)


def check_full_prediction(snapshot, config, d, previous_command):
    """Check the copied full native propagation against its exact packet grid.

    This verifies actual immutable packet prefix, future quantization, nodes,
    and linearization discrepancies. It does not launch a second simulation.
    """
    fixture = base.interval_fixture(snapshot,config,previous_command)
    # Explicit schema conversion only: the ROS serializer names send_time_s,
    # whereas the independent mathematical packet helper uses send_time.
    packets = [dict(send_time=p['send_time_s'], **{k:p[k] for k in ('q_ref','dq_ref','tau_ff','kp','kd')})
               for p in d['candidate_packets']]
    base.replay.check_packets(fixture,packets,d['plan'])
    h,nodes,queries,boundaries = base.interval.canonical_grid(fixture,packets)
    base.interval.close(d['horizon'],h,1e-12,'full actual horizon')
    base.interval.close(d['query_times'],queries,1e-12,'full canonical query grid')
    base.interval.close(d['candidate_boundary_times'],boundaries,1e-12,'full canonical integration grid')
    states = numeric_array(d['candidate_states'],4*len(queries),'candidate_states').reshape(-1,4)
    predicted = numeric_array(d['predicted_states'],12*config['nodes'],'predicted_states').reshape(-1,12)
    linear = numeric_array(d['linearized_states'],12*config['nodes'],'linearized_states').reshape(-1,12)
    actual = []
    for at in nodes:
        indices = np.flatnonzero(abs(np.asarray(queries)-at)<=1e-9)
        require(len(indices)==1,'Full canonical prediction node missing')
        actual.append(states[indices[0]])
    actual = np.asarray(actual)
    base.interval.close(predicted[:,6:10],actual,1e-13,'actual nonlinear prediction nodes')
    qe = float(abs(actual[:,:2]-linear[:,6:8]).max())
    ve = float(abs(actual[:,2:]-linear[:,8:10]).max())
    require(qe<=.006 and ve<=.10 and abs(qe-d['q_linearization_error'])<=1e-10
        and abs(ve-d['dq_linearization_error'])<=1e-10,'Full nonlinear surrogate discrepancy differs')
    envelope = numeric_array(d['candidate_envelope'],8,'candidate_envelope')
    require(np.all(states[:,:2]>=envelope[:2]-1e-12) and np.all(states[:,:2]<=envelope[2:4]+1e-12)
        and np.all(states[:,2:]>=envelope[4:6]-1e-12) and np.all(states[:,2:]<=envelope[6:8]+1e-12),
        'Full candidate query lies outside native continuous envelope')
    return dict(verified=True,query_count=len(queries),immutable_packets=len(fixture['input']['context']['packets']),
        nonlinear_q_error=qe, nonlinear_dq_error=ve)


def check_callable_record(value, snapshot, proposal, config):
    require(value['backend'] == 'mpc_callable' and 'nmpc' not in value
        and 0 <= proposal.compute_end_monotonic_ns-snapshot.issued_monotonic_ns <= 50_000_000,
        'Callable MPC backend was relabelled or original request deadline exceeded')
    return dict(verified=True, scope='Actual unchanged native result, full QP and original issue deadline; no nonlinear trial claim')


def check_online_record(value, snapshot, proposal, config, backend):
    """Check bounded nonlinear acceptance independently of the local QP cost."""
    if backend == 'mpc_callable':
        return check_callable_record(value, snapshot, proposal, config)
    online = value['nmpc']
    require(online['kind'] == 'x650_online_nmpc_call_v1'
        and online['accepted_by_solver'] is True
        and online['status'] == 'ready_checked_native_candidate'
        and online['offline_publication_flag'] is False
        and online['issue_age_finite'] is True,
        'Online NMPC did not independently accept a fresh native candidate')
    completed = online['completed_monotonic_ns']
    require(type(completed) is int and snapshot.issued_monotonic_ns <= completed <= proposal.compute_end_monotonic_ns
        and 0 <= online['call_elapsed_ms'] <= 50.
        and 0 <= online['issue_age_ms'] <= 50.
        and abs(online['issue_age_ms']-(completed-snapshot.issued_monotonic_ns)/1e6) <= 1e-9
        and online['call_elapsed_ms'] <= online['issue_age_ms']+1e-9,
        'Online NMPC re-aged source or exceeded original issue deadline')
    iterations = online['iterations']; selected = online['selected_iteration']
    require(isinstance(iterations,list) and 1 <= len(iterations) <= 4
        and type(selected) is int and 0 <= selected < len(iterations)
        and [row['index'] for row in iterations] == list(range(len(iterations))),
        'NMPC proposal count or selected index invalid')
    accepted = []; previous = None; last_elapsed = 0.
    for row in iterations:
        require(type(row['elapsed_ms']) in (int,float) and np.isfinite(row['elapsed_ms'])
            and last_elapsed <= row['elapsed_ms'] <= online['nonlinear_total_ms']+1e-9
            and type(row['accepted']) is bool, 'Nonlinear proposal timing/order invalid')
        last_elapsed = row['elapsed_ms']
        if row['accepted']:
            current = objective_value(row['objective'])
            require(row['native_passed'] is True and row['stopping_passed'] is True
                and 0 <= row['stopping_violation'] <= config['feasibility_tolerance']
                and 0 <= row['native_compute_ms'] <= 50. and row['elapsed_ms'] <= 40.
                and row['reason'] == ('first_feasible_proposal' if previous is None else 'nonlinear_objective_decreased'),
                'Selected nonlinear iteration lacks original native/budget gates')
            if previous is not None:
                require(current < previous-(1e-10+1e-7*abs(previous)), 'Nonlinear acceptance lacks strict exact-objective decrease')
            accepted.append(row); previous = current
        else:
            require(row is iterations[-1] and isinstance(row['reason'],str) and row['reason'],
                    'Rejected nonlinear trial was not terminal or has no reason')
    require(accepted and accepted[-1]['index'] == selected
        and online['first_objective'] == accepted[0]['objective']
        and online['selected_objective'] == accepted[-1]['objective'],
        'Selected nonlinear objective does not belong to the last accepted feasible trial')
    objective_value(online['selected_objective'])
    require(0 <= online['actual_stopping_violation'] <= config['feasibility_tolerance'],
            'Selected actual nonlinear stopping reserve exceeded')
    # A rejected later proposal may exhaust 40 ms while the earlier safe pair
    # is retained. It may NEVER renew the original 50 ms publication lease.
    allowed = min(40., 50.-(online['issue_age_ms']-online['call_elapsed_ms']))
    require(0 <= online['nonlinear_total_ms'] <= online['call_elapsed_ms']+1e-9
        and (abs(online['nonlinear_total_ms']-allowed) <= 1e-9
             or online['within_proposal_budget'] == (online['nonlinear_total_ms'] <= allowed))
        and accepted[-1]['elapsed_ms'] <= allowed+1e-9,
        'Nonlinear timing evidence disagrees with bounded proposal scope')
    return dict(verified=True, proposals=len(iterations), selected_iteration=selected)


def endpoint_observations(run, start, cfg, metric):
    """Same-time physical tip errors on declared windows; never phase-align."""
    import audit_x650_isolated_live as old
    case = cfg['upper_controller']['case_id']; duration = DURATIONS[case]
    if case == 'hold':
        target = old.shared.ConstantTarget(np.asarray(start['target_position_enu']))
        windows = [('endpoint',(0.,duration))]
    else:
        from simulations.x650_wholebody_mpc_v1.run_case import FigureEight
        parameters = {k:v for k,v in task.trajectory(cfg).items() if k not in ('kind','duration_s')}
        target = FigureEight(np.asarray(start['target_position_enu']), **parameters)
        windows = [('endpoint',(0.,duration)), ('tracking_endpoint',(2.,duration-4.)),
                   ('tail_endpoint',(duration-4.,duration))]
    limits = cfg['ts20_protocol']; result = {}; acceptance = {}
    for name, window in windows:
        value = metric(run, target.sample, start['origin_s'], window)
        require(value.get('same_time') is True and value.get('no_phase_alignment') is True
            and tuple(value.get('interval_task_s',())) == window and np.isfinite(value['duration_s'])
            and value['duration_s'] >= window[1]-window[0]-.04,
            'Original full same-time endpoint window incomplete: '+name)
        require(all(np.isfinite(value[k]) and value[k]>=0 for k in ('position_rmse_m','position_max_m')),
                'Nonfinite actual endpoint observation: '+name)
        result[name] = value
        acceptance[name] = dict(rms_passed=value['position_rmse_m']<=limits['tip_rms_max_m'],
            peak_passed=value['position_max_m']<=limits['tip_peak_max_m'],
            rms_limit_m=limits['tip_rms_max_m'], peak_limit_m=limits['tip_peak_max_m'])
        acceptance[name]['passed'] = acceptance[name]['rms_passed'] and acceptance[name]['peak_passed']
    return result, dict(passed=all(v['passed'] for v in acceptance.values()), windows=acceptance,
        metrics_recorded_before_performance_classification=True, thresholds_unchanged=True)


def classify_performance(endpoint, tilt, limits):
    require(np.isfinite(tilt) and tilt>=0., 'Nonfinite actual physical tilt')
    errors = []
    for name, value in endpoint['windows'].items():
        if not value['rms_passed']:
            errors.append(dict(check=name+'.rmse', reason='Original .05 m RMSE exceeded'))
        if not value['peak_passed']:
            errors.append(dict(check=name+'.peak', reason='Original .10 m peak exceeded'))
    if tilt > limits['tilt_max_deg']:
        errors.append(dict(check='tilt', reason='Original 8 degree peak exceeded'))
    return dict(passed=not errors, endpoint_passed=endpoint['passed'],
        tilt_passed=tilt<=limits['tilt_max_deg'], tilt_peak_deg=float(tilt),
        tilt_limit_deg=limits['tilt_max_deg'], thresholds_unchanged=True,
        performance_errors=errors)


def check_archives(launch, run, meta):
    record = read(launch/'launch.json')
    require(all(record.get(k) is True for k in ('complete','runtime_closed','sources_unchanged',
        'runtime_binaries_unchanged','launch_inputs_unchanged')) and record.get('exit_code') == 0,
        'Incomplete or changing new upper-controller launch')
    sources, binaries = qualification.archive_integrity(launch, record)
    cfg, protocol, startup = (read(launch/name) for name in ('task.json','protocol.json','startup_manifest.json'))
    require(task.validate_task_config(cfg) == cfg == record['task'] == meta['task'] == read(run/'controller_task.json')
        and record.get('execution_scenario') == cfg['scenario']
        and record.get('run_directories') == [str(run.resolve())] and protocol == record['protocol']
        and record['task_sha256'] == qualification.sha(launch/'task.json'),
        'Actual new task/reference/window identity differs')
    qualification.validate_protocol(protocol,cfg); check_case(cfg,protocol)
    for item in record['launch_inputs']:
        require(identity(item['path']) == item, 'Launch input changed')
    original = read(launch/'original_startup_manifest.json')
    require({k:v for k,v in startup.items() if k!='task_config'}
        == {k:v for k,v in original.items() if k!='task_config'}
        and startup['task_config']['sha256'] == original['task_config']['sha256'] == record['task_sha256'],
        'Startup derivation changed more than the path of identical task bytes')
    manifest = read(launch/'qualification_manifest.json')
    proof = qualification.verify(launch/'qualification_manifest.json', task_path=Path(manifest['task_config']['path']),
        startup_path=Path(manifest['startup_manifest']['path']), startup_sha256=manifest['startup_manifest']['sha256'],
        protocol_path=Path(manifest['protocol']['path']), binaries=binaries, options_path=launch/'options.json')
    require(proof.get('kind') == 'x650_upper_admission_v1' and proof.get('ready') is True
        and proof.get('physical_acceptance') is False, 'Software admission cannot replace this new physical window')
    for key, name in [('task_config','task.json'),('protocol','protocol.json'),
                      ('startup_manifest','original_startup_manifest.json'),('options','options.json')]:
        require(manifest[key]['sha256'] == qualification.sha(launch/name), 'Archived admission input differs: '+key)
    required = set(task.execution_sources(cfg)) | {
        'simulations/x650_upper_nmpc_v1/qualification.py', 'simulations/x650_upper_nmpc_v1/launch.py',
        'scripts/audit_x650_upper_controller.py',
        'control_packages/x650_nmpc_controller_v1/src/online_solver.cpp',
        'control_packages/x650_nmpc_controller_v1/src/offline_solver.cpp',
        'control_packages/x650_nmpc_controller_v1/src/objective.cpp',
        'ros2_ws/src/flyarm_nmpc_cpp/src/isolated_worker.cpp',
        'ros2_ws/src/flyarm_nmpc_cpp/include/flyarm_nmpc_cpp/send_commit.hpp'}
    require(required <= set(sources), 'Archive omits new solver/worker/actual-send transaction source')
    _, provenance = task.preflight(cfg)
    require(provenance == record['provenance'], 'Current physical/model preparation differs from launch')
    return cfg, protocol, startup, record, proof


def check_worker_startup(actual, startup, start, options, backend):
    require(actual.get('core_abi') == 4 and actual['startup_manifest'] == startup
        and actual['python_in_worker'] is False and actual['startup_controller_steps'] == 0
        and actual['implementation'] == IMPLEMENTATIONS[backend]
        and actual['diagnostic_mode'] == 'full' and actual['full_matrix_packet_capture'] is True
        and actual['session_id'] == start['session_id'] and actual['epoch'] == start['epoch']
        and actual['configuration_sha256'] == startup['task_config']['sha256']
        and actual['nmpc_options_manifest'] == options, 'Actual new worker startup/backend/Full capture differs')
    for key, loaded in [('core_library','loaded_core_path'),('optimization_library','loaded_optimization_path'),
                        ('predictor_library','loaded_predictor_path')]:
        require(str(Path(startup[key]['path']).resolve()) == actual[loaded], 'Actually loaded native library differs')
    return dict(verified=True, implementation=IMPLEMENTATIONS[backend], diagnostic_mode='full')


class Checks:
    """Continue independent evidence checks after an execution failure."""
    def __init__(self, report):
        self.report = report

    def __call__(self, name, action):
        try:
            value = action()
            self.report[name] = value
            return value
        except Exception as error:
            item = dict(check=name, error=repr(error), traceback=traceback.format_exc())
            self.report['execution_errors'].append(item)
            self.report[name] = dict(verified=False, error=repr(error))
            return None


def check_memory_commits(live, calls, sends, backend):
    ledger, final = read(live/'worker_commit_timing.json'), read(live/'worker_final.json')
    names = ['sequence','dispatch_sequence','sent_monotonic_ns','received_monotonic_ns',
             'committed_memory_steps','buffered_next_sequence']
    require(ledger['kind'] == 'upper_worker_actual_send_commits_v1'
        and ledger['columns'] == names and ledger['complete'] is True and ledger['requires_consumer_ack'] is False
        and all(len(row)==len(names) for row in ledger['rows']), 'Missing actual-send memory commit ledger')
    commits = [dict(zip(names,row)) for row in ledger['rows']]
    first = {}
    actual = {}
    for sent in sends:
        if not sent['stop_zero']:
            first.setdefault(sent['command_sequence'],sent)
            key = (sent['command_sequence'],sent['dispatch_sequence'])
            require(key not in actual, 'Repeated actual dispatch identity')
            actual[key] = sent
    require(set(first) == set(range(1,len(first)+1))
        and len(first) == len(commits) and len(calls)-1 <= len(commits) <= len(calls)
        and final['backend'] == backend and final['implementation'] == IMPLEMENTATIONS[backend]
        and final['actual_send_commits'] == final['committed_send_sequence'] == final['controller_steps'] == len(commits)
        and final['pending_send_sequence'] == 0,
        'Actual sends, committed controller steps or final pending state differ')
    by_call = {int(s.sequence):(s,p,v) for s,p,v in calls}
    for sequence, row in enumerate(commits,1):
        # A best-effort notification can be lost. A subsequent *actual* repeat
        # publication can commit the same pending candidate; never invent a
        # receipt for the first send or require a consumer round trip.
        key = (sequence,row['dispatch_sequence'])
        require(key in actual, 'Memory commit has no original actual-send record')
        sent = actual[key]; s,p,value = by_call[sequence]
        require(row['sequence'] == row['committed_memory_steps'] == sequence
            and row['dispatch_sequence'] == sent['dispatch_sequence']
            and row['sent_monotonic_ns'] == sent['sent_monotonic_ns']
            and row['sent_monotonic_ns'] <= row['received_monotonic_ns'] <= row['sent_monotonic_ns']+25_000_000
            and row['received_monotonic_ns'] >= p.compute_end_monotonic_ns
            and row['buffered_next_sequence'] in (0,sequence+1),
            'Plan memory committed without the matching fresh actual writer publication')
        if sequence+1 in by_call:
            require(row['received_monotonic_ns'] <= by_call[sequence+1][1].compute_start_monotonic_ns,
                    'Next solver started before previous actual-send memory commit')
    for s,_,value in calls:
        require(value['backend'] == backend
            and value['memory_before_step_count'] == int(s.sequence)-1
            and value['selected_next_step_count'] == int(s.sequence),
            'Unsent nonlinear trial or prepared candidate entered committed memory')
    discarded = len(calls) if len(calls)>len(commits) else 0
    require(final['discarded_unsent_sequence'] == ledger['discarded_unsent_sequence'] == discarded,
            'Unsent terminal candidate was committed or silently lost')
    return dict(verified=True, actual_send_commits=len(commits), discarded_unsent_sequence=discarded,
        no_consumer_ack=True, scope='Actual writer notification, not consumer round trip, commits one selected plan')


def audit(launch, run, out):
    report = dict(kind='x650_upper_closed_loop_audit_v1', passed=False, complete=False,
        execution_passed=False, performance_passed=False, execution_errors=[], performance_errors=[],
        physical_acceptance=False, hardware_qualified=False, backend=None, case_id=None,
        task_sha256=None, model_sha256=None, config_sha256=None, launch=None,
        launch_directory=str(launch), run_directory=str(run), timing_profile=profile.NAME,
        effective_profile=profile.effective(), auditor_identity=identity(Path(__file__)))
    checks = Checks(report); artifacts = []
    try:
        # Capture original evidence before derived views; never rewrite source data.
        paths = {p for directory in (launch,run) for p in directory.rglob('*') if p.is_file()}
        paths.update(ROOT/name for name in (
            'scripts/audit_x650_cpp_hold.py','scripts/report_x650_cpp_live.py',
            'scripts/audit_x650_cpp_source.py','scripts/audit_x650_cpp_normal_source.py',
            'scripts/audit_x650_cpp_fast_source.py','scripts/audit_x650_streaming_consumption.py',
            'scripts/audit_x650_isolated_live.py','scripts/audit_x650_precision_stage.py',
            'scripts/audit_x650_upper_stage.py','scripts/analyze_x650_hover_precision.py',
            'scripts/audit_x650_cpp_multirate.py','scripts/audit_x650_cpp_multirate_followup.py',
            'scripts/audit_x650_interval_robustness.py','scripts/audit_x650_recovery_robustness.py',
            'scripts/audit_x650_recovery_replay.py','scripts/audit_x650_interval_constraints.py',
            'simulations/x650_cpp_live_v1/qualification.py','simulations/x650_ts20_hold_v1/audit.py',
            'simulations/x650_ts20_hold_v1/task.py'))
        paths.update((ROOT/'simulations/x650_upper_nmpc_v1').glob('*.py'))
        paths.add(Path(__file__).resolve()); artifacts = [identity(p) for p in sorted(paths)]
        require(qualification.sha(Path(base.__file__)) == FROZEN_BASE_SHA256
            and qualification.sha(ROOT/'scripts/audit_x650_cpp_multirate_followup.py') == FROZEN_FOLLOWUP_SHA256,
            'Preserved original audit helper changed')
        meta = read(run/'controller_metadata.json'); stage = read(run/'stage_result.json')
        cfg, protocol, startup, launch_record, proof = check_archives(launch,run,meta)
        backend, case = check_case(cfg,protocol); duration = DURATIONS[case]
        report.update(backend=backend, case_id=case, task_sha256=qualification.sha(launch/'task.json'),
            model_sha256=launch_record['provenance']['model_sha256'],
            config_sha256=qualification.config_sha256(startup['config']),
            launch=identity(launch/'launch.json'), software_qualification=proof,
            task_config=identity(launch/'task.json'), startup_manifest=identity(launch/'startup_manifest.json'),
            runtime_binaries_sha256=launch_record['runtime_binaries_sha256'])
        def lifecycle_status():
            require(all(meta.get(k) is True for k in ('complete','landed_and_disarmed','diagnostic_parameters_restored'))
                and meta.get('diagnostic_restore_pending') is False, 'Incomplete flight lifecycle: '+str(meta.get('error')))
            require(all(stage.get(k) is True for k in ('complete','passed','success','landed_and_disarmed'))
                and read(run/'result.json').get('success') is True, 'Original provisional stage/run failed')
            return dict(verified=True, landed_and_disarmed=True, parameters_restored=True)
        checks('lifecycle_status', lifecycle_status)
        live = run/'isolated_live'; lifecycle = read(live/'lifecycle.json'); start = read(live/'live_start.json')
        window = [start['origin_s'],start['origin_s']+duration]; report['window_sim_s'] = window
        def process_lifecycle():
            require(lifecycle['started'] is True and lifecycle['error'] is None
                and all(p['returncode']==0 for p in lifecycle['processes'].values()), 'Native child lifecycle failed')
            require(start['history_count']>0 and abs(meta['task_end_sim_s']-window[1])<=.05,
                    'Incomplete declared full physical window')
            return dict(verified=True, duration_s=duration)
        checks('process_lifecycle', process_lifecycle)
        import audit_x650_isolated_live as old
        checks('ground_initialization', lambda: old.check_preflight_transport(live,meta,start))
        checks('fault_supervision', lambda: old.check_fault_supervision(live,start))
        resources = read(live/'execution_resources.json')
        def resource_check():
            require(resources==launch_record['execution_resources'] and resources['roles']==protocol['cpu_roles'], 'Actual resources changed')
            observed=read(live/'execution_affinity_at_handoff.json')
            require(set(observed)=={'source',*lifecycle['processes']}, 'Incomplete process placement observation')
            for name,threads in observed.items():
                if name!='worker':
                    require(threads and all(mask==resources['roles'].get(name,resources['background']) for mask in threads.values()),
                            'Handoff CPU partition changed: '+name)
            return dict(verified=True)
        checks('resources', resource_check)
        actual_worker = read(live/'worker_startup.json')
        checks('timing_selection', lambda: check_timing_selection(live,actual_worker))
        checks('worker_startup', lambda: check_worker_startup(actual_worker,startup,start,read(launch/'options.json'),backend))
        import audit_x650_streaming_consumption as streaming
        messages = streaming.original_messages(live)
        sends,receipts,ticks,events = (messages[k] for k in ('sends','receipts','dispatch','events'))
        worker_result = checks('worker_validation', lambda: check_worker(live,start,startup['config'],events,resources,protocol,backend=backend))
        calls = worker_result[0] if worker_result is not None else None
        if worker_result is not None:
            report.pop('worker_validation'); report['worker'] = worker_result[1]
            checks('optimizer_recovery', lambda: check_runtime_recoveries(calls,start,startup['config']))
            checks('optimizer_interval', lambda: check_runtime_intervals(calls,start,startup['config']))
            report['first_command_hold_guard'] = dict(verified=True,calls_checked=len(calls),duration_s=.16)
            checks('actual_send_barrier', lambda: check_actual_send_barrier(read(live/'ingress_timing.json'),sends,calls))
            checks('memory_commits', lambda: check_memory_commits(live,calls,sends,backend))
        source_rows = rows(live/'cpp_source/source_selections.csv')
        source_issues = {int(v['sequence']):int(v['issued_ns']) for v in source_rows}
        source_controls = {int(v['sequence']):float(v['control_s']) for v in source_rows}
        require(len(source_issues)==len(source_rows), 'Repeated source issue identity')
        if calls is not None:
            checks('timing', lambda: check_ticks(ticks,sends,calls,start,source_issues=source_issues,source_controls=source_controls))
        def consumption_check():
            publications = [json.loads(line) for line in (run/'control_execution_trace.jsonl').open() if line.strip()]
            result = streaming.audit_consumption(sends,receipts,rows(run/'ts20_state.csv'),start,lifecycle['release'],publications)
            require(result['passed'] is True, 'Asynchronous physical consumption accounting failed')
            from flyarm_control_interfaces.msg import ActuatorState
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.convert import message_to_ordereddict
            baseline = message_to_ordereddict(deserialize_message((live/'actuator_epoch_state.cdr').read_bytes(),ActuatorState))
            result['online_ledger'] = streaming.audit_async_ledger(read(live/'actuator_ledger.json'),baseline,
                sends,receipts,rows(run/'ts20_state.csv'),start,lifecycle['release'])
            require(result['online_ledger']['passed'] is True, 'Online asynchronous progress evidence failed')
            return result
        checks('consumption', consumption_check)
        reference = read(run/'hover_reference.json')
        def feedback_check():
            if case == 'hold':
                from audit_x650_cpp_source import audit_source
            elif case == 'normal':
                from audit_x650_cpp_normal_source import audit_source
            else:
                from audit_x650_cpp_fast_source import audit_source
            value = audit_source(live/'cpp_source',run/'ts20_state.csv',run/'ts20_packets.csv',reference,
                read(run/'estimated_feedback.json')['extra_reset_counters_at_bind'],worker_directory=live/'native_records',
                expected_binding=dict(configuration_sha256=actual_worker['configuration_sha256'],
                    position_gains=startup['config']['base_position_gain'],bus_period_s=startup['config']['bus_period'],delay_s=startup['config']['motion'][4]),
                **({'trajectory':startup['trajectory']} if case!='hold' else {}))
            require(all(value[k] is True for k in ('passed','record_integrity_passed','clean_source_stop','worker_link_checked')),
                    'Raw C++ feedback/source validation failed')
            require(calls is None or value['original_worker_snapshots_verified']==len(calls), 'Worker/source request count differs')
            return value
        checks('feedback', feedback_check)
        docs = {k:read(run/n) for k,n in {'session':'diagnostic_parameter_session.json','journal':'diagnostic_restore_journal_completed.json',
            'before':'diagnostic_parameters_before.json','applied':'diagnostic_parameters_applied.json','restored':'diagnostic_parameters_restored.json',
            'px4_before':'px4_parameters_before.json','px4_after':'px4_parameters_after.json'}.items()}
        _,pid,_ = old.frozen_research_pid(cfg)
        def parameters_check():
            for name, document in [('metadata',meta),('stage',stage)]:
                old.fixed.verify_condition(document,old.condition_for_task(cfg),name)
            old.precision.verify_reference(reference,old.precision.DEFAULT_REFERENCE_SOURCE)
            return old.fixed.verify_parameter_session(docs,meta,old.condition_for_task(cfg),pid)
        checks('parameters', parameters_check)
        def native_check():
            manifest = read(run/'native_trace_manifest.json')
            require(manifest['complete'] is True and manifest['sha256']==qualification.sha(run/'native_trace.csv'), 'Incomplete native physical trace')
            old.precision.capture_window_covers(manifest,window)
            return old.precision.native_states(run/'native_trace.csv',window)
        native = checks('native_trace', native_check)
        if native is not None:
            report['native_trace'] = dict(verified=True, window_sim_s=window)
            values = checks('endpoint_observations', lambda: endpoint_observations(run,start,cfg,old.shared.native_tip_metrics))
            if values is not None:
                report.pop('endpoint_observations'); report.update(**values[0],endpoint_performance=values[1])
                tilt = float(native['tilt'].max()); report['tilt_peak_deg'] = tilt
                report['performance'] = classify_performance(values[1],tilt,cfg['ts20_protocol'])
                report['performance_passed'] = report['performance']['passed']
                report['performance_errors'] = report['performance']['performance_errors']
        from pyulog import ULog
        ulog = ULog(str(run/'px4.ulg'))
        checks('recovery', lambda: check_recovery(old.precision.dataset(ulog,'vehicle_status'),
            old.precision.dataset(ulog,'vehicle_land_detected'),rows(run/'flight_samples.csv'),window[1]))
        if native is not None:
            checks('configuration', lambda: old.fixed.effective_configuration(ulog,native,reference,window,old.fixed.records(docs['applied'])))
        if calls is not None and native is not None:
            def px4_check():
                control,derived = old.command_view(run,sends,calls,start)
                (out/'derived_command_audit_view.json').write_text(json.dumps(dict(scope='Actual send CDR matched to original worker/source; originals untouched',rows=derived),allow_nan=False))
                view,proof_view,view_derived = observation_value_view(control,derived,sends,ticks,source_rows,start)
                (out/'distinct_pva_value_audit_view.json').write_text(json.dumps(dict(**proof_view,rows=view),allow_nan=False))
                report['px4_value_view'] = proof_view
                handoff = check_handoff_bridge(view,view_derived,start,ulog)
                handoff.update(full_physical_window_s=duration,
                    cpp_writer_observation_span_to_window_end_s=window[1]-handoff['first_cpp_writer_observation_sim_s'],
                    cpp_received_value_span_to_window_end_s=window[1]-handoff['first_cpp_value_received_s'])
                value = upper_commands(ulog,native,reference,view,window,pid,response=False,handoff=handoff)
                value['all_distinct_pva_value_rows_observed'] = value.pop('all_measurement_sends_observed')
                value['distinct_pva_measurement_value_rows'] = value.pop('measurement_sends')
                value['observation_scope'] = proof_view['scope']
                value['time_alignment'] = 'Original pre-send known observations; no physical publication timestamp or fitted shift'
                value['matched_observation_receipt_range_s'] = value.pop('matched_delivery_range_s')
                value['internal_minus_observation_time_logical_velocity'] = value.pop('internal_minus_original_time_logical_velocity')
                value['causal_value_matching_allowance_s'] = value.pop('causal_delivery_allowance_s')
                handoff['first_cpp_value_match_observation_sim_s'] = handoff.pop('first_cpp_writer_observation_sim_s')
                handoff['original_first_cpp_writer_observation_sim_s'] = derived[0]['sim_s']
                handoff['observation_scope'] = proof_view['scope']; report['handoff'] = handoff
                return value
            checks('px4_commands', px4_check)
        checks('flight', lambda: old.upper.flight_states(run/'flight_samples.csv',window))
        checks('ground', lambda: old.fixed.native_ground_check(run,read(run/'native_ground.json')))
        def actuator_check():
            value = old.prior_audit.actuator_check(run,task.baseline_config(cfg),window)
            value.pop('control_snapshot_to_plugin_consumption_lag',None)
            require(value['new_joint_prediction_verified'], 'Independent whole-window joint prediction failed')
            return value
        checks('actuator', actuator_check)
        require(all(identity(v['path'])==v for v in artifacts), 'Original evidence changed while auditing')
        report['complete'] = True
    except Exception as error:
        report['execution_errors'].append(dict(check='audit_prerequisites_or_integrity',error=repr(error),traceback=traceback.format_exc()))
    report['execution_passed'] = bool(report['complete'] and not report['execution_errors'])
    report['passed'] = bool(report['execution_passed'] and report['performance_passed'])
    report['physical_acceptance'] = report['passed']
    report['scope'] = 'One newly recorded declared simulation window; exact source/send/consumption/exit evidence and unchanged physical limits; not hardware qualification or NLP convergence'
    if not report['execution_passed']:
        try:
            from audit_x650_interval_robustness import failure_context
            report['failure_context'] = failure_context(run)
        except Exception as error:
            report['failure_context'] = dict(read_error=repr(error))
    report['artifact_identities'] = artifacts
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('launch-dir','run-dir','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args = parser.parse_args()
    launch,run,out = args.launch_dir.resolve(),args.run_dir.resolve(),args.output.resolve()
    require(not out.exists() and not out.is_relative_to(run) and not out.is_relative_to(launch),
            'New audit output must be outside original evidence')
    out.mkdir(parents=True)
    report = audit(launch,run,out)
    print(json.dumps({k:report[k] for k in ('passed','execution_passed','performance_passed','backend','case_id','execution_errors','performance_errors')},allow_nan=False))
    return 0 if report['passed'] else 1

def check_numeric_result(value, s, p, config, previous_command):
    """Check recorded owning native output against wire and physical limits."""
    require(value.get('kind') == 'cpp_full_pipeline_call_v1' and value.get('sequence') == int(s.sequence)
        and value.get('error') == '' and value.get('diagnostic_mode') == 'full' and value.get('full_matrix_packet_capture') is True, 'Failed/misidentified actual Core call')
    r = value['result']; d = r['diagnostics']; n = config['nodes']
    require(all(d.get(k) is True for k in ('accepted','nonlinear_passed','actual_bounds'))
        and d.get('previous_plan_fallback') is False, 'Native candidate rejected/fallback')
    for key, bound in [('primal_violation',config['feasibility_tolerance']),
                       ('kkt_stationarity',config['kkt_tolerance']), ('compute_ms',50.),
                       ('q_linearization_error',.006), ('dq_linearization_error',.10)]:
        require(type(d[key]) in (int,float) and np.isfinite(d[key]) and 0 <= d[key] <= bound,
                'Native quality/deadline rejected: '+key)
    require(d['subproblems'] == len(d['attempts']) and 1 <= d['subproblems'] <= 3
        and d['attempts'][-1]['passed'] is True and d['attempts'][-1]['actual_bounds'] is True
        and d['candidate_substeps'] > 0, 'Missing actual nonlinear candidate attempts/extrema')
    limits = numeric_array(config['velocity_limit'],5,'limits')
    command = numeric_array(r['command'],5,'command')
    plan = numeric_array(d['plan'],n*5,'plan').reshape(n,5)
    h = numeric_array(d['horizon'],n,'horizon')
    require(same(h, [s.dt_s]+[config['node_dt']]*(n-1)) and np.all(h > 0)
        and same(command,plan[0]) and np.all(abs(plan) <= limits+2e-7), 'Plan/horizon/command differs')
    delta = np.diff(np.vstack((previous_command,plan)),axis=0)
    require(np.all(abs(delta) <= h[:,None]*np.asarray(config['acceleration_limit'])+2e-7), 'Actual plan delta limit exceeded')
    for field, output in [('position_reference_enu','position_reference'),('velocity_feedforward_enu','velocity_feedforward'),
                           ('acceleration_feedforward_enu','acceleration_feedforward'),('joint_velocity','joint_velocity')]:
        require(same(getattr(p,field),r[output]), 'Recorded result and actual proposal differ: '+field)
    require(same(p.base_velocity_target,command[:3]) and same(p.joint_velocity,command[3:])
        and same(p.acceleration_feedforward_enu,[0,0,0],0)
        and same(p.velocity_feedforward_enu, command[:3]-np.asarray(config['base_position_gain'])*
                 (np.asarray(p.position_reference_enu)-np.asarray(s.position_enu))), 'PX4 command decomposition differs')
    require(same([p.primal_violation,p.kkt_residual],[d['primal_violation'],d['kkt_stationarity']],0)
        and int(p.subproblems) == d['subproblems'], 'Wire quality differs from actual Core result')
    envelope = numeric_array(d['candidate_envelope'],8,'envelope')
    lo, hi = np.asarray(config['q_lower']), np.asarray(config['q_upper'])
    require(np.all(envelope[:2] <= envelope[2:4]) and np.all(envelope[4:6] <= envelope[6:8])
        and np.all(envelope[:2]-.0003 >= lo-2e-7) and np.all(envelope[2:4]+.0003 <= hi+2e-7)
        and np.all(envelope[4:6]-.03 >= -limits[3:]-2e-7)
        and np.all(envelope[6:8]+.03 <= limits[3:]+2e-7), 'Actual candidate envelope exceeds tightened joint bounds')
    state = numeric_array(d['predicted_states'],n*12,'predicted_states').reshape(n,12)
    numeric_array(d['predicted_tip'],n*3,'predicted_tip')
    numeric_array(d['predicted_rotations'],n*9,'predicted_rotations')
    require(np.all(state[:,6:8] >= lo-2e-7) and np.all(state[:,6:8] <= hi+2e-7)
        and np.all(abs(state[:,8:10]) <= limits[3:]+2e-7), 'Candidate nodes exceed physical limits')
    return command



def check_worker(live, start, config, events, resources, protocol, *, backend, decode=None):
    if decode is None:
        from rclpy.serialization import deserialize_message
        from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal
        decode = lambda raw, proposal: deserialize_message(raw, ControlProposal if proposal else ControlSnapshot)
    final, sender, timing = (read(live/name) for name in ('worker_final.json','diagnostic_sender.json','worker_timing.json'))
    files = sorted((live/'native_records').glob('*.json'))
    require(files and final['implementation'] == IMPLEMENTATIONS[backend] and final['live_started'] is True
        and final['faulted'] is False and final['last_error'] == ''
        and final['requests'] == final['accepted'] == len(files), 'Native worker did not complete cleanly')
    require(sender['complete'] is True and sender['failed'] is False and sender['dropped'] == 0
        and sender['offered'] == sender['written'] == len(files), 'Lost actual Core evidence')
    columns = timing['columns']
    require(timing['kind'] == 'cpp_worker_phase_timing_v1' and timing['complete'] is True and timing['dropped'] == 0
        and len(columns) == len(set(columns)) and len(timing['rows']) == len(files)
        and all(len(r)==len(columns) for r in timing['rows']), 'Incomplete bounded native phase evidence')
    phase = [dict(zip(columns,r)) for r in timing['rows']]
    require(len(events) <= len(files) <= len(events)+1 and all(e['accepted'] is True for e in events)
        and [e['sequence'] for e in events] == list(range(1,len(events)+1))
        and [e['proposal_sequence'] for e in events] == [e['snapshot_sequence'] for e in events]
        == list(range(1,len(events)+1)), 'Native candidate/gate prefix differs')
    affinity = read(live/'worker_affinity_final.json')
    ready = read(live/'worker_affinity_ready.json')
    executor = protocol['worker_thread_roles']['executor']
    auxiliary = protocol['worker_thread_roles']['auxiliary']
    require(len(executor)==1 and sorted(executor+auxiliary)==resources['roles']['worker']
        and affinity['executor_cpu']==executor[0] and affinity['auxiliary_cpus']==auxiliary
        and affinity['stages'][:1]==ready['stages'] and [v['stage'] for v in affinity['stages']]==['ready','live_start','final'],
        'Worker thread placement declaration/stages differ')
    identity_set = None
    for stage in affinity['stages']:
        current={(r['tid'],r['starttime_ticks']) for r in stage['threads']}
        require(len(current)==len(stage['threads']) and (identity_set is None or current==identity_set), 'Worker TIDs changed')
        identity_set=current
        require(all(r['cpus']==(executor if r['tid']==affinity['main_tid'] else auxiliary) for r in stage['threads']),
                'Worker stage CPU masks differ')
    calls=[]; stats={name:[] for name in ('core_ms','callback_ms','thread_cpu_ms','publish_ms','record_after_publish_ms','gate_roundtrip_ms')}
    previous = np.asarray(start['history_command'][(start['history_count']-1)*5:start['history_count']*5])
    for seq,(path,t) in enumerate(zip(files,phase),1):
        value=read(path)
        for name in ('snapshot_cdr','proposal_cdr'):
            require(identity(value[name]['path'])==value[name], 'Native raw CDR bytes changed')
        s=decode(Path(value['snapshot_cdr']['path']).read_bytes(),False)
        p=decode(Path(value['proposal_cdr']['path']).read_bytes(),True)
        require(s.sequence==p.sequence==p.snapshot_sequence==seq==t['sequence']
            and s.session_id==p.session_id==start['session_id'] and s.epoch==p.epoch==start['epoch']
            and bytes(s.configuration_sha256)==bytes(p.configuration_sha256)==bytes(start['configuration_sha256'])
            and bytes(s.configuration_sha256).hex()==final['configuration_sha256']
            and p.status==0 and p.accepted_optimum and p.nonlinear_check_passed and not p.replay_only,
            'Native actual candidate identity/status differs')
        require(t['snapshot_issued_ns']==s.issued_monotonic_ns and t['callback_entry_ns']==p.compute_start_monotonic_ns,
                'Native phase/CDR timestamp differs')
        stamps=[s.issued_monotonic_ns,t['callback_entry_ns'],t['core_begin_ns'],t['core_end_ns'],p.compute_end_monotonic_ns,
                t['publish_begin_ns'],t['publish_end_ns'],t['record_offer_end_ns'],t['callback_exit_ns']]
        require(all(type(v) is int and v>0 for v in stamps) and all(b>=a for a,b in zip(stamps,stamps[1:]))
            and t['record_offered']==1 and t['thread_cpu_end_ns']>=t['thread_cpu_begin_ns']
            and t['tid_begin']==t['tid_end']==affinity['main_tid'] and t['cpu_begin']==t['cpu_end']==executor[0],
            'Native phase/CPU identity invalid')
        require(0<s.dt_s<=.1+1e-10 and 0<=s.control_time_s-s.joint_time_s<=.025+1e-10
            and p.compute_end_monotonic_ns-p.compute_start_monotonic_ns<=50_000_000,
            'Original native interval/joint freshness/computation deadline exceeded')
        if seq<=len(events):
            e=events[seq-1]
            require(e['session_id']==s.session_id and e['epoch']==s.epoch
                and e['received_monotonic_ns']>=t['publish_begin_ns']
                and e['roundtrip_ns']==e['received_monotonic_ns']-s.issued_monotonic_ns
                and e['roundtrip_ns']<=50_000_000, 'Gate altered source age or accepted a stale candidate')
            stats['gate_roundtrip_ms'].append(e['roundtrip_ns']/1e6)
        require(value.get('implementation') == IMPLEMENTATIONS[backend] and value.get('valid_proposal_published') is True,
                'Actual upper worker/backend publication identity differs')
        check_online_record(value, s, p, config, backend)
        check_full_qp(value['result']['diagnostics'], config)
        check_full_prediction(s, config, value['result']['diagnostics'], previous)
        previous=check_numeric_result(value,s,p,config,previous)
        check_hold_guard(s,config,value['result']['diagnostics'])
        for name,a,b in [('core_ms','core_begin_ns','core_end_ns'),('callback_ms','callback_entry_ns','callback_exit_ns'),
                         ('thread_cpu_ms','thread_cpu_begin_ns','thread_cpu_end_ns'),('publish_ms','publish_begin_ns','publish_end_ns'),
                         ('record_after_publish_ms','publish_end_ns','callback_exit_ns')]: stats[name].append((t[b]-t[a])/1e6)
        calls.append((s,p,value))
    # Admission source stamps and actual-send releases are independent of worker
    # phase observations; never infer them from a later accepted proposal.
    ingress=read(live/'ingress_timing.json'); names=ingress['columns']
    require(ingress['complete'] is True and ingress['overflow']==0 and len(names)==len(set(names))
        and all(len(r)==len(names) for r in ingress['rows']), 'Incomplete ingress timing')
    admissions=[dict(zip(names,r)) for r in ingress['rows'] if r[names.index('kind')]==4 and r[names.index('forward')]]
    by_seq={r['sequence']:r for r in admissions}
    require(len(by_seq)==len(admissions), 'Duplicate admission')
    for s,_,_ in calls:
        require(s.sequence in by_seq, 'Candidate has no actual ingress admission')
        a=by_seq[s.sequence]
        require(a['source_issued_ns']==s.issued_monotonic_ns and a['latest_control_s']==s.control_time_s
            and abs(a['latest_control_s']-a['last_control_s']-s.dt_s)<=1e-10
            and (s.sequence==1 or a['released_sequence']==a['last_admitted_sequence']==s.sequence-1),
            'Admission source/dt/actual-send barrier differs')
    ordered=[by_seq[int(s.sequence)] for s,_,_ in calls]
    require(all(b['entry_ns']-a['entry_ns']>=40_000_000 for a,b in zip(ordered,ordered[1:])),
        'New upper admission ran faster than its explicit 40 ms minimum')
    return calls, dict(verified=True, calls=len(calls), no_lost_diagnostics=True, executor_cpu=executor[0],
        numerical_scope='Actual owning outputs checked against original wire; frozen independent software proofs separately reverified',
        durations={k:summary(v) for k,v in stats.items()})


if __name__ == '__main__':
    raise SystemExit(main())
