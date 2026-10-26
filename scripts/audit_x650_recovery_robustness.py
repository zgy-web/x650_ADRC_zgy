#!/usr/bin/env python3
"""Audit a new complete disturbed flight with the ABI 2 recovery backend.

The physical waveform, limits, frame, consumption and PX4 exit checks are the
unchanged robustness checks. New software admission is verified explicitly;
historical flight certificates are never assigned to the changed Core/worker.
Incomplete experiments keep original evidence and prefix diagnostics, but
cannot produce a physical acceptance certificate.
"""
from pathlib import Path
import argparse
import json
import sys
import traceback

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents
            if (p/'simulations/x650_cpp_robustness_v1').is_dir())
sys.path[:0] = [str(ROOT), str(ROOT/'scripts'), str(ROOT/'control_packages/x650_wholebody_v1')]
from report_x650_cpp_live import identity, require, read
from simulations.x650_cpp_robustness_v1.task import baseline_config, validate_task_config
from audit_x650_cpp_hold import (rows, check_worker, check_ticks, check_actual_send_barrier,
    check_recovery, ulog_value_view, summary)
from audit_x650_cpp_robustness import (check_physical_model, check_wrench,
    upstream_shared_source_proof, endpoint_windows, robustness_performance)
import audit_x650_recovery_replay as replay


def recovery_qualification():
    # Keep partial-run evidence collection available even if an incomplete
    # deployment omitted the new admission package. Complete acceptance still
    # imports and executes the explicit new qualification below.
    from simulations.x650_cpp_recovery_v1 import qualification
    return qualification


def check_archives(launch, run, meta):
    qualification = recovery_qualification()
    record = read(launch/'launch.json')
    require(all(record.get(k) is True for k in ('complete','runtime_closed','sources_unchanged',
        'runtime_binaries_unchanged','launch_inputs_unchanged')), 'Incomplete/changing recovery launch')
    require(record.get('exit_code') == 0 and record.get('execution_scenario') == meta['task']['scenario']
        and record.get('run_directories') == [str(run.resolve())], 'Recovery launch/run lifecycle differs')
    sources, binaries = qualification.archive_integrity(launch, record)
    cfg, protocol, startup = (read(launch/name) for name in ('task.json','protocol.json','startup_manifest.json'))
    require(validate_task_config(cfg) == cfg == record['task'] == read(run/'controller_task.json') == meta['task'],
            'Actual physical robustness task differs')
    require(protocol == record['protocol'] and cfg['duration_s'] == protocol['duration_s'] == 24.,
            'Complete original 24 s disturbance task required')
    for artifact in record['launch_inputs']:
        require(identity(artifact['path']) == artifact, 'Recovery launch input changed')
    original = read(launch/'original_startup_manifest.json')
    require({k:v for k,v in startup.items() if k != 'task_config'} ==
            {k:v for k,v in original.items() if k != 'task_config'}
            and startup['task_config']['sha256'] == original['task_config']['sha256'] == qualification.sha(launch/'task.json'),
            'Startup derivation changed something besides the identical task path')
    manifest = read(launch/'qualification_manifest.json')
    proof = qualification.verify(launch/'qualification_manifest.json',
        task_path=Path(manifest['task_config']['path']), startup_path=Path(manifest['startup_manifest']['path']),
        startup_sha256=manifest['startup_manifest']['sha256'], protocol_path=Path(manifest['protocol']['path']),
        binaries=binaries)
    require(proof.get('ready') is True and proof.get('kind') == 'x650_cpp_recovery_admission_v1'
        and proof.get('physical_acceptance') is False, 'New backend admission does not replace physical acceptance')
    require(manifest['task_config']['sha256'] == qualification.sha(launch/'task.json')
        and manifest['protocol']['sha256'] == qualification.sha(launch/'protocol.json')
        and manifest['startup_manifest']['sha256'] == qualification.sha(launch/'original_startup_manifest.json'),
        'Recovery qualification differs from actual archived execution')
    proof['upstream_shared_source_proof'] = upstream_shared_source_proof(cfg)
    require(all(name in sources for name in (
        'simulations/x650_cpp_recovery_v1/qualification.py','simulations/x650_cpp_recovery_v1/launch.py',
        'simulations/x650_cpp_robustness_v1/task.py','simulations/x650_cpp_robustness_v1/run_case.py',
        'simulations/x650_cpp_robustness_v1/disturbance.py',
        'control_packages/x650_wholebody_v1/native/mpc_core.cpp',
        'control_packages/x650_wholebody_v1/native/mpc_core.hpp',
        'ros2_ws/src/flyarm_mpc_cpp/src/isolated_worker.cpp',
        'ros2_ws/src/flyarm_mpc_cpp/include/flyarm_mpc_cpp/diagnostics_json.hpp',
        'ros2_ws/src/flyarm_mpc_cpp/include/flyarm_mpc_cpp/worker_contract.hpp',
        'ros2_ws/src/flyarm_state_cpp/include/flyarm_state_cpp/source_core.hpp')),
        'Recovery source archive omits actual native recovery/worker or physical implementation')
    return cfg, protocol, startup, record, proof


def recovery_fixture(snapshot, config, previous_command):
    """Read the actual typed input, preserving its reference/command history."""
    return dict(config=config, memory=dict(previous_command=list(previous_command)), input=dict(
        t=float(snapshot.task_time_s), dt=float(snapshot.dt_s),
        state=dict(control_time=float(snapshot.control_time_s), joint_stamp=float(snapshot.joint_time_s)),
        context=dict(reference_time=float(snapshot.reference_time_s), reference=list(map(float,snapshot.reference_r)),
            commands=[dict(time=float(snapshot.command_time_s[i]),
                velocity=list(map(float,snapshot.command_velocity[2*i:2*i+2])))
                for i in range(snapshot.command_count)])))


def check_runtime_recoveries(calls, start, config):
    """Independently check compact native recovery evidence from original CDRs.

    Compact mode intentionally omits the full QP matrix and propagation stream.
    This checks its copied node/envelope/reference evidence, while the exact
    implementation and full-matrix replay remain bound by software admission.
    No prediction or optimizer is rerun and no missing matrix is reconstructed.
    """
    require(calls, 'No actual native calls available for recovery evidence')
    previous = list(start['history_command'][(start['history_count']-1)*5:start['history_count']*5])
    recovered, normal, total_projection = [], 0, 0
    for snapshot, proposal, value in calls:
        d = value['result']['diagnostics']; triggered = replay.bool_field(d, 'recovery_triggered')
        attempts = d['recovery_attempts']
        require(isinstance(attempts,list), 'Malformed actual recovery centre evidence')
        if not triggered:
            require(attempts == [] and d['recovery_reason'] == '' and d['recovery_trust_radius'] == 0.
                and d['recovery_stopping_violation'] == 0., 'Normal call carries untriggered recovery evidence')
            normal += 1
        else:
            fixture = recovery_fixture(snapshot, config, previous)
            require(d['recovery_reason'] in ('QP infeasible','constant QP infeasible')
                and replay.number(d['recovery_trust_radius']) == .02
                and min(replay.number(config['trust_radius']),.02) == .02,
                'Actual recovery changed its trigger or original initial radius')
            require(1 <= len(attempts) <= 2 and 2 <= d['subproblems'] <= 3,
                    'Actual recovery centre/control QP budget differs')
            require([a['centre_kind'] for a in attempts] ==
                ['slew_limited_brake','original_constraints_brake_projection'][:len(attempts)],
                'Actual recovery centre order differs')
            checked, projections = [], 0
            for a in attempts:
                kind = a['centre_kind']
                require(kind in ('slew_limited_brake','original_constraints_brake_projection'), 'Unknown recovery centre')
                projected = replay.bool_field(a,'projection_attempted')
                solved = replay.bool_field(a,'projection_solved')
                require(not solved or projected, 'Projection solved without an actual attempt')
                require(not projected or kind == 'original_constraints_brake_projection', 'Brake falsely declares projection')
                projections += projected
                require(0 <= replay.number(a['elapsed_ms']) <= replay.number(d['compute_ms']) <= 50.,
                        'Recovery centre clock exceeds original call/deadline')
                if not replay.bool_field(a,'trusted'):
                    require(isinstance(a['rejection'],str) and a['rejection'], 'Rejected centre lacks actual failure reason')
                    continue
                require(a['rejection'] == '' and replay.bool_field(a,'nonlinear_passed')
                    and replay.bool_field(a,'original_constraints_passed') and a['substeps'] > 0,
                    'Unvalidated recovery centre was trusted')
                require(kind != 'original_constraints_brake_projection' or (projected and solved),
                        'Trusted projected centre lacks an actual solved projection')
                times = replay.vector(a['node_times'], config['nodes'])
                require(max(abs(x-y) for x,y in zip(times,replay.node_times(fixture))) <= 1e-9,
                        'Trusted centre horizon differs from actual snapshot')
                slew = replay.check_plan(fixture,a['centre_plan'],a['node_references'])
                envelope = replay.check_envelope(fixture,a['envelope'])
                stopping = replay.stopping_violation(fixture,a['node_states'])
                tolerance = replay.number(config['feasibility_tolerance'])
                require(stopping <= tolerance and 0 <= replay.number(a['original_primal_violation']) <= tolerance
                    and abs(stopping-replay.number(a['stopping_violation'])) <= 1e-12,
                    'Trusted centre violates original nonlinear stopping reserve')
                checked.append(dict(kind=kind,plan_violation=slew,envelope_violation=envelope,
                    nonlinear_stopping_violation=stopping,elapsed_ms=a['elapsed_ms']))
            require(checked and projections <= 1 and d['subproblems']+projections <= 4,
                    'No trusted centre or original total QP budget exceeded')
            states = replay.vector(d['predicted_states'], 12*config['nodes'])
            nonlinear_nodes = [states[12*i+j] for i in range(config['nodes']) for j in (6,7,8,9)]
            stopping = replay.stopping_violation(fixture,nonlinear_nodes)
            require(stopping <= config['feasibility_tolerance']
                and abs(stopping-replay.number(d['recovery_stopping_violation'])) <= 1e-12,
                'Final actual nonlinear candidate violates original stopping reserve')
            total_projection += projections
            recovered.append(dict(sequence=int(snapshot.sequence),task_time_s=float(snapshot.task_time_s),
                centre_attempts=len(attempts),trusted_centres=checked,projection_solves=projections,
                control_subproblems=d['subproblems'],final_nonlinear_stopping_violation=stopping,
                core_ms=d['compute_ms']))
        previous = list(value['result']['command'])
    return dict(verified=True,normal_calls=normal,recovery_calls=len(recovered),projection_solves=total_projection,
        calls=recovered,initial_trust_radius=.02,core_budget_ms=50.,
        recovery_core_ms=summary([c['core_ms'] for c in recovered]) if recovered else None,
        compact_capture_scope='Copied actual native centre/final nodes, extrema and integrated reference; full QP rows and model rollout are independently bound by software admission, not recaptured/reconstructed here')


def failure_context(run):
    """Best-effort honest prefix observations; never an acceptance fallback."""
    result = dict(scope='Original failure/prefix observations only; incomplete task is not physical acceptance',
                  full_window_validated=False, physical_acceptance=False)
    for name in ('controller_metadata.json','stage_result.json','result.json','isolated_live/lifecycle.json',
                 'isolated_live/worker_final.json','physical_disturbance/applied_wrench.csv.status.json'):
        path = run/name
        if path.exists():
            try:
                doc = read(path)
                result[name] = {k:doc[k] for k in ('error','last_error','complete','passed','success','faulted',
                    'requests','accepted','landed_and_disarmed','diagnostic_parameters_restored',
                    'state','anchored','rows','task_rows','last_task_sim_s','t0_sim_s') if k in doc}
            except Exception as error:
                result[name] = dict(read_error=repr(error))
    native = sorted((run/'isolated_live/native_records').glob('*.json'))
    if native:
        try:
            observations=[];returned=rejected=0
            for path in native:
                value=read(path)
                returned+=int('result' in value and value.get('error')=='')
                rejected+=int(bool(value.get('error')))
                diagnostic=value.get('result',{}).get('diagnostics',value.get('rejected_diagnostics',{}))
                if diagnostic.get('recovery_triggered') is True:
                    observations.append(dict(sequence=value.get('sequence'),error=value.get('error'),
                        accepted=diagnostic.get('accepted'),compute_ms=diagnostic.get('compute_ms'),
                        control_subproblems=diagnostic.get('subproblems'),
                        centres=[{k:a.get(k) for k in ('centre_kind','trusted','rejection','elapsed_ms',
                            'projection_attempted','projection_solved')} for a in diagnostic.get('recovery_attempts',[])]))
            result['native_recovery_observations']=dict(record_count=len(native),returned_calls=returned,
                rejected_calls=rejected,recovery_calls=len(observations),calls=observations,
                scope='Recorded original status observations only; compact numerical validity and complete execution are checked separately')
            last = read(native[-1]); result['last_native_record'] = dict(identity=identity(native[-1]),
                sequence=last.get('sequence'),error=last.get('error'))
            d = last.get('result',{}).get('diagnostics',last.get('rejected_diagnostics',{}))
            result['last_native_record']['diagnostics'] = {k:d[k] for k in ('accepted','subproblems',
                'compute_ms','recovery_triggered','recovery_reason','recovery_attempts','recovery_stopping_violation') if k in d}
        except Exception as error:
            result['last_native_record'] = dict(read_error=repr(error))
    try:
        start = read(run/'isolated_live/live_start.json')
        require(native, 'No original worker snapshot identifies the controlled prefix boundary')
        last = read(native[-1]); binding = last['snapshot_cdr']; snapshot_path = Path(binding['path'])
        require(identity(snapshot_path) == binding, 'Original boundary snapshot bytes changed')
        from rclpy.serialization import deserialize_message
        from flyarm_control_interfaces.msg import ControlSnapshot
        snapshot = deserialize_message(snapshot_path.read_bytes(),ControlSnapshot)
        require(int(snapshot.sequence) == last['sequence'], 'Prefix boundary snapshot sequence differs')
        limit = float(snapshot.control_time_s)-float(start['origin_s'])
        require(0 <= limit <= 24.05, 'Original boundary snapshot is outside the task')
        limit = min(limit,24.)
        selected = [r for r in rows(run/'native_trace.csv')
            if -1e-9 <= float(r['sim_s'])-start['origin_s'] <= limit+1e-9]
        require(len(selected)>1 and all(all(r[k]=='1' for k in ('valid_base','valid_joint_state','valid_tip'))
            for r in selected), 'No valid physical trace covers the original controller prefix')
        t = np.asarray([float(r['sim_s'])-start['origin_s'] for r in selected])
        tip = np.asarray([[float(r['tip_'+axis]) for axis in 'xyz'] for r in selected])
        from simulations.x650_cpp_fast_v1.qualification import figure8
        target = figure8(start['target_position_enu'])
        desired = np.asarray([target.sample(at)[0] for at in t])
        error = np.linalg.norm(tip-desired,axis=1)
        require(np.isfinite(error).all() and np.isfinite(t).all() and np.all(np.diff(t)>0),
                'Invalid physical prefix chronology or coordinates')
        integrate = getattr(np,'trapezoid',np.trapz)
        result['native_prefix_error'] = dict(first_task_s=float(t[0]),last_task_s=float(t[-1]),samples=len(t),
            position_rmse_m=float(np.sqrt(integrate(error**2,t)/(t[-1]-t[0]))),position_max_m=float(error.max()),
            same_time=True,no_phase_alignment=True,time_weighted=True,
            boundary_snapshot=binding,boundary_control_task_s=limit,
            scope='Physical trace only through the final original worker input, excluding later takeover/landing; a rejected input boundary does not imply its candidate executed',
            full_24s_observed=bool(t[0]<=1e-8 and t[-1]>=23.996-1e-9))
    except Exception as error:
        result['native_prefix_error'] = dict(read_error=repr(error))
    return result


def audit(launch, run, out):
    report=dict(kind='x650_cpp_recovery_robustness_stage_certificate_v1',stage='fast',passed=False,complete=False,
        certificate=False,physical_acceptance=False,hardware_qualified=False,launch_directory=str(launch),run_directory=str(run),source_run_directory=str(run),
        integrity_and_execution_passed=False,recovery_performance_passed=None,overall_disturbance_test_passed=False,
        auditor_version='recovery_physical_v1',auditor_identity=identity(Path(__file__)))
    artifacts=[]
    try:
        # Even a failed audit retains the hashes of its exact original evidence.
        paths={p for base in (launch,run) for p in base.rglob('*') if p.is_file()}
        paths.add(Path(__file__).resolve())
        paths.update(ROOT/name for name in ('scripts/audit_x650_recovery_robustness.py','scripts/audit_x650_recovery_replay.py',
            'simulations/x650_cpp_recovery_v1/qualification.py','simulations/x650_cpp_recovery_v1/launch.py',
            'control_packages/x650_wholebody_v1/native/mpc_core.cpp','control_packages/x650_wholebody_v1/native/mpc_core.hpp',
            'ros2_ws/src/flyarm_mpc_cpp/src/isolated_worker.cpp','ros2_ws/src/flyarm_mpc_cpp/include/flyarm_mpc_cpp/diagnostics_json.hpp',
            'scripts/audit_x650_cpp_robustness.py','scripts/audit_x650_cpp_fast.py','scripts/audit_x650_cpp_hold.py','scripts/report_x650_cpp_live.py',
            'scripts/audit_x650_cpp_source.py','scripts/audit_x650_cpp_fast_source.py','scripts/audit_x650_streaming_consumption.py',
            'scripts/audit_x650_isolated_live.py','scripts/audit_x650_precision_stage.py','scripts/audit_x650_upper_stage.py',
            'scripts/analyze_x650_hover_precision.py','simulations/x650_cpp_live_v1/qualification.py',
            'simulations/x650_ts20_hold_v1/audit.py','simulations/x650_ts20_hold_v1/task.py',
            'simulations/x650_cpp_fast_v1/qualification.py','simulations/x650_cpp_fast_v1/task.py',
            'simulations/x650_cpp_normal_v1/qualification.py','simulations/x650_cpp_normal_v1/task.py',
            'simulations/x650_cpp_robustness_v1/qualification.py','simulations/x650_cpp_robustness_v1/task.py',
            'simulations/x650_cpp_robustness_v1/run_case.py','simulations/x650_cpp_robustness_v1/launch.py',
            'scripts/prepare_x650_robustness_models.py','scripts/verify_x650_robustness_model.py',
            'simulations/x650_cpp_robustness_v1/disturbance.py','benchmark/whole_body_closed_loop_v1/disturbance.cpp',
            'benchmark/whole_body_closed_loop_v1/disturbance_loader.py','benchmark/whole_body_closed_loop_v1/gz_world_entity_binding.py'))
        artifacts=[identity(p) for p in sorted(paths) if p.is_file()]
        report['failure_context']=failure_context(run)
        meta=read(run/'controller_metadata.json')
        require(all(meta.get(k) is True for k in ('complete','landed_and_disarmed','diagnostic_parameters_restored'))
            and meta.get('diagnostic_restore_pending') is False, 'Incomplete flight lifecycle: '+str(meta.get('error')))
        stage=read(run/'stage_result.json')
        require(all(stage.get(k) is True for k in ('complete','passed','success','landed_and_disarmed'))
            and read(run/'result.json').get('success') is True, 'Original provisional stage/run failed')
        cfg,protocol,startup,launch_record,proof=check_archives(launch,run,meta)
        report['software_qualification']=proof
        report['upstream_shared_source_proof']=proof['upstream_shared_source_proof']
        report['physical_model']=check_physical_model(cfg,meta,stage,proof)
        live=run/'isolated_live'; lifecycle=read(live/'lifecycle.json'); start=read(live/'live_start.json')
        require(lifecycle['started'] is True and lifecycle['error'] is None
            and all(p['returncode']==0 for p in lifecycle['processes'].values()), 'Native child lifecycle failed')
        require(start['history_count']>0 and abs(meta['task_end_sim_s']-start['origin_s']-24.)<=.05,
                'Incomplete full 24 s fast figure-eight task')
        window=[start['origin_s'],start['origin_s']+24.]
        import audit_x650_isolated_live as old
        report['ground_initialization']=old.check_preflight_transport(live,meta,start)
        report['fault_supervision']=old.check_fault_supervision(live,start)
        resources=read(live/'execution_resources.json')
        require(resources==launch_record['execution_resources'] and resources['roles']==protocol['cpu_roles'], 'Actual resources changed')
        observed=read(live/'execution_affinity_at_handoff.json')
        require(set(observed)=={'source',*lifecycle['processes']}, 'Incomplete process placement observation')
        for name,threads in observed.items():
            if name=='worker':continue
            require(threads and all(mask==resources['roles'].get(name,resources['background']) for mask in threads.values()),
                    'Handoff CPU partition changed: '+name)
        actual_startup=read(live/'worker_startup.json')
        require(actual_startup['startup_manifest']==startup and actual_startup['python_in_worker'] is False
            and actual_startup['core_abi']==2
            and actual_startup['startup_controller_steps']==0 and actual_startup['diagnostic_mode']=='compact'
            and actual_startup['session_id']==start['session_id'] and actual_startup['epoch']==start['epoch']
            and actual_startup['configuration_sha256']==recovery_qualification().sha(launch/'task.json'), 'Actual worker startup differs')
        for field,loaded in [('core_library','loaded_core_path'),('optimization_library','loaded_optimization_path'),('predictor_library','loaded_predictor_path')]:
            require(str(Path(startup[field]['path']).resolve())==actual_startup[loaded], 'Actually loaded native library differs')
        import audit_x650_streaming_consumption as streaming
        messages=streaming.original_messages(live)
        sends,receipts,ticks,events=(messages[k] for k in ('sends','receipts','dispatch','events'))
        calls,report['worker']=check_worker(live,start,startup['config'],events,resources,protocol)
        report['worker']['numerical_scope']='Actual owning outputs checked against original wire and physical limits; ABI 2 software qualification independently reverified'
        report['optimizer_recovery']=check_runtime_recoveries(calls,start,startup['config'])
        report['actual_send_barrier']=check_actual_send_barrier(read(live/'ingress_timing.json'),sends,calls)
        source_rows=rows(live/'cpp_source/source_selections.csv')
        source_issues={int(v['sequence']):int(v['issued_ns']) for v in source_rows}
        require(len(source_issues)==len(source_rows), 'Repeated source issue identity')
        report['timing']=check_ticks(ticks,sends,calls,start,source_issues=source_issues)
        publications=[json.loads(line) for line in (run/'control_execution_trace.jsonl').open() if line.strip()]
        report['consumption']=streaming.audit_consumption(sends,receipts,rows(run/'ts20_state.csv'),start,lifecycle['release'],publications)
        require(report['consumption']['passed'] is True, 'Asynchronous physical consumption accounting failed')
        from flyarm_control_interfaces.msg import ActuatorState
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        baseline=message_to_ordereddict(deserialize_message((live/'actuator_epoch_state.cdr').read_bytes(),ActuatorState))
        report['consumption']['online_ledger']=streaming.audit_async_ledger(read(live/'actuator_ledger.json'),baseline,
            sends,receipts,rows(run/'ts20_state.csv'),start,lifecycle['release'])
        require(report['consumption']['online_ledger']['passed'] is True, 'Online asynchronous progress evidence failed')
        from audit_x650_cpp_fast_source import audit_source
        reference=read(run/'hover_reference.json')
        report['feedback']=audit_source(live/'cpp_source',run/'ts20_state.csv',run/'ts20_packets.csv',reference,
            read(run/'estimated_feedback.json')['extra_reset_counters_at_bind'],worker_directory=live/'native_records',
            expected_binding=dict(configuration_sha256=actual_startup['configuration_sha256'],
                position_gains=startup['config']['base_position_gain'],bus_period_s=startup['config']['bus_period'],delay_s=startup['config']['motion'][4]),
            trajectory=startup['trajectory'])
        require(all(report['feedback'][key] is True for key in ('passed','record_integrity_passed','clean_source_stop','worker_link_checked'))
            and report['feedback']['original_worker_snapshots_verified']==len(calls), 'Raw C++ feedback/source validation failed')
        docs={k:read(run/n) for k,n in {'session':'diagnostic_parameter_session.json','journal':'diagnostic_restore_journal_completed.json',
            'before':'diagnostic_parameters_before.json','applied':'diagnostic_parameters_applied.json','restored':'diagnostic_parameters_restored.json',
            'px4_before':'px4_parameters_before.json','px4_after':'px4_parameters_after.json'}.items()}
        _,pid,_=old.frozen_research_pid(cfg)
        for name,document in [('metadata',meta),('stage',stage)]:
            old.fixed.verify_condition(document,old.condition_for_task(cfg),name)
        report['parameters']=old.fixed.verify_parameter_session(docs,meta,old.condition_for_task(cfg),pid)
        old.precision.verify_reference(reference,old.precision.DEFAULT_REFERENCE_SOURCE)
        manifest=read(run/'native_trace_manifest.json')
        require(manifest['complete'] is True and manifest['sha256']==recovery_qualification().sha(run/'native_trace.csv'), 'Incomplete native physical trace')
        for document in (stage,manifest,read(run/'native_ground.json'),read(run/'native_ground_manifest.json')):
            for key in ('model_sha256','source_urdf_sha256'):
                require(document[key]==meta[key], 'Actual physical model observer identity differs: '+key)
        old.precision.capture_window_covers(manifest,window)
        native=old.precision.native_states(run/'native_trace.csv',window)
        from pyulog import ULog
        ulog=ULog(str(run/'px4.ulg'))
        report['recovery']=check_recovery(old.precision.dataset(ulog,'vehicle_status'),
            old.precision.dataset(ulog,'vehicle_land_detected'),rows(run/'flight_samples.csv'),window[1])
        report['configuration']=old.fixed.effective_configuration(ulog,native,reference,window,old.fixed.records(docs['applied']))
        control,derived=old.command_view(run,sends,calls,start)
        (out/'derived_command_audit_view.json').write_text(json.dumps(dict(scope='Actual send CDR matched to original worker/source; originals untouched',rows=derived),allow_nan=False))
        value_control,value_proof=ulog_value_view(control,derived,sends)
        (out/'distinct_pva_value_audit_view.json').write_text(json.dumps(dict(**value_proof,rows=value_control),allow_nan=False))
        report['px4_value_view']=value_proof
        report['px4_commands']=old.upper.upper_commands(ulog,native,reference,value_control,window,pid,response=False)
        report['px4_commands']['all_distinct_pva_value_rows_observed']=report['px4_commands'].pop('all_measurement_sends_observed')
        report['px4_commands']['distinct_pva_measurement_value_rows']=report['px4_commands'].pop('measurement_sends')
        report['px4_commands']['observation_scope']=value_proof['scope']
        report['flight']=old.upper.flight_states(run/'flight_samples.csv',window)
        report['ground']=old.fixed.native_ground_check(run,read(run/'native_ground.json'))
        # The legacy TS20 predictor dispatches on its original hold scenario.
        # The fast task is an exact trajectory-only composition of that
        # bound plant. Keep its predictor identity and audit the actual 24 s.
        report['actuator']=old.prior_audit.actuator_check(run,baseline_config(cfg['frozen_cpp_hold']),window)
        report['actuator'].pop('control_snapshot_to_plugin_consumption_lag',None)
        require(report['actuator']['new_joint_prediction_verified'], 'Independent whole-window joint prediction failed')
        endpoints=endpoint_windows(run,start,cfg,old.shared.native_tip_metrics)
        tilt=float(native['tilt'].max())
        require(np.isfinite(tilt) and tilt<=cfg['ts20_protocol']['tilt_max_deg'], 'Fast tilt envelope exceeded')
        report.update(**endpoints,tilt_peak_deg=tilt,window_sim_s=window,figure8_protocol=cfg['figure8_protocol'],
            frozen_cpp_hold=cfg['frozen_cpp_hold'],cpp_hold_certificate=cfg['cpp_hold_certificate'],
            frozen_cpp_normal=cfg['frozen_cpp_normal'],cpp_normal_certificate=cfg['cpp_normal_certificate'],
            frozen_cpp_fast=cfg['frozen_cpp_fast'],cpp_fast_certificate=cfg['cpp_fast_certificate'],robustness=cfg['robustness'])
        report['external_wrench']=check_wrench(run,start,cfg,meta,stage)
        report['robustness_performance']=robustness_performance(run,start,cfg)
        report['recovery_performance_passed']=report['robustness_performance']['performance_passed']
        require(all(identity(v['path'])==v for v in artifacts), 'Original evidence changed while auditing')
        report.update(passed=True,complete=True,certificate=True,physical_acceptance=True,integrity_and_execution_passed=True,
            full_window_ulog_verified=True,parameter_restore_verified=True,native_ground_verified=True,actuator_log_verified=True,
            overall_disturbance_test_passed=report['recovery_performance_passed'] is True,
            acceptance_scope='passed/certificate concern full-window integrity, execution and original physical safety envelope; recovery performance retains the separately reported original 10 mm continuously for 1 s rule',
            scope='One complete original 24 s physical-robustness fast figure-eight; fixed same-time/world target, original disturbance and newly qualified ABI 2 recovery backend; no delay injection, fitting, phase alignment, hardware qualification or worst-case timing guarantee')
    except Exception as error:
        report.update(error=repr(error),traceback=traceback.format_exc(),integrity_and_execution_passed=False)
        if 'failure_context' not in report:report['failure_context']=failure_context(run)
    report['artifact_identities']=artifacts
    report['artifacts']={a['path']:a['sha256'] for a in artifacts}
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('launch-dir','run-dir','output'): parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args(); launch=args.launch_dir.resolve();run=args.run_dir.resolve();out=args.output.resolve()
    require(not out.exists() and not out.is_relative_to(run) and not out.is_relative_to(launch), 'New audit output must be outside originals')
    out.mkdir(parents=True)
    result=audit(launch,run,out)
    print(json.dumps({k:result.get(k) for k in ('passed','integrity_and_execution_passed','recovery_performance_passed','overall_disturbance_test_passed','error','endpoint','tracking_endpoint','tail_endpoint','timing','optimizer_recovery')},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
