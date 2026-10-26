#!/usr/bin/env python3
"""Audit a new complete disturbed flight with the ABI 3 recovery backend.

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


def interval_qualification():
    # Keep partial-run evidence collection available even if an incomplete
    # deployment omitted the new admission package. Complete acceptance still
    # imports and executes the explicit new qualification below.
    from simulations.x650_cpp_interval_v1 import qualification
    return qualification


def check_archives(launch, run, meta):
    qualification = interval_qualification()
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
    require(proof.get('ready') is True and proof.get('kind') == 'x650_cpp_interval_admission_v1'
        and proof.get('physical_acceptance') is False, 'New backend admission does not replace physical acceptance')
    require(manifest['task_config']['sha256'] == qualification.sha(launch/'task.json')
        and manifest['protocol']['sha256'] == qualification.sha(launch/'protocol.json')
        and manifest['startup_manifest']['sha256'] == qualification.sha(launch/'original_startup_manifest.json'),
        'Recovery qualification differs from actual archived execution')
    proof['upstream_shared_source_proof'] = upstream_shared_source_proof(cfg)
    require(all(name in sources for name in (
        'simulations/x650_cpp_interval_v1/qualification.py','simulations/x650_cpp_interval_v1/launch.py',
        'simulations/x650_cpp_robustness_v1/task.py','simulations/x650_cpp_robustness_v1/run_case.py',
        'simulations/x650_cpp_robustness_v1/disturbance.py',
        'control_packages/x650_wholebody_v1/native/mpc_core.cpp',
        'control_packages/x650_wholebody_v1/native/mpc_core.hpp',
        'control_packages/x650_wholebody_v1/native/canonical_interval.hpp',
        'control_packages/x650_wholebody_v1/native/joint_safety.hpp',
        'ros2_ws/src/flyarm_mpc_cpp/src/isolated_worker.cpp',
        'ros2_ws/src/flyarm_mpc_cpp/include/flyarm_mpc_cpp/diagnostics_json.hpp',
        'ros2_ws/src/flyarm_mpc_cpp/include/flyarm_mpc_cpp/worker_contract.hpp',
        'ros2_ws/src/flyarm_state_cpp/include/flyarm_state_cpp/source_core.hpp')),
        'Recovery source archive omits actual native recovery/worker or physical implementation')
    return cfg, protocol, startup, record, proof


from audit_x650_recovery_robustness import check_runtime_recoveries, failure_context as recovery_failure_context
import audit_x650_interval_constraints as interval


def interval_fixture(snapshot, config, previous_command):
    """Read actual CDR history; hypothetical centre packets are never execution history."""
    from audit_x650_recovery_robustness import recovery_fixture
    fixture=recovery_fixture(snapshot,config,previous_command)
    inp=fixture['input'];inp['state'].update(q=list(map(float,snapshot.q)),dq=list(map(float,snapshot.dq)))
    inp['context'].update(bus_origin=float(snapshot.bus_origin_s),packets=[dict(
        send_time=float(snapshot.packet_send_time_s[i]),
        **{field:list(map(float,getattr(snapshot,'packet_'+field)[2*i:2*i+2]))
           for field in ('q_ref','dq_ref','tau_ff','kp','kd')}) for i in range(snapshot.packet_count)])
    require(inp['context']['packets'],'Actual snapshot omits immutable packet history')
    return fixture


def failure_context(run):
    result=recovery_failure_context(run)
    try:
        observations=[]
        for path in sorted((run/'isolated_live/native_records').glob('*.json')):
            value=read(path);d=value.get('result',{}).get('diagnostics',value.get('rejected_diagnostics',{}))
            if d.get('interval_repairs'):
                observations.append(dict(sequence=value.get('sequence'),error=value.get('error'),
                    accepted=d.get('accepted'),actual_bounds=d.get('actual_bounds'),compute_ms=d.get('compute_ms'),
                    control_subproblems=d.get('subproblems'),interval_rows=d.get('interval_rows'),
                    candidate_envelope=d.get('candidate_envelope'),
                    repairs=[{k:a.get(k) for k in ('control_subproblem','added_points','total_points',
                        'elapsed_ms','rejection','envelope','trigger_points')} for a in d['interval_repairs']]))
        result['native_interval_observations']=dict(calls=observations,interval_calls=len(observations),
            scope='Original copied statuses only; incomplete or rejected call does not establish physical acceptance')
    except Exception as error:result['native_interval_observations']=dict(read_error=repr(error))
    return result


def check_runtime_intervals(calls,start,config):
    """Independent compact interval arithmetic, not a missing full-QP reconstruction.

    Every native final envelope is checked again. The copied centre events let
    us recompute the declared sliding gradients and encoding kernels. Actual
    physical errors, joint consumption and PX4 exit are separate checks below.
    Compact captures do not contain full QP rows or dense observer pieces; their
    exact implementation and full-matrix checks remain bound by admission.
    """
    require(calls,'No actual native calls for interval evidence')
    previous=list(start['history_command'][(start['history_count']-1)*5:start['history_count']*5])
    records=[];N=config['nodes'];n=5*N;base_min,base_max=9*N+5,15*N+5;base_counts={}
    def key(point):return (point['time'],point['joint'],point['component'])
    def same(a,b):return a[1:]==b[1:] and abs(a[0]-b[0])<=1e-10
    for snapshot,proposal,value in calls:
        d=value['result']['diagnostics'];fixture=interval_fixture(snapshot,config,previous)
        require(d['accepted'] is True and d['actual_bounds'] is True and d['nonlinear_passed'] is True,
                'Interval evidence attached to rejected actual candidate')
        require(1<=d['subproblems']<=3 and 0<=replay.number(d['compute_ms'])<=50.,'Interval control budget changed')
        replay.check_envelope(fixture,d['candidate_envelope'])
        replay.check_plan(fixture,d['plan'],replay.node_references(fixture,d['plan']))
        repairs=d['interval_repairs'];cuts=d['interval_constraints']
        require(isinstance(repairs,list) and isinstance(cuts,list) and len(repairs)<=2
            and type(d['interval_rows']) is int and d['interval_rows']==len(cuts)<=512
            and type(d['qp_columns']) is int and d['qp_columns']==n and type(d['qp_rows']) is int,
            'Actual interval shape, original QP rows or no-slack dimensions changed')
        # The unchanged assembler omits a Q/D/stopping row only when its
        # coefficients are exactly zero and its constant bounds are feasible.
        # Slew 5N, reference 2N, trust 2N and first-command 5 rows are nonzero.
        # Compact evidence reports the actual count, but omits the base matrix;
        # this range check does not independently prove the omitted rows.
        base=d['qp_rows']-len(cuts)
        require(base_min<=base<=base_max,'Actual original base QP row count outside assembly bounds')
        base_counts[base]=base_counts.get(base,0)+1
        if not repairs:
            require(not cuts and d['interval_canonical_substeps']==0
                and d['interval_canonical_envelope']==[0.]*8
                and d['interval_centre_plan']==[] and d['interval_canonical_boundaries']==[]
                and d['interval_canonical_states']==[], 'Untriggered interval evidence is not empty')
            previous=list(value['result']['command']);continue
        generated=[];cumulative=0;last_problem=0;centres=[];last_elapsed=-1.
        for repair in repairs:
            problem=repair['control_subproblem'];elapsed=replay.number(repair['elapsed_ms'])
            require(type(problem) is int and last_problem<problem<d['subproblems']
                and last_elapsed<=elapsed<=d['compute_ms'] and repair['rejection']=='',
                'Actual interval retry timing/order/status differs')
            last_problem,last_elapsed=problem,elapsed
            require(repair['risk_radius']==min(config['trust_radius'],.02)==.02,
                    'Actual interval search radius changed')
            plan=interval.finite(repair['centre_plan'],(N,5));packets=interval.centre_packets(fixture,plan)
            replay.check_packets(fixture,packets,plan.ravel().tolist())
            replay.check_plan(fixture,plan.ravel().tolist(),replay.node_references(fixture,plan.ravel().tolist()))
            _,_,_,boundaries=interval.canonical_grid(fixture,packets)
            duration=np.diff(boundaries)
            expected_substeps=int(sum(max(1,int(np.ceil(dt/config['integration_step']-1e-12))) for dt in duration if dt>0))
            require(repair['canonical_substeps']==expected_substeps>0,'Actual repair canonical substep grid changed')
            points=repair['points'];triggers=repair['trigger_points'];envelope=interval.finite(repair['envelope'],(8,))
            require(isinstance(points,list) and 1<=len(points)<=512 and isinstance(triggers,list)
                and 1<=len(triggers)<=8,'Actual interval trigger/risk coverage missing')
            margins=interval.finite(repair['risk_margin'],(len(points),))
            lows=interval.finite(repair['risk_encoding_lower'],(len(points),))
            highs=interval.finite(repair['risk_encoding_upper'],(len(points),))
            directions=set()
            for point in triggers:
                lo,hi=interval.point_limits(fixture,point)
                require(point['value']<lo-interval.TOL or point['value']>hi+interval.TOL,
                        'Actual interval batch lacks a hard native violation trigger')
                directions.add((point['joint'],point['component'],'lower' if point['value']<lo-interval.TOL else 'upper'))
                require(any(same(key(p),key(point)) and abs(p['value']-point['value'])<=1e-12 for p in points),
                        'Actual interval batch dropped original native violation')
            added=0
            for point,margin,low,high in zip(points,margins,lows,highs):
                lo,hi=interval.point_limits(fixture,point);j,component=point['joint'],point['component']
                gradient=interval.fast_branch_gradient(fixture,packets,boundaries,point)
                expected_margin=.02*sum(abs(gradient[k]) for k in range(n) if k%5>=3)
                require(abs(margin-expected_margin)<=1e-10,'Actual risk-point sliding range differs')
                encoding=interval.encoding_guard(fixture,packets,plan,point)
                interval.close([low,high],[encoding['encoding_lower_remainder'],encoding['encoding_upper_remainder']],
                    1e-12,'actual risk-point encoding range')
                require(((j,component,'lower') in directions and point['value']-margin+low<=lo+interval.TOL)
                    or ((j,component,'upper') in directions and point['value']+margin+high>=hi-interval.TOL),
                    'Actual risk point is outside the triggered search scope')
                require(point['source_kind'] in (0,2,3) or any(same(key(point),key(p)) for p in triggers),
                        'Untriggered substep endpoint entered actual risk batch')
                require(envelope[(0 if component==0 else 4)+j]-1e-12<=point['value']
                    <=envelope[(2 if component==0 else 6)+j]+1e-12,
                    'Actual risk point is outside its recorded native envelope')
                if not any(same(key(point),p) for p in generated):generated.append(key(point));added+=1
            cumulative+=added
            require(repair['added_points']==added and repair['total_points']==cumulative<=512,
                    'Actual merged interval point count differs')
            centres.append((elapsed,plan))
        require(cumulative==len(cuts)>0,'Actual final cut list omits generated interval points')
        # A trusted infeasibility-recovery seed can follow an interval retry.
        # Select the last copied centre event rather than assuming the last
        # failed plan remains the linearization centre in all mixed branches.
        for a in d['recovery_attempts']:
            if a['trusted'] is True:centres.append((replay.number(a['elapsed_ms']),interval.finite(a['centre_plan'],(N,5))))
        require(len({at for at,_ in centres})==len(centres),'Ambiguous compact centre event chronology')
        plan=max(centres,key=lambda item:item[0])[1];packets=interval.centre_packets(fixture,plan)
        _,_,_,boundaries=interval.canonical_grid(fixture,packets);z=(plan/interval.finite(config['velocity_limit'],(5,))).ravel()
        final_keys=[]
        for i,cut in enumerate(cuts):
            point=cut['point'];interval.point_limits(fixture,point);point_key=key(point)
            require(cut['qp_row']==base+i and any(same(point_key,p) for p in generated)
                and not any(same(point_key,p) for p in final_keys),'Actual interval cut identity or QP row differs')
            final_keys.append(point_key)
            gradient=interval.finite(cut['gradient'],(n,))
            interval.close(gradient,interval.fast_branch_gradient(fixture,packets,boundaries,point),1e-10,
                'actual interval sliding gradient')
            require(abs(replay.number(cut['affine_offset'])+gradient@z-point['value'])<=1e-10,
                    'Actual interval affine centre identity differs')
            encoding=interval.encoding_guard(fixture,packets,plan,point)
            for name,expected in encoding.items():interval.close([cut[name]],[expected],1e-12,'actual interval '+name)
        require(d['interval_canonical_substeps']==d['candidate_substeps']>0,
                'Actual interval canonical and final propagation grid differ')
        records.append(dict(sequence=int(snapshot.sequence),task_time_s=float(snapshot.task_time_s),
            control_subproblems=d['subproblems'],interval_repairs=len(repairs),interval_rows=len(cuts),
            core_ms=d['compute_ms'],final_native_envelope=d['candidate_envelope']))
        previous=list(value['result']['command'])
    return dict(verified=True,calls_checked=len(calls),interval_calls=len(records),calls=records,
        original_base_rows=dict(allowed_min=base_min,allowed_max=base_max,
            observed_min=min(base_counts),observed_max=max(base_counts),
            observed_counts={str(k):v for k,v in sorted(base_counts.items())},
            full_base_matrix_independently_verified=False,
            scope='Compact dynamic count range and appended cut identities only; original full base matrix/zero-row omission remains bound by qualified assembly implementation'),
        hard_q_padding_rad=.0003,hard_dq_padding_rad_s=.03,feasibility_tolerance=2e-7,
        no_slack_variables=True,control_qp_budget=3,core_budget_ms=50.,canonical_query_grid_unchanged_by_design=True,
        compact_capture_scope='Copied actual interval events and final native envelopes with independently recomputed sliding gradients and future-packet encoding kernels; full QP rows and native dense pieces remain bound by software admission, not recaptured or rerun here')


def audit(launch, run, out):
    report=dict(kind='x650_cpp_interval_robustness_stage_certificate_v1',stage='fast',passed=False,complete=False,
        certificate=False,physical_acceptance=False,hardware_qualified=False,launch_directory=str(launch),run_directory=str(run),source_run_directory=str(run),
        integrity_and_execution_passed=False,recovery_performance_passed=None,overall_disturbance_test_passed=False,
        auditor_version='interval_physical_v2',auditor_identity=identity(Path(__file__)))
    artifacts=[]
    try:
        # Even a failed audit retains the hashes of its exact original evidence.
        paths={p for base in (launch,run) for p in base.rglob('*') if p.is_file()}
        paths.add(Path(__file__).resolve())
        paths.update(ROOT/name for name in ('scripts/audit_x650_interval_robustness.py','scripts/audit_x650_recovery_robustness.py','scripts/audit_x650_recovery_replay.py','scripts/audit_x650_interval_constraints.py',
            'simulations/x650_cpp_interval_v1/qualification.py','simulations/x650_cpp_interval_v1/launch.py',
            'control_packages/x650_wholebody_v1/native/mpc_core.cpp','control_packages/x650_wholebody_v1/native/mpc_core.hpp',
        'control_packages/x650_wholebody_v1/native/canonical_interval.hpp',
        'control_packages/x650_wholebody_v1/native/joint_safety.hpp',
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
            and actual_startup['core_abi']==3
            and actual_startup['startup_controller_steps']==0 and actual_startup['diagnostic_mode']=='compact'
            and actual_startup['session_id']==start['session_id'] and actual_startup['epoch']==start['epoch']
            and actual_startup['configuration_sha256']==interval_qualification().sha(launch/'task.json'), 'Actual worker startup differs')
        for field,loaded in [('core_library','loaded_core_path'),('optimization_library','loaded_optimization_path'),('predictor_library','loaded_predictor_path')]:
            require(str(Path(startup[field]['path']).resolve())==actual_startup[loaded], 'Actually loaded native library differs')
        import audit_x650_streaming_consumption as streaming
        messages=streaming.original_messages(live)
        sends,receipts,ticks,events=(messages[k] for k in ('sends','receipts','dispatch','events'))
        calls,report['worker']=check_worker(live,start,startup['config'],events,resources,protocol)
        report['worker']['numerical_scope']='Actual owning outputs checked against original wire and physical limits; ABI 3 software qualification independently reverified'
        report['optimizer_recovery']=check_runtime_recoveries(calls,start,startup['config'])
        report['optimizer_interval']=check_runtime_intervals(calls,start,startup['config'])
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
        require(manifest['complete'] is True and manifest['sha256']==interval_qualification().sha(run/'native_trace.csv'), 'Incomplete native physical trace')
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
            scope='One complete original 24 s physical-robustness fast figure-eight; fixed same-time/world target, original disturbance and newly qualified ABI 3 recovery backend; no delay injection, fitting, phase alignment, hardware qualification or worst-case timing guarantee')
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
    print(json.dumps({k:result.get(k) for k in ('passed','integrity_and_execution_passed','recovery_performance_passed','overall_disturbance_test_passed','error','endpoint','tracking_endpoint','tail_endpoint','timing','optimizer_recovery','optimizer_interval')},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
