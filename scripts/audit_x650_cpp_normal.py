#!/usr/bin/env python3
"""Independent 32-second ordinary C++ MPC figure-eight acceptance.

Reuses established raw worker/dispatch/consumption/ULog checks. No physics,
optimizer, fitting or record repair is performed. A partial run never qualifies.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import traceback
import zipfile
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts'),str(ROOT/'control_packages/x650_wholebody_v1')]
from simulations.x650_cpp_normal_v1 import qualification
from simulations.x650_cpp_normal_v1.task import baseline_config
from report_x650_cpp_live import identity, require, read
from audit_x650_cpp_hold import (rows, check_worker, check_ticks, check_actual_send_barrier,
    check_recovery, ulog_value_view)


def check_archives(launch,run,meta):
    record=read(launch/'launch.json')
    require(all(record.get(k) is True for k in ('complete','runtime_closed','sources_unchanged',
        'runtime_binaries_unchanged','launch_inputs_unchanged')),'Incomplete/changing normal launch')
    require(record.get('exit_code')==0 and record.get('execution_scenario')=='x650_cpp_normal_v1'
        and record.get('run_directories')==[str(run.resolve())],'Normal launch/run lifecycle differs')
    sources,binaries=qualification.archive_integrity(launch,record)
    cfg,protocol,startup=(read(launch/name) for name in ('task.json','protocol.json','startup_manifest.json'))
    require(cfg==record['task']==read(run/'controller_task.json')==meta['task'],'Actual normal task differs')
    require(protocol==record['protocol'] and cfg['duration_s']==protocol['duration_s']==32.,
            'Complete frozen 32 s normal required')
    for artifact in record['launch_inputs']:require(identity(artifact['path'])==artifact,'Launch input changed')
    original=read(launch/'original_startup_manifest.json')
    require({k:v for k,v in startup.items() if k!='task_config'}=={k:v for k,v in original.items() if k!='task_config'}
        and startup['task_config']['sha256']==original['task_config']['sha256']==qualification.sha(launch/'task.json'),
        'Startup derivation changed something besides identical task path')
    manifest=read(launch/'qualification_manifest.json')
    proof=qualification.verify(launch/'qualification_manifest.json',task_path=Path(manifest['task_config']['path']),
        startup_path=Path(manifest['startup_manifest']['path']),startup_sha256=manifest['startup_manifest']['sha256'],
        protocol_path=Path(manifest['protocol']['path']),binaries=binaries)
    require(manifest['task_config']['sha256']==qualification.sha(launch/'task.json')
        and manifest['protocol']['sha256']==qualification.sha(launch/'protocol.json')
        and manifest['startup_manifest']['sha256']==qualification.sha(launch/'original_startup_manifest.json'),
        'Qualified normal inputs differ from execution')
    # The phase-law implementation and normal admission are both archived.
    require(all(name in sources for name in ('simulations/x650_cpp_normal_v1/task.py',
        'simulations/x650_cpp_normal_v1/qualification.py',
        'ros2_ws/src/flyarm_state_cpp/include/flyarm_state_cpp/source_core.hpp',
        'ros2_ws/src/flyarm_mpc_cpp/include/flyarm_mpc_cpp/worker_contract.hpp')),
        'Normal execution source archive omits dynamic contract/law')
    return cfg,protocol,startup,record,proof


def endpoint_windows(run,start,cfg,metric=None):
    if metric is None:
        from simulations.x650_wholebody_mpc_v1.recording import native_tip_metrics
        metric=native_tip_metrics
    require(cfg['duration_s']==32. and cfg['trajectory_parameters']=={k:v for k,v in qualification.normal_trajectory().items()
        if k not in {'kind','duration_s'}},'Endpoint metrics require the frozen normal definition')
    target=qualification.figure8(start['target_position_enu']);limits=cfg['ts20_protocol']
    result={}
    for name,interval in [('endpoint',(0.,32.)),('tracking_endpoint',(2.,28.)),('tail_endpoint',(28.,32.))]:
        values=metric(run,target.sample,start['origin_s'],interval)
        require(values.get('same_time') is True and values.get('no_phase_alignment') is True
            and tuple(values.get('interval_task_s',()))==interval
            and np.isfinite(values['duration_s']) and values['duration_s']>=interval[1]-interval[0]-.04,
            'Normal endpoint metric lacks complete same-time window: '+name)
        require(all(np.isfinite(values[k]) and 0<=values[k]<=limits[limit] for k,limit in
            [('position_rmse_m','tip_rms_max_m'),('position_max_m','tip_peak_max_m')]),
            'Normal endpoint envelope exceeded: '+name)
        result[name]=values
    return result


def audit(launch, run, out):
    report=dict(kind='x650_cpp_normal_stage_certificate_v1',stage='normal',passed=False,complete=False,
        certificate=False,physical_acceptance=False,hardware_qualified=False,launch_directory=str(launch),run_directory=str(run),source_run_directory=str(run))
    artifacts=[]
    try:
        # Even a failed audit retains the hashes of its exact original evidence.
        paths={p for base in (launch,run) for p in base.rglob('*') if p.is_file()}
        paths.update(ROOT/name for name in ('scripts/audit_x650_cpp_normal.py','scripts/audit_x650_cpp_hold.py','scripts/report_x650_cpp_live.py',
            'scripts/audit_x650_cpp_source.py','scripts/audit_x650_cpp_normal_source.py','scripts/audit_x650_streaming_consumption.py',
            'scripts/audit_x650_isolated_live.py','scripts/audit_x650_precision_stage.py','scripts/audit_x650_upper_stage.py',
            'scripts/analyze_x650_hover_precision.py','simulations/x650_cpp_live_v1/qualification.py',
            'simulations/x650_ts20_hold_v1/audit.py','simulations/x650_ts20_hold_v1/task.py',
            'simulations/x650_cpp_normal_v1/qualification.py','simulations/x650_cpp_normal_v1/task.py'))
        artifacts=[identity(p) for p in sorted(paths)]
        meta=read(run/'controller_metadata.json')
        require(all(meta.get(k) is True for k in ('complete','landed_and_disarmed','diagnostic_parameters_restored'))
            and meta.get('diagnostic_restore_pending') is False, 'Incomplete flight lifecycle: '+str(meta.get('error')))
        stage=read(run/'stage_result.json')
        require(all(stage.get(k) is True for k in ('complete','passed','success','landed_and_disarmed'))
            and read(run/'result.json').get('success') is True, 'Original provisional stage/run failed')
        cfg,protocol,startup,launch_record,proof=check_archives(launch,run,meta)
        report['software_qualification']=proof
        live=run/'isolated_live'; lifecycle=read(live/'lifecycle.json'); start=read(live/'live_start.json')
        require(lifecycle['started'] is True and lifecycle['error'] is None
            and all(p['returncode']==0 for p in lifecycle['processes'].values()), 'Native child lifecycle failed')
        require(start['history_count']>0 and abs(meta['task_end_sim_s']-start['origin_s']-32.)<=.05,
                'Incomplete full 32 s ordinary figure-eight task')
        window=[start['origin_s'],start['origin_s']+32.]
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
            and actual_startup['startup_controller_steps']==0 and actual_startup['diagnostic_mode']=='compact'
            and actual_startup['session_id']==start['session_id'] and actual_startup['epoch']==start['epoch']
            and actual_startup['configuration_sha256']==qualification.sha(launch/'task.json'), 'Actual worker startup differs')
        for field,loaded in [('core_library','loaded_core_path'),('optimization_library','loaded_optimization_path'),('predictor_library','loaded_predictor_path')]:
            require(str(Path(startup[field]['path']).resolve())==actual_startup[loaded], 'Actually loaded native library differs')
        import audit_x650_streaming_consumption as streaming
        messages=streaming.original_messages(live)
        sends,receipts,ticks,events=(messages[k] for k in ('sends','receipts','dispatch','events'))
        calls,report['worker']=check_worker(live,start,startup['config'],events,resources,protocol)
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
        from audit_x650_cpp_normal_source import audit_source
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
        require(manifest['complete'] is True and manifest['sha256']==qualification.sha(run/'native_trace.csv'), 'Incomplete native physical trace')
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
        # The normal task is an exact trajectory-only composition of that
        # bound plant. Keep its predictor identity and audit the actual 32 s.
        report['actuator']=old.prior_audit.actuator_check(run,baseline_config(cfg['frozen_cpp_hold']),window)
        report['actuator'].pop('control_snapshot_to_plugin_consumption_lag',None)
        require(report['actuator']['new_joint_prediction_verified'], 'Independent whole-window joint prediction failed')
        endpoints=endpoint_windows(run,start,cfg,old.shared.native_tip_metrics)
        tilt=float(native['tilt'].max())
        require(np.isfinite(tilt) and tilt<=cfg['ts20_protocol']['tilt_max_deg'], 'Normal tilt envelope exceeded')
        report.update(**endpoints,tilt_peak_deg=tilt,window_sim_s=window,figure8_protocol=cfg['figure8_protocol'],
            frozen_cpp_hold=cfg['frozen_cpp_hold'],cpp_hold_certificate=cfg['cpp_hold_certificate'])
        require(all(identity(v['path'])==v for v in artifacts), 'Original evidence changed while auditing')
        report.update(passed=True,complete=True,certificate=True,physical_acceptance=True,
            full_window_ulog_verified=True,parameter_restore_verified=True,native_ground_verified=True,actuator_log_verified=True,
            scope='One complete original 32 s normal figure-eight simulation; fixed same-time/world target, no alignment; not hardware qualification or a worst-case timing guarantee')
    except Exception as error:
        report.update(error=repr(error),traceback=traceback.format_exc())
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
    print(json.dumps({k:result.get(k) for k in ('passed','error','endpoint','tracking_endpoint','tail_endpoint','timing')},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
