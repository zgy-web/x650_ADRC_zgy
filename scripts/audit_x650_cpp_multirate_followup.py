#!/usr/bin/env python3
"""Follow-up value-clock audit; the accepted hold auditor and run stay frozen.

The chosen timestamp is the newest of two original pre-send observations in
the same simulation clock. It is not a reconstructed publication timestamp or
an identified transport delay. Actual sends remain independently checked using
their original monotonic clocks, CDRs, absolute slots and consumption records.
"""
from copy import deepcopy
from pathlib import Path
import argparse
import json
import sys
import traceback

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'control_packages/x650_wholebody_v1')]
import audit_x650_cpp_multirate as base
from audit_x650_cpp_multirate import (qualification,task,profile,identity,require,read,rows,
    check_archives,check_timing_selection,check_worker,check_runtime_recoveries,check_runtime_intervals,
    check_actual_send_barrier,check_ticks,check_recovery,ulog_value_view,check_physical_model,
    check_wrench,robustness_performance,upper_commands,check_handoff_bridge)

FROZEN_BASE_SHA256='d2a83b5e40abb6013784233000f258bd8d750131ca5eca06b1c01c5040ff5a2d'


def observation_value_view(control, derived, sends, ticks, source_rows, start):
    """Make a transparent value-only view from already observed raw clocks.

    Either input timestamp may repeat while the other advances. Taking their
    maximum selects an existing observation; it never adds a fitted offset or
    invents a sub-cycle publication time. Unsupported non-increasing selections
    are rejected rather than moved or silently folded.
    """
    normal=[s for s in sends if not s['stop_zero']]
    require(len(normal)==len(derived) and len({id(r) for r in derived})==len(derived)
        and all(sum(r is d for r in control)==1 for d in derived),
        'Observation view lost original actual-send/derived identities')
    by_tick={t['sequence']:t for t in ticks if t['output_valid']}
    sources={int(r['sequence']):r for r in source_rows}
    require(len(sources)==len(source_rows) and len(by_tick)==len(normal),
        'Observation view has repeated/missing raw source or tick identities')
    replacements={};new_derived=[];evidence=[];last=None
    for send,row in zip(normal,derived):
        require(send['dispatch_sequence'] in by_tick and float(row['sim_s'])==float(send['sim_s']),
            'Original writer observation or actual dispatch identity differs')
        tick=by_tick[send['dispatch_sequence']]
        require(tick['command_sequence']==send['command_sequence'] and tick['source_sequence'] in sources,
            'Actual value view references another candidate or unknown source')
        source=sources[tick['source_sequence']]
        issued=int(source['issued_ns']);received=int(tick['source_received_monotonic_ns'])
        actual=int(tick['actual_monotonic_ns']);sent=int(send['sent_monotonic_ns'])
        require(0<issued<=received<=actual<=sent and actual-issued<=25_000_000,
            'Source clock observation was not acquired within its original pre-send freshness bound')
        writer=float(send['sim_s']);observed=float(source['control_s'])
        require(np.isfinite([writer,observed]).all(), 'Nonfinite original observation clock')
        selected=max(writer,observed)
        require(last is None or selected>last,
            'Original observations cannot order the PVA value view; no fabricated time is permitted')
        last=selected
        value=dict(row,sim_s=selected,t_s=selected-float(start['origin_s']))
        require({k:v for k,v in row.items() if k not in {'sim_s','t_s'}}
            =={k:v for k,v in value.items() if k not in {'sim_s','t_s'}}, 'PVA value view altered payload/feedback')
        replacements[id(row)]=value;new_derived.append(value)
        evidence.append(dict(dispatch_sequence=send['dispatch_sequence'],command_sequence=send['command_sequence'],
            source_sequence=tick['source_sequence'],writer_observation_sim_s=writer,
            source_control_observation_sim_s=observed,value_match_observation_sim_s=selected,
            selected_observation='source_control' if observed>=writer else 'writer_actuator_state',
            source_issued_monotonic_ns=issued,source_received_monotonic_ns=received,
            tick_actual_monotonic_ns=actual,sent_monotonic_ns=sent))
    result=sorted([replacements.get(id(row),row) for row in control],key=lambda row:float(row['sim_s']))
    proof=dict(actual_normal_send_count=len(normal),distinct_pva_value_row_count=len(new_derived),
        original_send_records_preserved=True,original_derived_rows_preserved=True,
        original_physical_timestamps_changed=False,view_observation_clock_selected=True,
        actual_publication_timestamp_reconstructed=False,fitted_time_offset=False,observation_bindings=evidence,
        scope='Value-match observation lower-bound view: maximum of original raw source control observation and original writer actuator-state observation, both already held before the original wall send; not actual publication time or measured transport latency')
    return result,proof,new_derived


def endpoint_observations(run,start,cfg,metric):
    """Retain every original metric before classifying performance failure."""
    target=qualification.robust.figure8(start['target_position_enu'])
    limits=cfg['ts20_protocol'];result={};acceptance={}
    for name,interval in [('endpoint',(0.,24.)),('tracking_endpoint',(2.,20.)),('tail_endpoint',(20.,24.))]:
        value=metric(run,target.sample,start['origin_s'],interval)
        require(value.get('same_time') is True and value.get('no_phase_alignment') is True
            and tuple(value.get('interval_task_s',()))==interval and np.isfinite(value['duration_s'])
            and value['duration_s']>=interval[1]-interval[0]-.04,
            'Original full same-time endpoint window is incomplete: '+name)
        require(all(np.isfinite(value[k]) and value[k]>=0 for k in ('position_rmse_m','position_max_m')),
            'Nonfinite original endpoint observation: '+name)
        result[name]=value
        acceptance[name]=dict(rms_passed=value['position_rmse_m']<=limits['tip_rms_max_m'],
            peak_passed=value['position_max_m']<=limits['tip_peak_max_m'],
            rms_limit_m=limits['tip_rms_max_m'],peak_limit_m=limits['tip_peak_max_m'])
        acceptance[name]['passed']=acceptance[name]['rms_passed'] and acceptance[name]['peak_passed']
    return result,dict(passed=all(v['passed'] for v in acceptance.values()),windows=acceptance,
        metrics_recorded_before_performance_classification=True,thresholds_unchanged=True)



def audit(launch, run, out):
    report=dict(kind='x650_cpp_multirate_wrench_followup_report_v1',timing_profile=profile.NAME,effective_profile=profile.effective(),passed=False,complete=False,
        certificate=False,physical_acceptance=False,hardware_qualified=False,launch_directory=str(launch),run_directory=str(run),
        auditor_version='multirate_followup_v2',auditor_identity=identity(Path(__file__)))
    artifacts=[]
    try:
        # Even a failed audit retains the hashes of its exact original evidence.
        paths={p for base in (launch,run) for p in base.rglob('*') if p.is_file()}
        paths.update(ROOT/name for name in ('scripts/audit_x650_cpp_hold.py','scripts/report_x650_cpp_live.py',
            'scripts/audit_x650_cpp_source.py','scripts/audit_x650_streaming_consumption.py',
            'scripts/audit_x650_isolated_live.py','scripts/audit_x650_precision_stage.py','scripts/audit_x650_upper_stage.py',
            'scripts/analyze_x650_hover_precision.py','simulations/x650_cpp_live_v1/qualification.py',
            'simulations/x650_ts20_hold_v1/audit.py','simulations/x650_ts20_hold_v1/task.py'))
        paths.update(ROOT/name for name in ('scripts/audit_x650_cpp_multirate.py','scripts/audit_x650_interval_robustness.py','scripts/audit_x650_recovery_robustness.py','scripts/audit_x650_recovery_replay.py','scripts/audit_x650_interval_constraints.py'))
        paths.update((ROOT/'simulations/x650_cpp_multirate_v1').glob('*.py'))
        paths.add(Path(__file__).resolve())
        artifacts=[identity(p) for p in sorted(paths)]
        meta=read(run/'controller_metadata.json')
        require(all(meta.get(k) is True for k in ('complete','landed_and_disarmed','diagnostic_parameters_restored'))
            and meta.get('diagnostic_restore_pending') is False, 'Incomplete flight lifecycle: '+str(meta.get('error')))
        stage=read(run/'stage_result.json')
        require(all(stage.get(k) is True for k in ('complete','passed','success','landed_and_disarmed'))
            and read(run/'result.json').get('success') is True, 'Original provisional stage/run failed')
        cfg,protocol,startup,launch_record,proof=check_archives(launch,run,meta)
        report['software_qualification']=proof
        wrench=cfg['multirate']['case_id']=='wrench_fast'
        duration=cfg['duration_s']
        require(wrench and duration==24., 'Follow-up auditor is limited to the original complete 24 s wrench case')
        report['kind']='x650_cpp_multirate_wrench_followup_report_v1'
        report.update(task_config=identity(launch/'task.json'),startup_manifest=identity(launch/'startup_manifest.json'),runtime_binaries_sha256=launch_record['runtime_binaries_sha256'])
        if wrench:report['physical_model']=check_physical_model(cfg,meta,stage,proof)
        live=run/'isolated_live'; lifecycle=read(live/'lifecycle.json'); start=read(live/'live_start.json')
        require(lifecycle['started'] is True and lifecycle['error'] is None
            and all(p['returncode']==0 for p in lifecycle['processes'].values()), 'Native child lifecycle failed')
        require(start['history_count']>0 and abs(meta['task_end_sim_s']-start['origin_s']-duration)<=.05,
                'Incomplete full 30 s task')
        window=[start['origin_s'],start['origin_s']+duration]
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
        report['timing_selection']=check_timing_selection(live,actual_startup)
        require(actual_startup.get('core_abi')==4 and actual_startup['startup_manifest']==startup and actual_startup['python_in_worker'] is False
            and actual_startup['startup_controller_steps']==0 and actual_startup['diagnostic_mode']=='compact'
            and actual_startup['session_id']==start['session_id'] and actual_startup['epoch']==start['epoch']
            and actual_startup['configuration_sha256']==qualification.sha(launch/'task.json'), 'Actual worker startup differs')
        for field,loaded in [('core_library','loaded_core_path'),('optimization_library','loaded_optimization_path'),('predictor_library','loaded_predictor_path')]:
            require(str(Path(startup[field]['path']).resolve())==actual_startup[loaded], 'Actually loaded native library differs')
        import audit_x650_streaming_consumption as streaming
        messages=streaming.original_messages(live)
        sends,receipts,ticks,events=(messages[k] for k in ('sends','receipts','dispatch','events'))
        calls,report['worker']=check_worker(live,start,startup['config'],events,resources,protocol)
        report['optimizer_recovery']=check_runtime_recoveries(calls,start,startup['config'])
        report['optimizer_interval']=check_runtime_intervals(calls,start,startup['config'])
        report['first_command_hold_guard']=dict(verified=True,calls_checked=len(calls),duration_s=.16)
        report['actual_send_barrier']=check_actual_send_barrier(read(live/'ingress_timing.json'),sends,calls)
        source_rows=rows(live/'cpp_source/source_selections.csv')
        source_issues={int(v['sequence']):int(v['issued_ns']) for v in source_rows}
        source_controls={int(v['sequence']):float(v['control_s']) for v in source_rows}
        require(len(source_issues)==len(source_rows), 'Repeated source issue identity')
        report['timing']=check_ticks(ticks,sends,calls,start,source_issues=source_issues,source_controls=source_controls)
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
        if wrench:
            from audit_x650_cpp_fast_source import audit_source
        else:
            from audit_x650_cpp_source import audit_source
        reference=read(run/'hover_reference.json')
        report['feedback']=audit_source(live/'cpp_source',run/'ts20_state.csv',run/'ts20_packets.csv',reference,
            read(run/'estimated_feedback.json')['extra_reset_counters_at_bind'],worker_directory=live/'native_records',
            expected_binding=dict(configuration_sha256=actual_startup['configuration_sha256'],
                position_gains=startup['config']['base_position_gain'],bus_period_s=startup['config']['bus_period'],delay_s=startup['config']['motion'][4]),**({'trajectory':startup['trajectory']} if wrench else {}))
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
        endpoint_values,endpoint_acceptance=endpoint_observations(run,start,cfg,old.shared.native_tip_metrics)
        report.update(**endpoint_values,endpoint_performance=endpoint_acceptance)
        from pyulog import ULog
        ulog=ULog(str(run/'px4.ulg'))
        report['recovery']=check_recovery(old.precision.dataset(ulog,'vehicle_status'),
            old.precision.dataset(ulog,'vehicle_land_detected'),rows(run/'flight_samples.csv'),window[1])
        report['configuration']=old.fixed.effective_configuration(ulog,native,reference,window,old.fixed.records(docs['applied']))
        control,derived=old.command_view(run,sends,calls,start)
        (out/'derived_command_audit_view.json').write_text(json.dumps(dict(scope='Actual send CDR matched to original worker/source; originals untouched',rows=derived),allow_nan=False))
        value_control,value_proof,value_derived=observation_value_view(control,derived,sends,ticks,source_rows,start)
        (out/'distinct_pva_value_audit_view.json').write_text(json.dumps(dict(**value_proof,rows=value_control),allow_nan=False))
        report['px4_value_view']=value_proof
        report['handoff']=check_handoff_bridge(value_control,value_derived,start,ulog)
        report['handoff'].update(full_physical_window_s=window[1]-window[0],
            cpp_writer_observation_span_to_window_end_s=window[1]-report['handoff']['first_cpp_writer_observation_sim_s'],
            cpp_received_value_span_to_window_end_s=window[1]-report['handoff']['first_cpp_value_received_s'])
        report['px4_commands']=upper_commands(ulog,native,reference,value_control,window,pid,response=False,handoff=report['handoff'])
        report['px4_commands']['all_distinct_pva_value_rows_observed']=report['px4_commands'].pop('all_measurement_sends_observed')
        report['px4_commands']['distinct_pva_measurement_value_rows']=report['px4_commands'].pop('measurement_sends')
        report['px4_commands']['observation_scope']=value_proof['scope']
        report['px4_commands']['time_alignment']='Original pre-send known observation values only; no physical publication timestamp or fitted time shift'
        report['px4_commands']['matched_observation_receipt_range_s']=report['px4_commands'].pop('matched_delivery_range_s')
        report['px4_commands']['internal_minus_observation_time_logical_velocity']=report['px4_commands'].pop('internal_minus_original_time_logical_velocity')
        report['px4_commands']['causal_value_matching_allowance_s']=report['px4_commands'].pop('causal_delivery_allowance_s')
        report['handoff']['first_cpp_value_match_observation_sim_s']=report['handoff'].pop('first_cpp_writer_observation_sim_s')
        report['handoff']['original_first_cpp_writer_observation_sim_s']=derived[0]['sim_s']
        report['handoff']['observation_scope']=value_proof['scope']
        report['flight']=old.upper.flight_states(run/'flight_samples.csv',window)
        report['ground']=old.fixed.native_ground_check(run,read(run/'native_ground.json'))
        report['actuator']=old.prior_audit.actuator_check(run,task.baseline_config(cfg),window)
        report['actuator'].pop('control_snapshot_to_plugin_consumption_lag',None)
        require(report['actuator']['new_joint_prediction_verified'], 'Independent whole-window joint prediction failed')
        target=(qualification.robust.figure8(start['target_position_enu']) if wrench else old.shared.ConstantTarget(np.asarray(start['target_position_enu'])))
        endpoint=report['endpoint']
        limits=cfg['ts20_protocol'];tilt=float(native['tilt'].max())
        report['tilt_peak_deg']=tilt
        require(tilt<=limits['tilt_max_deg'], 'Original physical tilt envelope exceeded')
        if wrench:
            # Endpoint values/unchanged pass flags were retained before the later execution checks.
            report['external_wrench']=check_wrench(run,start,cfg,meta,stage)
            report['robustness_performance']=robustness_performance(run,start,cfg)
            report['recovery_performance_passed']=report['robustness_performance']['performance_passed']
        else:
            report['endpoint']=endpoint
        report.update(tilt_peak_deg=tilt,window_sim_s=window)
        require(all(identity(v['path'])==v for v in artifacts), 'Original evidence changed while auditing')
        overall=report['endpoint_performance']['passed'] and report['recovery_performance_passed'] is True
        report.update(passed=overall,complete=True,certificate=overall,physical_acceptance=overall,
            full_window_ulog_verified=True,parameter_restore_verified=True,native_ground_verified=True,
            actuator_log_verified=True,integrity_and_execution_passed=True,overall_disturbance_test_passed=overall,
            performance_rejection=None if overall else 'Original endpoint or recovery performance threshold failed',
            scope='Complete original 24 s simulation wrench execution audited independently from performance; observational PVA clock view is not publication timing or hardware qualification')
    except Exception as error:
        report.update(error=repr(error),traceback=traceback.format_exc())
    report['artifact_identities']=artifacts
    report['artifacts']={a['path']:a['sha256'] for a in artifacts}
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return report

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('launch-dir','run-dir','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    require(qualification.sha(Path(base.__file__))==FROZEN_BASE_SHA256,
        'Frozen v3 hold/flight auditor changed')
    launch,run,out=args.launch_dir.resolve(),args.run_dir.resolve(),args.output.resolve()
    require(not out.exists() and not out.is_relative_to(run) and not out.is_relative_to(launch),
        'New follow-up audit output must be outside original evidence')
    out.mkdir(parents=True)
    result=audit(launch,run,out)
    print(json.dumps({k:result.get(k) for k in ('passed','complete','integrity_and_execution_passed',
        'endpoint_performance','recovery_performance_passed','overall_disturbance_test_passed','error')},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
