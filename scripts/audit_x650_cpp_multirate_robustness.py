#!/usr/bin/env python3
"""Audit new 24 s rigid-mismatch flights with the unchanged 25/50 Hz backend.

Both mass and inertia of all seven physical rigid links must be scaled by the
declared .85 or 1.15. The controller, fast reference and timing profile remain
unchanged. Execution integrity and original performance thresholds are reported
separately; a large endpoint error never suppresses the remaining exit checks.
"""
from pathlib import Path
import argparse
import json
import sys
import traceback

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'control_packages/x650_wholebody_v1')]
import audit_x650_cpp_multirate as base
from audit_x650_cpp_multirate import (profile,identity,require,read,rows,
    check_timing_selection,check_worker,check_runtime_recoveries,check_runtime_intervals,
    check_actual_send_barrier,check_ticks,check_recovery,check_physical_model,
    check_wrench,robustness_performance,upper_commands,check_handoff_bridge)
from simulations.x650_cpp_multirate_robustness_v1 import qualification,task
from audit_x650_cpp_multirate_followup import observation_value_view,endpoint_observations

FROZEN_BASE_SHA256='d2a83b5e40abb6013784233000f258bd8d750131ca5eca06b1c01c5040ff5a2d'
FROZEN_FOLLOWUP_SHA256='69b4ed49238c4517f06607e83587848fa4def2b395bd971fc4dd47b41c51849a'
CASE_SCALES={'mass085_fast':.85,'mass115_fast':1.15}


def check_case(cfg,protocol):
    """Require the predeclared physical change, not a changed prediction model."""
    case=cfg['multirate']['case_id'];robust=cfg['robustness']
    require(case in CASE_SCALES and robust['case_id']==case
        and robust['kind']=='uniform_rigid_mismatch'
        and robust['mass_inertia_scale']==CASE_SCALES[case]
        and robust['schedule']==[] and robust['wrench_runtime'] is None
        and cfg['disturbance_enabled'] is False and protocol['delay_tests_enabled'] is False
        and cfg['duration_s']==protocol['duration_s']==24.,
        'Only a complete declared .85/1.15 rigid mismatch without wrench/delay is admitted')
    limits=cfg['ts20_protocol']
    require(limits['tip_rms_max_m']==.05 and limits['tip_peak_max_m']==.1
        and limits['tilt_max_deg']==8., 'Original endpoint/tilt acceptance thresholds changed')
    require(cfg['timing_profile']==profile.NAME
        and protocol['effective_timing_profile']==profile.effective(),
        'Rigid-mismatch case changed the accepted timing profile')
    return case


def classify_performance(endpoint_acceptance,tilt,limits):
    """Classify after recording values; no endpoint/tilt performance exception."""
    require(np.isfinite(tilt) and tilt>=0., 'Nonfinite actual physical tilt')
    tilt_passed=bool(tilt<=limits['tilt_max_deg'])
    passed=bool(endpoint_acceptance['passed'] and tilt_passed)
    return dict(passed=passed,endpoint_passed=bool(endpoint_acceptance['passed']),
        tilt_passed=tilt_passed,tilt_peak_deg=float(tilt),tilt_limit_deg=limits['tilt_max_deg'],
        thresholds_unchanged=True,metrics_recorded_before_performance_classification=True,
        recovery_applicable=False,
        performance_rejection=None if passed else 'Original endpoint or tilt performance threshold failed')


def check_archives(launch,run,meta):
    record=read(launch/'launch.json')
    require(all(record.get(k) is True for k in ('complete','runtime_closed','sources_unchanged',
        'runtime_binaries_unchanged','launch_inputs_unchanged')) and record.get('exit_code')==0,
        'Incomplete or changing new rigid-mismatch launch')
    sources,binaries=qualification.archive_integrity(launch,record)
    cfg,protocol,startup=(read(launch/name) for name in ('task.json','protocol.json','startup_manifest.json'))
    require(task.validate_task_config(cfg)==cfg==record['task']==meta['task']==read(run/'controller_task.json')
        and record.get('execution_scenario')==cfg['scenario']
        and record.get('run_directories')==[str(run.resolve())] and protocol==record['protocol'],
        'Actual new rigid-mismatch task/reference/window differs')
    qualification.validate_protocol(protocol,cfg)
    check_case(cfg,protocol)
    for item in record['launch_inputs']:
        require(identity(item['path'])==item, 'Launch input changed')
    original=read(launch/'original_startup_manifest.json')
    require({k:v for k,v in startup.items() if k!='task_config'}
        =={k:v for k,v in original.items() if k!='task_config'}
        and startup['task_config']['sha256']==original['task_config']['sha256']==qualification.sha(launch/'task.json'),
        'Startup derivation changed something besides the identical task path')
    manifest=read(launch/'qualification_manifest.json')
    proof=qualification.verify(launch/'qualification_manifest.json',task_path=Path(manifest['task_config']['path']),
        startup_path=Path(manifest['startup_manifest']['path']),startup_sha256=manifest['startup_manifest']['sha256'],
        protocol_path=Path(manifest['protocol']['path']),binaries=binaries)
    require(proof.get('kind')=='x650_cpp_multirate_robustness_admission_v1' and proof.get('ready') is True
        and proof.get('physical_acceptance') is False, 'New ground admission cannot replace actual physical acceptance')
    for key,name in [('task_config','task.json'),('protocol','protocol.json'),('startup_manifest','original_startup_manifest.json')]:
        require(manifest[key]['sha256']==qualification.sha(launch/name), 'Actual archived admission input differs')
    required=qualification.EXECUTION_SOURCES|{
        'simulations/x650_cpp_multirate_robustness_v1/task.py',
        'simulations/x650_cpp_multirate_robustness_v1/qualification.py',
        'simulations/x650_cpp_multirate_robustness_v1/run_case.py',
        'simulations/x650_cpp_multirate_robustness_v1/launch.py',
        'scripts/audit_x650_cpp_multirate_robustness.py',
        'scripts/audit_x650_cpp_multirate_followup.py','scripts/audit_x650_cpp_multirate.py'}
    require(required<=set(sources), 'New archive omits actual rigid-mismatch/timing/native implementation')
    return cfg,protocol,startup,record,proof


def audit(launch, run, out):
    report=dict(kind='x650_cpp_multirate_robustness_report_v1',timing_profile=profile.NAME,effective_profile=profile.effective(),passed=False,complete=False,
        certificate=False,physical_acceptance=False,hardware_qualified=False,launch_directory=str(launch),run_directory=str(run),
        integrity_and_execution_passed=False,recovery_performance_passed=None,recovery_applicable=False,
        auditor_version='multirate_rigid_mismatch_v1',auditor_identity=identity(Path(__file__)))
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
        paths.update(ROOT/name for name in ('scripts/audit_x650_cpp_robustness.py',
            'scripts/prepare_x650_robustness_models.py','scripts/verify_x650_robustness_model.py'))
        paths.add(ROOT/'scripts/audit_x650_cpp_multirate_followup.py')
        paths.update((ROOT/'simulations/x650_cpp_multirate_v1').glob('*.py'))
        paths.update((ROOT/'simulations/x650_cpp_multirate_robustness_v1').glob('*.py'))
        paths.add(Path(__file__).resolve())
        artifacts=[identity(p) for p in sorted(paths)]
        require(qualification.sha(Path(base.__file__))==FROZEN_BASE_SHA256
            and qualification.sha(ROOT/'scripts/audit_x650_cpp_multirate_followup.py')==FROZEN_FOLLOWUP_SHA256,
            'Frozen hold or observation/performance helper changed')
        meta=read(run/'controller_metadata.json')
        require(all(meta.get(k) is True for k in ('complete','landed_and_disarmed','diagnostic_parameters_restored'))
            and meta.get('diagnostic_restore_pending') is False, 'Incomplete flight lifecycle: '+str(meta.get('error')))
        stage=read(run/'stage_result.json')
        require(all(stage.get(k) is True for k in ('complete','passed','success','landed_and_disarmed'))
            and read(run/'result.json').get('success') is True, 'Original provisional stage/run failed')
        cfg,protocol,startup,launch_record,proof=check_archives(launch,run,meta)
        report['software_qualification']=proof
        case=check_case(cfg,protocol)
        duration=cfg['duration_s']
        report.update(case_id=case,physical_scale=CASE_SCALES[case],delay_injected=False,
            external_wrench_injected=False,reference_unchanged=True)
        report.update(task_config=identity(launch/'task.json'),startup_manifest=identity(launch/'startup_manifest.json'),runtime_binaries_sha256=launch_record['runtime_binaries_sha256'])
        report['physical_model']=check_physical_model(cfg,meta,stage,proof)
        live=run/'isolated_live'; lifecycle=read(live/'lifecycle.json'); start=read(live/'live_start.json')
        require(lifecycle['started'] is True and lifecycle['error'] is None
            and all(p['returncode']==0 for p in lifecycle['processes'].values()), 'Native child lifecycle failed')
        require(start['history_count']>0 and abs(meta['task_end_sim_s']-start['origin_s']-duration)<=.05,
                'Incomplete full 24 s rigid-mismatch task')
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
        from audit_x650_cpp_fast_source import audit_source
        reference=read(run/'hover_reference.json')
        report['feedback']=audit_source(live/'cpp_source',run/'ts20_state.csv',run/'ts20_packets.csv',reference,
            read(run/'estimated_feedback.json')['extra_reset_counters_at_bind'],worker_directory=live/'native_records',
            expected_binding=dict(configuration_sha256=actual_startup['configuration_sha256'],
                position_gains=startup['config']['base_position_gain'],bus_period_s=startup['config']['bus_period'],delay_s=startup['config']['motion'][4]),trajectory=startup['trajectory'])
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
        limits=cfg['ts20_protocol'];tilt=float(native['tilt'].max())
        report['tilt_peak_deg']=tilt
        report['performance']=classify_performance(endpoint_acceptance,tilt,limits)
        report['external_wrench']=check_wrench(run,start,cfg,meta,stage)
        require(report['external_wrench']==dict(verified=True,enabled=False),
            'Rigid-mismatch-only case unexpectedly includes an external wrench')
        report['robustness_performance']=robustness_performance(run,start,cfg)
        require(report['robustness_performance']['performance_passed'] is None
            and report['robustness_performance']['delay_injected'] is False,
            'No pulse-recovery qualification applies to a rigid-mismatch-only case')
        report.update(tilt_peak_deg=tilt,window_sim_s=window)
        require(all(identity(v['path'])==v for v in artifacts), 'Original evidence changed while auditing')
        overall=report['performance']['passed']
        report.update(passed=overall,complete=True,certificate=overall,physical_acceptance=overall,
            full_window_ulog_verified=True,parameter_restore_verified=True,native_ground_verified=True,
            actuator_log_verified=True,integrity_and_execution_passed=True,overall_robustness_test_passed=overall,
            performance_rejection=report['performance']['performance_rejection'],
            scope='One fresh complete 24 s rigid-mismatch simulation; unchanged numerical backend/PID/reference/limits; execution separated from performance; observational PVA clock view is not publication timing or hardware qualification')
    except Exception as error:
        report.update(error=repr(error),traceback=traceback.format_exc())
        # Partial/rejected runs retain available raw observations, never a
        # certificate or an invented complete-window performance result.
        try:
            from audit_x650_interval_robustness import failure_context
            report['failure_context']=failure_context(run)
        except Exception as detail:
            report['failure_context']=dict(read_error=repr(detail))
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
    require(qualification.sha(ROOT/'scripts/audit_x650_cpp_multirate_followup.py')==FROZEN_FOLLOWUP_SHA256,
        'Frozen observation/performance helper changed')
    launch,run,out=args.launch_dir.resolve(),args.run_dir.resolve(),args.output.resolve()
    require(not out.exists() and not out.is_relative_to(run) and not out.is_relative_to(launch),
        'New rigid-mismatch audit output must be outside original evidence')
    out.mkdir(parents=True)
    result=audit(launch,run,out)
    print(json.dumps({k:result.get(k) for k in ('passed','complete','integrity_and_execution_passed',
        'case_id','physical_model','endpoint_performance','performance','recovery_performance_passed',
        'overall_robustness_test_passed','error')},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
