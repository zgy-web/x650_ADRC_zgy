#!/usr/bin/env python3
"""Independent normal-trajectory acceptance; reuse actual transport/raw-state audits.

A normal certificate covers all 32 seconds, including approach and return.
The trajectory-only entry is explicit; it never authorizes fast flight.
"""
from audit_x650_isolated_live import *
from simulations.x650_isolated_figure8_v1 import task as normal_task


def check_sources(launch,folder,meta):
    d=read(launch/'launch.json');before=read(launch/'source_manifest.json')
    require(d['complete'] and d['runtime_closed'] and d['sources_unchanged']
            and d['runtime_binaries_unchanged'],'Incomplete launch or changing runtime')
    require(before==read(launch/'source_manifest_after.json'),'Source manifest changed')
    require(str(folder.resolve()) in d['run_directories'],'Run belongs to another launch')
    import hashlib
    with zipfile.ZipFile(launch/'sources.zip') as z:
        require(set(z.namelist())==set(before),'Source archive membership differs')
        for rel,h in before.items():require(hashlib.sha256(z.read(rel)).hexdigest()==h,'Archived source changed: '+rel)
    cfg=read(folder/'controller_task.json')
    require(cfg==d['task']==read(launch/'task.json')==meta['task'],'Task changed between processes')
    protocol=read(ROOT/'configs/experiments/x650_isolated_live_v1.json')
    cfg=normal_task.validate_task_config(cfg)
    normal_task.validate_live_hold(cfg)
    require(d['protocol']==protocol and sha(launch/'task.json')==d['task_sha256']
            and cfg['frozen_hold']['sha256']==protocol['frozen_hold_task_sha256'],
            'Frozen hold task or declared execution protocol changed')
    require(cfg['numerical_backend']['library_sha256']==protocol['frozen_numerical_library_sha256'],
            'Numerical library changed')
    resources=read(folder/'isolated_live/execution_resources.json')
    require(resources==d['execution_resources'] and resources['roles']==protocol['cpu_roles'],
            'Executed resource plan differs from declaration')
    observed=read(folder/'isolated_live/execution_affinity_at_handoff.json')
    require(set(observed)=={'source','worker','dispatcher','ingress','consumption','recorder','source_recorder'},
            'Owned resource evidence incomplete')
    for role,threads in observed.items():
        expected=resources['roles'].get(role,resources['background'])
        require(threads and all(cpus==expected for cpus in threads.values()),'CPU role differs: '+role)
    return {'verified':True,'source_files':len(before),'task_sha256':sha(launch/'task.json'),
            'native_library_sha256':protocol['frozen_numerical_library_sha256'],'new_execution_sources_archived':True}



def audit(launch,folder,out):
    report={'kind':'x650_isolated_normal_stage_certificate_v1','passed':False,'complete':False,
            'launch_directory':str(launch),'source_run_directory':str(folder),'hardware_qualified':False}
    try:
        meta=read(folder/'controller_metadata.json');cfg=read(folder/'controller_task.json')
        require(all(meta.get(k) is True for k in ('complete','landed_and_disarmed','diagnostic_parameters_restored'))
                and meta.get('diagnostic_restore_pending') is False,'Incomplete flight lifecycle: '+str(meta.get('error')))
        report['source_binding']=check_sources(launch,folder,meta)
        live=folder/'isolated_live';lifecycle=read(live/'lifecycle.json')
        require(lifecycle['started'] and lifecycle['error'] is None,'Execution did not start or failed')
        start=read(live/'live_start.json');window=[start['origin_s'],start['origin_s']+cfg['duration_s']]
        report['startup_gc']=check_preflight_gc(live,start)
        report['ground_initialization']=check_preflight_transport(live,meta,start)
        require(abs(meta['task_end_sim_s']-window[1])<=.05,'Incomplete declared normal figure eight')
        report.update(stage='normal',window_sim_s=window,figure8_protocol=cfg['figure8_protocol'],
                      frozen_hold=cfg['frozen_hold'],isolated_live_hold=cfg['isolated_live_hold'])
        report['recording']=reconcile_recording(live);require(report['recording']['complete'],'Diagnostic loss')
        from simulations.x650_isolated_live_v1.source_recording import materialize
        from flyarm_control_interfaces import msg
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        # Rebuild independently in memory and compare all derived views; never
        # repair or replace a recorded run while certifying it.
        def compare_view(path,value):
            require(read(path)==value,'Source-derived view differs from raw recorder: '+path.name)
        report['source_recording']=materialize(live,{'dispatch':msg.DispatchTick,'sends':msg.ActuatorSend,
            'receipts':msg.ExecutionReceipt,'events':msg.GateEvent},deserialize_message,message_to_ordereddict,compare_view)
        calls=load_calls(live);sends=read(live/'sends.json');receipts=read(live/'receipts.json');ticks=read(live/'dispatch.json')
        events=read(live/'events.json')
        from x650_worker_phase_audit import verify_worker_phases
        report['worker_phases']=verify_worker_phases(live,calls,events)
        require([t['sequence'] for t in ticks]==list(range(1,len(ticks)+1)),
                'DDS observer missed or reordered a dispatch tick')
        require([e['sequence'] for e in events]==list(range(1,len(events)+1)),
                'DDS observer missed or reordered a candidate gate event')
        require(len(events)<=len(calls)<=len(events)+1,'Candidate recording does not reconcile with gate events')
        require(not any(t['status']==3 for t in ticks) and all(e['accepted'] for e in events),
                'Candidate/output gate fault')
        report['consumption']=check_consumption(folder,sends,receipts,start,lifecycle['release'])
        valid=[t for t in ticks if t['output_valid']]
        require([t['sequence'] for t in valid]==[s['dispatch_sequence'] for s in sends if not s['stop_zero']],
                'Actual sends do not reconcile with all valid output ticks')
        intervals=np.diff([t['actual_monotonic_ns'] for t in valid])/1e6
        ages=[t['command_age_ns']/1e6 for t in valid]
        require(len(valid)>1 and float(intervals.max())<=40. and max(ages)<=50.,'Output period/lease exceeded')
        report['timing']={'output_interval_ms':stats(intervals.tolist()),'valid_command_age_ms':stats(ages)}
        docs={k:read(folder/n) for k,n in {'session':'diagnostic_parameter_session.json','journal':'diagnostic_restore_journal_completed.json',
            'before':'diagnostic_parameters_before.json','applied':'diagnostic_parameters_applied.json','restored':'diagnostic_parameters_restored.json',
            'px4_before':'px4_parameters_before.json','px4_after':'px4_parameters_after.json'}.items()}
        _,pid,_=frozen_research_pid(cfg)
        report['parameters']=fixed.verify_parameter_session(docs,meta,condition_for_task(cfg),pid)
        reference=read(folder/'hover_reference.json');precision.verify_reference(reference,precision.DEFAULT_REFERENCE_SOURCE)
        manifest=read(folder/'native_trace_manifest.json')
        require(manifest['complete'] and manifest['sha256']==sha(folder/'native_trace.csv'),'Invalid native recording')
        precision.capture_window_covers(manifest,window)
        native=precision.native_states(folder/'native_trace.csv',window)
        from pyulog import ULog
        ulog=ULog(str(folder/'px4.ulg'))
        report['configuration']=fixed.effective_configuration(ulog,native,reference,window,fixed.records(docs['applied']))
        control,derived=command_view(folder,sends,calls,start)
        shared.atomic_json(out/'derived_command_audit_view.json',{'scope':'derived from actual send CDR and matched source; original records untouched','rows':derived})
        report['px4_commands']=upper.upper_commands(ulog,native,reference,control,window,pid,response=False)
        report['feedback']=check_feedback(folder,calls,reference)
        report['flight']=upper.flight_states(folder/'flight_samples.csv',window)
        report['ground']=fixed.native_ground_check(folder,read(folder/'native_ground.json'))
        actuator=prior_audit.actuator_check(folder,cfg,window)
        actuator.pop('control_snapshot_to_plugin_consumption_lag',None)
        require(actuator['new_joint_prediction_verified'],'Independent joint prediction failed')
        report['actuator']=actuator
        trajectory=shared.FigureEight(np.asarray(start['target_position_enu']),**cfg['trajectory_parameters'])
        endpoint=shared.native_tip_metrics(folder,trajectory.sample,window[0],(0.,cfg['duration_s']))
        tracking=shared.native_tip_metrics(folder,trajectory.sample,window[0],(trajectory.start_s,trajectory.end_s))
        limits=cfg['ts20_protocol']
        require(endpoint['position_rmse_m']<=limits['tip_rms_max_m'] and endpoint['position_max_m']<=limits['tip_peak_max_m']
                and tracking['position_rmse_m']<=limits['tip_rms_max_m'] and tracking['position_max_m']<=limits['tip_peak_max_m']
                and float(native['tilt'].max())<=limits['tilt_max_deg'],'Physical normal trajectory envelope failed')
        report['endpoint']=endpoint;report['tracking_endpoint']=tracking;report['tilt_peak_deg']=float(native['tilt'].max())
        report['replay']=replay(live,launch/'task.json',calls,start)
        report['qualification']=cfg['ts20_protocol']['qualification']
        report['artifacts']={str(p):sha(p) for p in [*folder.rglob('*'),*launch.glob('*'),Path(__file__).resolve()]
                            if p.is_file() and p.suffix in ('.json','.jsonl','.csv','.ulg','.zip','.py')}
        report.update(passed=True,complete=True,full_window_ulog_verified=True,parameter_restore_verified=True,
                      native_ground_verified=True,actuator_log_verified=True)
    except BaseException as e:report.update(error=repr(e),traceback=traceback.format_exc())
    shared.atomic_json(out/'report.json',report)
    return report


def main():
    p=argparse.ArgumentParser()
    for n in ('launch-dir','run-dir','output'):p.add_argument('--'+n,type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    report=audit(a.launch_dir.resolve(),a.run_dir.resolve(),a.output.resolve())
    print(json.dumps({k:report.get(k) for k in ('passed','error','endpoint','timing','replay')},indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
