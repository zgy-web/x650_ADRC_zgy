#!/usr/bin/env python3
"""Independent actual-plant audit. Partial runs never grant a stage certificate.

Derived command rows are an explicitly labelled audit view, not replacement
flight records. Raw CDR, actuator counters, PX4 ULog and native physics remain
the evidence. No time fitting or controller retuning is performed here.
"""
import argparse
import base64
import csv
import json
from pathlib import Path
import sys
import time
import traceback
from types import SimpleNamespace
import zipfile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from simulations.x650_ts20_hold_v1 import task
from simulations.x650_ts20_hold_v1 import audit as prior_audit
from simulations.x650_wholebody_mpc_v1 import run_case as shared
from simulations.x650_wholebody_mpc_v1.research_condition import condition_for_task,frozen_research_pid
from simulations.x650_wholebody_mpc_v1.estimated_feedback import NED_TO_ENU,FRD_TO_FLU
from x650_wholebody.model import rotation_matrix
import audit_x650_precision_stage as fixed
import audit_x650_upper_stage as upper
import analyze_x650_hover_precision as precision
from run_x650_isolated_runtime import reconcile_recording,stats

read,sha,require=task.read,task.sha,fixed.require


def rows(p):
    with Path(p).open() as f:return list(csv.DictReader(f))


def same(a,b,tolerance=1e-10):
    a,b=np.asarray(a),np.asarray(b)
    return a.shape==b.shape and np.isfinite(a).all() and np.isfinite(b).all() and float(np.max(abs(a-b)))<=tolerance


def load_calls(live):
    from rclpy.serialization import deserialize_message
    from flyarm_control_interfaces.msg import ControlSnapshot,ControlProposal
    calls=[]
    for line in (live/'recording/diagnostics.jsonl').open():
        d=json.loads(line)
        s=deserialize_message(base64.b64decode(d['snapshot_cdr']['base64']),ControlSnapshot)
        p=deserialize_message(base64.b64decode(d['proposal_cdr']['base64']),ControlProposal)
        require(s.sequence==p.sequence==d['sequence'],'CDR sequence mismatch')
        calls.append((s,p,d))
    require(len(calls)>1,'No complete candidate sequence')
    require([int(s.sequence) for s,_,_ in calls]==list(range(1,len(calls)+1)),'Candidate gap')
    require(all(p.status==0 and p.accepted_optimum and p.nonlinear_check_passed and not p.replay_only
                and not d['error'] for s,p,d in calls),'Rejected or shadow candidate')
    return calls


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
    require(d['protocol']==protocol and sha(launch/'task.json')==protocol['frozen_hold_task_sha256'],
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


def check_preflight_gc(live,start):
    record=read(live/'worker_preflight_gc.json');final=read(live/'worker_final.json')
    require(record==final.get('preflight_gc'),'Preflight GC evidence differs')
    require(record['explicit_collection'] and record['before_ready'] and record['gc_policy_unchanged']
            and record['generation']==2 and record['before']['enabled'] and record['after']['enabled']
            and record['before']['thresholds']==record['after']['thresholds']
            and record['before']['frozen_count']==record['after']['frozen_count']
            and final['gc_enabled_before'] and final['gc_enabled_after'],
            'Natural GC policy changed or preflight collection missing')
    require(record['start_monotonic_ns']<=record['end_monotonic_ns']<start['issued_monotonic_ns'],
            'Setup collection was not completed before actual handoff')
    return {'verified':True,'preflight_duration_ms':record['duration_ns']/1e6,
            'collected':record['collected'],'natural_runtime_gc_unchanged':True,
            'runtime_gen2_collections':sum(g['generation']==2 and g['start_ns']>=start['issued_monotonic_ns']
                                          for g in final['gc_events'])}


def check_fault_supervision(live,start):
    record=read(live/'fault_supervision.json')
    require(record.get('kind')=='isolated_fault_supervision_v1'
            and all(record.get(k)==start[k] for k in ('session_id','epoch')),
            'Fault supervisor identity differs')
    require(record.get('first_fault') is None,'Independent supervisor observed a latched fault')
    require(record['counts']['dispatch']>0 and record['counts']['events']>0
            and record['counts']['ignored_identity']==0,'Fault supervisor was not connected to this run')
    return {'verified':True,'counts':record['counts'],'first_fault':None,
            'scope':'latest-value supervision; complete evidence is reconciled independently'}


def check_preflight_transport(live,meta,start):
    record=read(live/'preflight_transport.json')
    require(record==meta.get('isolated_preflight_transport'),'Ground initialization evidence differs')
    require(all(record.get(k) is True for k in ('ground_landed','ground_disarmed','loop_strict_false',
        'before_live_start','allocator_constructed_no_step','transport_ready')),
        'Allocator or transport was not prepared on the disarmed ground')
    require(record['allocator_step_count']==0 and record['normal_output_count']==0
            and record['initial_task_origin_s']==0.,'Preflight preparation advanced control or sent live output')
    takeoff=meta['takeoff_state_transition']['before']['command_boundary_wall_monotonic_s']
    require(0<record['start_monotonic_ns']<=record['end_monotonic_ns']<start['issued_monotonic_ns']
            and record['end_monotonic_ns']/1e9<takeoff,
            'Synchronous initialization did not finish before actual takeoff')
    return {'verified':True,'duration_ms':(record['end_monotonic_ns']-record['start_monotonic_ns'])/1e6,
            'allocator_prepare_ms':record['allocator_prepare_ns']/1e6,
            'transport_launch_ms':record['transport_launch_ns']/1e6,
            'no_allocator_step_or_live_output_before_handoff':True}


def check_receipt_identity(sends,receipts,start):
    sent={s['dispatch_sequence']:s for s in sends}
    received={r['dispatch_sequence']:r for r in receipts}
    require(len(sent)==len(sends) and len(received)==len(receipts),'Duplicate send/receipt identity')
    require(set(sent)==set(received),'Send and receipt ID sets differ')
    for key,s in sent.items():
        r=received[key]
        require(all(s[k]==r[k]==start[k] for k in ('session_id','epoch')),
                'Send/receipt belongs to another live session')
        require(s['command_sequence']==r['command_sequence'],'Receipt candidate identity differs')
    return received


def check_consumption(folder,sends,receipts,start,release):
    require(sends and len({s['dispatch_sequence'] for s in sends})==len(sends),'Send ledger incomplete')
    source=rows(folder/'ts20_state.csv')
    events=[{},{}]
    for row in source:
        for j in range(2):events[j].setdefault(int(row[f'upper_sequence{j+1}']),row)
    received=check_receipt_identity(sends,receipts,start)
    lag=[]
    for i,s in enumerate(sends):
        r=received.get(s['dispatch_sequence'])
        require(r is not None and r['joint_consumed'] and not r['failed'],'Missing actual consumption receipt')
        require(r['pva_send_returned'] is (not s['stop_zero']),'Receipt PVA semantics differ')
        require(same(r['consumed_counter'],s['expected_counter'],0),'Consumption receipt counters differ')
        if i:require(same(s['expected_counter'],np.asarray(sends[i-1]['expected_counter'])+1,0),
                     'Another writer or skipped send counter')
        if i and not s['stop_zero']:
            require(received[sends[i-1]['dispatch_sequence']]['observed_monotonic_ns']<=s['sent_monotonic_ns'],
                    'Next actuator output preceded previous actual joint consumption acknowledgement')
        for j in range(2):
            raw=events[j].get(s['expected_counter'][j])
            require(raw is not None,'Sent joint command not consumed in raw plugin log')
            require(abs(float(raw[f'u{j+1}'])-s['joint_velocity'][j])<=1e-12,'Raw consumed command differs')
            stamp=float(raw[f'receipt{j+1}_s'])
            require(abs(stamp-r['consumed_sim_s'][j])<=1e-9,'Receipt differs from raw plugin timestamp')
            require(-.008<=stamp-s['sim_s']<=.05,'Unbounded actual command delivery')
            lag.append(stamp-s['sim_s'])
    zeros=[s for s in sends if s['stop_zero']]
    require(len(zeros)==1 and zeros[0] is sends[-1] and same(zeros[0]['joint_velocity'],[0.,0.]),
            'Stop must be one final zero command')
    require(release['zero_consumed'] and release['writer_exited'],'Writer was not released before recovery')
    trace=[json.loads(line) for line in (folder/'control_execution_trace.jsonl').open() if line.strip()]
    # Legacy publications carry precise wall send markers. No publication by
    # that owner is permitted inside the handoff/release interval.
    publications=[e for e in trace if 'publish_before' in e]
    overlaps=[]
    for e in publications:
        wall=e['publish_before'].get('wall_monotonic_s')
        if wall is not None and start['issued_monotonic_ns']<=wall*1e9<release['wall_monotonic_ns']:overlaps.append(e)
    require(not overlaps,'Overlapping legacy and isolated output owners')
    return {'verified':True,'sent_commands':len(sends),'individual_joint_commands':2*len(sends),
            'all_raw_consumed_counters_matched':True,'zero_consumed':True,
            'exact_receipt_id_set_and_session_verified':True,
            'no_overlapping_legacy_publications':True,'legacy_publication_records':len(publications),
            'source_observation_to_consumption_s':stats(lag),
            'lag_clock_note':'Latest actual plugin sim clock at send; no fitted time offset'}


def command_view(folder,sends,calls,start):
    by_seq={int(s.sequence):(s,p) for s,p,_ in calls}
    first_send={}
    for event in sends:
        if not event['stop_zero']:first_send.setdefault(event['command_sequence'],event)
    for s,p,d in calls[1:]:
        previous=first_send.get(int(s.sequence)-1)
        require(previous is not None,'Worker advanced a candidate that was never actually sent')
        require(p.compute_start_monotonic_ns>=previous['sent_monotonic_ns']
                and p.compute_start_monotonic_ns>=s.issued_monotonic_ns,
                'Worker advanced before actual previous send or before its real source existed')
    derived=[]
    for event in sends:
        if event['stop_zero']:continue
        s,p=by_seq[event['command_sequence']]
        expected_p=np.asarray([p.position_reference_enu[1],p.position_reference_enu[0],start['ref_alt']-p.position_reference_enu[2]])
        expected_v=NED_TO_ENU@np.asarray(p.velocity_feedforward_enu)
        require(event['coordinate_frame']==1 and event['type_mask']==2048
                and same(event['position_ned'],expected_p) and same(event['velocity_ned'],expected_v)
                and same(event['acceleration_ned'],[0.,0.,0.])
                and same(event['joint_velocity'],p.joint_velocity),'Actual wire input differs from accepted proposal')
        require(abs(event['yaw_ned']-start['yaw_ned'])<1e-10,'Fixed yaw changed')
        fb=NED_TO_ENU@np.asarray(s.position_enu);fb[2]+=start['ref_alt']
        row={'phase':'measure','sim_s':event['sim_s'],'t_s':event['sim_s']-start['origin_s'],
            'pva_sent':'True','sent_coordinate_frame':1,'sent_type_mask':2048,'sent_yaw_ned':event['yaw_ned']}
        for prefix,axes,value in [('sent_position_target_','ned',event['position_ned']),
                ('sent_velocity_target_','ned',event['velocity_ned']),('sent_acceleration_target_','ned',event['acceleration_ned']),
                ('feedback_position_','ned',fb),('base_velocity_target_','xyz',p.base_velocity_target),
                ('command_acceleration_','xyz',p.acceleration_feedforward_enu)]:
            row.update({prefix+k:float(v) for k,v in zip(axes,value)})
        derived.append(row)
    # Preserve original prestream/recovery context for full-window ULog checks.
    legacy=[r for r in rows(folder/'control_trace.csv') if r.get('pva_sent') in ('True','true','1')]
    return sorted(legacy+derived,key=lambda r:float(r['sim_s'])),derived


def check_feedback(folder,calls,reference):
    source=read(folder/'isolated_live/feedback_sources.json')
    lookup={round(e['sim_s'],9):e['feedback'] for e in source}
    raw=rows(folder/'ts20_state.csv');times=np.asarray([float(r['sim_s']) for r in raw])
    packets=rows(folder/'ts20_packets.csv')
    packet_lookup={round(float(r['send_s']),9):r for r in packets}
    bound_feedback,bound_diagnostics=[],[]
    maxima={};bound=reference['px4_reference_at_bind']
    for s,p,d in calls:
        f=lookup.get(round(s.control_time_s,9));require(f is not None,'Admitted state lacks actual source snapshot')
        require(f['truth_feedback'] is False and f['truth_clock'] is False,'Truth substituted into controller feedback')
        state=f['control_state'];px=f['px4_snapshot']
        for name,key in [('position_enu','position_enu_m'),('velocity_enu','velocity_enu_m_s'),
                ('omega_body_flu','omega_flu_rad_s'),('acceleration_enu','acceleration_enu_m_s2'),
                ('q','q_rad'),('dq','dq_rad_s')]:
            require(same(getattr(s,name),state[key]),'Admitted state differs from actual feedback: '+name)
        require(same(s.rotation_row_major,np.asarray(state['rotation_flu_to_enu']).reshape(-1)),
                'Admitted rotation differs')
        pos=NED_TO_ENU@np.asarray(px['position_ned_m']);pos[2]+=bound['ref_alt']
        require(same(s.position_enu,pos) and same(s.velocity_enu,NED_TO_ENU@np.asarray(px['velocity_ned_m_s']))
                and same(s.acceleration_enu,NED_TO_ENU@np.asarray(px['acceleration_ned_m_s2'])),
                'PX4 frame or acceleration mapping differs')
        require(same(s.rotation_row_major,(NED_TO_ENU@rotation_matrix(px['attitude_wxyz'])@FRD_TO_FLU).reshape(-1)),
                'PX4 attitude frame mapping differs')
        index=int(np.argmin(abs(times-s.joint_time_s)));require(abs(times[index]-s.joint_time_s)<1e-9,
                'Joint source timestamp absent from actual complete row')
        q=[float(raw[index][f'q{j}_physical']) for j in (1,2)]
        dq=[float(raw[index][f'dq{j}_internal']) for j in (1,2)]
        require(same(s.q,q) and same(s.dq,dq),'Controller joint source is not actual plugin physical q/endpoint dq')
        row=raw[index];origin=s.control_time_s-s.task_time_s
        require(same(s.reference_r,[float(row['r1']),float(row['r2'])])
                and abs(s.reference_time_s-(s.joint_time_s-origin))<=1e-9,
                'Predictor reference is not the exact selected joint source')
        # The selected complete row is the newest already-read row <= the
        # control clock. Its last consumed two-axis value is held thereafter;
        # later postflight rows cannot revise this recorded causal history.
        require(s.command_count==1 and same(s.command_velocity[:2],[float(row['u1']),float(row['u2'])])
                and abs(s.command_time_s[0]+origin-max(float(row['receipt1_s']),float(row['receipt2_s'])))<=1e-9,
                'Prediction command history differs from actual consumed commands')
        sent_times=[float(s.packet_send_time_s[i])+origin for i in range(s.packet_count)]
        require(sent_times and max(sent_times)<=s.control_time_s+1e-9
                and (len(sent_times)==1 or max(abs(np.diff(sent_times)-.005))<=1e-9),
                'Prediction MIT history has future packets or a bus-grid gap')
        for i,stamp in enumerate(sent_times):
            packet=packet_lookup.get(round(stamp,9));require(packet is not None,'Predicted history packet absent from actual bus log')
            for field,short in [('q_ref','qref'),('dq_ref','dqref'),('tau_ff','tau'),('kp','kp'),('kd','kd')]:
                require(same(getattr(s,'packet_'+field)[2*i:2*i+2],[float(packet[short+str(j)]) for j in (1,2)]),
                        'Predictor packet differs from actual sent MIT packet: '+field)
        require(abs(sent_times[0]-float(row['active_send_s']))<=1e-9
                and sent_times[-1]>=float(row['last_send_s'])-1e-9,
                'Prediction packet history does not span the selected active/last-sent packets')
        bound_feedback.append({'phase':'measure','sim_s':s.control_time_s,'t_s':s.task_time_s,'feedback':f})
        bound_diagnostics.append({'phase':'measure','sim_s':s.control_time_s,'t_s':s.task_time_s,'diagnostics':d['diagnostic']})
        for name,c in f['callbacks'].items():
            age=float(c['sim_age_s']);maxima[name]=max(maxima.get(name,0.),age)
            limit=1.25 if name=='estimator_flags' else .25 if name=='joints' else .1
            require(-1e-9<=age<=limit+1e-9,'Stale admitted sensor source')
    context,selected=prior_audit.causal_reference_binding(raw,bound_feedback,bound_diagnostics,
        [calls[0][0].control_time_s,calls[-1][0].control_time_s])
    context['actual_packet_binding']=prior_audit.packet_state_binding(selected,packets)
    return {'verified':True,'admitted_samples':len(calls),'source_samples':len(source),
            'maximum_callback_age_s':maxima,'physical_joint_q_and_modeled_endpoint_dq':True,'truth_feedback':False,
            'prediction_context':context,'all_wire_packet_and_command_histories_verified':True}


def replay(live,task_path,calls,start):
    from flyarm_mpc_reference.live_factory import create_live_engine
    from flyarm_control_interfaces.msg import ControlProposal
    engine=create_live_engine(ROOT,task_path,sha(task_path),live/'initialization.json',sha(live/'initialization.json'),start['session_id'],start['epoch'])
    engine.begin_live(SimpleNamespace(**start))
    error=0.;compute=[]
    fields=('position_reference_enu','velocity_feedforward_enu','acceleration_feedforward_enu','base_velocity_target','joint_velocity')
    for s,actual,d in calls:
        s.issued_monotonic_ns=time.monotonic_ns()
        direct=engine.process(s,ControlProposal)
        require(direct.status==0,'Direct replay rejected: '+str(engine.last_error))
        error=max(error,*[float(np.max(abs(np.asarray(getattr(actual,f))-getattr(direct,f)))) for f in fields])
        compute.append((actual.compute_end_monotonic_ns-actual.compute_start_monotonic_ns)/1e6)
    require(error<=1e-7,'Numeric equivalence failed')
    return {'verified':True,'pairs':len(calls),'max_command_absolute_error':error,
            'worker_compute_ms':stats(compute),'only_transport_wall_clock_rebased':True}


def audit(launch,folder,out):
    report={'kind':'x650_isolated_live_stage_certificate_v1','passed':False,'complete':False,
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
        report['fault_supervision']=check_fault_supervision(live,start)
        require(abs(meta['task_end_sim_s']-window[1])<=.05,'Incomplete declared hold')
        report.update(stage='tip_hold',window_sim_s=window)
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
        endpoint=shared.native_tip_metrics(folder,shared.ConstantTarget(np.asarray(start['target_position_enu'])).sample,window[0],(0.,cfg['duration_s']))
        limits=cfg['ts20_protocol']
        require(endpoint['position_rmse_m']<=limits['tip_rms_max_m'] and endpoint['position_max_m']<=limits['tip_peak_max_m']
                and float(native['tilt'].max())<=limits['tilt_max_deg'],'Physical hold envelope failed')
        report['endpoint']=endpoint;report['tilt_peak_deg']=float(native['tilt'].max())
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
