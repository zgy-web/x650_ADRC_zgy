#!/usr/bin/env python3
"""Independent synchronous PIL audit, using original ULog/plant/NX records."""
import argparse
import csv
import json
from pathlib import Path
import sys
import traceback
import zipfile
import hashlib
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
for p in (ROOT,ROOT/'src',ROOT/'scripts',ROOT/'control_packages/x650_wholebody_v1'):
    sys.path.insert(0,str(p))
import audit_x650_isolated_live as prior
from simulations.x650_cpp_multirate_v1 import task
from simulations.x650_nx_sync_pil_v1.protocol import clock_barrier
from x650_pil_transport import validate_candidate
from run_x650_pil_stream import stats


def lines(path):return [json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()]


def audit(run,launch,out):
    out.mkdir(parents=True,exist_ok=False)
    report={'kind':'x650_sync_pil_audit_v1','passed':False,'physical_closed_loop':True,
        'wall_clock_qualified':False,'hardware_flight_qualified':False,'raw_px4_receipt_replay_qualified':False}
    try:
        read,require=prior.read,prior.require
        m=read(run/'controller_metadata.json');l=read(launch/'launch.json');cfg=m['task'];plan=l['plan']
        require(l['complete'] and l['runtime_closed'] and m['complete'] and m['landed_and_disarmed']
            and m['diagnostic_parameters_restored'] and not m['diagnostic_restore_pending'],'Incomplete lifecycle/restore')
        require(str(run.resolve()) in l['run_directories'],'Run does not belong to the bound launch')
        if (launch/'source_manifest.json').exists():
            before=read(launch/'source_manifest.json')
            require(l['sources_unchanged'] and l['stepper_unchanged'] and before==read(launch/'source_manifest_after.json'),
                'Sources or stepper changed during physical execution')
            with zipfile.ZipFile(launch/'sources.zip') as z:
                require(set(z.namelist())==set(before),'Source archive membership')
                for rel,h in before.items():require(hashlib.sha256(z.read(rel)).hexdigest()==h,'Source archive bytes: '+rel)
            report['sources_bound_before_and_after']=True
        else:report['sources_bound_before_and_after']=False
        require(m['sync_release']['zero_consumed'] and m['sync_release']['resumed_for_original_recovery'],'Unverified output release')
        require('nx_stop_error' not in m['sync_release'] and 'sync_cleanup_error' not in m
            and 'sync_helper_stop_error' not in m,'Synchronous session did not close cleanly')
        exchange=lines(run/'nx_exchange.jsonl');start=exchange[0]['request']['handoff'];calls=exchange[1:]
        ready=exchange[0]['response']
        require(ready['status']=='ready' and ready['scope']=='simulation_pil'
            and ready['session_id']==start['session_id'] and ready['epoch']==start['epoch']
            and ready['source_sha256']==plan['nx_source_sha256'],'NX handoff identity mismatch')
        start_time=start['origin_s'];duration=cfg['duration_s'];end=start_time+duration;window=[start_time,end]
        require(abs(m['task_end_sim_s']-end)<=.021,'Incomplete declared task window')
        previous=start_time;proposals={};core=[];wall=[]
        for seq,r in enumerate(calls,1):
            s=r['request']['snapshot'];p=r['response']['proposal_nx_clock']
            validate_candidate(r['request'],r['response'],sent_ns=r['sent_ns'],received_ns=r['received_ns'],
                issue_ns=s['issued_monotonic_ns'],source_sha256=plan['nx_source_sha256'],check_deadline=False)
            require(s['sequence']==seq and abs(s['control_time_s']-previous-.04)<1e-8,'Nonuniform solve grid or missing candidate')
            require(abs(s['dt_s']-.04)<1e-8,'Measured solve dt differs from actual step clock')
            previous=s['control_time_s'];proposals[seq]=(s,p)
            core.append(r['response']['core_wall_ns']/1e6);wall.append((r['received_ns']-r['sent_ns'])/1e6)
        require(len(calls)==round(duration/.04)-1,'Incomplete full solve sequence')
        sends=lines(run/'sync_dispatch.jsonl');normal=[x for x in sends if not x['zero']]
        raw=prior.rows(run/'ts20_state.csv');rt=np.array([float(x['sim_s']) for x in raw])
        first_sent={};derived=[];seen=[None,None]
        for send in sends:
            proof=send['consumption'];require(proof['verified'],'Unverified command consumption')
            idx=int(np.argmin(abs(rt-proof['sim_s'])));row=raw[idx]
            require(abs(rt[idx]-proof['sim_s'])<1e-9,'Missing actual consumption row')
            for j in range(2):
                actual=int(float(row[f'upper_sequence{j+1}']))
                require(actual==proof['sequences'][j] and (seen[j] is None or actual>seen[j]),'Consumption counter not increasing')
                require(abs(float(row[f'u{j+1}'])-send['joint'][j])<1e-10,'Consumed joint value differs')
                receipt=float(row[f'receipt{j+1}_s'])
                require(send['sim_s']-1e-9<=receipt<=send['sim_s']+.02+1e-9,'Consumption outside next physics segment')
                seen[j]=actual
            if send['zero']:continue
            s,p=proposals[send['proposal_sequence']];first_sent.setdefault(s['sequence'],send['wall_ns'])
            expected_p=[p['position_reference_enu'][1],p['position_reference_enu'][0],start['ref_alt']-p['position_reference_enu'][2]]
            expected_v=[p['velocity_feedforward_enu'][1],p['velocity_feedforward_enu'][0],-p['velocity_feedforward_enu'][2]]
            require(prior.same(send['position_ned'],expected_p) and prior.same(send['velocity_ned'],expected_v)
                and prior.same(send['joint'],p['joint_velocity']) and send['type_mask']==2048,'Wire encoding mismatch')
            fb=[s['position_enu'][1],s['position_enu'][0],start['ref_alt']-s['position_enu'][2]]
            d={'phase':'measure','sim_s':send['sim_s'],'t_s':send['sim_s']-start_time,'pva_sent':'True',
                'sent_coordinate_frame':1,'sent_type_mask':2048,'sent_yaw_ned':start['yaw_ned']}
            for prefix,axes,values in [('sent_position_target_','ned',expected_p),('sent_velocity_target_','ned',expected_v),
                ('sent_acceleration_target_','ned',[0.,0.,0.]),('feedback_position_','ned',fb),
                ('base_velocity_target_','xyz',p['base_velocity_target']),('command_acceleration_','xyz',[0.,0.,0.])]:
                d.update({prefix+k:float(v) for k,v in zip(axes,values)})
            derived.append(d)
        for seq,r in enumerate(calls,1):
            require(seq in first_sent,'Accepted candidate never sent')
            if seq>1:require(first_sent[seq-1]<=r['sent_ns'],'Next solve preceded previous actual dispatch')
            s=r['request']['snapshot'];idx=int(np.argmin(abs(rt-s['joint_time_s'])));row=raw[idx]
            require(abs(rt[idx]-s['joint_time_s'])<1e-9 and s['joint_time_s']<=s['control_time_s']+1e-9,'Noncausal joint source')
            require(prior.same(s['q'],[float(row[f'q{j}_physical']) for j in (1,2)]) and
                prior.same(s['dq'],[float(row[f'dq{j}_internal']) for j in (1,2)]),'NX joint inputs differ from plant')
        send_t=np.array([x['sim_s'] for x in normal]);require(len(send_t)>2 and np.max(abs(np.diff(send_t)-.02))<1e-8,'Dispatch grid differs')
        clocks=lines(run/'sync_clock.jsonl');position=None;steps=0;frozen_checks=0
        for c in clocks:
            if c['op']=='pause':position=c['sim_ns']
            elif c['op']=='step':
                require(position==c['before_sim_ns'],'Barrier continuity differs')
                clock_barrier(position,c['sim_ns'],ticks=c['requested_ticks']);position=c['sim_ns'];steps+=1
            elif c['op']=='status' and c['owns_pause']:
                clock_barrier(position,c['sim_ns']);frozen_checks+=1
        require(frozen_checks>=len(calls)+1,'Missing frozen clock checks around NX')
        native_manifest=read(run/'native_trace_manifest.json')
        require(native_manifest['complete'] and native_manifest['sha256']==prior.sha(run/'native_trace.csv'),'Native recording integrity')
        prior.precision.capture_window_covers(native_manifest,window)
        native=prior.precision.native_states(run/'native_trace.csv',window)
        reference=read(run/'hover_reference.json')
        docs={k:read(run/n) for k,n in {'session':'diagnostic_parameter_session.json','journal':'diagnostic_restore_journal_completed.json',
            'before':'diagnostic_parameters_before.json','applied':'diagnostic_parameters_applied.json','restored':'diagnostic_parameters_restored.json',
            'px4_before':'px4_parameters_before.json','px4_after':'px4_parameters_after.json'}.items()}
        _,pid,_=prior.frozen_research_pid(cfg)
        report['parameters']=prior.fixed.verify_parameter_session(docs,m,prior.condition_for_task(cfg),pid)
        from pyulog import ULog
        ulog=ULog(str(run/'px4.ulg'))
        report['configuration']=prior.fixed.effective_configuration(ulog,native,reference,window,prior.fixed.records(docs['applied']))
        old=[x for x in prior.rows(run/'control_trace.csv') if x.get('pva_sent') in ('True','true','1')]
        controls=sorted(old+derived,key=lambda x:float(x['sim_s']))
        report['px4_commands']=prior.upper.upper_commands(ulog,native,reference,controls,window,pid,response=False)
        report['flight']=prior.upper.flight_states(run/'flight_samples.csv',window)
        report['ground']=prior.fixed.native_ground_check(run,read(run/'native_ground.json'))
        # Original plant adapter selects its pinned motor variant; no retuning.
        source=read(plan['source_task']['path']);base=task.baseline_config(source)
        report['actuator']=prior.prior_audit.actuator_check(run,base,window)
        report['actuator'].pop('control_snapshot_to_plugin_consumption_lag',None)
        require(report['actuator']['new_joint_prediction_verified'],'Independent full-window actuator forecast failed')
        center=np.array(start['target_position_enu'])
        trajectory=(prior.shared.ConstantTarget(center) if cfg['stage']=='tip_hold' else
            prior.shared.FigureEight(center,**cfg['trajectory_parameters']))
        for record in calls:
            s=record['request']['snapshot']
            for field,value in zip(('target_position_enu','target_velocity_enu','target_acceleration_enu'),trajectory.sample(s['task_time_s'])):
                require(prior.same(s[field],value),'Actual NX target differs from declared trajectory')
        endpoint=prior.shared.native_tip_metrics(run,trajectory.sample,start_time,(0.,duration))
        tracking=(endpoint if cfg['stage']=='tip_hold' else prior.shared.native_tip_metrics(run,trajectory.sample,start_time,(trajectory.start_s,trajectory.end_s)))
        require(endpoint['position_rmse_m']<=cfg['ts20_protocol']['tip_rms_max_m'] and endpoint['position_max_m']<=cfg['ts20_protocol']['tip_peak_max_m']
            and float(native['tilt'].max())<=cfg['ts20_protocol']['tilt_max_deg'],'Physical hold error/tilt limit failed')
        report.update(case=cfg['stage'],endpoint=endpoint,tracking=tracking,tilt_peak_deg=float(native['tilt'].max()),nx_core_ms=stats(core),server_roundtrip_ms=stats(wall),
            nx_steps=len(calls),normal_dispatches=len(normal),zero_consumed=True,fixed_physics_segments=steps,
            frozen_checks=frozen_checks,window_sim_s=window,passed=True,complete=True,
            feedback_scope='Original validated PX4 estimated-state adapter; same-time physical q/dq independently matched; raw per-callback receipt replay not certified')
        (out/'derived_pva_view.json').write_text(json.dumps({'derived_from':'actual dispatch plus corresponding NX request/response','rows':derived}))
        with (out/'endpoint_series.csv').open('x',newline='') as f:
            writer=csv.writer(f);writer.writerow(['task_s','actual_x_m','actual_y_m','actual_z_m','target_x_m','target_y_m','target_z_m','error_3d_m'])
            for row in prior.rows(run/'native_trace.csv'):
                t=float(row['sim_s'])-start_time
                if -1e-9<=t<=duration+1e-9:
                    actual=np.array([float(row['tip_'+a]) for a in 'xyz']);target=trajectory.sample(t)[0]
                    writer.writerow([t,*actual,*target,float(np.linalg.norm(actual-target))])
        with (out/'timing_series.csv').open('x',newline='') as f:
            writer=csv.writer(f);writer.writerow(['task_s','nx_core_ms','server_roundtrip_ms'])
            writer.writerows([r['request']['snapshot']['task_time_s'],c,w] for r,c,w in zip(calls,core,wall))
    except BaseException as e:report.update(error=repr(e),traceback=traceback.format_exc())
    report['artifacts']={str(p):prior.sha(p) for p in sorted(run.iterdir()) if p.is_file() and p.suffix in ('.json','.jsonl','.csv','.ulg')}
    (out/'report.json').write_text(json.dumps(report,indent=2));return report


def main():
    p=argparse.ArgumentParser()
    for name in ('run','launch','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();r=audit(a.run,a.launch,a.output)
    print(json.dumps({k:r.get(k) for k in ('passed','error','nx_steps','normal_dispatches','endpoint','nx_core_ms','server_roundtrip_ms')},indent=2))
    return 0 if r['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
