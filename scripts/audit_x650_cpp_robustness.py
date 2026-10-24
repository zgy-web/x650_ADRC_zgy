#!/usr/bin/env python3
"""Independent physical robustness acceptance for the frozen 24-second fast MPC.

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
import math
import xml.etree.ElementTree as ET
import numpy as np
ROOT=next(p for p in Path(__file__).resolve().parents
    if (p/'simulations/x650_cpp_robustness_v1').is_dir())
sys.path[:0]=[str(ROOT),str(ROOT/'scripts'),str(ROOT/'control_packages/x650_wholebody_v1')]
from simulations.x650_cpp_robustness_v1 import qualification
from simulations.x650_cpp_robustness_v1.task import baseline_config, validate_task_config
from report_x650_cpp_live import identity, require, read
from audit_x650_cpp_hold import (rows, check_worker, check_ticks, check_actual_send_barrier,
    check_recovery, ulog_value_view)


def bound_identity(binding):
    require(isinstance(binding,dict) and isinstance(binding.get('path'),str)
        and isinstance(binding.get('sha256'),str), 'Missing physical artifact binding')
    actual=identity(binding['path'])
    require(all(actual.get(k)==v for k,v in binding.items()), 'Physical artifact changed: '+binding['path'])
    return Path(binding['path'])


def sdf_inertias(path):
    """Recompute the seven rigid inertias without trusting the generator ledger."""
    tree=ET.parse(path);models=tree.getroot().findall('model')
    require(tree.getroot().tag=='sdf' and len(models)==1, 'One actual SDF model required')
    values={}
    for link in models[0].findall('link'):
        name=link.get('name');inertial=link.find('inertial')
        require(name not in values and inertial is not None, 'Missing or repeated rigid link')
        fields={'mass':inertial.findtext('mass'), **{k:inertial.findtext('inertia/'+k)
            for k in ('ixx','ixy','ixz','iyy','iyz','izz')}}
        result={k:float(v) for k,v in fields.items()}
        require(all(math.isfinite(v) for v in result.values()) and result['mass']>0,
                'Invalid independent SDF inertia')
        values[name]=result
    require(set(values)=={'base_link','arm_link_1','arm_link_2','link_002','link_003','link_004','link_005'},
            'Actual X650 rigid topology differs')
    return tree,models[0].get('name'),values


def check_physical_model(cfg,meta,stage,proof):
    """Separate actual rigid plant identity from the unchanged nominal controller."""
    from amsim.modeling.manager import resolve
    from prepare_x650_robustness_models import verify_scaled_tree
    nominal=read(bound_identity(cfg['frozen_cpp_fast']))
    model=resolve(cfg['model']);nominal_model=resolve(nominal['model'])
    actual_tree,actual_name,actual_values=sdf_inertias(model.sdf_path)
    nominal_tree,nominal_name,nominal_values=sdf_inertias(nominal_model.sdf_path)
    require(meta['model_sha256']==stage['model_sha256']==model.sha256==qualification.sha(model.sdf_path)
        and cfg['preparation_report']==nominal['preparation_report'],
        'Actual model or frozen controller preparation identity differs')
    # The profile binding may change to name the real plant. Its numerical PID
    # values and their canonical digest must stay exactly the nominal values.
    actual_pid,nominal_pid=read(cfg['pid_profile']),read(nominal['pid_profile'])
    require(actual_pid['model_sha256']==model.sha256
        and {k:v for k,v in actual_pid.items() if k!='model_sha256'}==
            {k:v for k,v in nominal_pid.items() if k!='model_sha256'},
        'Robustness changed controller PID values or misbound its plant')
    robust=cfg['robustness'];scale=robust['mass_inertia_scale']
    qualified=proof['physical_condition']
    require(qualified['verified'] is True and qualified['model_sha256']==model.sha256
        and qualified['scale']==scale, 'Ground qualification names a different actual plant')
    if robust['kind']=='external_wrench':
        require(scale==1. and robust['physical_manifest'] is None and model.sha256==nominal_model.sha256
            and actual_name==nominal_name and actual_values==nominal_values,
            'External-wrench case altered the nominal rigid plant')
        return dict(verified=True,kind=robust['kind'],scale=scale,model_sha256=model.sha256,
            nominal_model_sha256=nominal_model.sha256,total_mass_kg=sum(v['mass'] for v in actual_values.values()),
            controller_preparation_unchanged=True,controller_pid_values_unchanged=True)
    require(robust['kind']=='uniform_rigid_mismatch' and scale in (.85,1.15), 'Unreviewed physical mismatch')
    manifest_path=bound_identity(robust['physical_manifest']);manifest=read(manifest_path)
    from verify_x650_robustness_model import verify_model
    independent=verify_model(manifest_path,expected_scale=scale)
    require(independent['verified'] is True and independent['model_sha256']==model.sha256,
            'Independent actual model resource verification failed')
    require(manifest['kind']=='x650_uniform_rigid_mismatch_v1' and manifest['scale']==scale
        and manifest['model_sha256']==model.sha256 and manifest['model_name']==actual_name
        and bound_identity(manifest['nominal_sdf']).resolve()==nominal_model.sdf_path.resolve()
        and bound_identity(manifest['nominal_preparation_report']).resolve()==Path(cfg['preparation_report']).resolve(),
        'Physical mismatch manifest is not the actual nominal/variant pair')
    for key in ('nominal_model_manifest','nominal_plant_manifest','unchanged_airframe_source','generator'):
        bound_identity(manifest[key])
    require(verify_scaled_tree(nominal_tree,actual_tree,scale=scale) is True,
            'An unreviewed SDF semantic difference exists')
    ledger=manifest['per_link'];require(len(ledger)==len(actual_values), 'Incomplete physical ledger')
    by_name={item['link']:item for item in ledger}
    require(len(by_name)==len(ledger) and set(by_name)==set(actual_values), 'Duplicate or missing ledger link')
    for name,actual in actual_values.items():
        expected={k:v*scale for k,v in nominal_values[name].items()}
        require(actual==expected and by_name[name]['nominal']==nominal_values[name]
            and by_name[name]['actual']==actual and by_name[name]['inertial_pose_preserved'] is True,
            'Independently recomputed rigid scaling/ledger differs: '+name)
    total=sum(v['mass'] for v in actual_values.values());prior=sum(v['mass'] for v in nominal_values.values())
    require(abs(total-manifest['total_mass_kg'])<=1e-12 and abs(prior-manifest['nominal_total_mass_kg'])<=1e-12
        and abs(total-prior*scale)<=1e-12, 'Aggregate physical mismatch differs')
    plant=read(cfg['ts20_inputs']['plant_manifest']);original=read(nominal['ts20_inputs']['plant_manifest'])
    require(plant['model_sha256']==model.sha256 and plant['parameters']==original['parameters']
        and plant['rigidity_mismatch_manifest_sha256']==qualification.sha(manifest_path),
        'Changed actual TS20 plant parameters or lost rigid binding')
    return dict(verified=True,kind=robust['kind'],scale=scale,model_sha256=model.sha256,
        nominal_model_sha256=nominal_model.sha256,total_mass_kg=total,nominal_total_mass_kg=prior,
        rigid_links_independently_recomputed=len(actual_values),per_link=ledger,
        all_other_sdf_semantics_unchanged=True,controller_preparation_unchanged=True,
        controller_pid_values_unchanged=True,ts20_internal_model_unchanged=True,readonly_resource_verification=independent,
        scope='Actual rigid mass/COM inertia mismatch against unchanged identified upper response; not an identified loaded-joint motor model')


WRENCH_HEADER=('sim_s,t_s,step_dt_s,in_task,event_active,fx_command_N,fy_command_N,fz_command_N,'
    'tx_command_Nm,ty_command_Nm,tz_command_Nm,fx_enqueued_N,fy_enqueued_N,fz_enqueued_N,'
    'tx_origin_enqueued_Nm,ty_origin_enqueued_Nm,tz_origin_enqueued_Nm,tx_com_enqueued_Nm,'
    'ty_com_enqueued_Nm,tz_com_enqueued_Nm,r_com_world_x,r_com_world_y,r_com_world_z,valid_enqueued').split(',')


def wrench_waveform(records,status,*,origin,schedule,duration=24.):
    """Recompute every native enqueue row and every individual pulse impulse."""
    require(records and set(records[0])==set(WRENCH_HEADER), 'Incomplete native wrench columns')
    a=np.asarray([[float(r[k]) for k in WRENCH_HEADER] for r in records])
    require(a.shape==(len(records),len(WRENCH_HEADER)) and np.isfinite(a).all(), 'Nonfinite wrench evidence')
    sim,t,dt=a[:,:3].T
    require(np.all(np.diff(sim)>0) and np.all(dt>0) and np.max(dt)<=.004+1e-9
        and np.max(abs(np.diff(sim)-dt[1:]))<=1e-8, 'Missing/nonmonotone native wrench step')
    require(status.get('state')=='SUCCESS' and all(status.get(k) is True for k in ('ready','anchored','complete','success'))
        and status['rows']==len(records) and status['duration_s']==duration and abs(status['t0_sim_s']-origin)<=1e-9,
        'Native wrench task incomplete or misanchored')
    require(np.all((t==-1)|(abs(t-(sim-origin))<=1e-9)) and np.all(a[:,-1]==1),
        'Wrench timestamp/validity mismatch')
    anchored=t>=0
    require(anchored.any() and np.all(anchored[np.flatnonzero(anchored)[0]:]),
            'Native disturbance anchor regressed')
    inside=anchored&(t<duration)
    require(np.array_equal(a[:,3],inside.astype(float)) and int(inside.sum())==status['task_rows']
        and inside.any() and sim[inside][0]-origin<=.05+1e-9
        and origin+duration-sim[inside][-1]<=.004+1e-9
        and duration<=t[-1]<=duration+.004+1e-9,
        'Wrench record does not cover the exact complete task')
    commanded=np.zeros((len(records),6));events=[]
    for event in schedule:
        lo,hi=event['start_s'],event['end_s'];v=np.r_[event['force_world_n'],event['torque_world_nm']]
        require(0<=lo<hi<=duration and v.shape==(6,) and np.isfinite(v).all(), 'Invalid fixed wrench schedule')
        mask=inside&(t>=lo)&(t<hi)
        require(mask.any() and abs(float(dt[mask].sum())-(hi-lo))<=2*float(dt.max())+1e-9,
                'Missing/truncated individual force pulse')
        commanded[mask]+=v
        actual=(a[mask,11:14]*dt[mask,None]).sum(axis=0).tolist()+\
            (a[mask,17:20]*dt[mask,None]).sum(axis=0).tolist()
        expected=(v*(hi-lo)).tolist()
        require(np.all(abs(np.asarray(actual)-expected)<=2*float(dt.max())*abs(v)+1e-8),
                'Individual pulse impulse differs from immutable schedule')
        events.append(dict(start_s=lo,end_s=hi,samples=int(mask.sum()),sampled_duration_s=float(dt[mask].sum()),
            actual_enqueued_impulse=actual,expected_ideal_impulse=expected))
    requested=a[:,5:11];force=a[:,11:14];origin_moment=a[:,14:17];couple=a[:,17:20];offset=a[:,20:23]
    error=max(float(abs(requested-commanded).max()),float(abs(force-commanded[:,:3]).max()),
        float(abs(couple-commanded[:,3:]).max()),float(abs(origin_moment-np.cross(offset,force)-couple).max()))
    require(error<=1e-9 and status['max_enqueue_component_error']<=1e-9
        and np.array_equal(a[:,4],np.any(commanded!=0,axis=1).astype(float)),
        'Physical-step command/enqueue/free-couple ledger differs')
    total=np.r_[(force[inside]*dt[inside,None]).sum(axis=0),(couple[inside]*dt[inside,None]).sum(axis=0)]
    ideal=sum((np.r_[e['force_world_n'],e['torque_world_nm']]*(e['end_s']-e['start_s']) for e in schedule),np.zeros(6))
    for name,value in [('impulse_force_Ns_couple_Nms',total),('expected_impulse_force_Ns_couple_Nms',ideal)]:
        require(np.allclose(status[name],value,rtol=0,atol=1e-8), 'Wrench status aggregate differs: '+name)
    require(abs(status['first_task_sim_s']-sim[inside][0])<=1e-9
        and abs(status['last_task_sim_s']-sim[inside][-1])<=1e-9
        and abs(status['max_step_dt_s']-dt.max())<=1e-9, 'Wrench status coverage differs from original rows')
    return dict(verified=True,rows=len(records),task_rows=int(inside.sum()),max_native_step_s=float(dt.max()),
        maximum_enqueue_component_error=error,events=events,total_enqueued_impulse=total.tolist(),
        semantics='Same-PreUpdate ExternalWorldWrenchCmd increment at base_link own CoM, world ENU force and free couple; not a force sensor or independent solver-consumption acknowledgement')


def recovery_metrics(t,error,event,*,next_start,duration=24.,threshold=.01,hold_s=1.):
    """A failed/truncated recovery remains false; no fitted baseline or phase."""
    t,error=np.asarray(t,float),np.asarray(error,float)
    require(t.ndim==error.ndim==1 and t.shape==error.shape and len(t)>1
        and np.isfinite(t).all() and np.isfinite(error).all() and np.all(np.diff(t)>0)
        and np.all(error>=0) and 0<threshold and 0<hold_s, 'Invalid recovery trace')
    start,end=float(event['start_s']),float(event['end_s']);limit=min(float(next_start),duration)
    require(0<=start<end<=limit<=duration and t[0]<=0.+1e-8 and t[-1]>=duration-.004-1e-9
        and np.diff(t).max()<=.004+1e-8, 'Incomplete physical recovery observation')
    before=(t>=max(0.,start-1.)-1e-9)&(t<start)
    pulse=(t>=start-1e-9)&(t<=end+1e-9)
    post=(t>=end-1e-9)&(t<=limit+1e-9)
    require(before.any() and pulse.any() and post.any(), 'Empty pulse/response window')
    idx=np.flatnonzero(post);run_start=None;recovery=None;confirmed=None
    for i in idx:
        if error[i]<=threshold:
            if run_start is None:run_start=i
            if t[i]-t[run_start]>=hold_s-1e-9:
                recovery=max(0.,float(t[run_start]-end));confirmed=float(t[i]);break
        else:run_start=None
    integrate=getattr(np,'trapezoid',np.trapz)
    return dict(start_s=start,end_s=end,recovery_window_end_s=limit,threshold_m=threshold,hold_s=hold_s,
        pre_pulse_1s_peak_m=float(error[before].max()),pulse_peak_m=float(error[pulse].max()),
        post_pulse_peak_m=float(error[post].max()),post_pulse_iae_m_s=float(integrate(error[post],t[post])),
        recovered=recovery is not None,recovery_after_end_s=recovery,confirmed_at_task_s=confirmed,
        recovery_already_inside_band_at_end=bool(recovery is not None and recovery<=.004+1e-9),
        scope='Same-time absolute 3D tip error, no pre-pulse bias subtraction; interval ends at next pulse or full task end')


def native_error_trace(run,start):
    selected=[r for r in rows(run/'native_trace.csv') if -1e-9<=float(r['sim_s'])-start['origin_s']<=24.+1e-9]
    require(selected and all(all(r[k]=='1' for k in ('valid_base','valid_joint_state','valid_tip')) for r in selected),
            'Invalid independent physical response state')
    t=np.asarray([float(r['sim_s'])-start['origin_s'] for r in selected])
    tip=np.asarray([[float(r['tip_'+axis]) for axis in 'xyz'] for r in selected])
    base=np.asarray([[float(r['base_p'+axis]) for axis in 'xyz'] for r in selected])
    q=np.asarray([[float(r['arm_joint_'+str(j)+'_q']) for j in (1,2)] for r in selected])
    target=qualification.figure8(start['target_position_enu'])
    desired=np.asarray([target.sample(at)[0] for at in t])
    require(all(np.isfinite(a).all() for a in (t,tip,base,q,desired)), 'Nonfinite independent response trace')
    return t,np.linalg.norm(tip-desired,axis=1),base,q


def robustness_performance(run,start,cfg):
    robust=cfg['robustness'];t,error,base,q=native_error_trace(run,start)
    result=dict(case_id=robust['case_id'],same_time=True,no_phase_alignment=True,
        body_position_peak_to_peak_m=np.ptp(base,axis=0).tolist(),
        joint_position_peak_to_peak_rad=np.ptp(q,axis=0).tolist(),delay_injected=False)
    if robust['kind']=='external_wrench':
        schedule=robust['schedule'];values=[]
        for i,event in enumerate(schedule):
            limit=schedule[i+1]['start_s'] if i+1<len(schedule) else 24.
            v=recovery_metrics(t,error,event,next_start=limit,threshold=robust['recovery_threshold_m'],hold_s=robust['recovery_hold_s'])
            mask=(t>=event['start_s'])&(t<=limit+1e-9)
            v.update(body_position_peak_to_peak_m=np.ptp(base[mask],axis=0).tolist(),
                joint_position_peak_to_peak_rad=np.ptp(q[mask],axis=0).tolist())
            values.append(v)
        result.update(events=values,performance_passed=all(v['recovered'] for v in values),
            performance_rule='Every predeclared pulse returns inside 10 mm absolute same-time error continuously for 1 s before the next pulse/task end',
            physical_motion_observed=True,causal_force_identification=False,
            scope='Independent native motion and error during/after the immutable enqueued disturbance; not a paired causal force-response identification')
    else:
        result.update(performance_passed=None,scope='Rigid mismatch tracking is reported against original safety envelope; no new post-hoc precision threshold')
    return result


def check_wrench(run,start,cfg,meta,stage):
    robust=cfg['robustness'];folder=run/'physical_disturbance'
    if robust['kind']!='external_wrench':
        require(not folder.exists() and not meta.get('external_force_disturbance') and not stage.get('external_force_disturbance'),
                'Unexpected physical wrench in rigid-mismatch-only case')
        return dict(verified=True,enabled=False)
    require(meta.get('external_force_disturbance') is True and stage.get('external_force_disturbance') is True,
            'Original run does not declare actual external-wrench integration')
    summary=read(folder/'final_summary.json')
    require(meta.get('robustness_external')==stage.get('robustness_external')==summary,
            'Physical wrench final metadata/stage/summary differ')
    require(summary.get('complete') is True and summary.get('runtime')==robust['wrench_runtime']
        and summary.get('directory')==str(folder) and summary.get('actuator_ownership_changed') is False,
        'Physical wrench final lifecycle is incomplete or uses another runtime')
    for binding in summary['artifacts'].values():bound_identity(binding)
    arm=read(folder/'arm_receipt.json');marker=(folder/'disturbance_start.txt').read_text().strip()
    require(marker=='t0_sim_s='+format(start['origin_s'],'.17g') and arm['origin_s']==start['origin_s']
        and arm==summary['arm_receipt'] and arm['same_origin_as_live_start'] is True
        and arm['max_activation_lag_s']==.05 and 0<=arm['activation_lag_s']<=.05+1e-9
        and arm['begin_monotonic_ns']<=arm['end_monotonic_ns']
        and 0<=arm['observed_before_sim_s']-start['origin_s']<=arm['activation_lag_s']+1e-9
        and abs(arm['observed_after_sim_s']-start['origin_s']-arm['activation_lag_s'])<=1e-9,
            'Physical wrench did not use the exact original LiveStart origin')
    bound_identity(arm['start_marker'])
    binding=read(folder/'entity_identity/entity_binding.json');probe=binding['probe']
    require(binding['complete'] is True and probe['verified'] is True and probe['read_only'] is True and probe['link']=='base_link'
        and probe['model_parent_id']==probe['world_id'] and probe['link_parent_id']==probe['model_id'],
        'Invalid original world/model/link ECS binding')
    require(probe['model']==read(run/'native_trace_manifest.json')['instance'],
            'Physical wrench acted on a different actual captured model instance')
    for name,digest in binding['evidence'].items():
        require(Path(name).name==name and qualification.sha(folder/'entity_identity'/name)==digest,
                'Changed original ECS identity evidence')
    require(read(folder/'entity_identity/probe_stdout.json')==probe, 'ECS probe document differs from raw stdout')
    load=read(folder/'disturbance_load_binding.json')
    require(load['schema']=='verified_disturbance_load_ack_v1' and load['service_acknowledged'] is True
        and load['force_experiment_complete'] is False and load['verified_binding']['binding_sha256']==qualification.sha(folder/'entity_identity/entity_binding.json')
        and load['config_sha256']==qualification.sha(folder/'disturbance_config.txt')
        and load['load_log_sha256']==qualification.sha(folder/'disturbance_load.log')
        and load['library_sha256']==qualification.sha(ROOT/'benchmark/whole_body_closed_loop_v1/libdisturbance.so')
        and load['loader_source_sha256']==qualification.sha(ROOT/'benchmark/whole_body_closed_loop_v1/disturbance_loader.py'),
        'Original physical disturbance load/library/config binding differs')
    require(load==summary['load_ack'] and summary['entity_binding']==identity(folder/'entity_identity/entity_binding.json'),
            'Final external-wrench identity differs from original ground load')
    ready=read(folder/'ground_ready.json')
    require(ready['disarmed_ground'] is True and ready['no_actuator_output_added'] is True
        and ready['ready']==summary['ready'] and ready['load_ack']==load
        and ready['runtime']==robust['wrench_runtime'] and ready['entity_binding']==summary['entity_binding']
        and ready['ready']['state']=='READY_WAITING_START' and ready['ready']['anchored'] is False,
        'Original external-wrench setup was not ready on the disarmed ground')
    require('data: true' in (folder/'disturbance_load.log').read_text(), 'Original load service did not acknowledge')
    config={};events=[]
    for line in (folder/'disturbance_config.txt').read_text().splitlines():
        k,v=line.split('=',1)
        if k=='event':events.append([float(x) for x in v.split(',')])
        else:
            require(k not in config,'Duplicate force config field');config[k]=v
    expected=[[e['start_s'],e['end_s'],*e['force_world_n'],*e['torque_world_nm']] for e in robust['schedule']]
    require(events==expected and config==dict(model=probe['model'],link='base_link',output=str(folder/'applied_wrench.csv'),
        start_path=str(folder/'disturbance_start.txt'),stop_path=str(folder/'disturbance_stop.txt'),duration='24'),
        'Loaded original physical waveform/path differs from predeclared schedule')
    status=read(folder/'applied_wrench.csv.status.json')
    require(status==summary['status'] and status['model']==probe['model'] and status['link']=='base_link',
            'Native wrench status differs from final metadata/entity')
    result=wrench_waveform(rows(folder/'applied_wrench.csv'),status,origin=start['origin_s'],schedule=robust['schedule'])
    result.update(enabled=True,load_ack_verified=True,actual_live_start_origin_verified=True,
        original_ecs_identity_verified=True,model_instance=probe['model'])
    return result


def upstream_shared_source_proof(cfg):
    """Bind reused static Python helpers to accepted upstream original bytes.

    The current robustness launch archives its actual robustness task/adapter
    and C++ ACTIVE implementation. The two inherited fast admission helpers
    are already independently hash-bound by the original fast certificate.
    This proof does not add them to or relabel the current sources.zip.
    """
    cert_path=bound_identity(cfg['cpp_fast_certificate']);cert=read(cert_path)
    require(cert.get('kind')=='x650_cpp_fast_stage_certificate_v1'
        and all(cert.get(k) is True for k in ('passed','complete','certificate','physical_acceptance')),
        'Accepted original fast source certificate required')
    entries=cert['artifact_identities'];index={v['path']:v['sha256'] for v in entries}
    require(len(index)==len(entries) and index==cert['artifacts'], 'Upstream original artifact index differs')
    prior=Path(cert['launch_directory']);archive=prior/'sources.zip';manifest=prior/'source_manifest.json'
    for path in (archive,manifest):
        require(index.get(str(path))==qualification.sha(path), 'Unbound or changed immutable upstream source archive/manifest')
    source_manifest=read(manifest);matched=[]
    with zipfile.ZipFile(archive) as original:
        for name in ('simulations/x650_cpp_fast_v1/task.py','simulations/x650_cpp_fast_v1/qualification.py'):
            current=identity(ROOT/name);data=original.read(name);digest=hashlib.sha256(data).hexdigest()
            require(source_manifest.get(name)==digest==current['sha256']==index.get(current['path']),
                    'Inherited fast helper differs from accepted immutable upstream bytes: '+name)
            matched.append(dict(relative_path=name,current_artifact=current,upstream_archive_entry=name,
                upstream_original_sha256=digest,upstream_original_bytes=len(data)))
    return dict(verified=True,certificate=identity(cert_path),upstream_archive=identity(archive),
        upstream_source_manifest=identity(manifest),matched_sources=matched,
        source_added_to_current_launch=False,current_launch_archive_modified=False,
        scope='Two reused static fast admission helpers from the accepted immutable upstream archive, matched to current files and original certificate artifact identities; not claimed to be duplicated in this robustness launch sources.zip')


def check_archives(launch,run,meta):
    record=read(launch/'launch.json')
    require(all(record.get(k) is True for k in ('complete','runtime_closed','sources_unchanged',
        'runtime_binaries_unchanged','launch_inputs_unchanged')),'Incomplete/changing fast launch')
    require(record.get('exit_code')==0 and record.get('execution_scenario')==meta['task']['scenario']
        and record.get('run_directories')==[str(run.resolve())],'Fast launch/run lifecycle differs')
    sources,binaries=qualification.archive_integrity(launch,record)
    cfg,protocol,startup=(read(launch/name) for name in ('task.json','protocol.json','startup_manifest.json'))
    require(validate_task_config(cfg)==cfg==record['task']==read(run/'controller_task.json')==meta['task'],'Actual fast task differs')
    require(protocol==record['protocol'] and cfg['duration_s']==protocol['duration_s']==24.,
            'Complete frozen 24 s fast required')
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
        'Qualified fast inputs differ from execution')
    proof['upstream_shared_source_proof']=upstream_shared_source_proof(cfg)
    # The phase-law implementation and fast admission are both archived.
    require(all(name in sources for name in (
        'simulations/x650_cpp_robustness_v1/task.py','simulations/x650_cpp_robustness_v1/qualification.py',
        'simulations/x650_cpp_robustness_v1/run_case.py','simulations/x650_cpp_robustness_v1/launch.py',
        'simulations/x650_cpp_robustness_v1/disturbance.py',
        'ros2_ws/src/flyarm_state_cpp/include/flyarm_state_cpp/source_core.hpp',
        'ros2_ws/src/flyarm_mpc_cpp/include/flyarm_mpc_cpp/worker_contract.hpp')),
        'Fast execution source archive omits dynamic contract/law')
    return cfg,protocol,startup,record,proof


def endpoint_windows(run,start,cfg,metric=None):
    if metric is None:
        from simulations.x650_wholebody_mpc_v1.recording import native_tip_metrics
        metric=native_tip_metrics
    require(cfg['duration_s']==24. and cfg['trajectory_parameters']=={k:v for k,v in qualification.fast_trajectory().items()
        if k not in {'kind','duration_s'}},'Endpoint metrics require the frozen fast definition')
    target=qualification.figure8(start['target_position_enu']);limits=cfg['ts20_protocol']
    result={}
    for name,interval in [('endpoint',(0.,24.)),('tracking_endpoint',(2.,20.)),('tail_endpoint',(20.,24.))]:
        values=metric(run,target.sample,start['origin_s'],interval)
        require(values.get('same_time') is True and values.get('no_phase_alignment') is True
            and tuple(values.get('interval_task_s',()))==interval
            and np.isfinite(values['duration_s']) and values['duration_s']>=interval[1]-interval[0]-.04,
            'Fast endpoint metric lacks complete same-time window: '+name)
        require(all(np.isfinite(values[k]) and 0<=values[k]<=limits[limit] for k,limit in
            [('position_rmse_m','tip_rms_max_m'),('position_max_m','tip_peak_max_m')]),
            'Fast endpoint envelope exceeded: '+name)
        result[name]=values
    return result


def audit(launch, run, out):
    report=dict(kind='x650_cpp_robustness_stage_certificate_v1',stage='fast',passed=False,complete=False,
        certificate=False,physical_acceptance=False,hardware_qualified=False,launch_directory=str(launch),run_directory=str(run),source_run_directory=str(run),
        auditor_version='robustness_source_proof_v2',auditor_identity=identity(Path(__file__)))
    artifacts=[]
    try:
        # Even a failed audit retains the hashes of its exact original evidence.
        paths={p for base in (launch,run) for p in base.rglob('*') if p.is_file()}
        paths.add(Path(__file__).resolve())
        paths.update(ROOT/name for name in ('scripts/audit_x650_cpp_robustness.py','scripts/audit_x650_cpp_fast.py','scripts/audit_x650_cpp_hold.py','scripts/report_x650_cpp_live.py',
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
        artifacts=[identity(p) for p in sorted(paths)]
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
        require(manifest['complete'] is True and manifest['sha256']==qualification.sha(run/'native_trace.csv'), 'Incomplete native physical trace')
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
        require(all(identity(v['path'])==v for v in artifacts), 'Original evidence changed while auditing')
        report.update(passed=True,complete=True,certificate=True,physical_acceptance=True,
            full_window_ulog_verified=True,parameter_restore_verified=True,native_ground_verified=True,actuator_log_verified=True,
            scope='One complete original 24 s physical-robustness fast figure-eight; fixed same-time/world target and original controller; no delay injection, fitting, phase alignment, hardware qualification or worst-case timing guarantee')
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
