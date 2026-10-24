"""Read-only ordinary-figure-eight admission from archived hold and new software evidence.

Historical binaries/sources are verified in their archives. They are not required
to remain the active worker: the new dynamic worker needs its own ROS proof.
"""
from pathlib import Path
import hashlib
import math
import zipfile

import numpy as np

from simulations.x650_cpp_live_v1 import qualification as hold

Evidence, read, sha, identity = hold.Evidence, hold.read, hold.sha, hold.identity
require, same_artifact, finite_bounded = hold.require, hold.same_artifact, hold.finite_bounded


def normal_trajectory():
    from .task import trajectory
    return trajectory()


def figure8(center):
    from simulations.x650_wholebody_mpc_v1.run_case import FigureEight
    return FigureEight(np.asarray(center, dtype=float), **{k:v for k,v in normal_trajectory().items()
        if k not in {'kind','duration_s'}})


def archive_integrity(launch, record):
    """Archived bytes remain authoritative after a legitimate source upgrade."""
    sources=read(launch/'source_manifest.json')
    require(sources==read(launch/'source_manifest_after.json'), 'Execution sources changed')
    with zipfile.ZipFile(launch/'sources.zip') as archive:
        require(len(archive.namelist())==len(sources) and set(archive.namelist())==set(sources),
                'Source archive membership differs')
        for name,digest in sources.items():
            require(hashlib.sha256(archive.read(name)).hexdigest()==digest,'Archived source changed: '+name)
    before=read(launch/'runtime_binary_manifest.json');after=read(launch/'runtime_binary_manifest_after.json')
    require(len({v['path'] for v in before})==len(before)
        and {v['path']:v['sha256'] for v in before}==after==record['runtime_binaries_sha256'],
        'Runtime identities differ')
    with zipfile.ZipFile(launch/'runtime_binaries.zip') as archive:
        require(len(archive.namelist())==len(before) and set(archive.namelist())=={v['archive_entry'] for v in before},
                'Runtime archive membership differs')
        for item in before:
            data=archive.read(item['archive_entry'])
            require(len(data)==item['bytes'] and hashlib.sha256(data).hexdigest()==item['sha256'],
                    'Archived binary changed')
    return sources,after


def verify_hold_gate(binding, baseline_binding):
    evidence=Evidence()
    # Do not recursively require historical worker source paths to equal the new
    # source. The certificate's original launch/run archive files are the proof.
    cert=read(evidence.artifact(binding));baseline=read(evidence.artifact(baseline_binding))
    require(cert.get('kind')=='x650_cpp_live_complete_hold_certificate_v1'
        and all(cert.get(k) is True for k in ('passed','complete','certificate','physical_acceptance',
            'full_window_ulog_verified','parameter_restore_verified','native_ground_verified')),
        'A complete independently accepted C++ hold is required')
    window=cert.get('window_sim_s',[])
    require(len(window)==2 and all(type(x) in (int,float) and math.isfinite(x) for x in window)
        and abs(window[1]-window[0]-30.)<=1e-9, 'Prior hold is not a complete 30 s window')
    require(baseline.get('stage')=='tip_hold' and baseline.get('controller')=='mpc'
        and baseline.get('duration_s')==30. and 'numerical_backend' not in baseline,
        'Prior task is not the complete C++ hold')
    artifacts=cert.get('artifacts',[])
    require(isinstance(artifacts,list) and artifacts, 'Prior hold has no original artifacts')
    original={}
    for item in artifacts:
        path=evidence.artifact(item)
        require(str(path) not in original,'Repeated prior hold artifact')
        original[str(path)]=item
    launch,run=Path(cert['launch_directory']),Path(cert['run_directory'])
    required=[launch/n for n in ('launch.json','task.json','protocol.json','startup_manifest.json',
        'source_manifest.json','source_manifest_after.json','sources.zip','runtime_binary_manifest.json',
        'runtime_binary_manifest_after.json','runtime_binaries.zip')]
    required += [run/n for n in ('controller_metadata.json','controller_task.json','native_trace.csv','px4.ulg',
        'isolated_live/lifecycle.json','isolated_live/live_start.json','ts20_state.csv','diagnostic_parameters_restored.json')]
    require(all(str(p.resolve()) in original for p in required),'Prior certificate omits essential original evidence')
    record=read(launch/'launch.json');meta=read(run/'controller_metadata.json')
    require(all(record.get(k) is True for k in ('complete','runtime_closed','sources_unchanged',
        'runtime_binaries_unchanged','launch_inputs_unchanged')) and record.get('exit_code')==0
        and record.get('execution_scenario')=='x650_cpp_live_v1'
        and record.get('run_directories')==[str(run.resolve())], 'Prior hold launch did not complete')
    require(all(meta.get(k) is True for k in ('complete','landed_and_disarmed','diagnostic_parameters_restored'))
        and meta.get('diagnostic_restore_pending') is False, 'Prior hold recovery incomplete')
    require(baseline==read(launch/'task.json')==record['task']==read(run/'controller_task.json')==meta['task']
        and sha(launch/'task.json')==baseline_binding['sha256'], 'Prior hold task binding differs')
    source,binaries=archive_integrity(launch,record)
    protocol=read(launch/'protocol.json');startup=read(launch/'startup_manifest.json')
    require(protocol==record['protocol'] and protocol.get('trajectory_kind')=='hold'
        and protocol.get('tip_hold_duration_s')==30. and protocol.get('consumption_policy')=='latest_value_v1'
        and startup['trajectory']=={'kind':'hold'}, 'Prior execution is not the accepted streaming hold')
    return dict(verified=True,certificate=identity(evidence.artifact(binding)),baseline_task=identity(evidence.artifact(baseline_binding)),
        launch_directory=str(launch),run_directory=str(run),window_sim_s=window,
        startup=startup,protocol=protocol,runtime_binaries=binaries,source_manifest=source,
        original_artifact_count=len(original),historical_scope='Archived old execution; not proof of the replacement worker binary')


def validate_protocol(protocol, prior):
    require(protocol.get('kind')=='x650_cpp_normal_protocol_v1' and protocol.get('simulation_only') is True
        and protocol.get('trajectory_kind')=='normal_figure8' and protocol.get('trajectory')==normal_trajectory()
        and protocol.get('duration_s')==32. and protocol.get('fast_admitted') is False
        and protocol.get('required_prior_stage')=='complete_cpp_tip_hold', 'Frozen normal execution protocol differs')
    changes={'kind','trajectory_kind','trajectory','duration_s','required_prior_stage','fast_admitted','claims_excluded'}
    require({k:v for k,v in protocol.items() if k not in changes}=={k:v for k,v in prior.items() if k not in changes},
            'Normal altered original execution timing, allocation, physics or protection')
    require(set(protocol.get('claims_excluded',[]))==set(prior.get('claims_excluded',[]))|{'software_replay_as_new_physical_normal'},
            'Normal qualification claims changed')


def verify_reference(report, startup, evidence):
    require(report.get('kind')=='cpp_normal_reference_preview_v1' and report.get('passed') is True
        and report.get('complete') is True and report.get('trajectory')==normal_trajectory(),
        'Independent normal target/preview proof incomplete')
    errors=report.get('maximum_absolute_error',{})
    expected={'position','velocity','acceleration','preview_position','preview_velocity','preview_acceleration'}
    require(set(errors)==expected and all(finite_bounded(x,2e-12) for x in errors.values()),
            'Independent current/preview reference errors invalid')
    require(report.get('case_count',0)>0 and report.get('physical_acceptance') is False,
            'Reference cases/scope missing')
    evidence.artifact(report['binary'])
    sources=report.get('source_artifacts',[])
    require(sources, 'Reference proof lacks bound source files')
    for item in sources:evidence.artifact(item)
    source_hashes={v['sha256'] for v in startup['source_artifacts']}
    require(any(v['path'].endswith('source_core.hpp') and v['sha256'] in source_hashes for v in sources),
            'Reference law differs from the actual shared C++ source')
    cases=evidence.document(report['cases']);actual=np.loadtxt(evidence.artifact(report['actual_output']))
    require(len(cases)==report['case_count'] and report.get('time_tolerance')==0.
        and report.get('pva_tolerance')==2e-12,'Reference corpus/count/tolerances differ')
    expected_rows=[];times=[];steps=[]
    for i,c in enumerate(cases):
        require(c['kind'] in (0,1) and c['nodes']==startup['config']['nodes']
            and c['node_dt']==startup['config']['node_dt'],'Reference shape differs from worker')
        target=figure8(c['center']);offset=0.
        for node in range(-1,c['nodes']):
            if node>=0:offset+=c['dt'] if node==0 else c['node_dt']
            at=c['t']+offset
            value=np.concatenate(target.sample(at)) if c['kind'] else np.r_[c['center'],np.zeros(6)]
            expected_rows.append([i,node,at,*value])
        if c['kind']==1:times.append(c['t']);steps.append(c['dt'])
    expected_rows=np.asarray(expected_rows)
    require(actual.shape==expected_rows.shape and np.isfinite(actual).all()
        and len(actual)==report['target_sample_count'] and np.array_equal(actual[:,:3],expected_rows[:,:3]),
        'Original compiled reference output case/node/time differs')
    for prefix,mask in [('',actual[:,1]==-1),('preview_',actual[:,1]!=-1)]:
        for i,name in enumerate(('position','velocity','acceleration')):
            value=float(np.max(abs(actual[mask,3+3*i:6+3*i]-expected_rows[mask,3+3*i:6+3*i])))
            require(value<=2e-12 and abs(value-errors[prefix+name])<=1e-15,
                    'Compiled reference output differs from independent original law')
    require(set(steps)>={.012,.016,.020,.024} and min(times)<=0. and max(times)>=32.
        and all(b in times for b in (2.,4.,26.,28.)),'Reference transition/actual-dt coverage incomplete')


def verify_numerical(report,startup,evidence):
    require(report.get('kind')=='cpp_normal_numerical_qualification_v1'
        and report.get('trajectory')==normal_trajectory(), 'Normal mathematical proof required')
    total=report.get('total')
    require(type(total) is int and total>0, 'No dynamic mathematical cases')
    if 'original_numerical_proof' in report:
        original=evidence.document(report['original_numerical_proof'])
        require(original.get('kind')=='cpp_owning_core_equivalence_v1'
            and all(report.get(k)==v for k,v in original.items() if k!='kind'),
            'Normal derived report altered original mathematical proof')
        source=report.get('center_source',{})
        require(source.get('json_pointer')=='/input/target/position','Unsupported independent centre source')
        center_case=evidence.document(source['artifact'])
        require(center_case['input']['t']==0. and report.get('trajectory_center')==center_case['input']['target']['position'],
                'Normal centre is not the original zero-time fixture target')
    hold.verify_numerical(dict(report,kind='cpp_owning_core_equivalence_v1'),total,startup,evidence,matrices=True)
    corpus=evidence.document(report['cases']);times=[];steps=[];centers=[]
    if 'original_numerical_proof' in report:
        require(same_artifact(report['center_source']['artifact'],corpus['rows'][0]),
                'Independent centre is not the first original numerical case')
    for entry in corpus['rows']:
        row=evidence.document(entry);raw=row['input'];config=row['config']
        require(config==startup['config'], 'Dynamic numerical case changes frozen configuration')
        center=row['memory']['anchor']['target'] if 'target' in row['memory']['anchor'] else None
        # Explicit metadata records the original actual anchor target. Never infer
        # it by subtracting the requested sine from a potentially wrong target.
        center=row.get('trajectory_center',report.get('trajectory_center',center))
        require(center is not None,'Dynamic oracle omits independent trajectory centre')
        target=figure8(center);t=float(raw['t']);dt=float(raw['dt'])
        require(math.isfinite(t) and 0<dt<=.05, 'Invalid dynamic input clock')
        expected=target.sample(t)
        for name,value in zip(('position','velocity','acceleration'),expected):
            require(np.allclose(raw['target'][name],value,rtol=0,atol=2e-12),'Dynamic current target differs: '+name)
        horizon=np.cumsum([dt]+[config['node_dt']]*(config['nodes']-1))
        require(len(raw['preview'])==len(horizon),'Missing actual dynamic preview nodes')
        for preview,offset in zip(raw['preview'],horizon):
            for name,value in zip(('position','velocity','acceleration'),target.sample(t+offset)):
                require(np.allclose(preview[name],value,rtol=0,atol=2e-12),'Dynamic horizon phase/dt differs: '+name)
        times.append(t);steps.append(dt);centers.append(center)
    require(set(steps)>={.012,.016,.020,.024} and min(times)<=2. and max(times)>=28.,
            'Dynamic matrix cases lack interval/phase coverage')
    for boundary in (2.,4.,26.,28.):
        require(any(abs(t-boundary)<=.05 for t in times),'Missing C2 boundary numerical case')
    return {'verified':True,'cases':total,'times_s':times,'actual_dt_s':sorted(set(steps))}


def verify_worker(report,startup,evidence,binaries):
    require(report.get('kind')=='cpp_normal_worker_actual_ros_v1' and report.get('complete') is True
        and report.get('passed') is True and report.get('no_actuator_publishers') is True
        and report.get('physical_acceptance') is False and report.get('trajectory')==normal_trajectory(),
        'New dynamic-reference ROS worker evidence required')
    tested=evidence.document(report['startup_manifest'])
    require({k:v for k,v in tested.items() if k!='task_config'}=={k:v for k,v in startup.items() if k!='task_config'},
            'Dynamic worker tested another startup configuration/source/library')
    require(read(evidence.artifact(tested['task_config']))==read(evidence.artifact(startup['task_config'])),
            'Dynamic ROS task differs')
    require(binaries.get(str(Path(report['binary']['path']).resolve()))==report['binary']['sha256'],
            'Installed dynamic worker is not the tested binary')
    require(same_artifact(report['core_library'],startup['core_library']),'Dynamic ROS core differs')
    for name,key in [('optimization_library','optimization_library'),('predictor','predictor_library')]:
        require(same_artifact(report[name],startup[key]),'Dynamic ROS library differs: '+name)
    require(report.get('continuous_positive_steps')==160 and report.get('software_task_time_s')==[.02,3.2],
            'Dynamic ROS software coverage differs')
    sequence=evidence.document(report['sequence'])
    require(sequence.get('kind')=='cpp_normal_synthetic_sequence_v1' and sequence.get('passed') is True
        and sequence.get('complete') is True and sequence.get('physical_acceptance') is False
        and sequence.get('trajectory')==normal_trajectory() and sequence.get('config')==startup['config']
        and sequence.get('step_count')==160 and len(sequence.get('rows',[]))==160
        and sequence.get('startup_sha256')==report['startup_manifest']['sha256'],
        'Dynamic ROS input sequence binding/coverage differs')
    origin=evidence.document(dict(path=sequence['live_start_path'],sha256=sequence['live_start_sha256']))
    target=figure8(origin['target_position_enu'])
    cases=report.get('cases',[])
    expected={'dynamic_sequence_sigint','wrong_target','wrong_phase','invalid_hash_latched'}
    require(len(cases)==len(expected) and {c.get('name') for c in cases}==expected,'Dynamic ROS lifecycle/negative cases missing')
    for case in cases:
        good=case['name']=='dynamic_sequence_sigint';final=case.get('final',{});count=final.get('requests')
        require(case.get('passed') is True and case.get('child_exit')==(0 if good else 1)
            and type(count) is int and count==(160 if good else 120) and final.get('accepted')==(count if good else count-1)
            and final.get('faulted') is (not good) and final.get('no_python_gc_or_pickle') is True,
            'Dynamic ROS completion/negative latch differs')
        recorder=case.get('recorder',{})
        require(recorder.get('complete') is True and recorder.get('failed') is False and recorder.get('dropped')==0
            and recorder.get('written')==recorder.get('offered')==count and len(case.get('records',[]))==count,
            'Dynamic ROS raw recording incomplete')
        for sequence,item in enumerate(case['records'],1):
            record=evidence.document(item)
            require(record.get('kind')=='cpp_full_pipeline_call_v1' and record.get('sequence')==sequence,
                    'Dynamic native record duplicated/reordered')
        rows=case.get('rows',[])
        require(len(rows)==count and [r.get('sequence') for r in rows]==list(range(1,count+1)),
                'Dynamic ROS inputs incomplete')
        for row in rows if good else rows[:-1]:
            require(row.get('status')==0 and set(row.get('absolute_errors',{}))=={
                'base_velocity_target','joint_velocity','position_reference_enu','velocity_feedforward_enu','acceleration_feedforward_enu'}
                and all(finite_bounded(x,1e-7) for x in row['absolute_errors'].values()), 'Dynamic ROS output differs')
            require(set(row.get('diagnostic_absolute_errors',{}))=={'plan','predicted_states','predicted_tip',
                'predicted_rotations','candidate_envelope','horizon'} and
                all(finite_bounded(x,1e-7) for x in row['diagnostic_absolute_errors'].values()),
                'Dynamic ROS internal prediction/candidate differs')
            require(abs(row.get('task_time_s',float('nan'))-.02*row['sequence'])<=1e-12,
                    'Dynamic ROS software phase differs')
            for field,value in zip(('target_position','target_velocity','target_acceleration'),target.sample(row['task_time_s'])):
                require(np.allclose(row[field],value,rtol=0,atol=2e-12),'Dynamic ROS current reference differs: '+field)
            require(finite_bounded(row.get('native_callback_ms'),50),'Dynamic worker callback exceeded original 50 ms')
        if not good:
            require(rows[-1].get('intentionally_invalid') is True and rows[-1].get('status',0)!=0,
                    'Dynamic negative input did not reject')
    return {'verified':True,'actual_new_binary':report['binary'],'cases':len(cases),
        'scope':'Actual new ROS worker software evidence; old 1056 replay is historical, not this binary'}


def verify(manifest_path,*,task_path,startup_path,startup_sha256,protocol_path,binaries):
    from . import task
    evidence=Evidence();manifest=read(manifest_path)
    required={'kind','simulation_only','task_config','startup_manifest','protocol','reference_proof',
        'numerical_proof','worker_proof','transport_proof'}
    require(set(manifest)==required and manifest['kind']=='x650_cpp_normal_qualification_v1'
        and manifest['simulation_only'] is True,'Explicit normal qualification manifest required')
    for key,path in [('task_config',task_path),('startup_manifest',startup_path),('protocol',protocol_path)]:
        require(same_artifact(manifest[key],identity(path)),'Selected normal artifact differs: '+key)
    require(manifest['startup_manifest']['sha256']==startup_sha256,'Selected normal startup SHA differs')
    cfg=task.validate_task_config(read(evidence.artifact(manifest['task_config'])))
    prior=verify_hold_gate(cfg['cpp_hold_certificate'],cfg['frozen_cpp_hold'])
    startup=evidence.document(manifest['startup_manifest']);protocol=read(evidence.artifact(manifest['protocol']))
    require(startup.get('kind')=='cpp_full_pipeline_v1' and startup.get('simulation_only') is True
        and startup.get('trajectory')==normal_trajectory()
        and same_artifact(startup['task_config'],manifest['task_config']), 'Normal startup/task mismatch')
    require(startup['config']==prior['startup']['config'],'Normal controller parameters changed')
    for key in ('core_library','optimization_library','predictor_library'):
        require(same_artifact(startup[key],prior['startup'][key]),'Normal changed accepted numerical library: '+key)
    validate_protocol(protocol,prior['protocol'])
    # Unchanged source/gateway executables keep their archived hold identities;
    # only the new worker may have changed and is independently qualified below.
    for path,digest in prior['runtime_binaries'].items():
        if Path(path).name=='isolated_worker':continue
        require(binaries.get(path)==digest,'Normal changed a source/gateway/numerical binary without new qualification: '+path)
    verify_reference(evidence.document(manifest['reference_proof']),startup,evidence)
    numeric=verify_numerical(evidence.document(manifest['numerical_proof']),startup,evidence)
    require(numeric['cases']==48,'The complete frozen 48 dynamic phase/dt cases are required')
    worker=verify_worker(evidence.document(manifest['worker_proof']),startup,evidence,binaries)
    hold.verify_transport(evidence.document(manifest['transport_proof']),evidence,binaries)
    return dict(kind='cpp_normal_ground_admission_v1',ready=True,physical_normal_qualified=False,
        qualification_manifest=identity(manifest_path),task_sha256=manifest['task_config']['sha256'],
        startup_sha256=startup_sha256,protocol_sha256=manifest['protocol']['sha256'],
        prior_hold={k:prior[k] for k in ('certificate','baseline_task','window_sim_s','original_artifact_count')},
        numerical=numeric,worker=worker,verified_artifact_count=len(evidence.checked),
        verified_artifacts=list(evidence.checked.values()))
