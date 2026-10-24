"""Negative admission cases; no ROS, simulation or optimizer is started."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

from simulations.x650_cpp_normal_v1 import qualification as q


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,allow_nan=False),encoding='utf-8')
    return q.identity(path)


def hold_fixture(root):
    launch,run=root/'launch',root/'run';launch.mkdir();run.mkdir()
    baseline=dict(stage='tip_hold',controller='mpc',duration_s=30.)
    task=write(launch/'task.json',baseline)
    protocol=dict(trajectory_kind='hold',tip_hold_duration_s=30.,consumption_policy='latest_value_v1')
    write(launch/'protocol.json',protocol)
    write(launch/'startup_manifest.json',dict(trajectory=dict(kind='hold'),config={}))
    write(run/'controller_task.json',baseline)
    write(run/'controller_metadata.json',dict(task=baseline,complete=True,landed_and_disarmed=True,
        diagnostic_parameters_restored=True,diagnostic_restore_pending=False))
    for name in ('native_trace.csv','px4.ulg','isolated_live/lifecycle.json','isolated_live/live_start.json',
                 'ts20_state.csv','diagnostic_parameters_restored.json'):write(run/name,{})
    source={'worker.cpp':hashlib.sha256(b'old worker').hexdigest()}
    write(launch/'source_manifest.json',source);write(launch/'source_manifest_after.json',source)
    with zipfile.ZipFile(launch/'sources.zip','w') as z:z.writestr('worker.cpp',b'old worker')
    binary=root/'old_worker';binary.write_bytes(b'old executable');bid=q.identity(binary)
    binaries={bid['path']:bid['sha256']}
    write(launch/'runtime_binary_manifest.json',[dict(bid,archive_entry='worker')])
    write(launch/'runtime_binary_manifest_after.json',binaries)
    with zipfile.ZipFile(launch/'runtime_binaries.zip','w') as z:z.writestr('worker',binary.read_bytes())
    write(launch/'launch.json',dict(complete=True,runtime_closed=True,sources_unchanged=True,
        runtime_binaries_unchanged=True,launch_inputs_unchanged=True,exit_code=0,
        execution_scenario='x650_cpp_live_v1',run_directories=[str(run.resolve())],
        task=baseline,protocol=protocol,runtime_binaries_sha256=binaries))
    cert=dict(kind='x650_cpp_live_complete_hold_certificate_v1',passed=True,complete=True,certificate=True,
        physical_acceptance=True,full_window_ulog_verified=True,parameter_restore_verified=True,native_ground_verified=True,
        window_sim_s=[10.,40.],launch_directory=str(launch),run_directory=str(run),
        artifacts=[q.identity(p) for base in (launch,run) for p in sorted(base.rglob('*')) if p.is_file()],
        software_qualification={'verified_artifacts':[dict(path=str(root/'current_worker.cpp'),sha256='a'*64)]})
    return cert,write(root/'certificate.json',cert),{k:v for k,v in task.items() if k!='bytes'}


class HoldGateTests(unittest.TestCase):
    def test_original_archives_survive_intentional_active_worker_upgrade(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);cert,bind,base=hold_fixture(root)
            (root/'current_worker.cpp').write_text('new dynamic wrapper')
            proof=q.verify_hold_gate(bind,base)
            self.assertTrue(proof['verified']);self.assertEqual(proof['window_sim_s'],[10.,40.])

    def test_incomplete_and_wrong_stage_cannot_inherit_pass(self):
        for field,value in [('complete',False),('physical_acceptance',False),('window_sim_s',[10.,39.99]),
                            ('kind','x650_isolated_live_stage_certificate_v1')]:
            with self.subTest(field=field),tempfile.TemporaryDirectory() as folder:
                root=Path(folder);cert,bind,base=hold_fixture(root);cert[field]=value
                bind=write(root/'certificate.json',cert)
                with self.assertRaises(ValueError):q.verify_hold_gate(bind,base)

    def test_missing_original_and_changed_original_rejected(self):
        for mode in ('omit','change','duplicate'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as folder:
                root=Path(folder);cert,bind,base=hold_fixture(root)
                if mode=='omit':cert['artifacts']=[v for v in cert['artifacts'] if not v['path'].endswith('px4.ulg')]
                elif mode=='duplicate':cert['artifacts'].append(cert['artifacts'][0])
                else:(root/'run/px4.ulg').write_text('changed')
                bind=write(root/'certificate.json',cert)
                with self.assertRaises(ValueError):q.verify_hold_gate(bind,base)

    def test_rehashing_corrupt_archive_does_not_change_internal_identity(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);cert,bind,base=hold_fixture(root)
            with zipfile.ZipFile(root/'launch/sources.zip','w') as z:z.writestr('worker.cpp',b'new worker')
            cert['artifacts']=[q.identity(v['path']) for v in cert['artifacts']];bind=write(root/'certificate.json',cert)
            with self.assertRaisesRegex(ValueError,'Archived source changed'):q.verify_hold_gate(bind,base)


def artifact(name):return {'path':str(Path.cwd()/name),'sha256':'a'*64}


class MemoryEvidence:
    def __init__(self,documents):self.documents=documents
    def document(self,item):return self.documents[Path(item['path']).name]
    def artifact(self,item):return Path(item['path'])


def dynamic_fixture():
    config=dict(nodes=2,node_dt=.04)
    startup={k:artifact(k) for k in ('core_library','optimization_library','predictor_library')}
    startup['config']=config
    results=[];inputs={};entries=[]
    for i,(time,dt) in enumerate(zip([2.,4.,26.,28.],[.012,.016,.020,.024])):
        center=[1.,2.,3.];target=q.figure8(center)
        def value(at):return {k:v.tolist() for k,v in zip(('position','velocity','acceleration'),target.sample(at))}
        raw=dict(t=time,dt=dt,target=value(time),preview=[value(time+dt),value(time+dt+.04)])
        entry=artifact('input_'+str(i));entries.append(entry)
        inputs['input_'+str(i)]=dict(config=config,memory={'anchor':{}},input=raw,trajectory_center=center)
        results.append(dict(index=i,pass_=True,exit_code=0,normalized_plan_peak=0.,predicted_state_peak=0.,
            predicted_tip_peak=0.,objective_relative=0.,native_compute_ms=3.,core_step_wall_ms=4.,
            independently_recomputed_quality=dict(primal_violation=0.,kkt_stationarity_inf=0.),
            matrix_peak_errors=dict.fromkeys(('A','b','H','g','C','lo','hi'),0.),
            mit_wire_fields_equal=True,independent_matrix_oracle=True))
    inputs['cases']=dict(complete=True,source_files_unchanged=True,rows=entries)
    report=dict(kind='cpp_normal_numerical_qualification_v1',trajectory=q.normal_trajectory(),complete=True,
        pass_=True,passed=4,total=4,libraries_unchanged=True,runtime_python_calls=0,cases=artifact('cases'),rows=results,
        core_library=startup['core_library'],optimization_library=startup['optimization_library'],predictor=startup['predictor_library'])
    report['pass']=report.pop('pass_')
    return startup,report,MemoryEvidence(inputs)


class DynamicNumericalTests(unittest.TestCase):
    def test_raw_dynamic_preview_and_unchanged_quality_checked(self):
        s,r,e=dynamic_fixture();self.assertTrue(q.verify_numerical(r,s,e)['verified'])

    def test_repeating_current_target_is_not_a_valid_horizon(self):
        s,r,e=dynamic_fixture();v=e.documents['input_1']['input'];v['preview']=[deepcopy(v['target'])]*2
        with self.assertRaisesRegex(ValueError,'horizon phase'):q.verify_numerical(r,s,e)

    def test_fitted_center_and_wrong_clock_do_not_pass(self):
        for field in ('trajectory_center','dt','target'):
            s,r,e=dynamic_fixture();row=e.documents['input_1']
            if field=='trajectory_center':row[field][0]+=.001
            elif field=='dt':row['input']['dt']=.02
            else:row['input']['target']['velocity'][0]+=.001
            with self.subTest(field=field),self.assertRaises(ValueError):q.verify_numerical(r,s,e)

    def test_old_hold_proof_and_relaxed_timing_rejected(self):
        s,r,e=dynamic_fixture();r['kind']='cpp_owning_core_equivalence_v1'
        with self.assertRaisesRegex(ValueError,'Normal mathematical'):q.verify_numerical(r,s,e)
        s,r,e=dynamic_fixture();r['rows'][0]['core_step_wall_ms']=50.01
        with self.assertRaisesRegex(ValueError,'50 ms'):q.verify_numerical(r,s,e)

    def test_real_matrix_and_mit_checks_are_retained(self):
        s,r,e=dynamic_fixture();r['rows'][1]['mit_wire_fields_equal']=False
        with self.assertRaisesRegex(ValueError,'MIT'):q.verify_numerical(r,s,e)


class ProtocolTests(unittest.TestCase):
    def test_all_original_timing_cpu_and_consumption_limits_preserved(self):
        root=Path(__file__).resolve().parents[2]
        original=q.read(root/'configs/experiments/x650_cpp_live_v1.json')
        normal=q.read(root/'configs/experiments/x650_cpp_normal_v1.json')
        q.validate_protocol(normal,original)
        for key,value in [('command_age_max_s',.06),('output_period_s',.03),('consumption_progress_budget_ms',50),
                          ('source_wall_age_max_s',.03),('trajectory_kind','fast')]:
            bad=deepcopy(normal);bad[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):q.validate_protocol(bad,original)


def worker_fixture(root):
    startup={k:artifact(k) for k in ('core_library','optimization_library','predictor_library')}
    startup.update(config={'nodes':2,'node_dt':.04},task_config=write(root/'task.json',{'stage':'normal'}))
    report=dict(kind='cpp_normal_worker_actual_ros_v1',complete=True,passed=True,no_actuator_publishers=True,
        physical_acceptance=False,trajectory=q.normal_trajectory(),startup_manifest=artifact('startup'),
        binary=artifact('isolated_worker'),core_library=startup['core_library'],
        optimization_library=startup['optimization_library'],predictor=startup['predictor_library'],
        continuous_positive_steps=160,software_task_time_s=[.02,3.2],sequence=artifact('sequence'),cases=[])
    docs={'startup':startup,'live_start':{'target_position_enu':[1.,2.,3.]},'sequence':dict(
        kind='cpp_normal_synthetic_sequence_v1',passed=True,complete=True,physical_acceptance=False,
        trajectory=q.normal_trajectory(),config=startup['config'],step_count=160,rows=[artifact(str(i)) for i in range(160)],
        startup_sha256=report['startup_manifest']['sha256'],live_start_path=artifact('live_start')['path'],
        live_start_sha256='a'*64)}
    law=q.figure8([1.,2.,3.])
    for name in ('dynamic_sequence_sigint','wrong_target','wrong_phase','invalid_hash_latched'):
        good=name=='dynamic_sequence_sigint';count=160 if good else 120;records=[];rows=[]
        for i in range(1,count+1):
            key=name+str(i);records.append(artifact(key));docs[key]={'kind':'cpp_full_pipeline_call_v1','sequence':i}
            if not good and i==count:rows.append(dict(sequence=i,status=1,intentionally_invalid=True));continue
            p,v,a=law.sample(i*.02)
            rows.append(dict(sequence=i,status=0,task_time_s=i*.02,target_position=p.tolist(),target_velocity=v.tolist(),
                target_acceleration=a.tolist(),native_callback_ms=10.,absolute_errors=dict.fromkeys((
                    'base_velocity_target','joint_velocity','position_reference_enu','velocity_feedforward_enu','acceleration_feedforward_enu'),0.),
                diagnostic_absolute_errors=dict.fromkeys(('plan','predicted_states','predicted_tip','predicted_rotations','candidate_envelope','horizon'),0.)))
        report['cases'].append(dict(name=name,passed=True,child_exit=0 if good else 1,
            final=dict(requests=count,accepted=count if good else count-1,faulted=not good,no_python_gc_or_pickle=True),
            recorder=dict(complete=True,failed=False,dropped=0,written=count,offered=count),records=records,rows=rows))
    return startup,report,MemoryEvidence(docs),{report['binary']['path']:report['binary']['sha256']}


class DynamicWorkerTests(unittest.TestCase):
    def test_dynamic_new_binary_and_actual_software_scope(self):
        with tempfile.TemporaryDirectory() as folder:
            s,r,e,b=worker_fixture(Path(folder));self.assertTrue(q.verify_worker(r,s,e,b)['verified'])

    def test_missing_dynamics_or_wrong_reference_cannot_inherit_pass(self):
        for defect in ('coverage','phase','target','diagnostics','negative','library','deadline'):
            with self.subTest(defect=defect),tempfile.TemporaryDirectory() as folder:
                s,r,e,b=worker_fixture(Path(folder));row=r['cases'][0]['rows'][-1]
                if defect=='coverage':r['continuous_positive_steps']=3
                elif defect=='phase':row['task_time_s']=3.0
                elif defect=='target':row['target_position'][0]+=.001
                elif defect=='diagnostics':row['diagnostic_absolute_errors']['predicted_tip']=.001
                elif defect=='negative':r['cases'][1]['rows'][-1]['status']=0
                elif defect=='library':r['optimization_library']=artifact('other')
                else:row['native_callback_ms']=50.01
                with self.assertRaises(ValueError):q.verify_worker(r,s,e,b)


if __name__=='__main__':unittest.main()
