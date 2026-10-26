"""Focused new-timing admission/derivation counterexamples; no ROS or flight."""
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from simulations.x650_cpp_multirate_v1 import profile, task, qualification as q


def original():
    return dict(scenario='original_hold',stage='tip_hold',controller='mpc',duration_s=30.,
        disturbance_enabled=False,model='original_x650',pid_profile='/original/pid.json',
        mpc_config=dict(nodes=15,node_dt_s=.04,max_dt_s=.05,velocity_limit=[.3,.3,.2,.45,.35],
            q_lower=[-.75,-.35],q_upper=[.45,.85]),
        ts20_protocol=dict(kind='original_protocol',control_period_s=.02,max_control_interval_s=.05,
            max_compute_s=.05,tip_rms_max_m=.05,tip_peak_max_m=.1,tilt_max_deg=8.))


class DerivationTests(unittest.TestCase):
    def build(self):
        with patch.object(task,'original_config',side_effect=lambda *args:original()):
            return task.build_task_config('tip_hold',dict(path='/original/task.json',sha256='a'*64))

    def test_derivation_changes_timing_but_preserves_physical_limits(self):
        cfg=self.build()
        self.assertEqual(cfg['mpc_config'],dict(original()['mpc_config'],max_dt_s=.1,command_hold_s=.16))
        self.assertEqual(cfg['model'],original()['model'])
        self.assertFalse(cfg['multirate']['physical_acceptance_inherited'])

    def test_hidden_retune_model_waveform_or_extra_field_rejects(self):
        cfg=self.build()
        for change in ('limit','model','extra','profile'):
            value=deepcopy(cfg)
            if change=='limit':value['mpc_config']['velocity_limit'][3]=.8
            elif change=='model':value['model']='other_x650'
            elif change=='profile':value['timing_profile']='legacy_v1'
            else:value['new_untested_field']=True
            with self.subTest(change=change), patch.object(task,'original_config',side_effect=lambda *args:original()):
                with self.assertRaises(ValueError):task.validate_task_config(value)

    def test_hold_cannot_borrow_a_physical_certificate(self):
        with patch.object(task,'original_config',return_value=original()):
            with self.assertRaises(ValueError):
                task.build_task_config('tip_hold',{},hold_certificate=dict(path='/old/report',sha256='a'*64))

    def test_profile_rejects_a_single_unadvertised_budget_change(self):
        profile.validate(profile.effective())
        for key,value in [('command_lease_ns',170_000_000),('source_wall_age_max_ns',40_000_000),
                          ('command_hold_s',.2),('hard_realtime_qualified',True),('control_dt_max_s',.2)]:
            changed=profile.effective();changed[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):profile.validate(changed)

    def test_protocol_keeps_role_counts_and_disjoint_thread_partition(self):
        expected=dict(kind='new',cpu_roles={'source':[0],'worker':[2,4]},
            worker_thread_roles={'executor':[2],'auxiliary':[4]},cpu_scope='owned',
            effective_timing_profile=profile.effective(),output_period_s=.02)
        remap=deepcopy(expected);remap['cpu_roles']={'source':[6],'worker':[8,10]}
        remap['worker_thread_roles']={'executor':[8],'auxiliary':[10]}
        with patch.object(task,'protocol',return_value=expected):
            q.validate_protocol(remap,{})
            for changed in (dict(remap,output_period_s=.04),
                    dict(remap,cpu_roles={'source':[6],'worker':[8]}),
                    dict(remap,worker_thread_roles={'executor':[8],'auxiliary':[8,10]})):
                with self.assertRaises(ValueError):q.validate_protocol(changed,{})


class Graph:
    def __init__(self,root):self.root=root;self.documents={}
    def put(self,name,value):
        path=str(self.root/name);self.documents[path]=value
        return dict(path=path,sha256='a'*64)
    def document(self,binding):return self.documents[binding['path']]
    def artifact(self,binding):return Path(binding['path'])


def worker_graph(root):
    e=Graph(root);count=20
    path=root/'task.json';path.write_bytes(b'{"new_task":true}\n')
    receipt=dict(path=str(path),sha256=q.sha(path))
    libs={r:dict(path=str(root/(r+'.so')),sha256='b'*64) for r in q.LIBRARY_FIELDS}
    worker=dict(path=str(root/'isolated_worker'),sha256='c'*64)
    startup=dict(config=dict(nodes=15,max_dt=.1,command_hold_s=.16),trajectory={'kind':'hold'},task_config=receipt,
        **{key:libs[role] for role,key in q.LIBRARY_FIELDS.items()})
    start=e.put('startup.json',startup)
    ws=e.put('worker_startup.json',dict(core_abi=4,diagnostic_mode='compact',python_in_worker=False,
        startup_controller_steps=0,startup_manifest=deepcopy(startup),timing_profile=profile.NAME,
        effective_profile=profile.effective(),**{'loaded_'+r+'_path':v['path'] for r,v in libs.items()}))
    sources=[];original_rows=[]
    for i in range(1,count+1):
        snapshot=dict(sequence=2*i,configuration_sha256=[0]*32,task_time_s=.04*i,
            dt_s=.02,position_enu=[1.,2.,3.],actual_packet_history=[1.,2.,3.])
        source=e.put(f'original_source{i}.json',dict(snapshot=deepcopy(snapshot),sequence=2*i))
        source['sequence']=2*i;original_rows.append(source)
        snapshot.update(sequence=i,configuration_sha256=list(bytes.fromhex(receipt['sha256'])),
            dt_s=.04*i-.04*(i-1))
        derived=e.put(f'source{i}.json',dict(sequence=i,original_sequence=2*i,source_row=source,snapshot=snapshot))
        derived['sequence']=i;sources.append(derived)
    original_id=e.put('original_recording.json',dict(originals_verified_unchanged=True,rows=original_rows))
    recording=e.put('recording.json',dict(kind='x650_cpp_multirate_derived_recording_v1',
        originals_verified_unchanged=True,original_states_and_actual_history_preserved=True,
        physical_validation=False,counterfactual_closed_loop_claimed=False,timing_profile=profile.NAME,
        original_recording=original_id,config=startup['config'],trajectory=startup['trajectory'],rows=sources))
    calls=[]
    for i,source in enumerate(sources,1):
        call=e.put(f'oracle{i}.json',dict(sequence=i,returned=True,source_row=source,
            memory_before={'step_count':i-1},memory_after={'step_count':i}))
        call['sequence']=i;calls.append(call)
    oracle=e.put('oracle.json',dict(kind='x650_cpp_multirate_sequence_oracle_v1',core_abi=4,complete=True,passed=True,
        row_count=count,step_count_after=count,memory_sequentially_reconstructed_in_cpp=True,
        original_proposals_used_as_output_oracle=False,recorded_states_and_actual_history_preserved=True,
        physical_validation=False,counterfactual_closed_loop_claimed=False,recording=recording,rows=calls))
    probe=dict(kind='x650_cpp_multirate_actual_worker_probe_v1',core_abi=4,complete=True,passed=True,
        test_sequence_complete=True,rows_count=count,no_actuator_publishers=True,physical_acceptance=False,
        timing_profile=profile.NAME,effective_profile=profile.effective(),worker=worker,startup=start,
        worker_startup=ws,libraries=libs,oracle=oracle,recording=recording,
        rows=[dict(sequence=i,passed=True,native_callback_ms=26.,absolute_errors={'command':0.},
            full_native_output_absolute_errors={'command':0.},diagnostic_absolute_error=0.) for i in range(1,count+1)],
        wire_crosscheck=[dict(sequence=i,snapshot_all_fields_exact=True,proposal_all_fields_exact=True) for i in range(1,count+1)],
        records=[{}]*count,final=dict(requests=count,accepted=count,faulted=False),
        recorder=dict(complete=True,dropped=0,written=count,offered=count))
    return e,probe,startup,{worker['path']:worker['sha256']},{'worker':worker}


class WorkerProofTests(unittest.TestCase):
    def test_sequential_actual_worker_software_can_be_admitted(self):
        with tempfile.TemporaryDirectory() as directory:
            result=q.verify_worker(*self.arguments(directory))
            self.assertTrue(result['verified']);self.assertFalse(result['physical_acceptance'])

    @staticmethod
    def arguments(directory):
        e,p,s,b,c=worker_graph(Path(directory));return p,s,e,b,c

    def test_wrong_timing_abi_wire_deadline_or_memory_rejects(self):
        for case in ('timing','abi','wire','late','memory','counterfactual','output_oracle','state','dt'):
            with tempfile.TemporaryDirectory() as directory:
                p,s,e,b,c=self.arguments(directory)
                if case=='timing':p['effective_profile']['command_lease_ns']=170_000_000
                elif case=='abi':p['core_abi']=3
                elif case=='wire':p['wire_crosscheck'][3]['snapshot_all_fields_exact']=False
                elif case=='late':p['rows'][4]['native_callback_ms']=50.01
                elif case=='memory':e.document(e.document(p['oracle'])['rows'][4])['memory_before']={'step_count':0}
                elif case=='counterfactual':e.document(p['oracle'])['counterfactual_closed_loop_claimed']=True
                elif case=='output_oracle':e.document(p['oracle'])['original_proposals_used_as_output_oracle']=True
                elif case=='state':e.document(e.document(p['recording'])['rows'][0])['snapshot']['position_enu'][0]+=.001
                else:e.document(e.document(p['recording'])['rows'][0])['snapshot']['dt_s']=.05
                with self.subTest(case=case),self.assertRaises(ValueError):q.verify_worker(p,s,e,b,c)

    def test_old_or_partial_hold_certificate_cannot_unlock_wrench(self):
        for value in ({'kind':'x650_cpp_live_complete_hold_certificate_v1','passed':True},
                      {'kind':'x650_cpp_multirate_tip_hold_certificate_v1','passed':True,'complete':False}):
            with tempfile.TemporaryDirectory() as directory:
                e=Graph(Path(directory));binding=e.put('cert.json',value)
                with patch.object(q,'read',return_value=value),self.assertRaises(ValueError):
                    q.verify_hold_acceptance(binding,{},evidence=e)


if __name__=='__main__':unittest.main()
