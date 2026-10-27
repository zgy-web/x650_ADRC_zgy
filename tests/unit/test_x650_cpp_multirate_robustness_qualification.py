"""Reject drift in changed-plant admission without importing a ROS runtime."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from simulations.x650_cpp_multirate_robustness_v1 import task, qualification as q
from simulations.x650_cpp_multirate_v1 import profile


def original(case='mass085_fast'):
    return dict(scenario='original_'+case, stage='fast', controller='mpc', duration_s=24.,
        disturbance_enabled=False, model='mass085_model' if case=='mass085_fast' else 'mass115_model',
        frozen_cpp_hold={'path':'/original/hold.json','sha256':'a'*64},
        pid_profile='/original/unchanged_pid.json',
        mpc_config=dict(nodes=15,node_dt_s=.04,max_dt_s=.05,velocity_limit=[.3,.3,.2,.45,.35]),
        trajectory_parameters={'period_s':8.,'amplitude_x_m':.25,'amplitude_z_m':.1},
        ts20_protocol=dict(kind='original',control_period_s=.02,max_control_interval_s=.05,
            max_compute_s=.05,tip_rms_max_m=.05,tip_peak_max_m=.1,tilt_max_deg=8.),
        robustness=dict(case_id=case,kind='uniform_rigid_mismatch',mass_inertia_scale=task.SCALES[case],
            physical_manifest={'path':'/original/physical.json','sha256':'b'*64},
            schedule=[],wrench_runtime=None))


class TaskTests(unittest.TestCase):
    def build(self, case='mass085_fast'):
        with patch.object(task,'original_config',side_effect=lambda case,binding:original(case)), \
             patch.object(task,'bound_document',return_value={}):
            return task.build_task_config(case,{'path':'/original/task.json','sha256':'a'*64},
                hold_certificate={'path':'/current/hold.json','sha256':'c'*64})

    def test_two_original_plants_only_and_no_wrench_or_delay(self):
        for case in task.SCENARIOS:
            with self.subTest(case=case), patch.object(task,'bound_document',return_value=original(case)), \
                 patch.object(task.robust,'validate_task_config',side_effect=deepcopy):
                self.assertEqual(task.original_config(case,{}),original(case))
                for bad in ('wrench_fast','tip_hold','mass100_fast'):
                    with self.assertRaises(ValueError):task.original_config(bad,{})
                wrong=original(case);wrong['robustness']['schedule']=[{'force_world_n':[1.5,0,0]}]
                with patch.object(task,'bound_document',return_value=wrong),self.assertRaises(ValueError):
                    task.original_config(case,{})

    def test_derivation_only_adds_matching_timing_and_keeps_plants_reference_limits(self):
        for case in task.SCENARIOS:
            cfg=self.build(case);old=original(case)
            self.assertEqual(cfg['mpc_config'],dict(old['mpc_config'],max_dt_s=.1,command_hold_s=.16))
            for name in ('model','pid_profile','robustness','trajectory_parameters','duration_s'):
                self.assertEqual(cfg[name],old[name])
            self.assertFalse(cfg['multirate']['physical_acceptance_inherited'])
            self.assertFalse(cfg['multirate']['hardware_qualified'])

    def test_no_retune_relaxed_limit_waveform_or_case_substitution(self):
        cfg=self.build()
        for case in ('gain','model','waveform','case','limit'):
            changed=deepcopy(cfg)
            if case=='gain':changed['mpc_config']['nodes']=20
            elif case=='model':changed['model']='nominal'
            elif case=='waveform':changed['robustness']['schedule']=[{}]
            elif case=='case':changed['robustness']['mass_inertia_scale']=1.15
            else:changed['ts20_protocol']['tip_peak_max_m']=.2
            with self.subTest(case=case),patch.object(task,'original_config',side_effect=lambda *args:original()), \
                 patch.object(task,'bound_document',return_value={}),self.assertRaises(ValueError):
                task.validate_task_config(changed)

    def test_protocol_only_allows_resource_reassignment(self):
        expected=dict(kind='new',cpu_roles={'source':[0],'worker':[2,4]},
            worker_thread_roles={'executor':[2],'auxiliary':[4]},cpu_scope='owned',
            effective_timing_profile=profile.effective(),delay_tests_enabled=False,output_period_s=.02)
        remap=deepcopy(expected);remap['cpu_roles']={'source':[6],'worker':[8,10]}
        remap['worker_thread_roles']={'executor':[8],'auxiliary':[10]}
        with patch.object(task,'protocol',return_value=expected):
            q.validate_protocol(remap,{})
            for changed in (dict(remap,delay_tests_enabled=True),dict(remap,output_period_s=.04)):
                with self.assertRaises(ValueError):q.validate_protocol(changed,{})


class HoldBindingTests(unittest.TestCase):
    def fixture(self, directory):
        root=Path(directory)
        def write(name,value):
            p=root/name;p.write_text(json.dumps(value));return q.identity(p)
        cfg=dict(mpc_config={'nodes':15,'max_dt_s':.1,'command_hold_s':.16})
        hold_cfg=dict(cfg,duration_s=30.,multirate={'case_id':'tip_hold'})
        hold_task=write('task.json',hold_cfg)
        artifacts=[write('native.json',{'native':True})]
        libs={k:write(role+'.json',{'role':role}) for role,k in q.LIBRARY_FIELDS.items()}
        startup=dict(config={'nodes':15,'max_dt':.1,'command_hold_s':.16},**libs)
        start=write('startup.json',startup)
        binaries={'/accepted/isolated_worker':'d'*64}
        cert=dict(kind='x650_cpp_multirate_tip_hold_certificate_v1',timing_profile=profile.NAME,
            effective_profile=profile.effective(),hardware_qualified=False,task_config=hold_task,
            window_sim_s=[10.,40.],endpoint={'position_rmse_m':.003,'position_max_m':.005},
            tilt_peak_deg=.08,artifact_identities=artifacts,artifacts={a['path']:a['sha256'] for a in artifacts},
            runtime_binaries_sha256=binaries,startup_manifest=start,
            **{k:True for k in ('passed','complete','certificate','physical_acceptance',
                'full_window_ulog_verified','parameter_restore_verified','native_ground_verified','actuator_log_verified')})
        return write('certificate.json',cert),cfg,startup,binaries,cert

    def call(self,binding,cfg,startup,binaries):
        with patch.object(q.accepted_task,'validate_task_config',side_effect=deepcopy), \
             patch.object(q.accepted_task,'baseline_config',return_value={'nominal':'same'}), \
             patch.object(task,'baseline_config',return_value={'nominal':'same'}):
            return q.verify_hold_acceptance(binding,cfg,startup=startup,binaries=binaries)

    def test_complete_same_nominal_hold_is_not_changed_plant_acceptance(self):
        with tempfile.TemporaryDirectory() as directory:
            binding,cfg,startup,binaries,_=self.fixture(directory)
            result=self.call(binding,cfg,startup,binaries)
            self.assertTrue(result['verified'])
            self.assertFalse(result['changed_plant_acceptance'])

    def test_changed_controller_library_runtime_or_missing_artifact_rejects(self):
        for case in ('controller','config','library','runtime','artifact'):
            with tempfile.TemporaryDirectory() as directory,self.subTest(case=case):
                binding,cfg,startup,binaries,cert=self.fixture(directory)
                if case=='controller':cfg['mpc_config']['nodes']=16
                elif case=='config':startup['config']['nodes']=16
                elif case=='library':startup['core_library']=dict(startup['core_library'],sha256='0'*64)
                elif case=='runtime':binaries={'/accepted/isolated_worker':'0'*64}
                else:Path(cert['artifact_identities'][0]['path']).write_text('{}')
                with self.assertRaises(ValueError):self.call(binding,cfg,startup,binaries)

    def test_incomplete_or_wrong_stage_certificate_rejects(self):
        for change in ({'complete':False},{'kind':'x650_cpp_fast_stage_certificate_v1'}, {'hardware_qualified':True}):
            with tempfile.TemporaryDirectory() as directory,self.subTest(change=change):
                binding,cfg,startup,binaries,cert=self.fixture(directory)
                cert.update(change);Path(binding['path']).write_text(json.dumps(cert));binding=q.identity(binding['path'])
                with self.assertRaises(ValueError):self.call(binding,cfg,startup,binaries)


if __name__=='__main__':unittest.main()
