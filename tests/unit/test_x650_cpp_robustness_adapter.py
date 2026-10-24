"""Robustness changes the real plant/test input, never the accepted controller."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]
from simulations.x650_cpp_robustness_v1 import task,qualification


def save(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False),encoding='utf-8')
    return {'path':str(path),'sha256':task.sha(path)}


class RobustTaskTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory();self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name)
        self.pid={'model_sha256':'a'*64,'parameters':{'MPC_XY_P':1.5,'MC_ROLL_P':6.5},'pid_parameters_sha256':'b'*64}
        self.pid_binding=save(self.root/'nominal_pid.json',self.pid)
        self.nominal=dict(scenario='x650_cpp_fast_v1',stage='fast',controller='mpc',duration_s=24.,
            model='nominal_model',pid_profile=self.pid_binding['path'],preparation_report='/original/nominal_preparation.json',
            disturbance_enabled=False,mpc_config={'nodes':15,'weight':400.},
            trajectory_parameters={k:v for k,v in task.trajectory().items() if k not in ('kind','duration_s')},
            ts20_inputs={'plant_manifest':'/original/nominal_plant.json','native_library':'/original/predictor.so'},
            ts20_protocol={'max_compute_s':.05,'tip_rms_max_m':.05,'tip_peak_max_m':.1})
        self.fast_binding=save(self.root/'fast.json',self.nominal)
        self.certificate=save(self.root/'fast_certificate.json',{'passed':True})
        self.patches=patch.object(task,'fast_config',side_effect=lambda binding:deepcopy(self.nominal))
        self.patches.start();self.addCleanup(self.patches.stop)
        self.runtime={'library':{'path':'/frozen/force.so','sha256':'c'*64}}

    def config(self,case='wrench_fast'):
        if case=='wrench_fast':
            return task.build_task_config(case,self.fast_binding,self.certificate,disturbance_binding=self.runtime)
        scale=.85 if case=='mass085_fast' else 1.15
        manifest=save(self.root/(case+'_physical.json'),dict(kind='x650_uniform_rigid_mismatch_v1',scale=scale,
            model_name='variant_'+case,model_sha256='d'*64))
        profile=dict(self.pid,model_sha256='d'*64)
        pid=save(self.root/(case+'_pid.json'),profile)
        return task.build_task_config(case,self.fast_binding,self.certificate,physical_binding=manifest,pid_binding=pid)

    def test_three_cases_preserve_controller_preparation_and_exact_fast_reference(self):
        for case,scale in (('wrench_fast',1.),('mass085_fast',.85),('mass115_fast',1.15)):
            with self.subTest(case=case):
                cfg=self.config(case)
                for key in ('stage','controller','duration_s','mpc_config','ts20_protocol','trajectory_parameters','preparation_report'):
                    self.assertEqual(cfg[key],self.nominal[key])
                self.assertEqual(cfg['robustness']['mass_inertia_scale'],scale)
                self.assertEqual(task.validate_task_config(cfg),cfg)
                self.assertEqual(cfg['disturbance_enabled'],case=='wrench_fast')
                self.assertEqual(cfg['ts20_inputs']['native_library'],self.nominal['ts20_inputs']['native_library'])
                if case!='wrench_fast':self.assertNotEqual(cfg['model'],self.nominal['model'])

    def test_task_rejects_retuning_schedule_threshold_delay_and_physical_scale_edits(self):
        cfg=self.config()
        variants=[]
        for path,value in [(('mpc_config','weight'),401.),(('ts20_protocol','max_compute_s'),.06),
                (('trajectory_parameters','period_s'),10.),(('robustness','recovery_threshold_m'),.015),
                (('robustness','recovery_hold_s'),.5),(('robustness','mass_inertia_scale'),.9)]:
            bad=deepcopy(cfg);bad[path[0]][path[1]]=value;variants.append(bad)
        bad=deepcopy(cfg);bad['robustness']['schedule'][0]['end_s']=7.7;variants.append(bad)
        bad=deepcopy(cfg);bad['delay_injection_s']=.02;variants.append(bad)
        for bad in variants:
            with self.subTest(change=bad),self.assertRaises(ValueError):task.validate_task_config(bad)

    def test_model_binding_cannot_change_pid_values_or_mix_force_and_mass_cases(self):
        cfg=self.config('mass085_fast');c=cfg['robustness']
        bad_pid=deepcopy(self.pid);bad_pid['model_sha256']='d'*64;bad_pid['parameters']['MPC_XY_P']+=.1
        bad_binding=save(self.root/'retuned_pid.json',bad_pid)
        with self.assertRaises(ValueError):
            task.build_task_config('mass085_fast',self.fast_binding,self.certificate,
                physical_binding=c['physical_manifest'],pid_binding=bad_binding)
        with self.assertRaises(ValueError):
            task.build_task_config('mass085_fast',self.fast_binding,self.certificate,
                physical_binding=c['physical_manifest'],pid_binding=c['pid_profile_binding'],disturbance_binding=self.runtime)
        Path(c['physical_manifest']['path']).write_text('{}')
        with self.assertRaises(ValueError):task.validate_task_config(cfg)

    def test_protocol_preserves_protection_and_explicitly_excludes_delay_injection(self):
        cfg=self.config();actual=task.protocol(cfg)
        old=json.loads((ROOT/'configs/experiments/x650_cpp_fast_v1.json').read_text())
        changes={'kind','required_prior_stage','robustness','delay_tests_enabled'}
        self.assertEqual({k:v for k,v in actual.items() if k not in changes},
            {k:v for k,v in old.items() if k not in changes})
        self.assertFalse(actual['delay_tests_enabled'])
        self.assertEqual(actual['robustness'],cfg['robustness'])
        for key in ('output_period_s','command_age_max_s','source_wall_age_max_s','consumption_progress_budget_ms'):
            self.assertEqual(actual[key],old[key])


class RobustQualificationTests(unittest.TestCase):
    def fixture(self,folder):
        nominal=save(folder/'nominal_task.json',{'nominal':True})
        cfg=dict(frozen_cpp_fast=nominal,cpp_fast_certificate=save(folder/'cert.json',{}),
            robustness=dict(case_id='mass115_fast',schedule=[],mass_inertia_scale=1.15))
        current=save(folder/'task.json',cfg)
        core=folder/'core.so';core.write_bytes(b'the accepted numerical library')
        base=dict(kind='cpp_full_pipeline_v1',config={'nodes':15,'weights':[400.,20.]},trajectory=task.trajectory(),
            core_library=qualification.identity(core),source_artifacts=[])
        original_startup=save(folder/'nominal_startup.json',dict(base,task_config=nominal))
        startup=save(folder/'startup.json',dict(base,task_config=current))
        protocol=save(folder/'protocol.json',task.protocol(cfg))
        nominal_protocol=save(folder/'nominal_protocol.json',{'unchanged':True})
        nominal_qualification=save(folder/'nominal_qualification.json',dict(task_config=nominal,
            startup_manifest=original_startup,protocol=nominal_protocol))
        manifest=save(folder/'qualification.json',dict(kind='x650_cpp_robustness_qualification_v1',simulation_only=True,
            task_config=current,startup_manifest=startup,protocol=protocol,nominal_qualification=nominal_qualification))
        prior=dict(startup=json.loads(Path(original_startup['path']).read_text()),certificate=cfg['cpp_fast_certificate'])
        return cfg,current,startup,protocol,manifest,prior

    def verify(self,data,binaries):
        cfg,current,startup,protocol,manifest,prior=data
        with patch.object(task,'validate_task_config',return_value=cfg), \
                patch.object(qualification,'verify_fast_gate',return_value=prior), \
                patch.object(qualification.fast,'verify',return_value={'ready':True}) as nominal_verify, \
                patch.object(task,'physical_condition',return_value=(None,{'verified':True,'scale':1.15})):
            result=qualification.verify(manifest['path'],task_path=Path(current['path']),startup_path=Path(startup['path']),
                startup_sha256=startup['sha256'],protocol_path=Path(protocol['path']),binaries=binaries)
            return result,nominal_verify.call_args

    def test_rebound_task_reuses_original_math_binary_proof_without_online_ack_change(self):
        with tempfile.TemporaryDirectory() as name:
            data=self.fixture(Path(name));binaries={'/original/isolated_worker':'f'*64}
            result,call=self.verify(data,binaries)
            self.assertTrue(result['ready']);self.assertFalse(result['physical_acceptance'])
            self.assertEqual(result['physical_condition'],{'verified':True,'scale':1.15})
            self.assertIs(call.kwargs['binaries'],binaries)
            self.assertEqual(call.kwargs['task_path'],Path(data[0]['frozen_cpp_fast']['path']))

    def test_changed_numerical_startup_or_protocol_is_rejected_even_with_fresh_hashes(self):
        for mode in ('config','trajectory','protocol'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as name:
                data=list(self.fixture(Path(name)));cfg,current,startup,protocol,manifest,prior=data
                path=Path(protocol['path'] if mode=='protocol' else startup['path'])
                document=json.loads(path.read_text())
                if mode=='config':document['config']['weights'][0]+=1
                elif mode=='trajectory':document['trajectory']['period_s']=12.
                else:document['output_period_s']=.03
                updated=save(path,document);bound=json.loads(Path(manifest['path']).read_text())
                key='protocol' if mode=='protocol' else 'startup_manifest';bound[key]=updated
                save(Path(manifest['path']),bound)
                data[3 if mode=='protocol' else 2]=updated
                with self.assertRaises(ValueError):self.verify(data,{'/original/isolated_worker':'f'*64})


if __name__=='__main__':unittest.main()
