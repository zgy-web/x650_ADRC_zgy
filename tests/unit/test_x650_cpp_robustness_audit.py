"""Fail-closed physical-wrench/recovery evidence, separate from software MPC proof."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('robustness_audit',ROOT/'scripts/audit_x650_cpp_robustness.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


SCHEDULE=[dict(start_s=7.,end_s=7.6,force_world_n=[1.5,0,0],torque_world_nm=[0,0,0]),
    dict(start_s=13.,end_s=13.6,force_world_n=[-1.5,0,0],torque_world_nm=[0,0,0]),
    dict(start_s=20.5,end_s=20.9,force_world_n=[0,0,0],torque_world_nm=[0,.04,0])]


def waveform():
    """A known rectangular three-pulse record including an offset force moment."""
    result=[];impulse=np.zeros(6);offset=np.array([.02,-.03,.01])
    for tick in range(-2,6001):
        sim=10.+tick*.004;t=tick*.004 if tick>=0 else -1.
        force=np.zeros(3);couple=np.zeros(3)
        for e in SCHEDULE:
            if e['start_s']<=t<e['end_s']:
                force=np.asarray(e['force_world_n'],float);couple=np.asarray(e['torque_world_nm'],float)
        in_task=0<=tick<6000
        if in_task:impulse+=np.r_[force,couple]*.004
        value=np.r_[sim,t,.004,float(in_task),float(bool(np.any(force) or np.any(couple))),
            force,couple,force,np.cross(offset,force)+couple,couple,offset,1.]
        result.append(dict(zip(audit.WRENCH_HEADER,(format(v,'.17g') for v in value))))
    status=dict(state='SUCCESS',ready=True,anchored=True,complete=True,success=True,rows=len(result),
        task_rows=6000,duration_s=24.,t0_sim_s=10.,max_step_dt_s=.004,max_enqueue_component_error=0.,
        first_task_sim_s=10.,last_task_sim_s=33.996,
        impulse_force_Ns_couple_Nms=impulse.tolist(),expected_impulse_force_Ns_couple_Nms=[0,0,0,0,.016,0])
    return result,status


class PhysicalWrenchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.original,cls.status=waveform()

    def check(self,records=None,status=None,origin=10.):
        return audit.wrench_waveform(self.original if records is None else records,
            self.status if status is None else status,origin=origin,schedule=SCHEDULE)

    def test_complete_known_pulses_and_com_moment_are_verified(self):
        result=self.check()
        self.assertTrue(result['verified']);self.assertEqual(result['task_rows'],6000)
        self.assertEqual([e['samples'] for e in result['events']],[150,150,100])
        np.testing.assert_allclose(result['events'][0]['actual_enqueued_impulse'],[.9,0,0,0,0,0],atol=1e-12)
        np.testing.assert_allclose(result['events'][1]['actual_enqueued_impulse'],[-.9,0,0,0,0,0],atol=1e-12)
        np.testing.assert_allclose(result['events'][2]['actual_enqueued_impulse'],[0,0,0,0,.016,0],atol=1e-12)
        self.assertIn('not a force sensor',result['semantics'])

    def test_net_zero_positive_and_negative_pulses_cannot_hide_both_missing(self):
        records=deepcopy(self.original)
        for r in records:
            if 7<=float(r['t_s'])<13.6:
                for k in ('fx_command_N','fx_enqueued_N','tx_origin_enqueued_Nm','ty_origin_enqueued_Nm','tz_origin_enqueued_Nm'):
                    r[k]='0'
                r['event_active']='0'
        with self.assertRaises(ValueError):self.check(records)

    def test_force_direction_and_free_couple_are_independently_checked(self):
        for key in ('fx_command_N','fx_enqueued_N','ty_com_enqueued_Nm','ty_origin_enqueued_Nm','r_com_world_z'):
            with self.subTest(key=key):
                records=deepcopy(self.original)
                index=next(i for i,r in enumerate(records) if float(r['t_s'])>=7.)
                records[index][key]=str(float(records[index][key])+.01)
                with self.assertRaises(ValueError):self.check(records)

    def test_missing_duplicate_or_nonfinite_physics_row_is_rejected(self):
        for mode in ('missing','duplicate','nan','too_long_step'):
            with self.subTest(mode=mode):
                records=deepcopy(self.original)
                if mode=='missing':records.pop(100)
                elif mode=='duplicate':records.insert(100,deepcopy(records[100]))
                elif mode=='nan':records[100]['fx_enqueued_N']='nan'
                else:records[100]['step_dt_s']='.005'
                with self.assertRaises(ValueError):self.check(records)

    def test_task_count_impulse_and_full_tail_cannot_be_relabelled(self):
        for field,value in [('task_rows',5999),('rows',6000),('duration_s',23.9),('state','ABORTED'),
            ('impulse_force_Ns_couple_Nms',[0,0,0,0,0,0]),('t0_sim_s',10.1),('last_task_sim_s',33.992)]:
            with self.subTest(field=field):
                status=deepcopy(self.status);status[field]=value
                with self.assertRaises(ValueError):self.check(status=status)
        records=self.original[:-100]
        with self.assertRaises(ValueError):self.check(records)

    def test_late_original_marker_requires_honest_first_task_coverage(self):
        records=deepcopy(self.original);status=deepcopy(self.status)
        for r in records:
            if 0<=float(r['t_s'])<.02:r['t_s']='-1';r['in_task']='0'
        status['task_rows']-=5;status['first_task_sim_s']=10.02
        self.assertTrue(self.check(records,status)['verified'])
        status['first_task_sim_s']=10.06
        with self.assertRaises(ValueError):self.check(records,status)


class RecoveryPerformanceTests(unittest.TestCase):
    def setUp(self):
        self.t=np.arange(6001)*.004
        self.event=dict(start_s=7.,end_s=7.6)

    def check(self,error,next_start=13.):
        return audit.recovery_metrics(self.t,error,self.event,next_start=next_start)

    def test_recovery_uses_continuous_full_one_second_and_absolute_error(self):
        error=np.full(len(self.t),.005)
        error[(self.t>=7)&(self.t<8.2)]=.025
        result=self.check(error)
        self.assertTrue(result['recovered']);self.assertAlmostEqual(result['recovery_after_end_s'],.6,places=10)
        self.assertAlmostEqual(result['confirmed_at_task_s'],9.2,places=10)
        self.assertAlmostEqual(result['pulse_peak_m'],.025)

    def test_short_in_band_excursion_does_not_count_as_recovery(self):
        error=np.full(len(self.t),.02)
        error[(self.t>=8.)&(self.t<8.5)]=.005
        result=self.check(error)
        self.assertFalse(result['recovered']);self.assertIsNone(result['recovery_after_end_s'])
        self.assertIsNone(result['confirmed_at_task_s'])

    def test_next_pulse_cuts_off_dwell_instead_of_borrowing_future_samples(self):
        error=np.full(len(self.t),.02);error[self.t>=12.5]=.005
        self.assertFalse(self.check(error)['recovered'])
        self.assertTrue(self.check(error,next_start=24.)['recovered'])

    def test_constant_position_bias_is_not_subtracted_to_create_recovery(self):
        result=self.check(np.full(len(self.t),.012))
        self.assertFalse(result['recovered']);self.assertEqual(result['pre_pulse_1s_peak_m'],.012)

    def test_already_in_band_at_withdrawal_is_zero_recovery_time(self):
        result=self.check(np.full(len(self.t),.005))
        self.assertTrue(result['recovered']);self.assertTrue(result['recovery_already_inside_band_at_end'])
        self.assertAlmostEqual(result['recovery_after_end_s'],0.,places=10)

    def test_missing_native_tail_or_gap_is_rejected(self):
        error=np.zeros(len(self.t))
        for t,e in ((self.t[:-20],error[:-20]),(np.delete(self.t,100),np.delete(error,100))):
            with self.assertRaises(ValueError):audit.recovery_metrics(t,e,self.event,next_start=13.)


class CertificateTests(unittest.TestCase):
    def test_partial_run_never_becomes_physical_robustness_certificate(self):
        with tempfile.TemporaryDirectory() as name:
            base=Path(name);launch,run,out=(base/n for n in ('launch','run','out'))
            for folder in (launch,run,out):folder.mkdir()
            (run/'controller_metadata.json').write_text(json.dumps(dict(complete=False,error='partial')))
            report=audit.audit(launch,run,out)
            self.assertFalse(report['passed']);self.assertFalse(report['certificate'])
            self.assertFalse(report['physical_acceptance']);self.assertFalse(report['hardware_qualified'])
            self.assertIn(str(run/'controller_metadata.json'),report['artifacts'])


class UpstreamStaticSourceTests(unittest.TestCase):
    def fixture(self,folder):
        prior=folder/'prior_launch';prior.mkdir()
        names=('simulations/x650_cpp_fast_v1/task.py','simulations/x650_cpp_fast_v1/qualification.py')
        manifest={}
        with zipfile.ZipFile(prior/'sources.zip','w') as archive:
            for index,name in enumerate(names):
                path=folder/name;path.parent.mkdir(parents=True,exist_ok=True)
                path.write_bytes(('accepted original helper '+str(index)+'\n').encode())
                archive.write(path,name);manifest[name]=audit.qualification.sha(path)
        (prior/'source_manifest.json').write_text(json.dumps(manifest))
        entries=[audit.identity(folder/name) for name in names]
        entries += [audit.identity(prior/name) for name in ('sources.zip','source_manifest.json')]
        certificate=dict(kind='x650_cpp_fast_stage_certificate_v1',passed=True,complete=True,
            certificate=True,physical_acceptance=True,launch_directory=str(prior),
            artifact_identities=entries,artifacts={v['path']:v['sha256'] for v in entries})
        path=folder/'accepted_certificate.json';path.write_text(json.dumps(certificate))
        return dict(cpp_fast_certificate=audit.identity(path)),names,prior,certificate

    def test_upstream_static_helpers_are_explicitly_bound_without_current_archive_relabelling(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name);cfg,names,prior,cert=self.fixture(folder)
            before=(prior/'sources.zip').read_bytes()
            with patch.object(audit,'ROOT',folder):result=audit.upstream_shared_source_proof(cfg)
            self.assertTrue(result['verified']);self.assertEqual(len(result['matched_sources']),2)
            self.assertFalse(result['source_added_to_current_launch']);self.assertFalse(result['current_launch_archive_modified'])
            self.assertEqual(before,(prior/'sources.zip').read_bytes())

    def test_changed_current_helper_or_upstream_archive_is_rejected(self):
        for mode in ('current_helper','upstream_archive'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as name:
                folder=Path(name);cfg,names,prior,cert=self.fixture(folder)
                path=folder/names[0] if mode=='current_helper' else prior/'sources.zip'
                path.write_bytes(path.read_bytes()+b'changed')
                with patch.object(audit,'ROOT',folder),self.assertRaises(ValueError):
                    audit.upstream_shared_source_proof(cfg)

    def test_source_hash_without_original_certificate_identity_is_insufficient(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name);cfg,names,prior,cert=self.fixture(folder)
            excluded=str((folder/names[0]).resolve())
            cert['artifact_identities']=[v for v in cert['artifact_identities'] if v['path']!=excluded]
            del cert['artifacts'][excluded]
            path=Path(cfg['cpp_fast_certificate']['path']);path.write_text(json.dumps(cert))
            cfg['cpp_fast_certificate']=audit.identity(path)
            with patch.object(audit,'ROOT',folder),self.assertRaises(ValueError):audit.upstream_shared_source_proof(cfg)


if __name__=='__main__':unittest.main()
