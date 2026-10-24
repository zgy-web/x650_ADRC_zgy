"""Physical metrics use the original world/time reference; never fit a trajectory."""
from copy import deepcopy
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('fast_audit',ROOT/'scripts/audit_x650_cpp_fast.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def config():
    return dict(duration_s=24.,trajectory_parameters={k:v for k,v in audit.qualification.fast_trajectory().items()
        if k not in {'kind','duration_s'}},ts20_protocol=dict(tip_rms_max_m=.05,tip_peak_max_m=.10))


class FastMetricTests(unittest.TestCase):
    def test_full_tracking_and_tail_use_independent_native_tip(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name);start=dict(origin_s=10.,target_position_enu=[1.,-.5,2.])
            trajectory=audit.qualification.figure8(start['target_position_enu'])
            with (folder/'native_trace.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=['sim_s','valid_tip','tip_x','tip_y','tip_z']);writer.writeheader()
                for t in np.linspace(0,24,2401):
                    p=trajectory.sample(t)[0]+[.003,0,0]
                    writer.writerow(dict(sim_s=t+10,valid_tip=1,tip_x=p[0],tip_y=p[1],tip_z=p[2]))
            result=audit.endpoint_windows(folder,start,config())
            for key,interval in [('endpoint',(0.,24.)),('tracking_endpoint',(2.,20.)),('tail_endpoint',(20.,24.))]:
                self.assertEqual(result[key]['interval_task_s'],interval)
                self.assertAlmostEqual(result[key]['position_rmse_m'],.003,places=12)
                self.assertTrue(result[key]['same_time']);self.assertTrue(result[key]['no_phase_alignment'])

    def test_tail_window_independently_rejects_error_or_truncation(self):
        for mode in ('error','short','phasefit','wrongwindow','nan'):
            def metric(folder,sample,origin,interval):
                result=dict(same_time=True,no_phase_alignment=True,interval_task_s=interval,
                    duration_s=interval[1]-interval[0],position_rmse_m=0.,position_max_m=0.)
                if interval==(20.,24.):
                    if mode=='error':result['position_rmse_m']=.051
                    elif mode=='short':result['duration_s']=3.5
                    elif mode=='phasefit':result['no_phase_alignment']=False
                    elif mode=='wrongwindow':result['interval_task_s']=(0.,24.)
                    else:result['position_max_m']=float('nan')
                return result
            with self.subTest(mode=mode),self.assertRaises(ValueError):
                audit.endpoint_windows(Path('.'),dict(origin_s=10.,target_position_enu=[0,0,2]),config(),metric)

    def test_partial_run_never_produces_certificate_and_keeps_hashes(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);launch,run,out=(root/n for n in ('launch','run','out'))
            for path in (launch,run,out):path.mkdir()
            (run/'controller_metadata.json').write_text(json.dumps(dict(complete=False,error='prefix only')))
            report=audit.audit(launch,run,out)
            self.assertFalse(report['passed']);self.assertFalse(report['physical_acceptance'])
            self.assertFalse(report['certificate']);self.assertEqual(report['stage'],'fast')
            self.assertIn(str(run/'controller_metadata.json'),report['artifacts'])

    def test_normal_or_short_definition_rejected(self):
        for key,value in [('period_s',12.),('amplitude_x',.2)]:
            cfg=config();cfg['trajectory_parameters'][key]=value
            with self.assertRaisesRegex(ValueError,'frozen fast'):
                audit.endpoint_windows(Path('.'),dict(origin_s=10.,target_position_enu=[0,0,2]),cfg)

    def test_source_extension_requires_explicit_frozen_fast(self):
        import audit_x650_cpp_fast_source as source
        with self.assertRaisesRegex(ValueError,'Exact fast source trajectory'):
            source.audit_source(Path('missing'),Path('missing'),Path('missing'),{}, {},trajectory={'kind':'hold'})


class FastAuditIsolationTests(unittest.TestCase):
    def test_normal_and_hold_source_definitions_are_not_accepted_as_fast(self):
        import audit_x650_cpp_fast_source as source
        from simulations.x650_cpp_normal_v1.qualification import normal_trajectory
        for trajectory in ({'kind':'hold'}, normal_trajectory(), None):
            with self.subTest(trajectory=trajectory), self.assertRaisesRegex(ValueError,'Exact fast source trajectory'):
                source.audit_source(Path('missing'),Path('missing'),Path('missing'),{}, {},trajectory=trajectory)

    def test_fast_reference_covers_two_cycles_and_four_second_tail(self):
        target=audit.qualification.figure8([1.,-.5,2.])
        for t in (0.,1.,2.,20.,22.,24.):
            p,v,a=target.sample(t)
            np.testing.assert_allclose(p,[1.,-.5,2.],atol=1e-14)
            np.testing.assert_allclose(v,0.,atol=1e-14)
            np.testing.assert_allclose(a,0.,atol=1e-14)
        # The accepted source phase law is at its constant-speed region here.
        # Four seconds of fast motion advance half a cycle; normal would not.
        p1=audit.qualification.figure8([0.,0.,0.]).sample(6.)[0]
        p2=audit.qualification.figure8([0.,0.,0.]).sample(10.)[0]
        self.assertGreater(abs(p1[0]),.1)
        self.assertAlmostEqual(p1[0],-p2[0],places=13)
        self.assertAlmostEqual(p1[2],p2[2],places=13)

    def test_raw_source_verifier_preserves_original_safety_logic(self):
        # This is an integrity comparison, not a second implementation oracle.
        # A future safety fix must be deliberately reviewed for both stages.
        original=(ROOT/'scripts/audit_x650_cpp_normal_source.py').read_text(encoding='utf-8')
        expected=original.replace('Normal-reference','Fast-reference').replace('ordinary-figure-eight','fast-figure-eight').replace('x650_cpp_normal','x650_cpp_fast').replace('normal_trajectory','fast_trajectory').replace('normal','fast')
        self.assertEqual((ROOT/'scripts/audit_x650_cpp_fast_source.py').read_text(encoding='utf-8'),expected)


if __name__=='__main__':unittest.main()
