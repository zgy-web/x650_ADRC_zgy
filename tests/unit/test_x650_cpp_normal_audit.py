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
spec=importlib.util.spec_from_file_location('normal_audit',ROOT/'scripts/audit_x650_cpp_normal.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def config():
    return dict(duration_s=32.,trajectory_parameters={k:v for k,v in audit.qualification.normal_trajectory().items()
        if k not in {'kind','duration_s'}},ts20_protocol=dict(tip_rms_max_m=.05,tip_peak_max_m=.10))


class NormalMetricTests(unittest.TestCase):
    def test_full_tracking_and_tail_use_independent_native_tip(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name);start=dict(origin_s=10.,target_position_enu=[1.,-.5,2.])
            trajectory=audit.qualification.figure8(start['target_position_enu'])
            with (folder/'native_trace.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=['sim_s','valid_tip','tip_x','tip_y','tip_z']);writer.writeheader()
                for t in np.linspace(0,32,3201):
                    p=trajectory.sample(t)[0]+[.003,0,0]
                    writer.writerow(dict(sim_s=t+10,valid_tip=1,tip_x=p[0],tip_y=p[1],tip_z=p[2]))
            result=audit.endpoint_windows(folder,start,config())
            for key,interval in [('endpoint',(0.,32.)),('tracking_endpoint',(2.,28.)),('tail_endpoint',(28.,32.))]:
                self.assertEqual(result[key]['interval_task_s'],interval)
                self.assertAlmostEqual(result[key]['position_rmse_m'],.003,places=12)
                self.assertTrue(result[key]['same_time']);self.assertTrue(result[key]['no_phase_alignment'])

    def test_tail_window_independently_rejects_error_or_truncation(self):
        for mode in ('error','short','phasefit','wrongwindow','nan'):
            def metric(folder,sample,origin,interval):
                result=dict(same_time=True,no_phase_alignment=True,interval_task_s=interval,
                    duration_s=interval[1]-interval[0],position_rmse_m=0.,position_max_m=0.)
                if interval==(28.,32.):
                    if mode=='error':result['position_rmse_m']=.051
                    elif mode=='short':result['duration_s']=3.5
                    elif mode=='phasefit':result['no_phase_alignment']=False
                    elif mode=='wrongwindow':result['interval_task_s']=(0.,32.)
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
            self.assertFalse(report['certificate']);self.assertEqual(report['stage'],'normal')
            self.assertIn(str(run/'controller_metadata.json'),report['artifacts'])

    def test_fast_or_short_definition_rejected(self):
        for key,value in [('period_s',8.),('amplitude_x',.2)]:
            cfg=config();cfg['trajectory_parameters'][key]=value
            with self.assertRaisesRegex(ValueError,'frozen normal'):
                audit.endpoint_windows(Path('.'),dict(origin_s=10.,target_position_enu=[0,0,2]),cfg)

    def test_source_extension_requires_explicit_frozen_normal(self):
        import audit_x650_cpp_normal_source as source
        with self.assertRaisesRegex(ValueError,'Exact normal source trajectory'):
            source.audit_source(Path('missing'),Path('missing'),Path('missing'),{}, {},trajectory={'kind':'hold'})


if __name__=='__main__':unittest.main()
