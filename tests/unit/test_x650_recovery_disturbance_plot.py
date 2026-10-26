"""Clock/cutoff regressions: a partial failed flight must not use landing data."""
import csv
from copy import deepcopy
from pathlib import Path
import struct
import sys
import tempfile
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'control_packages/x650_wholebody_v1')]
import plot_x650_recovery_disturbance as plot


class RecoveryPlotTest(unittest.TestCase):
    def packet(self,order='<'):
        raw=bytearray(7628);raw[:4]=b'\x00\x01\x00\x00' if order=='<' else b'\x00\x00\x00\x00'
        struct.pack_into(order+'H',raw,4,1)
        struct.pack_into(order+'QQQ',raw,12,123,4,379)
        raw[37:69]=bytes(range(32))
        struct.pack_into(order+'Qddd',raw,76,100000000,53.42,7.544,.02)
        return raw

    def test_original_cdr_prefix_alignment_and_endian(self):
        # Encapsulation is outside the alignment base: offset mistakes would
        # produce plausible but wrong physical cutoffs and include landing.
        for order in ('<','>'):
            got=plot.snapshot_identity(self.packet(order))
            self.assertEqual((got['session_id'],got['epoch'],got['sequence']),(123,4,379))
            self.assertEqual(got['configuration_sha256'],list(range(32)))
            self.assertAlmostEqual(got['control_time_s'],53.42)
            self.assertAlmostEqual(got['task_time_s'],7.544)

    def test_unknown_cdr_or_truncated_snapshot_rejected(self):
        for raw in (self.packet()[:-1],b'\x00\x07\x00\x00'+self.packet()[4:]):
            with self.assertRaises(ValueError):plot.snapshot_identity(raw)

    def test_failed_snapshot_wins_over_later_landing_finish(self):
        calls=[dict(sequence=378,task_time_s=7.524,error=''),
               dict(sequence=379,task_time_s=7.544,error='QP infeasible')]
        cutoff,source=plot.controlled_cutoff({'complete':False},
            {'complete':False,'task_end_sim_s':90.}, {'complete':False,'passed':False},calls,24.)
        self.assertEqual(cutoff,7.544)
        self.assertIn('rejected',source)

    def test_clean_finish_selects_declared_window(self):
        cutoff,_=plot.controlled_cutoff({'complete':True},{'complete':True},
            {'complete':True,'passed':True},[dict(task_time_s=23.98,error='')],24.)
        self.assertEqual(cutoff,24.)

    def test_incomplete_without_core_rejection_uses_last_actual_snapshot(self):
        cutoff,_=plot.controlled_cutoff({'complete':False},{'complete':False},
            {'complete':False,'passed':False},[dict(task_time_s=3.24,error='')],24.)
        self.assertEqual(cutoff,3.24)

    def test_future_landing_rows_do_not_enter_native_prefix(self):
        fields=['sim_s','valid_base','valid_joint_state','valid_tip','tip_x','tip_y','tip_z',
                'arm_joint_1_q','arm_joint_2_q','arm_joint_1_qd','arm_joint_2_qd']
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'native.csv'
            with path.open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
                for elapsed in (0.,.004,.008,.012):
                    row={name:0 for name in fields}
                    row.update(sim_s=10.+elapsed,valid_base=1,valid_joint_state=1,valid_tip=1,
                               tip_x=1000. if elapsed>.008 else .001)
                    writer.writerow(row)
            t,values=plot.native_values(path,10.,.008)
            self.assertEqual(len(t),3)
            self.assertLess(values[:,1].max(),.01)

    def test_irregular_samples_use_time_weighted_rms(self):
        t=np.array([0.,.001,1.]);error=np.array([[0.,0.,0.],[1.,0.,0.],[1.,0.,0.]])
        norm=np.linalg.norm(error,axis=1)
        got=plot.metric(t,norm,error)
        self.assertAlmostEqual(got['position_rmse_m'],np.sqrt(.9995))
        self.assertNotAlmostEqual(got['position_rmse_m'],np.sqrt(np.mean(norm**2)))


class AuditChartBindingTest(unittest.TestCase):
    def setUp(self):
        self.run=ROOT/'tmp'/'plot_test_run'
        self.inputs={name:dict(path=str(self.run/path),sha256=('%064x'%(i+1))) for i,(name,path) in enumerate(
            [('native_trace','native_trace.csv'),('actual_task','controller_task.json'),('live_start','isolated_live/live_start.json')])}
        self.metrics=dict(sample_count=6001,position_rmse_m=.004,position_max_m=.01,
                          axis_rmse_m=[.002,.001,.003],duration_s=24.)
        self.report=dict(kind='x650_cpp_multirate_wrench_certificate_v1',run_directory=str(self.run),
            passed=True,physical_acceptance=True,recovery_performance_passed=False,
            endpoint=deepcopy(self.metrics),artifacts={v['path']:v['sha256'] for v in self.inputs.values()})

    def check(self,report=None,**kwargs):
        return plot.check_audit_binding(self.report if report is None else report,self.run,self.inputs,self.metrics,
            kwargs.pop('full',True),relocated=kwargs.pop('relocated',False),
            original_run=kwargs.pop('original_run',str(self.run)),**kwargs)

    def test_multirate_exact_inputs_and_recovery_status_remain_distinct(self):
        got=self.check()
        self.assertTrue(got['passed'])
        self.assertIs(got['recovery_performance_passed'],False)

    def test_followup_performance_failure_keeps_original_input_binding(self):
        report=deepcopy(self.report)
        report.update(kind='x650_cpp_multirate_wrench_followup_report_v1',
                      passed=False,physical_acceptance=False,recovery_performance_passed=True)
        self.assertIs(self.check(report),report)
        self.assertFalse(report['passed'])
        report['artifacts'].clear()
        with self.assertRaises(ValueError):self.check(report)

    def test_legacy_certificate_fields_still_required(self):
        for kind in ('x650_cpp_recovery_robustness_stage_certificate_v1',
                     'x650_cpp_interval_robustness_stage_certificate_v1'):
            report=deepcopy(self.report);report['kind']=kind
            report['source_run_directory']=report.pop('run_directory')
            self.assertIs(self.check(report),report)
            report['run_directory']=report.pop('source_run_directory')
            with self.assertRaises(ValueError):self.check(report)

    def test_new_kind_cannot_use_legacy_field_or_unrelated_same_named_run(self):
        report=deepcopy(self.report);report['source_run_directory']=report.pop('run_directory')
        with self.assertRaises(ValueError):self.check(report)
        report=deepcopy(self.report);report['run_directory']=str(ROOT/'unrelated'/self.run.name)
        with self.assertRaises(ValueError):self.check(report)

    def test_changed_or_missing_chart_hash_is_rejected(self):
        for name,item in self.inputs.items():
            for mode in ('changed','missing'):
                report=deepcopy(self.report)
                if mode=='changed':report['artifacts'][item['path']]='f'*64
                else:report['artifacts'].pop(item['path'])
                with self.subTest(name=name,mode=mode):
                    with self.assertRaises(ValueError):self.check(report)

    def test_hold_or_unknown_certificate_cannot_authorize_wrench_plot(self):
        for kind in ('x650_cpp_multirate_tip_hold_certificate_v1','unknown'):
            report=deepcopy(self.report);report['kind']=kind
            with self.assertRaises(ValueError):self.check(report)

    def test_passed_certificate_still_needs_full_physics_and_matching_metrics(self):
        with self.assertRaises(ValueError):self.check(full=False)
        report=deepcopy(self.report);report['physical_acceptance']=False
        with self.assertRaises(ValueError):self.check(report)
        for field,value in (('sample_count',6000),('position_max_m',.009),('duration_s',23.99)):
            report=deepcopy(self.report);report['endpoint'][field]=value
            with self.subTest(field=field):
                with self.assertRaises(ValueError):self.check(report)

    def test_failed_prefix_is_not_granted_acceptance(self):
        report=deepcopy(self.report);report.update(passed=False,physical_acceptance=False)
        report.pop('endpoint')
        self.assertIs(self.check(report,full=False)['physical_acceptance'],False)

    def test_relocated_archive_preserves_original_relation_and_unique_hashes(self):
        original='/root/runs/'+self.run.name
        report=deepcopy(self.report);report['run_directory']=original
        report['artifacts']={original+'/'+str(Path(v['path']).relative_to(self.run)).replace('\\','/'):v['sha256']
                             for v in self.inputs.values()}
        self.assertIs(self.check(report,relocated=True,original_run=original),report)
        report['artifacts']['/other/'+self.run.name+'/native_trace.csv']=self.inputs['native_trace']['sha256']
        with self.assertRaises(ValueError):self.check(report,relocated=True,original_run=original)
        report['artifacts'].pop('/other/'+self.run.name+'/native_trace.csv')
        with self.assertRaises(ValueError):self.check(report,relocated=True,original_run='/different/'+self.run.name)


if __name__=='__main__':unittest.main()
