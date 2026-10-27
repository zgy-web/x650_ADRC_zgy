"""Failure cases for the independent, unchanged-controller mass-sweep audit."""
from copy import deepcopy
from pathlib import Path
import json
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'scripts')]
import audit_x650_cpp_multirate_robustness as a


def case_fixture(case='mass085_fast'):
    scale={'mass085_fast':.85,'mass115_fast':1.15}[case]
    cfg=dict(multirate=dict(case_id=case),robustness=dict(case_id=case,
        kind='uniform_rigid_mismatch',mass_inertia_scale=scale,schedule=[],wrench_runtime=None),
        disturbance_enabled=False,duration_s=24.,timing_profile=a.profile.NAME,
        ts20_protocol=dict(tip_rms_max_m=.05,tip_peak_max_m=.1,tilt_max_deg=8.))
    protocol=dict(delay_tests_enabled=False,duration_s=24.,effective_timing_profile=a.profile.effective())
    return cfg,protocol


class CaseTests(unittest.TestCase):
    def test_both_physical_scales_keep_original_thresholds_and_no_extra_disturbance(self):
        for case in ('mass085_fast','mass115_fast'):
            cfg,protocol=case_fixture(case)
            before=deepcopy((cfg,protocol))
            self.assertEqual(a.check_case(cfg,protocol),case)
            self.assertEqual((cfg,protocol),before)

    def test_relabelled_plant_force_delay_window_and_relaxed_limits_are_rejected(self):
        for change in ('scale','case','kind','force','runtime','enabled','delay',
                       'duration','rms','peak','tilt','timing','profile'):
            with self.subTest(change=change):
                cfg,protocol=case_fixture()
                if change=='scale':cfg['robustness']['mass_inertia_scale']=1.15
                elif change=='case':cfg['multirate']['case_id']='wrench_fast'
                elif change=='kind':cfg['robustness']['kind']='external_wrench'
                elif change=='force':cfg['robustness']['schedule']=[dict(force_world_n=[1.5,0.,0.])]
                elif change=='runtime':cfg['robustness']['wrench_runtime']={}
                elif change=='enabled':cfg['disturbance_enabled']=True
                elif change=='delay':protocol['delay_tests_enabled']=True
                elif change=='duration':cfg['duration_s']=23.
                elif change=='rms':cfg['ts20_protocol']['tip_rms_max_m']=.06
                elif change=='peak':cfg['ts20_protocol']['tip_peak_max_m']=.2
                elif change=='tilt':cfg['ts20_protocol']['tilt_max_deg']=9.
                elif change=='timing':cfg['timing_profile']='legacy'
                else:protocol['effective_timing_profile']['command_hold_s']=.2
                with self.assertRaises(ValueError):a.check_case(cfg,protocol)


class PerformanceTests(unittest.TestCase):
    def test_peak_fail_retains_all_original_windows_and_is_not_execution_exception(self):
        cfg,_=case_fixture()
        observed=[]
        def metric(run,target,origin,interval):
            observed.append(interval)
            return dict(same_time=True,no_phase_alignment=True,interval_task_s=interval,
                duration_s=interval[1]-interval[0],position_rmse_m=.042,position_max_m=.185)
        values,acceptance=a.endpoint_observations(None,dict(target_position_enu=[0,0,0],origin_s=1.),cfg,metric)
        result=a.classify_performance(acceptance,4.2,cfg['ts20_protocol'])
        self.assertEqual(observed,[(0.,24.),(2.,20.),(20.,24.)])
        self.assertEqual(set(values),{'endpoint','tracking_endpoint','tail_endpoint'})
        self.assertFalse(result['passed'])
        self.assertTrue(result['tilt_passed'])
        self.assertFalse(result['recovery_applicable'])
        self.assertEqual(values['endpoint']['position_max_m'],.185)
        json.dumps(dict(values=values,acceptance=acceptance,result=result),allow_nan=False)

    def test_tilt_failure_is_recorded_without_losing_endpoint_results(self):
        cfg,_=case_fixture()
        result=a.classify_performance(dict(passed=True),8.1,cfg['ts20_protocol'])
        self.assertFalse(result['passed']);self.assertTrue(result['endpoint_passed'])
        self.assertFalse(result['tilt_passed'])
        self.assertEqual(result['tilt_peak_deg'],8.1)
        self.assertEqual(result['tilt_limit_deg'],8.)

    def test_original_boundary_values_pass_and_nonfinite_observation_rejects(self):
        cfg,_=case_fixture()
        result=a.classify_performance(dict(passed=True),8.,cfg['ts20_protocol'])
        self.assertTrue(result['passed']);self.assertIsNone(result['performance_rejection'])
        for tilt in (float('nan'),float('inf'),-1.):
            with self.subTest(tilt=tilt),self.assertRaises(ValueError):
                a.classify_performance(dict(passed=True),tilt,cfg['ts20_protocol'])

    def test_incomplete_record_cannot_be_classified_as_ordinary_performance_failure(self):
        cfg,_=case_fixture()
        def metric(run,target,origin,interval):
            return dict(same_time=True,no_phase_alignment=True,interval_task_s=interval,
                duration_s=1.,position_rmse_m=.001,position_max_m=.002)
        with self.assertRaises(ValueError):
            a.endpoint_observations(None,dict(target_position_enu=[0,0,0],origin_s=1.),cfg,metric)


class IncompleteRunTests(unittest.TestCase):
    def test_partial_run_keeps_original_evidence_and_never_inherits_hold_acceptance(self):
        with tempfile.TemporaryDirectory() as name:
            folder=Path(name);launch,run,out=(folder/n for n in ('launch','run','out'))
            for p in (launch,run,out):p.mkdir()
            original=json.dumps(dict(complete=False,error='test incomplete stage'))
            (run/'controller_metadata.json').write_text(original)
            result=a.audit(launch,run,out)
            self.assertEqual(result['kind'],'x650_cpp_multirate_robustness_report_v1')
            for key in ('complete','passed','certificate','physical_acceptance','integrity_and_execution_passed','hardware_qualified'):
                self.assertFalse(result[key],key)
            self.assertIsNone(result['recovery_performance_passed'])
            self.assertIn(str((run/'controller_metadata.json').resolve()),result['artifacts'])
            self.assertEqual((run/'controller_metadata.json').read_text(),original)
            self.assertIn('failure_context',result)
            self.assertEqual(json.loads((out/'report.json').read_text())['error'],result['error'])


if __name__=='__main__':unittest.main()
