"""Offline observation-clock and performance-separation counterexamples."""
from copy import deepcopy
from pathlib import Path
import json
import sys
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
import audit_x650_cpp_multirate_followup as a


def clock_fixture():
    sends=[];derived=[];ticks=[];sources=[]
    for i,(writer,source,candidate) in enumerate(((10.,10.,1),(10.,10.02,2),(10.04,10.02,3)),1):
        sent=1_000_000_000+i*20_000_000
        sends.append(dict(stop_zero=False,dispatch_sequence=i,command_sequence=candidate,
            sim_s=writer,sent_monotonic_ns=sent))
        derived.append(dict(phase='measure',sim_s=writer,t_s=writer-9.,original_payload=candidate))
        ticks.append(dict(sequence=i,output_valid=True,command_sequence=candidate,source_sequence=i,
            source_received_monotonic_ns=sent-2_000_000,actual_monotonic_ns=sent-1000))
        sources.append(dict(sequence=str(i),issued_ns=str(sent-3_000_000),control_s=str(source)))
    return derived.copy(),derived,sends,ticks,sources,dict(origin_s=9.)


class ObservationClockTests(unittest.TestCase):
    def test_different_candidates_under_one_writer_clock_are_retained(self):
        args=clock_fixture();before=deepcopy(args)
        view,proof,new=a.observation_value_view(*args)
        self.assertEqual([r['sim_s'] for r in new],[10.,10.02,10.04])
        self.assertEqual([r['original_payload'] for r in new],[1,2,3])
        self.assertEqual([r['selected_observation'] for r in proof['observation_bindings']],
            ['source_control','source_control','writer_actuator_state'])
        self.assertFalse(proof['actual_publication_timestamp_reconstructed'])
        self.assertFalse(proof['original_physical_timestamps_changed'])
        self.assertEqual(args,before)
        for item in proof['observation_bindings']:
            self.assertIn(item['value_match_observation_sim_s'],
                [item['writer_observation_sim_s'],item['source_control_observation_sim_s']])
        # Exercise the exact persisted view shape used by the full auditor.
        persisted=json.loads(json.dumps(dict(**proof,rows=view),allow_nan=False))
        self.assertEqual(len(persisted['rows']),3)
        self.assertEqual(len(persisted['observation_bindings']),3)

    def test_unorderable_stale_postsend_or_unknown_source_is_not_repaired(self):
        for case in ('same_clock','stale','future','missing','candidate'):
            args=clock_fixture();control,derived,sends,ticks,sources,start=args
            if case=='same_clock':
                sends[1]['sim_s']=derived[1]['sim_s']=10.;sources[1]['control_s']='10.'
            elif case=='stale':sources[1]['issued_ns']=str(ticks[1]['actual_monotonic_ns']-25_000_001)
            elif case=='future':sources[1]['issued_ns']=str(sends[1]['sent_monotonic_ns']+1)
            elif case=='missing':ticks[1]['source_sequence']=999
            else:ticks[1]['command_sequence']=999
            with self.subTest(case=case),self.assertRaises(ValueError):a.observation_value_view(*args)


class PerformanceTests(unittest.TestCase):
    def test_original_peak_failure_keeps_full_metrics_and_limits(self):
        cfg=dict(ts20_protocol=dict(tip_rms_max_m=.05,tip_peak_max_m=.1))
        def metric(run,target,origin,interval):
            return dict(same_time=True,no_phase_alignment=True,interval_task_s=interval,
                duration_s=interval[1]-interval[0],position_rmse_m=.042358,position_max_m=.185172)
        values,acceptance=a.endpoint_observations(None,dict(target_position_enu=[0,0,0],origin_s=1.),cfg,metric)
        self.assertAlmostEqual(values['endpoint']['position_max_m'],.185172)
        self.assertFalse(acceptance['passed'])
        self.assertTrue(acceptance['windows']['endpoint']['rms_passed'])
        self.assertFalse(acceptance['windows']['endpoint']['peak_passed'])
        self.assertEqual(acceptance['windows']['endpoint']['peak_limit_m'],.1)

    def test_incomplete_native_window_is_never_just_a_performance_failure(self):
        cfg=dict(ts20_protocol=dict(tip_rms_max_m=.05,tip_peak_max_m=.1))
        def metric(run,target,origin,interval):
            return dict(same_time=True,no_phase_alignment=True,interval_task_s=interval,
                duration_s=1.,position_rmse_m=.001,position_max_m=.002)
        with self.assertRaises(ValueError):
            a.endpoint_observations(None,dict(target_position_enu=[0,0,0],origin_s=1.),cfg,metric)


if __name__=='__main__':unittest.main()
