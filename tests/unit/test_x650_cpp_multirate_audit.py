"""Counterexamples for the new timing checks; original evidence is never retimed."""
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
import audit_x650_cpp_multirate as a


def held_fixture():
    c=dict(nodes=15,node_dt=.04,max_dt=.1,command_hold_s=.16,feasibility_tolerance=2e-7,
        bus_period=.005,integration_step=.001,motion=[.01,.02,.01,0.,.012,.02,.001,.1],
        kp=[6.,6.],kd=[.25,.25],velocity_limit=[.3,.3,.2,.45,.35],
        q_lower=[-.75,-.35],q_upper=[.45,.85])
    s=SimpleNamespace(task_time_s=1.,dt_s=.044,control_time_s=11.,joint_time_s=10.998,
        q=[0.,0.],dq=[0.,0.],reference_time_s=.95,reference_r=[0.,0.],
        command_count=1,command_time_s=[.9],command_velocity=[0.,0.],bus_origin_s=0.,
        packet_count=1,packet_send_time_s=[.995],packet_q_ref=[0.,0.],packet_dq_ref=[0.,0.],
        packet_tau_ff=[0.,0.],packet_kp=[6.,6.],packet_kd=[.25,.25])
    fixture=a.interval_fixture(s,c,[0.]*5);plan=np.zeros((15,5))
    pp=a.interval.centre_packets(fixture,plan);end=s.task_time_s+.16
    packets=[p for i,p in enumerate(pp) if i<s.packet_count or p['send_time']+c['motion'][4]<=end]
    offsets=np.cumsum([s.dt_s]+[c['node_dt']]*14)
    queries=sorted(set([s.task_time_s,end,*[s.task_time_s+v for v in offsets if v<.16],
        *[p['send_time']+c['motion'][4] for p in pp if s.task_time_s<=p['send_time']+c['motion'][4]<=end]]))
    begin=s.task_time_s-(s.control_time_s-s.joint_time_s)
    boundaries=sorted(set([begin,*queries,*[p['send_time']+c['motion'][4] for p in pp
        if begin<p['send_time']+c['motion'][4]<=end]]))
    count=sum(max(1,int(np.ceil(dt/c['integration_step']-1e-12))) for dt in np.diff(boundaries) if dt>0)
    wire=[dict(send_time_s=p['send_time'],**{k:v for k,v in p.items() if k!='send_time'}) for p in packets]
    g=dict(required=True,checked=True,passed=True,start_task_s=1.,end_task_s=end,duration_s=.16,
        source_control_time_s=s.control_time_s,source_joint_time_s=s.joint_time_s,
        command=[0.,0.],reference_initial=[0.,0.],reference_final=[0.,0.],envelope=[0.]*8,
        actual_history_packet_count=1,substeps=count,packets=wire,query_times=queries,
        boundary_times=boundaries,boundary_states=[0.]*(4*len(boundaries)))
    return s,c,dict(plan=plan.ravel().tolist(),hold_guard=g)


def tick_fixture():
    issue=1_000_000_000;source=SimpleNamespace(sequence=1,issued_monotonic_ns=issue,control_time_s=10.)
    proposal={'owning_candidate':'original bytes'};ticks=[];sends=[]
    for i,age in enumerate((50_000_000,70_000_000),1):
        at=issue+age
        ticks.append(dict(sequence=i,session_id=4,epoch=1,status=1,output_valid=True,
            command_sequence=1,command_issued_monotonic_ns=issue,command_age_ns=age,
            actual_monotonic_ns=at,scheduled_monotonic_ns=at,source_received_monotonic_ns=at-1_000_000,
            source_sequence=i,repeated=i>1,command=proposal.copy()))
        sends.append(dict(stop_zero=False,dispatch_sequence=i,command_sequence=1,
            sent_monotonic_ns=at+1000,repeated=i>1,sim_s=10.+age/1e9))
    return ticks,sends,[(source,proposal,{})],dict(session_id=4,epoch=1)


class HoldGuardTests(unittest.TestCase):
    def test_nonbus_aligned_prediction_nodes_remain_in_native_grid(self):
        s,c,d=held_fixture();result=a.check_hold_guard(s,c,d)
        self.assertTrue(result['verified'])
        self.assertIn(1.044,d['hold_guard']['query_times'])
        self.assertNotIn(1.044,[p['send_time_s']+c['motion'][4] for p in d['hold_guard']['packets']])

    def test_shortened_guard_fabricated_history_encoding_and_wrong_grid_reject(self):
        for case in ('duration','history','encoding','node','envelope','reference','source_time'):
            s,c,d=held_fixture();g=d['hold_guard']
            if case=='duration':g['duration_s']=.05
            elif case=='history':g['actual_history_packet_count']=0
            elif case=='encoding':g['packets'][-1]['dq_ref'][0]=.001
            elif case=='node':g['query_times'].remove(1.044)
            elif case=='envelope':g['envelope'][6]=.421
            elif case=='reference':g['reference_initial'][0]=.001
            else:g['source_control_time_s']+=.001
            with self.subTest(case=case),self.assertRaises(ValueError):a.check_hold_guard(s,c,d)


class MultiRateTickTests(unittest.TestCase):
    def test_repetition_retains_original_issue_under_explicit_new_lease(self):
        t,s,c,start=tick_fixture()
        result=a.check_ticks(t,s,c,start,convert=lambda x:x,source_controls={1:10.05,2:10.07})
        self.assertEqual(result['command_age_ms']['maximum'],70.)
        self.assertEqual(result['original_absolute_period_ns'],20_000_000)

    def test_wall_or_physical_expiration_renewed_issue_and_stale_source_reject(self):
        for case in ('wall','physical','renewal','source','slot'):
            t,s,c,start=tick_fixture()
            clocks={1:10.05,2:10.07}
            if case=='wall':
                t[1]['actual_monotonic_ns']=c[0][0].issued_monotonic_ns+161_000_000
                t[1]['command_age_ns']=161_000_000
            elif case=='physical':clocks[2]=10.160001
            elif case=='renewal':t[1]['command_issued_monotonic_ns']+=20_000_000
            elif case=='source':t[1]['source_received_monotonic_ns']-=25_000_001
            else:t[1]['scheduled_monotonic_ns']+=20_000_000
            with self.subTest(case=case),self.assertRaises(ValueError):
                a.check_ticks(t,s,c,start,convert=lambda x:x,source_controls=clocks)

    def test_lagging_writer_observation_is_not_the_tick_physical_clock(self):
        t,s,c,start=tick_fixture();s[0]['sim_s']=9.988
        result=a.check_ticks(t,s,c,start,convert=lambda x:x,source_controls={1:10.004,2:10.024})
        self.assertEqual(result['writer_observation_precedes_candidate'],1)
        self.assertAlmostEqual(result['writer_observation_to_candidate_ms']['minimum'],-12.)
        self.assertAlmostEqual(result['physical_hold_age_ms']['maximum'],24.)

    def test_missing_original_clock_or_negative_same_clock_age_rejects(self):
        t,s,c,start=tick_fixture()
        for clocks in (None,{}, {1:10.05},{1:9.988,2:10.024}):
            with self.subTest(clocks=clocks),self.assertRaises(ValueError):
                a.check_ticks(t,s,c,start,convert=lambda x:x,source_controls=clocks)


def handoff_fixture():
    def row(time,phase,value):
        result=dict(phase=phase,sim_s=time,t_s=time-10. if phase=='measure' else -1.,
            pva_sent='True',sent_coordinate_frame=1,sent_type_mask=2048,sent_yaw_ned=0.)
        for part in ('position','velocity','acceleration'):
            for axis in 'ned':result['sent_'+part+'_target_'+axis]=value if part=='velocity' else 0.
        return result
    legacy=[row(9.972,'settle',.001),row(10.,'settle',.002)]
    derived=[row(10.056+.02*i,'measure',.003) for i in range(1497)]
    received=np.vstack((a.pva_values([legacy[0]]),a.pva_values([legacy[1]]),a.pva_values([derived[0]])))
    return legacy+derived,derived,dict(origin_s=10.,history_count=1,history_time_s=[10.]),\
        np.asarray([9.976,10.004,10.060]),received


class HandoffCoverageTests(unittest.TestCase):
    def test_real_prestream_and_received_hold_cover_initial_56ms_without_relabelling(self):
        control,derived,start,times,received=handoff_fixture();original=deepcopy(control)
        with patch.object(a,'trajectory_matrix',return_value=(times,received)):
            bridge=a.check_handoff_bridge(control,derived,start,object())
        selected,t=a.selected_rows(control,[10.,40.],handoff=bridge)
        self.assertEqual(len(selected),1497)
        self.assertAlmostEqual(t[0]-10.,.056)
        self.assertAlmostEqual(bridge['initial_received_value_span_s'],.060)
        self.assertFalse(bridge['legacy_rows_relabelled_as_mpc'])
        self.assertFalse(bridge['timestamps_changed'])
        self.assertEqual(control,original)

    def test_missing_actual_boundary_context_or_mismatched_ulog_payload_rejects(self):
        for case in ('actual_boundary','context','legacy_value','cpp_value','history'):
            control,derived,start,times,received=handoff_fixture()
            if case=='actual_boundary':control[1]['sim_s']=9.999
            elif case=='context':times=times[1:];received=received[1:]
            elif case=='legacy_value':received[1,3]=.123
            elif case=='cpp_value':received[2,3]=.123
            else:start['history_time_s']=[9.999]
            with self.subTest(case=case),patch.object(a,'trajectory_matrix',return_value=(times,received)),self.assertRaises(ValueError):
                a.check_handoff_bridge(control,derived,start,object())

    def test_bridge_does_not_waive_original_50ms_internal_or_final_coverage(self):
        control,derived,start,times,received=handoff_fixture()
        with patch.object(a,'trajectory_matrix',return_value=(times,received)):
            bridge=a.check_handoff_bridge(control,derived,start,object())
        for rows in (control[:2]+derived[:5]+derived[8:],control[:2]+derived[:-5]):
            with self.assertRaises(ValueError):a.selected_rows(rows,[10.,40.],handoff=bridge)
        with self.assertRaises(ValueError):a.selected_rows(control,[10.,40.],handoff={})


if __name__=='__main__':unittest.main()
