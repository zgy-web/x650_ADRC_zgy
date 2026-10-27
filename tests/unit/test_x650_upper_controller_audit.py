"""Counterexamples for new physical acceptance; these tests do not run physics."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'scripts'),str(Path(__file__).parent)]
import audit_x650_upper_controller as a
from test_x650_cpp_hold_audit import fixture as old_numeric_fixture
from test_x650_cpp_multirate_audit import held_fixture


def online_fixture():
    s = SimpleNamespace(issued_monotonic_ns=1_000_000_000)
    p = SimpleNamespace(compute_end_monotonic_ns=1_030_000_000)
    c = dict(feasibility_tolerance=2e-7)
    def cost(x): return dict(total=x,components=[x,0.,0.,0.,0.,0.])
    rows = [dict(index=0,accepted=True,native_passed=True,stopping_passed=True,
        native_compute_ms=4.,elapsed_ms=5.,stopping_violation=0.,objective=cost(10.),reason='first_feasible_proposal'),
        dict(index=1,accepted=True,native_passed=True,stopping_passed=True,
        native_compute_ms=4.,elapsed_ms=10.,stopping_violation=0.,objective=cost(9.),reason='nonlinear_objective_decreased')]
    online = dict(kind='x650_online_nmpc_call_v1',accepted_by_solver=True,status='ready_checked_native_candidate',
        offline_publication_flag=False,issue_age_finite=True,completed_monotonic_ns=1_020_000_000,
        call_elapsed_ms=19.,issue_age_ms=20.,iterations=rows,selected_iteration=1,
        first_objective=cost(10.),selected_objective=cost(9.),actual_stopping_violation=0.,
        nonlinear_total_ms=11.,within_proposal_budget=True)
    return dict(nmpc=online),s,p,c


def qp_fixture():
    c = dict(nodes=1,velocity_limit=[.3,.3,.2,.45,.35],feasibility_tolerance=2e-7,kkt_tolerance=5e-5)
    n,m = 5,14; C=np.zeros((m,n));C[7,3]=1.;C[8,4]=1.
    lo=-np.ones(m);hi=np.ones(m);lo[7:9]=-.02;hi[7:9]=.02
    d = dict(qp_columns=n,qp_rows=m,interval_rows=0,H=(2*np.eye(n)).ravel().tolist(),
        g=[0.]*n,C=C.ravel().tolist(),lower=lo.tolist(),upper=hi.tolist(),solution=[0.]*n,
        plan=[0.]*n,A=np.eye(n).ravel().tolist(),b=[0.]*n,
        nominal_packets=[dict(original=True)],candidate_packets=[dict(original=True)],query_times=[.04])
    return d,c


class FullEvidenceTests(unittest.TestCase):
    def test_explicit_full_record_does_not_relabel_old_compact(self):
        c,s,p,v=old_numeric_fixture();v.update(diagnostic_mode='full',full_matrix_packet_capture=True)
        before=deepcopy(v)
        self.assertEqual(a.check_numeric_result(v,s,p,c,[0.]*5).tolist(),[0.]*5)
        self.assertEqual(v,before)
        v['diagnostic_mode']='compact'
        with self.assertRaises(ValueError):a.check_numeric_result(v,s,p,c,[0.]*5)

    def test_full_capture_cannot_hide_changed_original_inner_velocity_bound(self):
        c,s,p,v=old_numeric_fixture();v.update(diagnostic_mode='full',full_matrix_packet_capture=True)
        v['result']['diagnostics']['candidate_envelope'][6]=.421
        with self.assertRaisesRegex(ValueError,'envelope'):a.check_numeric_result(v,s,p,c,[0.]*5)

    def test_independent_full_row_kkt_and_normalized_plan(self):
        d,c=qp_fixture()
        self.assertTrue(a.check_full_qp(d,c)['verified'])
        for kind in ('hard_row','trust','stationarity','plan','missing_matrix','slack'):
            bad=deepcopy(d)
            if kind=='hard_row':bad['upper'][0]=-.1
            elif kind=='trust':bad['upper'][7]=.03
            elif kind=='stationarity':bad['g'][0]=.1
            elif kind=='plan':bad['plan'][0]=.001
            elif kind=='missing_matrix':bad['H']=[]
            else:bad['qp_columns']+=1
            with self.subTest(kind=kind),self.assertRaises(ValueError):a.check_full_qp(bad,c)

    def test_full_prediction_preserves_actual_packet_prefix_and_nonbus_nodes(self):
        s,c,d=held_fixture();fixture=a.base.interval_fixture(s,c,[0.]*5)
        packets=a.base.interval.centre_packets(fixture,np.zeros((15,5)))
        h,nodes,queries,boundaries=a.base.interval.canonical_grid(fixture,packets)
        d.update(candidate_packets=[dict(send_time_s=p['send_time'],**{k:v for k,v in p.items() if k!='send_time'}) for p in packets],
            horizon=h,query_times=queries,candidate_boundary_times=boundaries,
            candidate_states=[0.]*(4*len(queries)),predicted_states=[0.]*180,linearized_states=[0.]*180,
            q_linearization_error=0.,dq_linearization_error=0.,candidate_envelope=[0.]*8)
        self.assertTrue(a.check_full_prediction(s,c,d,[0.]*5)['verified'])
        for kind in ('history','query','nodes','discrepancy'):
            value=deepcopy(d)
            if kind=='history':value['candidate_packets'][0]['dq_ref'][0]=.01
            elif kind=='query':value['query_times'][1]+=.0001
            elif kind=='nodes':value['predicted_states'][6]=.001
            else:value['q_linearization_error']=.001
            with self.subTest(kind=kind),self.assertRaises(ValueError):a.check_full_prediction(s,c,value,[0.]*5)


class NonlinearDecisionTests(unittest.TestCase):
    def test_strict_nonlinear_decrease_accepts_original_timestamps(self):
        value,s,p,c=online_fixture();before=deepcopy(value)
        self.assertEqual(a.check_online_record(value,s,p,c,'nmpc')['selected_iteration'],1)
        self.assertEqual(value,before)

    def test_decrease_selected_native_flags_and_original_deadline_are_independent(self):
        for kind in ('cost','sum','native','selected','offline_flag','age','expired','budget'):
            value,s,p,c=online_fixture();d=value['nmpc']
            if kind=='cost':d['iterations'][1]['objective']=deepcopy(d['first_objective']);d['selected_objective']=deepcopy(d['first_objective'])
            elif kind=='sum':d['iterations'][0]['objective']['components'][0]=9.
            elif kind=='native':d['iterations'][1]['native_passed']=False
            elif kind=='selected':d['selected_iteration']=0
            elif kind=='offline_flag':d['offline_publication_flag']=True
            elif kind=='age':d['issue_age_ms']=1.
            elif kind=='expired':d.update(completed_monotonic_ns=1_051_000_000,issue_age_ms=51.,call_elapsed_ms=50.);p.compute_end_monotonic_ns=1_052_000_000
            else:d['iterations'][1]['elapsed_ms']=40.1;d['nonlinear_total_ms']=41.
            with self.subTest(kind=kind),self.assertRaises(ValueError):a.check_online_record(value,s,p,c,'nmpc')

    def test_failed_late_trial_can_retain_selected_pair_but_cannot_renew_lease(self):
        value,s,p,c=online_fixture();d=value['nmpc']
        bad=deepcopy(d['iterations'][-1]);bad.update(index=2,accepted=False,native_passed=False,
            stopping_passed=False,elapsed_ms=41.,reason='native_proposal_failed: bounded trial')
        d['iterations'].append(bad)
        d.update(completed_monotonic_ns=1_045_000_000,issue_age_ms=45.,call_elapsed_ms=44.,
                 nonlinear_total_ms=42.,within_proposal_budget=False)
        p.compute_end_monotonic_ns=1_046_000_000
        self.assertTrue(a.check_online_record(value,s,p,c,'nmpc')['verified'])
        d.update(completed_monotonic_ns=1_051_000_000,issue_age_ms=51.,call_elapsed_ms=50.)
        p.compute_end_monotonic_ns=1_052_000_000
        with self.assertRaises(ValueError):a.check_online_record(value,s,p,c,'nmpc')

    def test_callable_mpc_has_own_original_issue_gate_without_nonlinear_claim(self):
        value=dict(backend='mpc_callable');_,s,p,c=online_fixture()
        self.assertTrue(a.check_online_record(value,s,p,c,'mpc_callable')['verified'])
        p.compute_end_monotonic_ns=s.issued_monotonic_ns+50_000_001
        with self.assertRaises(ValueError):a.check_online_record(value,s,p,c,'mpc_callable')


def commit_fixture():
    calls=[];sends=[];rows=[]
    for seq in (1,2):
        calls.append((SimpleNamespace(sequence=seq),SimpleNamespace(compute_end_monotonic_ns=seq*100,
            compute_start_monotonic_ns=seq*100-40),
            dict(backend='nmpc',memory_before_step_count=seq-1,selected_next_step_count=seq)))
        sends.append(dict(stop_zero=False,command_sequence=seq,dispatch_sequence=seq*2,sent_monotonic_ns=seq*100+10))
        rows.append([seq,seq*2,seq*100+10,seq*100+20,seq,0])
    ledger=dict(kind='upper_worker_actual_send_commits_v1',requires_consumer_ack=False,complete=True,
        columns=['sequence','dispatch_sequence','sent_monotonic_ns','received_monotonic_ns','committed_memory_steps','buffered_next_sequence'],
        rows=rows,discarded_unsent_sequence=0)
    final=dict(backend='nmpc',implementation='cpp_upper_nmpc_v1',actual_send_commits=2,
        committed_send_sequence=2,controller_steps=2,pending_send_sequence=0,discarded_unsent_sequence=0)
    return calls,sends,ledger,final


class ActualSendMemoryTests(unittest.TestCase):
    def check(self,fixture):
        calls,sends,ledger,final=fixture
        with patch.object(a,'read',side_effect=lambda path: ledger if path.name=='worker_commit_timing.json' else final):
            return a.check_memory_commits(Path('/fixture'),calls,sends,'nmpc')

    def test_real_send_commits_once_and_unsent_terminal_proposal_is_discarded(self):
        args=commit_fixture();self.assertEqual(self.check(args)['actual_send_commits'],2)
        calls,sends,ledger,final=args
        sends.pop();ledger['rows'].pop();ledger['discarded_unsent_sequence']=2
        final.update(actual_send_commits=1,committed_send_sequence=1,controller_steps=1,discarded_unsent_sequence=2)
        self.assertEqual(self.check(args)['discarded_unsent_sequence'],2)

    def test_early_duplicate_unmatched_or_consumer_serialized_commit_rejected(self):
        for kind in ('early','dispatch','duplicate','consumer','next_memory'):
            args=commit_fixture();calls,sends,ledger,final=args
            if kind=='early':ledger['rows'][0][3]=90
            elif kind=='dispatch':ledger['rows'][0][1]=99
            elif kind=='duplicate':ledger['rows'][1][0]=1
            elif kind=='consumer':ledger['requires_consumer_ack']=True
            else:calls[1][2]['memory_before_step_count']=0
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.check(args)

    def test_later_actual_repeat_can_commit_after_first_notification_loss(self):
        args=commit_fixture();calls,sends,ledger,final=args
        repeated=deepcopy(sends[0]);repeated.update(dispatch_sequence=3,sent_monotonic_ns=130,repeated=True)
        sends.insert(1,repeated);ledger['rows'][0][1:4]=[3,130,140]
        self.assertTrue(self.check(args)['verified'])


class PhysicalMetricTests(unittest.TestCase):
    def test_declared_windows_and_reference_periods_are_not_reused_from_fast(self):
        for case,windows in [('hold',[(0.,30.)]),('normal',[(0.,32.),(2.,28.),(28.,32.)]),('fast',[(0.,24.),(2.,20.),(20.,24.)])]:
            seen=[]
            def metric(run,target,origin,window):
                seen.append(window)
                return dict(same_time=True,no_phase_alignment=True,interval_task_s=window,
                    duration_s=window[1]-window[0],position_rmse_m=.04,position_max_m=.11)
            cfg=dict(upper_controller=dict(case_id=case),ts20_protocol=dict(tip_rms_max_m=.05,tip_peak_max_m=.1,tilt_max_deg=8.))
            values,acceptance=a.endpoint_observations(None,dict(target_position_enu=[0.,0.,1.],origin_s=1.),cfg,metric)
            self.assertEqual(seen,windows);self.assertFalse(acceptance['passed'])
            result=a.classify_performance(acceptance,1.,cfg['ts20_protocol'])
            self.assertFalse(result['passed']);self.assertTrue(result['tilt_passed'])
            self.assertEqual(values['endpoint']['position_max_m'],.11)
            self.assertEqual(len(result['performance_errors']),len(windows))

    def test_execution_failures_are_retained_and_later_exit_check_still_runs(self):
        report=dict(execution_errors=[],performance_errors=[dict(check='endpoint')])
        checks=a.Checks(report)
        def rejected():raise ValueError('original source mismatch')
        checks('source',rejected);checks('landed',lambda:dict(verified=True))
        self.assertEqual(report['execution_errors'][0]['check'],'source')
        self.assertTrue(report['landed']['verified'])
        self.assertEqual(report['performance_errors'],[dict(check='endpoint')])

    def test_incomplete_native_window_is_execution_failure_not_small_error_success(self):
        cfg=dict(upper_controller=dict(case_id='hold'),ts20_protocol=dict(tip_rms_max_m=.05,tip_peak_max_m=.1,tilt_max_deg=8.))
        def metric(*args):return dict(same_time=True,no_phase_alignment=True,interval_task_s=(0.,30.),
            duration_s=20.,position_rmse_m=.0001,position_max_m=.0002)
        with self.assertRaises(ValueError):a.endpoint_observations(None,dict(target_position_enu=[0.,0.,1.],origin_s=1.),cfg,metric)


if __name__=='__main__':unittest.main()
