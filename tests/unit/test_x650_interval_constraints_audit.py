"""Fail-closed interval-audit counterexamples; synthetic arithmetic only."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
SPEC=importlib.util.spec_from_file_location('interval_audit',ROOT/'scripts/audit_x650_interval_constraints.py')
audit=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(audit)


def accepted_fixture():
    N,n,m=15,75,230
    c=dict(nodes=N,node_dt=.04,max_dt=.05,feasibility_tolerance=2e-7,kkt_tolerance=.002,
        q_lower=[-.75,-.35],q_upper=[.45,.85],velocity_limit=[.55,.35,.45,.45,.35],
        acceleration_limit=[.45,.30,.45,.60,.60],motion=[1.,0.,0.,0.,.003346379092576424,0.,0.,1.],
        kp=[6.,6.],kd=[.25,.25],bus_period=.005,trust_radius=.15)
    packet=lambda t:dict(send_time=t,q_ref=[0.,0.],dq_ref=[0.,0.],tau_ff=[0.,0.],kp=c['kp'],kd=c['kd'])
    memory=dict(initialized=True,history_primed=True,has_last_t=False,has_stamp=False,has_origin=True,
        anchor=dict(position=[0.]*3,rotation=[1.,0.,0.,0.,1.,0.,0.,0.,1.],q=[0.,0.]),
        previous_command=[0.]*5,last_t=0.,previous_stamp=0.,origin=10.,plan_t=0.,step_count=0,
        plan=[],plan_h=[],joint_history=[dict(time=-.005,velocity=[0.,0.])])
    f=dict(config=c,memory=memory,input=dict(t=0.,dt=.02,state=dict(control_time=10.,joint_stamp=10.,
        stamp=10.,q=[0.,0.],dq=[0.,0.]),context=dict(reference_time=-.005,reference=[0.,0.],bus_origin=0.,
        commands=[dict(time=-.005,velocity=[0.,0.])],packets=[packet(-.005)])))
    packets=[packet(-.005)]+[packet(i*.005) for i in range(117)]
    h,nodes,queries,boundaries=audit.canonical_grid(f,packets)
    C=np.zeros((m,n));lo=np.full(m,-1.);hi=np.full(m,1.)
    for i in range(N):
        for j in range(2):
            row=13*N+2*i+j;C[row,5*i+3+j]=1.;lo[row]=-.02;hi[row]=.02
    d=dict(accepted=True,nonlinear_passed=True,actual_bounds=True,previous_plan_fallback=False,
        subproblems=1,attempts=[dict(passed=True,actual_bounds=True)],compute_ms=4.,
        primal_violation=0.,kkt_stationarity=0.,qp_rows=m,qp_columns=n,
        H=np.eye(n).ravel().tolist(),g=[0.]*n,C=C.ravel().tolist(),lower=lo.tolist(),upper=hi.tolist(),
        solution=[0.]*n,plan=[0.]*n,horizon=h,candidate_packets=deepcopy(packets),nominal_packets=deepcopy(packets),
        query_times=queries,candidate_boundary_times=boundaries,candidate_states=[0.]*(4*len(queries)),
        nominal_states=[0.]*(4*len(queries)),candidate_substeps=600,
        predicted_states=[0.]*(12*N),linearized_states=[0.]*(12*N),q_linearization_error=0.,dq_linearization_error=0.,
        candidate_envelope=[0.]*8,recovery_triggered=False,recovery_reason='',recovery_attempts=[],
        interval_repairs=[],interval_constraints=[],interval_rows=0)
    output={k:[0.]*size for k,size in audit.replay.OUTPUT_DIMENSIONS.items()}
    after=deepcopy(memory);after.update(plan=d['plan'],plan_h=h,plan_t=0.,previous_command=output['command'],
        last_t=0.,has_last_t=True,previous_stamp=10.,has_stamp=True,origin=10.,has_origin=True,step_count=1)
    after['joint_history'].append(dict(time=0.,velocity=[0.,0.]))
    libraries=dict(core='a'*64,optimization='b'*64,predictor='c'*64)
    raw=dict(kind='x650_cpp_recovery_raw_replay_v1',schema_version=1,complete=True,core_abi=3,
        input_sha256='f'*64,loaded_code={k:dict(sha256=v) for k,v in libraries.items()},
        scope=dict(offline_fixed_snapshot=True,actuator_io=False,closed_loop_or_physical_validation=False,
            injected_delay=False,original_core_deadline_ms=50.,initial_trust_radius=.02),
        memory_before=deepcopy(memory),memory_after=after,memory_unchanged=False,memory_commit_valid=True,
        returned=True,core_error=False,unexpected_exception=False,error=None,output=output,
        core_step_wall_ms=5.,diagnostics=d)
    return f,raw,libraries


def add_cut(f,raw):
    d=raw['diagnostics'];n=5*f['config']['nodes'];base=d['qp_rows'];at=.03
    point=dict(time=at,value=-.421,lower=-.42,upper=.42,joint=0,component=1,source_kind=3)
    risk_gradient=audit.fast_branch_gradient(f,d['nominal_packets'],d['candidate_boundary_times'],point)
    margin=.02*sum(abs(risk_gradient[k]) for k in range(n) if k%5 in (3,4))
    guard=audit.encoding_guard(f,d['nominal_packets'],[0.]*n,point)
    d['interval_repairs']=[dict(control_subproblem=1,added_points=1,total_points=1,canonical_substeps=600,
        elapsed_ms=2.,rejection='',envelope=[0.,0.,0.,0.,-.421,0.,0.,0.],points=[deepcopy(point)],
        trigger_points=[deepcopy(point)],centre_plan=[0.]*n,risk_radius=.02,risk_margin=[margin],
        risk_encoding_lower=[guard['encoding_lower_remainder']],risk_encoding_upper=[guard['encoding_upper_remainder']])]
    # Synthetic final assembly at another centre keeps original generated time.
    anchor=deepcopy(point);anchor['value']=0.
    gradient=audit.fast_branch_gradient(f,d['nominal_packets'],d['candidate_boundary_times'],anchor).tolist()
    d['interval_constraints']=[dict(point=anchor,affine_offset=0.,gradient=gradient,qp_row=base,**guard)]
    d.update(interval_rows=1,qp_rows=base+1,subproblems=2,
        attempts=[dict(passed=False,actual_bounds=False),dict(passed=True,actual_bounds=True)],
        interval_centre_plan=[0.]*n,interval_canonical_boundaries=d['candidate_boundary_times'],
        interval_canonical_states=[0.]*(4*len(d['candidate_boundary_times'])),
        interval_canonical_envelope=[0.]*8,interval_canonical_substeps=600)
    d['C']+=gradient;d['lower'].append(-.42-guard['encoding_lower_remainder']);d['upper'].append(.42-guard['encoding_upper_remainder'])


class IntervalAuditTests(unittest.TestCase):
    def setUp(self):self.fixture,self.raw,self.libs=accepted_fixture()
    def check(self):return audit.audit(self.fixture,self.raw,'f'*64,self.libs)

    def test_complete_normal_arithmetic_graph_does_not_claim_native_repropagation(self):
        result=self.check()
        self.assertTrue(result['accepted_control_validated'])
        self.assertFalse(result['native_repropagation_independent']);self.assertFalse(result['physical_validation'])

    def test_appended_hard_cut_has_same_canonical_anchor_and_original_trust_rows(self):
        add_cut(self.fixture,self.raw);result=self.check()
        self.assertEqual(result['interval']['rows'],1);self.assertTrue(result['qp']['no_slack_variables'])

    def test_tampered_padding_or_final_envelope_is_rejected(self):
        self.raw['diagnostics']['candidate_envelope'][4]=-.42001
        with self.assertRaisesRegex(ValueError,'nonlinear envelope'):self.check()
        self.raw['diagnostics']['candidate_envelope'][4]=0.
        add_cut(self.fixture,self.raw)
        self.raw['diagnostics']['interval_constraints'][0]['point']['lower']=-.45
        with self.assertRaisesRegex(ValueError,'padding/limit'):self.check()

    def test_added_query_or_rewritten_integration_grid_cannot_hide_peak(self):
        for name in ('query_times','candidate_boundary_times'):
            with self.subTest(name=name):
                f,raw,libs=accepted_fixture();raw['diagnostics'][name]=sorted(raw['diagnostics'][name]+[.030001])
                with self.assertRaisesRegex(ValueError,'canonical'):audit.audit(f,raw,'f'*64,libs)

    def test_wrong_hash_library_or_committed_memory_is_rejected(self):
        for mode in ('input','library','memory'):
            with self.subTest(mode=mode):
                f,raw,libs=accepted_fixture()
                if mode=='input':raw['input_sha256']='0'*64
                elif mode=='library':raw['loaded_code']['predictor']['sha256']='0'*64
                else:raw['memory_after']['previous_stamp']=10.01
                with self.assertRaises(ValueError):audit.audit(f,raw,'f'*64,libs)

    def test_risk_batch_requires_real_trigger_and_unchanged_sliding_range(self):
        add_cut(self.fixture,self.raw);repair=self.raw['diagnostics']['interval_repairs'][0]
        repair['trigger_points'][0]['value']=-.41
        with self.assertRaisesRegex(ValueError,'actual native violation trigger'):self.check()
        repair['trigger_points'][0]['value']=-.421
        repair['risk_margin'][0]+=.001
        with self.assertRaisesRegex(ValueError,'sliding search range'):self.check()

    def test_changed_cut_row_or_missing_centre_evidence_is_rejected(self):
        add_cut(self.fixture,self.raw)
        original=self.raw['diagnostics']['interval_constraints'][0]['gradient'][3]
        self.raw['diagnostics']['interval_constraints'][0]['gradient'][3]=.5
        self.raw['diagnostics']['C'][230*75+3]=.5
        with self.assertRaisesRegex(ValueError,'sensitivity|gradient'):self.check()
        self.raw['diagnostics']['interval_constraints'][0]['gradient'][3]=original
        self.raw['diagnostics']['C'][230*75+3]=original
        del self.raw['diagnostics']['interval_canonical_states']
        with self.assertRaises(KeyError):self.check()

    def test_encoding_bound_and_matching_qp_rewrite_cannot_hide_tampering(self):
        add_cut(self.fixture,self.raw);d=self.raw['diagnostics'];cut=d['interval_constraints'][0]
        cut['encoding_lower_remainder']-=.001;d['lower'][230]+=.001
        with self.assertRaisesRegex(ValueError,'encoding_lower_remainder'):self.check()

    def test_encoding_risk_range_and_missing_guard_evidence_are_rejected(self):
        add_cut(self.fixture,self.raw);d=self.raw['diagnostics']
        d['interval_repairs'][0]['risk_encoding_lower'][0]-=.001
        with self.assertRaisesRegex(ValueError,'risk-point encoding range'):self.check()
        d['interval_repairs'][0]['risk_encoding_lower'][0]+=.001
        del d['interval_constraints'][0]['encoding_effort_half_lsb']
        with self.assertRaises(KeyError):self.check()

    def test_half_lsb_kernel_telescopes_and_actual_history_is_not_perturbed(self):
        f=self.fixture;d=self.raw['diagnostics'];point=dict(time=.03,joint=0,component=1)
        guard=audit.encoding_guard(f,d['nominal_packets'],d['plan'],point)
        epsilon=.5*(6.*(2*np.pi/(2**24))+.25/512)
        expected=audit.sliding_transition(f['config'],.03-f['config']['motion'][4],0)[1][1]
        self.assertAlmostEqual(guard['encoding_kernel_l1'],expected,places=13)
        self.assertAlmostEqual(guard['encoding_lower_remainder'],-expected*epsilon,places=13)
        self.assertAlmostEqual(guard['encoding_upper_remainder'],expected*epsilon,places=13)
        packets=deepcopy(d['nominal_packets']);packets[0]['q_ref'][0]=.1;packets[0]['dq_ref'][0]=.2
        self.assertEqual(audit.encoding_guard(f,packets,d['plan'],point),guard)
        point['time']=.001
        early=audit.encoding_guard(f,packets,d['plan'],point)
        self.assertEqual(early['encoding_kernel_l1'],0.)
        self.assertEqual(early['encoding_anchor_bias'],0.)

    def test_encoding_anchor_bias_uses_actual_rounding_not_zero_or_full_lsb(self):
        f=self.fixture;plan=np.zeros((15,5));plan[:,3]=.2/512
        packets=audit.centre_packets(f,plan);point=dict(time=.03,joint=0,component=1)
        guard=audit.encoding_guard(f,packets,plan,point)
        self.assertNotEqual(guard['encoding_anchor_bias'],0.)
        radius=guard['encoding_kernel_l1']*guard['encoding_effort_half_lsb']
        self.assertAlmostEqual(guard['encoding_lower_remainder'],-radius-guard['encoding_anchor_bias'],places=14)
        self.assertAlmostEqual(guard['encoding_upper_remainder'],radius-guard['encoding_anchor_bias'],places=14)
        self.assertGreaterEqual(guard['encoding_lower_remainder'],-2*radius-1e-14)
        self.assertLessEqual(guard['encoding_upper_remainder'],2*radius+1e-14)

    def test_original_trust_location_survives_new_matrix_tail(self):
        add_cut(self.fixture,self.raw);self.raw['diagnostics']['upper'][195]=.2
        with self.assertRaisesRegex(ValueError,'trust width'):self.check()

    def test_original_full_row_quality_and_no_slack_dimension_are_required(self):
        self.raw['diagnostics']['lower'][0]=.001
        with self.assertRaisesRegex(ValueError,'QP residual'):self.check()
        self.raw['diagnostics']['lower'][0]=-1.
        self.raw['diagnostics']['qp_columns']=76
        with self.assertRaisesRegex(ValueError,'dimensions'):self.check()

    def test_rejection_verifies_full_rollback_without_control_pass_claim(self):
        self.raw.update(returned=False,core_error=True,error='interval budget exhausted',output=None,
            memory_after=deepcopy(self.raw['memory_before']),memory_unchanged=True,memory_commit_valid=False)
        result=self.check()
        self.assertTrue(result['passed']);self.assertFalse(result['controller_call_accepted'])
        self.assertFalse(result['accepted_control_validated']);self.assertTrue(result['memory_rollback_verified'])
        self.raw['memory_after']['step_count']=1
        with self.assertRaises(ValueError):self.check()


if __name__=='__main__':unittest.main()
