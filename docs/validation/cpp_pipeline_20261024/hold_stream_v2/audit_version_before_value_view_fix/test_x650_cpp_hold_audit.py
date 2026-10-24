"""Acceptance negatives use raw numeric/wire-shaped evidence, never a simulator."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('cpp_hold_audit',ROOT/'scripts/audit_x650_cpp_hold.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def fixture():
    c=dict(nodes=2,node_dt=.04,velocity_limit=[.55,.35,.45,.45,.35],acceleration_limit=[.45,.3,.45,.6,.6],
        q_lower=[-1,-1],q_upper=[1,1],base_position_gain=[1,1,1],feasibility_tolerance=2e-7,kkt_tolerance=5e-5)
    s=SimpleNamespace(sequence=1,dt_s=.02,position_enu=[0.,0.,0.])
    p=SimpleNamespace(position_reference_enu=[0.,0.,0.],velocity_feedforward_enu=[0.,0.,0.],
        acceleration_feedforward_enu=[0.,0.,0.],base_velocity_target=[0.,0.,0.],joint_velocity=[0.,0.],
        primal_violation=0.,kkt_residual=0.,subproblems=1)
    d=dict(accepted=True,nonlinear_passed=True,actual_bounds=True,previous_plan_fallback=False,
        primal_violation=0.,kkt_stationarity=0.,compute_ms=10.,q_linearization_error=.001,dq_linearization_error=.04,
        subproblems=1,attempts=[dict(passed=True,actual_bounds=True)],candidate_substeps=60,
        plan=[0.]*10,horizon=[.02,.04],candidate_envelope=[-.1,-.1,.1,.1,-.1,-.1,.1,.1],
        predicted_states=[0.]*24,predicted_tip=[0.]*6,predicted_rotations=[0.]*18)
    r=dict(command=[0.]*5,position_reference=[0.]*3,velocity_feedforward=[0.]*3,
        acceleration_feedforward=[0.]*3,joint_velocity=[0.]*2,diagnostics=d)
    v=dict(kind='cpp_full_pipeline_call_v1',sequence=1,error='',diagnostic_mode='compact',result=r)
    return c,s,p,v


class NumericEvidenceTests(unittest.TestCase):
    def test_original_nonlinear_tolerances_not_tightened_to_envelope_margin(self):
        c,s,p,v=fixture()
        self.assertEqual(module.check_numeric_result(v,s,p,c,[0.]*5).tolist(),[0.]*5)

    def test_result_and_sent_wire_must_be_identical(self):
        c,s,p,v=fixture();p.joint_velocity=[.001,0.]
        with self.assertRaisesRegex(ValueError,'proposal differ'):
            module.check_numeric_result(v,s,p,c,[0.]*5)

    def test_non_linear_candidate_cannot_pass_using_flag_alone(self):
        c,s,p,v=fixture();v['result']['diagnostics']['candidate_envelope'][0]=-1.
        with self.assertRaisesRegex(ValueError,'envelope exceeds'):
            module.check_numeric_result(v,s,p,c,[0.]*5)

    def test_nan_and_deadline_and_fallback_rejected(self):
        for field,value in [('compute_ms',50.00001),('primal_violation',float('nan')),('previous_plan_fallback',True)]:
            with self.subTest(field=field):
                c,s,p,v=fixture();v['result']['diagnostics'][field]=value
                with self.assertRaises(ValueError):module.check_numeric_result(v,s,p,c,[0.]*5)

    def test_unsent_previous_command_is_not_silently_zeroed(self):
        c,s,p,v=fixture()
        with self.assertRaisesRegex(ValueError,'delta limit'):
            module.check_numeric_result(v,s,p,c,[.1,0.,0.,0.,0.])

    def test_complete_candidate_plan_must_be_recorded(self):
        c,s,p,v=fixture();v['result']['diagnostics']['predicted_tip'].pop()
        with self.assertRaisesRegex(ValueError,'Malformed native array'):
            module.check_numeric_result(v,s,p,c,[0.]*5)


def wire_fixture():
    start=dict(session_id=7,epoch=2)
    proposals=[SimpleNamespace(sequence=n) for n in (1,2)]
    issued=[100_000_000,120_000_000]
    calls=[(SimpleNamespace(sequence=n,issued_monotonic_ns=i),p,{}) for n,i,p in zip((1,2),issued,proposals)]
    ticks=[];sends=[]
    for n,i,p in zip((1,2),issued,proposals):
        ticks.append(dict(session_id=7,epoch=2,sequence=n,command_sequence=n,status=1,output_valid=True,
            scheduled_monotonic_ns=i+10_000_000,actual_monotonic_ns=i+11_000_000,
            command_issued_monotonic_ns=i,command_age_ns=11_000_000,source_received_monotonic_ns=i+10_000_000,
            repeated=False,command=vars(p)))
        sends.append(dict(dispatch_sequence=n,command_sequence=n,sent_monotonic_ns=i+11_100_000,repeated=False,stop_zero=False))
    return start,calls,ticks,sends


class OutputTimingTests(unittest.TestCase):
    def test_new_command_stream_does_not_require_per_send_ack(self):
        start,calls,ticks,sends=wire_fixture()
        self.assertTrue(module.check_ticks(ticks,sends,calls,start,convert=vars)['verified'])

    def test_issued_clock_cannot_be_refreshed(self):
        start,calls,ticks,sends=wire_fixture();ticks[1]['command_issued_monotonic_ns']+=1
        with self.assertRaisesRegex(ValueError,'age/source/candidate'):
            module.check_ticks(ticks,sends,calls,start,convert=vars)

    def test_absolute_slot_cannot_be_skipped(self):
        start,calls,ticks,sends=wire_fixture();ticks[1]['scheduled_monotonic_ns']+=20_000_000
        with self.assertRaisesRegex(ValueError,'slots skipped|original absolute slot'):
            module.check_ticks(ticks,sends,calls,start,convert=vars)

    def test_actual_publish_lease_is_checked_beyond_gate_tick(self):
        start,calls,ticks,sends=wire_fixture();sends[1]['sent_monotonic_ns']+=45_000_000
        with self.assertRaisesRegex(ValueError,'age/source/candidate'):
            module.check_ticks(ticks,sends,calls,start,convert=vars)

    def test_actual_send_barrier_rejects_self_reported_early_release(self):
        calls=[(SimpleNamespace(sequence=1),SimpleNamespace(compute_start_monotonic_ns=100),{}),
               (SimpleNamespace(sequence=2),SimpleNamespace(compute_start_monotonic_ns=200),{})]
        sends=[dict(command_sequence=1,sent_monotonic_ns=150,stop_zero=False)]
        trace=dict(columns=['kind','sequence','forward','entry_ns'],rows=[[5,1,True,160],[4,2,True,170]])
        self.assertTrue(module.check_actual_send_barrier(trace,sends,calls)['verified'])
        trace['rows'][0][-1]=140
        with self.assertRaisesRegex(ValueError,'preceded actual send'):
            module.check_actual_send_barrier(trace,sends,calls)

    def test_failed_prefix_retains_input_hashes_without_certificate(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);launch=root/'launch';run=root/'run';out=root/'out'
            for p in (launch,run,out):p.mkdir()
            (run/'controller_metadata.json').write_text(json.dumps(dict(complete=False,error='recorded hold_v1 fault')))
            result=module.audit(launch,run,out)
            self.assertFalse(result['passed']);self.assertFalse(result['certificate'])
            self.assertFalse(result['physical_acceptance']);self.assertIn('Incomplete flight',result['error'])
            self.assertTrue(any(a['path'].endswith('controller_metadata.json') for a in result['artifacts']))

    def test_metadata_cannot_substitute_for_real_disarm(self):
        status=dict(timestamp=[31_000_000,32_000_000],arming_state=[2,1],failsafe=[0,0])
        land=dict(timestamp=[31_000_000,32_000_000],landed=[1,1])
        flight=[dict(sim_time_s='33',armed='0',landed='1',failsafe='0')]
        self.assertTrue(module.check_recovery(status,land,flight,30.)['verified'])
        status['arming_state'][-1]=2
        with self.assertRaisesRegex(ValueError,'independently end'):
            module.check_recovery(status,land,flight,30.)


if __name__=='__main__':unittest.main()
