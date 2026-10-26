"""Counterexamples for the read-only paired planning-margin audit."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[2]
SPEC=importlib.util.spec_from_file_location('joint_margin_audit',ROOT/'scripts/audit_x650_joint_safety_margin.py')
audit=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(audit)


def problem(nodes=2):
    n,m=nodes*5,nodes*15+5
    old=dict(qp_columns=n,qp_rows=m,A=[0.]*(21*nodes*n),b=[0.]*(21*nodes),
        H=[0.]*(n*n),g=[0.]*n,C=[0.]*(m*n),lower=[-1.]*m,upper=[1.]*m)
    new=deepcopy(old)
    for i in range(nodes):
        for offset,padding in ((0,.0003),(1,.0003),(2,.03),(3,.03)):
            row=5*nodes+6*i+offset
            new['lower'][row]+=padding;new['upper'][row]-=padding
    return old,new


def metadata():
    fixture=dict(memory=dict(step_count=428,actual_history='preserved'),
        config=dict(feasibility_tolerance=2e-7,kkt_tolerance=.002))
    raw=dict(kind='x650_cpp_recovery_raw_replay_v1',schema_version=1,complete=True,core_abi=2,
        input_sha256='a'*64,returned=False,core_error=True,unexpected_exception=False,output=None,
        error='TS20 nonlinear candidate rejected before command publication',memory_unchanged=True,
        memory_commit_valid=False,memory_before=deepcopy(fixture['memory']),memory_after=deepcopy(fixture['memory']),
        scope=dict(offline_fixed_snapshot=True,actuator_io=False,closed_loop_or_physical_validation=False,
            injected_delay=False,original_core_deadline_ms=50.,initial_trust_radius=.02),
        diagnostics=dict(accepted=False,actual_bounds=False,nonlinear_passed=False,previous_plan_fallback=False,
            recovery_triggered=False,recovery_attempts=[],subproblems=1),loaded_code=dict(predictor=dict(sha256='b'*64)))
    return fixture,raw,deepcopy(raw)


class ProblemChangeTests(unittest.TestCase):
    def test_only_declared_node_rows_tighten_in_both_directions(self):
        old,new=problem();result=audit.compare_problem(old,new,2)
        self.assertTrue(result['verified']);self.assertEqual(result['changed_lower_count'],8)
        self.assertEqual(result['changed_upper_count'],8)

    def test_hessian_objective_or_sensitivity_change_is_not_hidden(self):
        for key in ('A','b','H','g','C'):
            with self.subTest(key=key):
                old,new=problem();new[key][0]=.001
                with self.assertRaisesRegex(ValueError,'Unreviewed matrix'):audit.compare_problem(old,new,2)

    def test_reference_trust_slew_or_stopping_bound_change_rejects(self):
        for row in (0,14,22,26,30):
            with self.subTest(row=row):
                old,new=problem();new['lower'][row]+=.0001
                with self.assertRaisesRegex(ValueError,'Only the reviewed node padding'):audit.compare_problem(old,new,2)

    def test_wrong_padding_sign_or_amount_is_rejected(self):
        for row,key,delta in ((10,'lower',.0001),(12,'upper',.001),(10,'lower',-.0006)):
            with self.subTest(row=row,key=key,delta=delta):
                old,new=problem();new[key][row]+=delta
                with self.assertRaisesRegex(ValueError,'Only the reviewed node padding'):audit.compare_problem(old,new,2)


class MetadataTests(unittest.TestCase):
    def test_same_fixed_input_rejection_and_rollback_remain_explicit(self):
        f,old,new=metadata();result=audit.check_metadata(old,new,f,'a'*64)
        self.assertTrue(result['rejected_plan_never_returned'])

    def test_changed_input_predictor_or_memory_cannot_be_compared_as_same_case(self):
        for mode in ('input','predictor','memory'):
            with self.subTest(mode=mode):
                f,old,new=metadata()
                if mode=='input':new['input_sha256']='c'*64
                elif mode=='predictor':new['loaded_code']['predictor']['sha256']='c'*64
                else:new['memory_after']['step_count']=429
                with self.assertRaises(ValueError):audit.check_metadata(old,new,f,'a'*64)

    def test_physical_success_scope_or_relaxed_deadline_is_rejected(self):
        for key,value in (('closed_loop_or_physical_validation',True),('original_core_deadline_ms',60.),
                          ('initial_trust_radius',.15)):
            with self.subTest(key=key):
                f,old,new=metadata();new['scope'][key]=value
                with self.assertRaisesRegex(ValueError,'scope or original gates'):audit.check_metadata(old,new,f,'a'*64)


if __name__=='__main__':unittest.main()
