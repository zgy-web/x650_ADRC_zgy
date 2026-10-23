"""Independent raw-row diagnostics for a frozen rejected QP; no native library needed."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import uuid
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
import check_x650_frozen_qp_case as checker


def problem():
    return {'H':np.eye(2),'g':np.zeros(2),'C':np.diag([2.,4.]),
            'lo':np.array([1e-7,-1.]),'hi':np.array([1.,-3e-7]),
            'warm':np.zeros(2),'scale':np.ones(2),'solution':np.zeros(2)}


class FrozenQpCaseTests(unittest.TestCase):
    def test_accept_expectation_requires_original_gates_but_not_archived_solution(self):
        result={'independent_primal_gate_matches_native':True,'accepted_by_frozen_qp_quality':True,
                'independent_residual':{'primal_violation':1e-9}}
        args=dict(inputs_unchanged=True,identical_solution=True,identical_quality=True,
                  reproduced=False,results=[result],feasibility_tolerance=2e-7)
        self.assertTrue(checker.expectation_satisfied('accept',**args))
        self.assertFalse(checker.expectation_satisfied('reproduce',**args))
        result['independent_residual']['primal_violation']=3e-7
        self.assertFalse(checker.expectation_satisfied('accept',**args))
        result['independent_residual']['primal_violation']=1e-9
        result['accepted_by_frozen_qp_quality']=False
        self.assertFalse(checker.expectation_satisfied('accept',**args))
        result['accepted_by_frozen_qp_quality']=True
        for key in ('inputs_unchanged','identical_solution','identical_quality'):
            changed={**args,key:False}
            self.assertFalse(checker.expectation_satisfied('accept',**changed))
        result['independent_primal_gate_matches_native']=False
        self.assertFalse(checker.expectation_satisfied('accept',**args))

    def test_raw_lower_upper_and_box_are_separate_from_row_normalized_distance(self):
        p=problem();r=checker.independent_residuals(p,np.zeros(2))
        self.assertEqual(r['worst_source'],'C_upper')
        self.assertEqual(r['primal_violation'],3e-7)
        self.assertEqual(r['lower']['positive_violation'],1e-7)
        self.assertEqual(r['upper']['row_index_zero_based'],1)
        self.assertEqual(r['upper']['coefficient_l2_norm'],4.)
        self.assertEqual(r['upper']['distance_in_normalized_z_coordinates'],3e-7/4.)
        self.assertFalse(r['upper']['distance_is_acceptance_metric'])
        p['lo'][:]=-100.;p['hi'][:]=100.
        r=checker.independent_residuals(p,np.array([1.01,0.]))
        self.assertEqual(r['worst_source'],'box')
        self.assertAlmostEqual(r['primal_violation'],.01)

    def test_identical_rejection_is_regression_consistency_not_acceptance(self):
        directory=ROOT/'tmp'/('qp_case_test_'+uuid.uuid4().hex);directory.mkdir(parents=True)
        def cleanup():
            for p in directory.iterdir():p.unlink()
            directory.rmdir()
        self.addCleanup(cleanup)
        case=directory/'case.npz';np.savez(case,**problem(),max_iterations=10)
        task=directory/'task.json'
        task.write_text(json.dumps({'numerical_backend':{'library_sha256':'a'*64},
            'mpc_config':{'max_iterations':10,'feasibility_tolerance':2e-7,'kkt_tolerance':.002}}))
        class FakeNative:
            def __init__(self,path,digest):self.receipt={'path':str(path),'sha256':digest}
            def solve(self,p,max_iterations):
                return p['solution'].copy(),{'primal_violation':3e-7,'kkt_stationarity_inf':0.,
                    'kkt_complementarity_inf':0.,'active_constraints':2},False
        with patch.object(checker,'NativeOptimization',FakeNative):
            report=checker.run(case,task,checker.sha(task),directory/'fake.so','a'*64,3)
        self.assertTrue(report['complete'])
        self.assertTrue(report['regression_consistent'])
        self.assertFalse(report['controller_or_flight_accepted'])
        self.assertTrue(all(not result['accepted_by_frozen_qp_quality'] for result in report['results']))
        with self.assertRaisesRegex(ValueError,'iteration limit'):
            checker.load_case(case,11)
        with self.assertRaisesRegex(ValueError,'task SHA256'):
            checker.pinned_task(task,'0'*64,'a'*64)
        with self.assertRaisesRegex(ValueError,'Library SHA256'):
            checker.pinned_task(task,checker.sha(task),'b'*64)


if __name__=='__main__':unittest.main()
