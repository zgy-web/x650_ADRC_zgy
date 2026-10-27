"""Numerical-contract checks; real bound fixtures are qualified separately."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import uuid

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"control_packages/x650_nmpc_standard_v1/python"))
from x650_nmpc_standard.solver import SolverOptions,deadline_flags,independent_residuals,check_derivatives,CondensedNlp


class Quadratic:
    """min (z+1)^2 subject z>=0, optimum z=0 with a positive lower multiplier."""
    lower_g=np.array([0.]);upper_g=np.array([np.inf])
    lower_z=np.array([-2.]);upper_z=np.array([2.])

    def evaluate(self,z):
        return float((z[0]+1)**2),z.copy(),2*(z+1),np.array([[1.]])

    def lagrangian_hessian(self,z,multipliers):
        return np.array([[2.]])


class ClockAndOptionsTests(unittest.TestCase):
    def test_original_issue_never_refreshed(self):
        self.assertTrue(deadline_flags(1_000_000_000,1_049_000_000,50)["deadline_passed"])
        self.assertFalse(deadline_flags(1_000_000_000,1_051_000_000,50)["deadline_passed"])
        self.assertFalse(deadline_flags(None,1_001_000_000,50)["deadline_passed"])
        with self.assertRaises(ValueError): deadline_flags(2_000,1_000,50)

    def test_thresholds_explicit(self):
        SolverOptions().validate()
        with self.assertRaises(ValueError): SolverOptions(primal_tolerance=-1).validate()
        with self.assertRaises(ValueError): SolverOptions(original_issue_budget_ms=51).validate()


@unittest.skipUnless(importlib.util.find_spec("scipy"),"scipy required for independent active-multiplier fit")
class ResidualTests(unittest.TestCase):
    def test_active_constraint_sign_and_multiplier(self):
        report=independent_residuals(Quadratic(),np.array([0.]))
        self.assertTrue(report["first_order_kkt_passed"])
        self.assertAlmostEqual(report["reconstructed_multipliers"][0],2.)
        self.assertLess(report["stationarity_inf"],1e-10)

    def test_feasibility_and_stationarity_not_confused(self):
        feasible=independent_residuals(Quadratic(),np.array([.1]))
        self.assertTrue(feasible["feasible"])
        self.assertFalse(feasible["first_order_kkt_passed"])
        infeasible=independent_residuals(Quadratic(),np.array([-.001]))
        self.assertFalse(infeasible["feasible"])
        self.assertFalse(infeasible["first_order_kkt_passed"])

    def test_analytic_derivatives(self):
        self.assertTrue(check_derivatives(Quadratic(),np.array([.1]))["passed"])

    def test_incorrect_derivative_is_detected(self):
        class Bad(Quadratic):
            def evaluate(self,z):
                f,g,grad,jac=super().evaluate(z)
                return f,g,grad+1,jac
        self.assertFalse(check_derivatives(Bad(),np.array([.1]))["passed"])


@unittest.skipUnless(importlib.util.find_spec("casadi"),"CasADi required for graph checks")
class SymbolicGraphTests(unittest.TestCase):
    @staticmethod
    def fixture():
        import casadi as ca
        x=ca.SX.sym("x",23);u=ca.SX.sym("u",5);p=ca.SX.sym("p",2)
        next_x=ca.vertcat(x[:5]+p[0]*u,x[5:])
        residual=next_x[:5]-p[1]
        model=SimpleNamespace(x=x,u=u,p=p,discrete=next_x,
            cost=p[0]*ca.dot(residual,residual),constraints=next_x[:5],
            control_lower=np.full(5,-1.),control_upper=np.full(5,1.),
            constraint_lower=np.full(5,-3.),constraint_upper=np.full(5,3.))
        h=np.full(15,.04)
        case=SimpleNamespace(x0=np.zeros(23),parameters=np.tile([.04,.1],(16,1)),
            initial_plan=np.full((15,5),.1),horizon=h,metadata={"scope":"synthetic mathematical test"})
        return model,case

    def test_exact_stage_objective_no_double_h_or_half(self):
        model,case=self.fixture();nlp=CondensedNlp(model,case)
        f,g,grad,jac=nlp.evaluate(case.initial_plan)
        samples=np.arange(1,16)*.04*.1
        expected=float(np.sum(.04*5*(samples-.1)**2))
        self.assertAlmostEqual(f,expected,places=13)
        np.testing.assert_allclose(nlp.rollout(case.initial_plan)[-1,:5],.06,atol=1e-14)
        self.assertEqual(jac.shape,(75,75))
        self.assertTrue(check_derivatives(nlp,case.initial_plan)["passed"])

    def test_rebinding_preserves_source_and_graph(self):
        model,case=self.fixture();nlp=CondensedNlp(model,case)
        original=nlp.parameter_values.copy()
        changed=SimpleNamespace(**vars(case));changed.x0=case.x0.copy();changed.x0[0]=.2
        bound=nlp.with_case(changed)
        self.assertEqual(bound.graph_sha256,nlp.graph_sha256)
        self.assertNotEqual(bound.binding_sha256,nlp.binding_sha256)
        np.testing.assert_array_equal(nlp.parameter_values,original)

    def test_numeric_case_export_roundtrip(self):
        model,case=self.fixture();nlp=CondensedNlp(model,case)
        directory=ROOT/"artifacts"/("standard_solver_contract_test_"+uuid.uuid4().hex)
        directory.mkdir(parents=True)
        path=directory/"case.json"
        try:
            identity=nlp.dump_case_json(path,source_identity={"test":True},config_sha256="0"*64)
            loaded=json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(loaded["model_name"],"x650_standard")
            self.assertEqual(loaded["terminal"]["cost"],0.)
            np.testing.assert_array_equal(loaded["guess_u"],case.initial_plan)
            np.testing.assert_array_equal(loaded["p"],case.parameters)
            np.testing.assert_array_equal(loaded["cost_scaling"],np.ones(16))
            self.assertGreater(identity["bytes"],0)
        finally:
            if path.exists(): path.unlink()
            directory.rmdir()


if __name__=="__main__":
    unittest.main()
