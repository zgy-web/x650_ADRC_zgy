"""Prediction/constraint/preview checks without claiming a native flight."""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

import numpy as np
from scipy.linalg import expm

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from x650_wholebody.mpc import MpcController, MpcSolveError, response_matrices
from x650_wholebody.model import tip_world, joint_jacobian_body, rotation_matrix
from test_model import example_model


def state():
    return SimpleNamespace(position=np.array([0.,0.,2.]), velocity=np.zeros(3),
                           rotation=np.eye(3), omega_body=np.zeros(3),
                           q=np.zeros(2), dq=np.zeros(2), stamp_s=1.)


def target(p, v=(0.,0.,0.), a=(0.,0.,0.)):
    return SimpleNamespace(position=np.array(p), velocity=np.array(v), acceleration=np.array(a))


class MpcTests(unittest.TestCase):
    def make(self, trajectory=None, config=None):
        model = example_model(elbow_offset_body_m=(0.,.04,0.))
        controller = MpcController(model, {"MPC_XY_P":1.7,"MPC_Z_P":.9},
                                   trajectory=trajectory, config={"nodes":6, **(config or {})})
        s = state()
        controller.reset(s.position, s.rotation, s.q)
        d = target(tip_world(s.position, s.rotation, s.q, model))
        return model, controller, s, d

    def test_closed_loop_response_is_exact_zoh_not_an_instantaneous_velocity_jump(self):
        h, tau = .02, np.array([.2,.2,.25])
        A, B = response_matrices(h, tau)
        # Independent continuous-time matrix exponential for one base axis.
        continuous = np.array([[0.,1.,0.],[0.,-1/tau[0],1/tau[0]],[0.,0.,0.]])
        transition = expm(h*continuous)
        np.testing.assert_allclose(A[np.ix_([0,3],[0,3])], transition[:2,:2], atol=1e-14)
        np.testing.assert_allclose(B[[0,3],0], transition[:2,2], atol=1e-14)
        self.assertLess(B[3,0], .1)
        self.assertAlmostEqual(B[6,3], h)
        self.assertAlmostEqual(B[8,3], 1.)
        self.assertEqual(A[6,8], 0.)

    def test_rest_equilibrium_and_truthful_model_metadata(self):
        _, c, s, d = self.make()
        out = c.step(0., .02, s, d)
        np.testing.assert_allclose(out["base_velocity_cmd"], 0., atol=1e-12)
        np.testing.assert_allclose(out["qd_cmd"], 0., atol=1e-12)
        diag, meta = out["diagnostics"], c.metadata()
        self.assertTrue(diag["accepted_optimum"])
        self.assertLess(diag["primal_violation"], 1e-12)
        self.assertLess(diag["kkt_stationarity_inf"], 1e-10)
        self.assertAlmostEqual(diag["predicted_horizon_s"], .02+5*.04)
        self.assertFalse(meta["response_identified"])
        self.assertFalse(meta["realtime_verified"])
        self.assertFalse(diag["command_delay_compensated"])
        self.assertFalse(diag["state_delay_compensated"])

    def test_first_command_respects_actual_dt_not_the_longer_prediction_node(self):
        _, c, s, d = self.make()
        d.position[0] += .04
        out = c.step(0., .01, s, d)
        command = np.r_[out["base_velocity_cmd"],out["qd_cmd"]]
        self.assertGreater(np.linalg.norm(command), 1e-5)
        self.assertTrue(np.all(abs(command) <= c.acceleration_limit*.01+1e-9))
        np.testing.assert_allclose(out["diagnostics"]["command_acceleration"], command/.01, atol=1e-13)
        self.assertAlmostEqual(out["diagnostics"]["node_intervals_s"][0], .01)

    def test_actual_state_feedback_encodes_without_duplicate_position_or_joint_P(self):
        _, c, s, d = self.make()
        s.position[0] += .015
        out = c.step(0., .02, s, d)
        np.testing.assert_allclose(c.base_position_gain*(out["p_ref"]-s.position)+out["v_ref"],
                                   out["base_velocity_cmd"], atol=1e-14)
        np.testing.assert_allclose(c.joint_position_gain*(out["q_ref"]-s.q)+out["qd_ref"],
                                   out["qd_cmd"], atol=1e-14)
        self.assertLess(out["base_velocity_cmd"][0], 0.)
        self.assertGreater(np.linalg.norm(out["qd_cmd"]), 0.)

    def test_measured_base_velocity_and_attitude_produce_active_arm_compensation(self):
        m, c, s, d = self.make()
        s.velocity[0] = .04
        out = c.step(0., .02, s, d)
        J = joint_jacobian_body(s.q,m)
        self.assertGreater(np.linalg.norm(out["qd_cmd"]), 0.)
        self.assertLess(np.linalg.norm(s.velocity+J@out["qd_cmd"]), np.linalg.norm(s.velocity))
        m, c, s, d = self.make()
        s.rotation = rotation_matrix([np.cos(.03),0.,np.sin(.03),0.])
        out = c.step(0., .02, s, d)
        self.assertGreater(np.linalg.norm(out["qd_cmd"]), 0.)

    def test_future_reference_changes_present_command_with_same_current_target(self):
        m, c, s, d = self.make()
        center = d.position.copy()
        class Preview:
            def preview(self, t):
                # Smooth acceleration starts in the future; current p/v/a are zero.
                elapsed = max(t-.05, 0.)
                return target(center+[.4*elapsed**3,0.,0.], [.4*3*elapsed**2,0.,0.], [.4*6*elapsed,0.,0.])
        _, preview, sp, dp = self.make(trajectory=Preview())
        no_preview = c.step(0., .02, s, d)
        with_preview = preview.step(0., .02, sp, dp)
        self.assertLess(np.linalg.norm(np.r_[no_preview["base_velocity_cmd"],no_preview["qd_cmd"]]), 1e-10)
        self.assertGreater(np.linalg.norm(np.r_[with_preview["base_velocity_cmd"],with_preview["qd_cmd"]]), 1e-5)
        self.assertEqual(with_preview["diagnostics"]["reference_preview"], "trajectory")

    def test_horizon_joint_rate_delta_rate_and_terminal_reserve_are_feasible(self):
        _, c, s, d = self.make()
        d.position += [.02,0.,.01]
        out = c.step(0., .02, s, d)
        diag = out["diagnostics"]
        U, X, h = diag["predicted_commands"], diag["predicted_states"], diag["node_intervals_s"]
        self.assertTrue(np.all(abs(U) <= c.velocity_limit+1e-9))
        self.assertTrue(np.all(abs(np.diff(np.vstack((np.zeros(5),U)),axis=0))/h[:,None] <= c.acceleration_limit+1e-8))
        self.assertTrue(np.all(X[:,6:8] >= c.q_lower-1e-9))
        self.assertTrue(np.all(X[:,6:8] <= c.q_upper+1e-9))
        reserve = c.velocity_limit[3:]/(2*c.acceleration_limit[3:])+c.max_dt
        terminal = X[-1,6:8]+reserve*X[-1,8:10]
        self.assertTrue(np.all(terminal >= c.q_lower-1e-9))
        self.assertTrue(np.all(terminal <= c.q_upper+1e-9))

    def test_analytic_qp_gradient_agrees_with_central_difference(self):
        _, c, s, d = self.make()
        d.position[0] += .01
        p = c._problem(0.,.02,s,d)
        z = np.linspace(-.01,.01,c.nodes*5)
        expected = p["H"]@z+p["g"]
        objective = lambda x: np.linalg.norm(p["A"]@x-p["b"])**2
        finite = []
        for i in range(len(z)):
            offset = np.zeros(len(z)); offset[i] = 1e-6
            finite.append((objective(z+offset)-objective(z-offset))/(2e-6))
        np.testing.assert_allclose(expected, finite, atol=2e-9, rtol=1e-6)

    def test_solver_failure_raises_explicitly_without_mutating_previous_command(self):
        _, c, s, d = self.make()
        failure = SimpleNamespace(x=np.zeros(c.nodes*5), success=False, status=9, nit=1,
                                  message="simulated iteration budget exhausted")
        with patch("x650_wholebody.mpc.minimize", return_value=failure):
            with self.assertRaises(MpcSolveError) as raised:
                c.step(0.,.02,s,d)
        self.assertFalse(raised.exception.diagnostics["accepted_optimum"])
        self.assertFalse(raised.exception.diagnostics["previous_plan_fallback"])
        np.testing.assert_array_equal(c.previous_command, 0.)
        self.assertIsNone(c.last_t)

    def test_shift_fallback_is_explicit_and_rechecks_current_joint_constraints(self):
        _, c, s, d = self.make(config={"failure_policy":"shift"})
        c.step(0.,.02,s,d)
        failure = SimpleNamespace(x=np.zeros(c.nodes*5), success=False, status=9,nit=1,message="forced failure")
        with patch("x650_wholebody.mpc.minimize", return_value=failure):
            out = c.step(.02,.02,s,d)
            self.assertTrue(out["diagnostics"]["previous_plan_fallback"])
            self.assertFalse(out["diagnostics"]["accepted_optimum"])
            s.q[0] = c.q_upper[0]+.001
            with self.assertRaises(ValueError):
                c.step(.04,.02,s,d)

    def test_time_clock_reset_and_invalid_configuration_are_not_silent(self):
        m, c, s, d = self.make()
        c.step(0.,.02,s,d)
        for t,dt in ((0.,.02),(.02,.06),(.02,0.)):
            with self.subTest(t=t,dt=dt), self.assertRaises(ValueError):
                c.step(t,dt,s,d)
        s.stamp_s = .5
        with self.assertRaises(ValueError):
            c.step(.02,.02,s,d)
        c.reset(s.position,s.rotation,s.q)
        c.step(0.,.02,s,d)
        with self.assertRaises(ValueError):
            MpcController(m,{"MPC_XY_P":1.,"MPC_Z_P":1.},config={"base_time_constant_s":[0.,.2,.2]})
        for config in ({"nodes":2.5},{"nodes":"15"},{"max_iterations":3.5}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                MpcController(m,{"MPC_XY_P":1.,"MPC_Z_P":1.},config=config)


if __name__ == "__main__":
    unittest.main()
