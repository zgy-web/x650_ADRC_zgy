"""Servo algebra, actual-state compensation, limits and archived-core parity."""
from __future__ import annotations
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE.parent/"pid_ff_wholebody_v1"))
from x650_wholebody.baseline import BaselineController, nominal_reference, bounded_least_squares
from x650_wholebody.model import tip_world, joint_jacobian_body, rotation_matrix
from test_model import example_model


def state(p=(0., 0., 2.), q=(0., 0.), v=(0., 0., 0.), R=None, omega=(0., 0., 0.)):
    return SimpleNamespace(position=np.array(p), rotation=np.eye(3) if R is None else R,
                           velocity=np.array(v), omega_body=np.array(omega), q=np.array(q), dq=np.zeros(2), stamp_s=1.)


def target(p, v=(0., 0., 0.), a=(0., 0., 0.)):
    return SimpleNamespace(position=np.array(p), velocity=np.array(v), acceleration=np.array(a))


class BaselineTests(unittest.TestCase):
    def make(self, **kw):
        m = example_model(elbow_offset_body_m=(0., .04, 0.))
        c = BaselineController(m, {"MPC_XY_P": 1.7, "MPC_Z_P": .9}, **kw)
        s = state()
        c.reset(s.position, s.rotation, s.q)
        return m, c, s

    def test_rest_equilibrium_and_actual_position_gain_encoding(self):
        m, c, s = self.make()
        d = target(tip_world(s.position, s.rotation, s.q, m))
        out = c.step(0., .02, s, d)
        np.testing.assert_allclose(out["base_velocity_cmd"], 0., atol=1e-13)
        np.testing.assert_allclose(out["qd_cmd"], 0., atol=1e-13)
        s.position[0] += .01
        out = c.step(.02, .02, s, d)
        np.testing.assert_allclose(c.base_position_gain*(out["p_ref"]-s.position)+out["v_ref"], out["base_velocity_cmd"], atol=1e-15)
        np.testing.assert_allclose(c.joint_position_gain*(out["q_ref"]-s.q)+out["qd_ref"], out["qd_cmd"], atol=1e-15)
        self.assertIn("requires separate", c.metadata()["pid_status"])

    def test_measured_body_velocity_and_attitude_drive_arm_compensation(self):
        m, c, s = self.make()
        d = target(tip_world(s.position, s.rotation, s.q, m))
        s.velocity[0] = .04
        out = c.step(0., .02, s, d)
        self.assertGreater(np.linalg.norm(out["qd_cmd"]), 0.)
        J = s.rotation@joint_jacobian_body(s.q, m)
        self.assertLess(np.linalg.norm(s.velocity+J@out["qd_cmd"]), np.linalg.norm(s.velocity))
        m, c, s = self.make()
        d = target(tip_world(s.position, s.rotation, s.q, m))
        s.rotation = rotation_matrix([np.cos(.03), 0., np.sin(.03), 0.])
        out = c.step(0., .02, s, d)
        self.assertGreater(np.linalg.norm(out["qd_cmd"]), 0.)

    def test_nominal_fk_velocity_acceleration_closure_with_nonzero_yaw(self):
        m = example_model(elbow_offset_body_m=(0., .04, 0.))
        p0, q0 = np.array([.1, -.2, 2.]), np.array([0., 0.])
        R0 = rotation_matrix([np.cos(.35), 0., 0., np.sin(.35)])
        d = target(tip_world(p0, R0, q0, m)+[.02, .01, -.005], [.01, -.02, .003], [.02, .01, -.01])
        n = nominal_reference(p0, R0, q0, d, m)
        J = joint_jacobian_body(n["q_nom"], m)
        from x650_wholebody.model import joint_centripetal_body
        np.testing.assert_allclose(tip_world(n["p_nom"], R0, n["q_nom"], m), d.position, atol=1e-14)
        np.testing.assert_allclose(n["v_nom"]+R0@J@n["qd_nom"], d.velocity, atol=1e-14)
        np.testing.assert_allclose(n["a_nom"]+R0@(J@n["qdd_nom"]+joint_centripetal_body(n["q_nom"], n["qd_nom"], m)), d.acceleration, atol=1e-14)

    def test_invalid_time_limits_and_missing_external_pid_are_not_silent(self):
        m, c, s = self.make()
        d = target(tip_world(s.position, s.rotation, s.q, m))
        with self.assertRaises(ValueError):
            c.step(0., .06, s, d)
        c.step(0., .02, s, d)
        with self.assertRaises(ValueError):
            c.step(0., .02, s, d)
        s.q[0] = -.8
        with self.assertRaises(ValueError):
            c.step(.02, .02, s, d)
        with self.assertRaises(KeyError):
            BaselineController(m, {"MPC_XY_P": 2.})
        with self.assertRaises(ValueError):
            BaselineController(m, {"MPC_XY_P": 2., "MPC_Z_P": 1.}, config={"q_upper": [1., 1.]})

    def test_archived_controller_outputs_are_preserved_for_legacy_geometry(self):
        from flyarm_pid_ff._core.controller import WholeBodyClosedLoop
        m = example_model(mount_body=(0., 0., -.1), total_mass=2.6493076923076924, base_com_body=(0., 0., 0.))
        p0, R0, q0 = np.array([0., 0., 2.]), np.eye(3), np.zeros(2)
        center = tip_world(p0, R0, q0, m)
        class Source:
            def sample(self, t):
                return (center+np.array([.02*np.sin(t), 0., .01*np.sin(2*t)]),
                        np.array([.02*np.cos(t), 0., .02*np.cos(2*t)]),
                        np.array([-.02*np.sin(t), 0., -.04*np.sin(2*t)]))
        source = Source()
        for mode in ("baseline", "compensated"):
            old = WholeBodyClosedLoop(p0, q0, R0, source, mode=mode)
            new = BaselineController(m, {"MPC_XY_P": 2., "MPC_Z_P": 1.}, mode=mode)
            new.reset(p0, R0, q0)
            for i in range(160):
                t = i*.02
                s = state(p=p0+[.001*np.sin(t), .001*np.cos(t), .001*np.sin(2*t)], q=[.01*np.sin(t), .01*np.cos(t)],
                          v=[.001*np.cos(t), -.001*np.sin(t), .002*np.cos(2*t)], omega=[.001, .002, -.001])
                a = old.step(t, .02, s.position, s.rotation, s.velocity, s.rotation@s.omega_body, s.q, s.dq)
                b = new.step(t, .02, s, target(*source.sample(t)))
                for key in ("p_ref", "v_ref", "a_ref", "q_ref", "qd_ref", "qdd_ref", "qd_cmd", "base_velocity_cmd", "tip_ref", "tip_actual"):
                    np.testing.assert_allclose(a[key], b[key], atol=1e-10, rtol=1e-9, err_msg=key)

    def test_box_solver_kkt_and_collapsed_bound(self):
        rng = np.random.default_rng(17)
        for _ in range(15):
            A, b = rng.normal(size=(14, 5)), rng.normal(size=14)
            lower, upper = -.2*np.ones(5), .3*np.ones(5)
            lower[0] = upper[0] = .1
            out = bounded_least_squares(A, b, lower, upper)
            g = A.T@(A@out.x-b)
            self.assertEqual(out.x[0], .1)
            for i in range(1, 5):
                if abs(out.x[i]-lower[i]) < 1e-9:
                    self.assertGreaterEqual(g[i], -1e-8)
                elif abs(out.x[i]-upper[i]) < 1e-9:
                    self.assertLessEqual(g[i], 1e-8)
                else:
                    self.assertLess(abs(g[i]), 1e-8)

    def test_archived_feedforward_is_preserved_at_com_origin(self):
        from flyarm_pid_ff._core.feedforward import ArmParameters, arm_reaction_feedforward as old_ff
        from x650_wholebody.feedforward import arm_reaction_feedforward
        old_model = ArmParameters()
        model = example_model(mount_body=old_model.mount_body, total_mass=old_model.total_mass,
                              angle_offsets=old_model.angle_offsets, base_com_body=(0., 0., 0.))
        args = ([.1, -.2], [.21, -.14], [-.3, .5])
        kw = dict(R_world_body=rotation_matrix([.9, .1, .2, -.1]), omega_body=[.1, -.2, .3],
                  alpha_body=[-.2, .3, -.4], base_acceleration_world=[.2, -.1, .3],
                  gravity_world=[0., 0., -9.81])
        old = old_ff(*args, params=old_model, **kw)
        new = arm_reaction_feedforward(*args, model=model, **kw)
        for name in ("acceleration_world", "raw_acceleration_world", "relative_force_world",
                     "predicted_reaction_force_world", "predicted_reaction_moment_body",
                     "required_arm_moment_body", "arm_com_position_body", "arm_com_acceleration_world"):
            np.testing.assert_allclose(new[name], old[name], atol=1e-13, rtol=1e-12, err_msg=name)


if __name__ == "__main__":
    unittest.main()
