"""Exact horizon-cache boundaries and accelerated nominal equivalence.

These verify reusable numerical work; they never establish flight performance
or a wall-time deadline. The shared PID nominal implementation stays untouched.
"""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from x650_wholebody.mpc import MpcController, response_matrices, rotation_increment
from x650_wholebody.baseline import nominal_reference
from x650_wholebody.model import tip_world
from test_model import example_model


class MpcCacheTests(unittest.TestCase):
    def make(self):
        model = example_model(elbow_offset_body_m=(0., .04, 0.))
        controller = MpcController(model, {"MPC_XY_P": 1.7, "MPC_Z_P": .9}, config={"nodes": 6})
        controller.reset([.1, -.2, 2.], np.eye(3), [0., 0.])
        return controller

    def test_full_and_preview_nominals_match_common_pid_reference_for_varied_anchors(self):
        rng, controller = np.random.default_rng(650), self.make()
        for _ in range(50):
            p0, R0, q0 = rng.normal(size=3), rotation_increment(rng.uniform(-.2, .2, 3)), rng.uniform(-.1, .1, 2)
            controller.reset(p0, R0, q0)
            targets = []
            for _ in range(6):
                p = tip_world(p0, R0, q0, controller.model)+R0@rng.uniform(-.01, .01, 3)
                targets.append(SimpleNamespace(position=p, velocity=rng.uniform(-.02, .02, 3), acceleration=rng.uniform(-.03, .03, 3)))
            preview = controller._preview_nominals(targets)
            for i, target in enumerate(targets):
                expected = nominal_reference(p0, R0, q0, target, controller.model, controller.arm_share)
                full = controller._nominal(target)
                for key, value in expected.items():
                    np.testing.assert_allclose(full[key], value, atol=2e-13, rtol=2e-12)
                for key in ("p_nom", "v_nom", "q_nom", "qd_nom"):
                    np.testing.assert_allclose(preview[key][i], expected[key], atol=2e-13, rtol=2e-12)

    def test_cache_is_exactly_keyed_on_dt_and_response_geometry_parameters(self):
        controller = self.make()
        first = controller._horizon(.02)
        self.assertIs(first, controller._horizon(.02))
        changed_dt = controller._horizon(.024)
        self.assertIsNot(first, changed_dt)
        self.assertEqual(changed_dt["h"][0], .024)
        self.assertIs(first, controller._horizon(.02))
        controller.base_tau[0] *= 1.1
        changed_tau = controller._horizon(.02)
        self.assertIsNot(first, changed_tau)
        np.testing.assert_allclose(changed_tau["transitions"][0], response_matrices(.02, controller.base_tau)[0])
        controller.velocity_limit[0] *= .9
        changed_speed = controller._horizon(.02)
        self.assertIsNot(changed_tau, changed_speed)
        controller.attitude_tau *= 1.1
        changed_attitude = controller._horizon(.02)
        self.assertIsNot(changed_speed, changed_attitude)
        np.testing.assert_allclose(changed_attitude["attitude_decay"], np.exp(-changed_attitude["times"]/controller.attitude_tau))
        for dt in np.linspace(.01, .04, 20):
            controller._horizon(float(dt))
        self.assertLessEqual(len(controller._horizon_cache), 8)

    def test_cache_does_not_reuse_state_target_or_attitude_dependent_problem(self):
        controller = self.make()
        state = SimpleNamespace(position=np.array([.1, -.2, 2.]), velocity=np.zeros(3),
            rotation=np.eye(3), omega_body=np.zeros(3), q=np.zeros(2), dq=np.zeros(2), stamp_s=1.)
        center = tip_world(state.position, state.rotation, state.q, controller.model)
        target = SimpleNamespace(position=center, velocity=np.zeros(3), acceleration=np.zeros(3))
        first = controller._problem(0., .02, state, target)
        state.position[0] += .01
        state.velocity[1] = .03
        state.q[0] = .04
        state.rotation = rotation_increment([.01, -.02, .03])
        state.omega_body = np.array([.02, -.03, .01])
        target.position = center+[.01, 0., -.005]
        second = controller._problem(.02, .02, state, target)
        np.testing.assert_array_equal(first["jacobians"], second["jacobians"])
        np.testing.assert_array_equal(first["C"], second["C"])
        self.assertGreater(np.linalg.norm(first["offsets"]-second["offsets"]), .01)
        for key in ("H", "g", "b"):
            self.assertFalse(np.allclose(first[key], second[key], rtol=0., atol=1e-12), key)

    def test_preview_still_validates_acceleration_and_rejects_unreachable_ik(self):
        controller = self.make()
        center = tip_world(controller.p0, controller.R0, controller.q0, controller.model)
        target = SimpleNamespace(position=center, velocity=np.zeros(3), acceleration=[0., np.nan, 0.])
        with self.assertRaises(ValueError):
            controller._preview_nominals([target])
        target.acceleration = np.zeros(3)
        target.position = center+[10., 0., 0.]
        with self.assertRaisesRegex(ValueError, "unreachable"):
            controller._preview_nominals([target])


if __name__ == "__main__":
    unittest.main()
