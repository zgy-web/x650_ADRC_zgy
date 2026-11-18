"""Closed-loop and contract checks independent of ROS/PX4/Gazebo."""
from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "comparison_controllers", ROOT/"simulations/x650_control_comparison_v1/controllers.py")
C = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(C)

MODEL = {"gain": [.9, 1.1, 1.], "tau": [.25, .32, .4], "delay_s": [0., 0., 0.]}


def experiment(name, duration=15., disturbance=False, config=None):
    controller = C.make_controller(name, config or {}, MODEL)
    position, velocity, applied = np.zeros(3), np.zeros(3), np.zeros(3)
    controller.reset(position, velocity, applied)
    trace = []
    for k in range(round(duration/.02)):
        target = np.array([.2, -.15, .1]) if not disturbance else np.zeros(3)
        request = controller.step(.02, position, velocity, target)
        applied = np.clip(np.clip(request, -.4, .4), applied-.02, applied+.02)
        controller.accept_applied(applied, .02)
        force = np.array([.2, -.1, .15]) if disturbance and k*.02 >= 2 else np.zeros(3)
        # Small-step integration of the continuous plant, independent of the
        # MPC's analytic discrete state matrices.
        for _ in range(20):
            acceleration = (np.array(MODEL["gain"])*applied-velocity)/MODEL["tau"] + force
            position += velocity*.001
            velocity += acceleration*.001
        trace.append((position.copy(), applied.copy(), target.copy()))
    return controller, trace


class ComparisonControllersTests(unittest.TestCase):
    def test_requires_reset_and_rejects_invalid_measurements(self):
        controller = C.make_controller("pid", {}, MODEL)
        with self.assertRaises(C.ControllerError):
            controller.step(.02, [0]*3, [0]*3, [0]*3)
        controller.reset([0]*3, [0]*3, [0]*3)
        for bad_dt in (0, -.02, float("nan"), .2):
            with self.subTest(dt=bad_dt), self.assertRaises(ValueError):
                controller.step(bad_dt, [0]*3, [0]*3, [0]*3)
        with self.assertRaises(ValueError):
            controller.step(.02, [float("nan"), 0, 0], [0]*3, [0]*3)
        with self.assertRaises(ValueError):
            C.make_controller("pid", {}, {"gain": [1]*3, "tau": [0]*3})

    def test_pid_derivative_has_no_target_step_kick(self):
        controller = C.make_controller("pid", {"pid": {"ki": 0., "kd": 10.}}, MODEL)
        controller.reset([0]*3, [0]*3, [0]*3)
        command = controller.step(.02, [0]*3, [0]*3, [.2]*3)
        np.testing.assert_allclose(command, [.24]*3, atol=1e-12)

    def test_pid_tracks_actual_clipped_output_for_antiwindup(self):
        controller = C.make_controller("pid", {}, MODEL)
        controller.reset([0]*3, [0]*3, [0]*3)
        request = controller.step(.02, [0]*3, [0]*3, [1]*3)
        before = controller.integral.copy()
        controller.accept_applied([.02]*3, .02)
        self.assertTrue(np.all(controller.integral < before))
        np.testing.assert_allclose(controller.applied, [.02]*3)
        for _ in range(500):
            controller.step(.02, [0]*3, [0]*3, [10]*3)
            controller.accept_applied([.4]*3, .02)
        self.assertTrue(np.all(np.abs(controller.integral) <= .3))

    def test_pid_and_ladrc_track_three_axis_steps(self):
        for name in ("pid", "ladrc"):
            with self.subTest(controller=name):
                _, trace = experiment(name)
                positions = np.array([r[0] for r in trace])
                self.assertLess(np.linalg.norm(positions[-1]-trace[-1][2]), .015)
                self.assertLess(np.max(np.abs(positions)), .3)

    def test_ladrc_observer_rejects_constant_acceleration_disturbance(self):
        controller, trace = experiment("ladrc", duration=15., disturbance=True)
        self.assertLess(np.linalg.norm(trace[-1][0]), .005)
        np.testing.assert_allclose(controller.z[:, 2], [.2, -.1, .15], atol=.01)

    def test_ladrc_exact_observer_remains_stable_at_high_bandwidth(self):
        controller = C.make_controller("ladrc", {"ladrc": {"wo": 100.}}, MODEL)
        controller.reset([0]*3, [0]*3, [0]*3)
        for _ in range(200):
            controller.step(.05, [.1]*3, [0]*3, [.1]*3)
            controller.accept_applied([0]*3, .05)
        np.testing.assert_allclose(controller.z[:, 0], [.1]*3, atol=1e-9)
        np.testing.assert_allclose(controller.z[:, 1:], 0, atol=1e-8)

    def test_ladrc_uses_sent_command_in_observer(self):
        sent = C.make_controller("ladrc", {}, MODEL)
        zero = C.make_controller("ladrc", {}, MODEL)
        for controller in (sent, zero):
            controller.reset([0]*3, [0]*3, [0]*3)
            controller.step(.02, [0]*3, [0]*3, [1]*3)
        sent.accept_applied([.1]*3, .02)
        zero.accept_applied([0]*3, .02)
        sent.step(.02, [0]*3, [0]*3, [1]*3)
        zero.step(.02, [0]*3, [0]*3, [1]*3)
        self.assertGreater(np.linalg.norm(sent.z-zero.z), .001)

    def test_reset_removes_previous_trial_memory(self):
        for name in ("pid", "ladrc"):
            controller = C.make_controller(name, {}, MODEL)
            controller.reset([0]*3, [0]*3, [0]*3)
            first = controller.step(.02, [0]*3, [0]*3, [.2]*3)
            for _ in range(20):
                controller.accept_applied([.2]*3, .02)
                controller.step(.02, [.1]*3, [.1]*3, [.2]*3)
            controller.reset([0]*3, [0]*3, [0]*3)
            np.testing.assert_allclose(controller.step(.02, [0]*3, [0]*3, [.2]*3), first)

    @unittest.skipUnless(importlib.util.find_spec("osqp"), "OSQP comparison environment required")
    def test_mpc_closes_loop_and_obeys_speed_and_slew_bounds(self):
        _, trace = experiment("mpc", duration=8.)
        commands = np.array([r[1] for r in trace])
        self.assertLess(np.linalg.norm(trace[-1][0]-trace[-1][2]), .005)
        self.assertLessEqual(np.max(np.abs(commands)), .4+1e-8)
        self.assertLessEqual(np.max(np.abs(np.diff(np.vstack((np.zeros(3),commands)),axis=0))), .020001)

    @unittest.skipUnless(importlib.util.find_spec("osqp"), "OSQP comparison environment required")
    def test_mpc_failure_is_explicit_and_never_falls_back(self):
        controller = C.make_controller("mpc", {}, MODEL)
        controller.reset([0]*3, [0]*3, [0]*3)
        info = SimpleNamespace(status="primal infeasible", status_val=3, iter=25,
                               run_time=.001, prim_res=1., dual_res=0.)
        controller.solver = SimpleNamespace(update=lambda **_: None,
                                           solve=lambda **_: SimpleNamespace(info=info,x=None))
        with self.assertRaisesRegex(C.ControllerError, "primal infeasible"):
            controller.step(.02, [0]*3, [0]*3, [.2]*3)
        np.testing.assert_array_equal(controller.applied, np.zeros(3))
        self.assertEqual(controller.diagnostics["solver_status_val"], 3)

    @unittest.skipUnless(importlib.util.find_spec("osqp"), "OSQP comparison environment required")
    def test_conservative_configuration_is_bounded_with_unmodelled_delay(self):
        # A robustness check for the identified effective model: the independent
        # stress plant has explicit delays that none of these controllers sees.
        # This is not a Gazebo result or a ranking of controller performance.
        effective = {"gain": [1.197187, 1.109495, 1.079271],
                     "tau": [.830645, .620470, .262687], "delay_s": [0,0,0]}
        config = {"ladrc": {"wc": [1,1,2], "wo": [4,4,8]},
                  "mpc": {"q_position": [9,9,25], "q_velocity": [4,4,1],
                          "r_input": [2,2,.2], "r_delta": [5,5,1]}}
        true_gain, true_tau = np.array([1.13,1.11,1.05]), np.array([.5,.4,.16])
        delayed_steps = np.rint(np.array([.25,.25,.10])/.02).astype(int)
        for name in ("pid", "ladrc", "mpc"):
            with self.subTest(controller=name):
                controller = C.make_controller(name, config, effective)
                position, velocity, applied = np.zeros(3), np.zeros(3), np.zeros(3)
                controller.reset(position, velocity, applied)
                history = [np.zeros(3)]*(max(delayed_steps)+1)
                largest_position = 0.
                for k in range(1500):
                    target = np.ones(3)*(.2 if 2 <= k*.02 < 12 else 0.)
                    raw = controller.step(.02, position, velocity, target)
                    applied = np.clip(np.clip(raw,-.4,.4),applied-.02,applied+.02)
                    controller.accept_applied(applied,.02)
                    history.append(applied.copy())
                    delayed = np.array([history[-1-int(n)][i] for i,n in enumerate(delayed_steps)])
                    for _ in range(20):
                        velocity += (true_gain*delayed-velocity)/true_tau*.001
                        position += velocity*.001
                    largest_position = max(largest_position,float(np.max(np.abs(position))))
                self.assertLess(largest_position,.3)
                self.assertLess(np.linalg.norm(position),.01)


if __name__ == "__main__":
    unittest.main()
