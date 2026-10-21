"""Shared FF, timestamps, servo encoding and transparent solver failure."""
from __future__ import annotations
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE.parent/"pid_ff_wholebody_v1"))
from x650_wholebody.adapter import ControllerAdapter
from x650_wholebody.api import ControlState, EndpointTarget
from x650_wholebody.baseline import BaselineController
from x650_wholebody.feedforward import arm_reaction_feedforward
from x650_wholebody.model import tip_world, rotation_matrix
from test_model import example_model

GAINS = {"MPC_XY_P": 1.6, "MPC_Z_P": .8}


def state(*, stamp=1., omega=(0., 0., 0.), q=(.1, .2), dq=(.2, -.1)):
    return ControlState([.1, -.2, 2.], rotation_matrix([.9, .1, -.2, .1]),
                        [.03, -.02, .01], omega, q, dq, stamp)


class AllocatorStub:
    def __init__(self, model, *, a=(.2, -.1, .3), qdd=(.4, -.3), name="stub"):
        self.model, self.a, self.qdd, self.name = model, np.array(a), np.array(qdd), name
        self.calls, self.resets = 0, 0
    def reset(self, p0, R0, q0):
        self.resets += 1
    def step(self, t, dt, measured, target):
        self.calls += 1
        return {"p_ref": np.array([.11, -.23, 2.04]), "base_velocity_cmd": np.array([.05, -.06, .07]),
                "qd_cmd": np.array([.1, -.05]), "qdd_ref": self.qdd.copy(), "a_ref": self.a.copy(),
                "diagnostics": {"allocator_name": self.name, "solver_success": True}}
    def metadata(self):
        return {"name": self.name}


class AdapterTests(unittest.TestCase):
    def adapter(self, *, model=None, allocator=None, **kw):
        model = example_model(elbow_offset_body_m=(0., .04, 0.)) if model is None else model
        allocator = AllocatorStub(model) if allocator is None else allocator
        wrapper = ControllerAdapter(allocator, model, GAINS, **kw)
        s = state()
        wrapper.reset(s.position, s.rotation, s.q)
        return wrapper, allocator, s

    def target(self, s, model):
        return EndpointTarget(tip_world(s.position, s.rotation, s.q, model))

    def test_both_allocator_groups_use_identical_physical_feedforward(self):
        model = example_model(elbow_offset_body_m=(0., .04, 0.))
        outputs = []
        for name in ("PID allocator", "MPC allocator"):
            alloc = AllocatorStub(model, name=name)
            wrapper, _, s = self.adapter(model=model, allocator=alloc)
            out = wrapper.step(0., .02, s, self.target(s, model))
            expected = arm_reaction_feedforward(s.q, s.dq, alloc.qdd, model=model,
                R_world_body=s.rotation, omega_body=s.omega_body, alpha_body=[0., 0., 0.],
                base_acceleration_world=alloc.a, acceleration_limit=1.)
            np.testing.assert_allclose(out.acceleration_feedforward, alloc.a+expected["acceleration_world"], atol=1e-14)
            np.testing.assert_allclose(out.diagnostics["arm_feedforward"]["fixed_body_origin_transport_force_world"], expected["fixed_body_origin_transport_force_world"])
            self.assertFalse(out.diagnostics["reaction_moment_applied"])
            outputs.append(out)
        np.testing.assert_array_equal(outputs[0].acceleration_feedforward, outputs[1].acceleration_feedforward)
        self.assertNotEqual(outputs[0].diagnostics["allocator_name"], outputs[1].diagnostics["allocator_name"])

    def test_actual_external_position_gains_are_cancelled_once(self):
        wrapper, alloc, s = self.adapter()
        out = wrapper.step(0., .02, s, self.target(s, wrapper.model))
        desired = np.array([.05, -.06, .07])
        np.testing.assert_allclose(wrapper.position_gains*(out.position_reference-s.position)+out.velocity_feedforward,
                                   desired, atol=1e-15)
        np.testing.assert_array_equal(out.joint_velocity, [.1, -.05])
        np.testing.assert_allclose(out.base_velocity_target, desired)

    def test_filter_uses_unique_physical_stamps_and_rejects_time_regression(self):
        wrapper, alloc, s = self.adapter()
        np.testing.assert_array_equal(wrapper.observe(s), 0.)
        a = wrapper.observe(state(stamp=1.02, omega=(.2, -.4, .6)))
        np.testing.assert_allclose(a, [.2/.14, -.4/.14, .6/.14], atol=1e-13)
        # A repeated control call with a stale/new value on the same stamp must
        # not replace the omega used by the next true measurement derivative.
        np.testing.assert_array_equal(wrapper.observe(state(stamp=1.02, omega=(99., 99., 99.))), a)
        b = wrapper.observe(state(stamp=1.04, omega=(.4, -.8, 1.2)))
        np.testing.assert_allclose(b, (.12/.14)*a+(.02/.14)*np.array([10., -20., 30.]), atol=1e-12)
        with self.assertRaisesRegex(ValueError, "timestamp regressed"):
            wrapper.step(0., .02, state(stamp=1.01), self.target(s, wrapper.model))
        self.assertEqual(alloc.calls, 0)

    def test_reset_preserves_filter_only_when_requested(self):
        wrapper, alloc, s = self.adapter()
        wrapper.observe(s)
        alpha = wrapper.observe(state(stamp=1.1, omega=(.2, .1, -.1)))
        wrapper.reset(s.position, s.rotation, s.q, preserve_motion_filter=True)
        np.testing.assert_array_equal(wrapper.observe(state(stamp=1.1)), alpha)
        wrapper.reset(s.position, s.rotation, s.q)
        np.testing.assert_array_equal(wrapper.observe(state(stamp=.1)), 0.)
        self.assertEqual(alloc.resets, 3)

    def test_arm_and_total_accelerations_have_separate_norm_caps(self):
        model = example_model(elbow_offset_body_m=(0., .04, 0.))
        alloc = AllocatorStub(model, a=(10., 10., 10.), qdd=(10000., -10000.))
        wrapper, _, s = self.adapter(model=model, allocator=alloc)
        out = wrapper.step(0., .02, s, self.target(s, model))
        self.assertTrue(out.diagnostics["arm_feedforward"]["saturated"])
        self.assertAlmostEqual(np.linalg.norm(out.diagnostics["arm_acceleration_world"]), 1.)
        self.assertTrue(out.diagnostics["acceleration_saturated"])
        self.assertAlmostEqual(np.linalg.norm(out.acceleration_feedforward), 3.)
        self.assertIn("allocation.qdd_ref", out.diagnostics["feedforward_joint_acceleration_source"])

    def test_command_acceleration_source_not_measured_dq_difference(self):
        wrapper, alloc, s = self.adapter()
        d = self.target(s, wrapper.model)
        first = wrapper.step(0., .02, s, d)
        s2 = state(stamp=1.02, dq=(100., -100.))
        second = wrapper.step(.02, .02, s2, d)
        # Measured dq contributes to centrifugal force, but the supplied ddq
        # stays precisely the allocator's bounded command difference.
        np.testing.assert_array_equal(second.diagnostics["allocation"]["qdd_ref"], alloc.qdd)
        direct = arm_reaction_feedforward(s2.q, s2.dq, alloc.qdd, model=wrapper.model,
            R_world_body=s2.rotation, omega_body=s2.omega_body, alpha_body=[0., 0., 0.],
            base_acceleration_world=alloc.a, acceleration_limit=1.)
        np.testing.assert_allclose(second.diagnostics["arm_feedforward"]["raw_acceleration_world"], direct["raw_acceleration_world"], atol=1e-13)

    def test_failure_is_propagated_and_malformed_allocator_output_rejected(self):
        wrapper, alloc, s = self.adapter()
        def fail(*args):
            raise RuntimeError("QP infeasible")
        alloc.step = fail
        with self.assertRaisesRegex(RuntimeError, "QP infeasible"):
            wrapper.step(0., .02, s, self.target(s, wrapper.model))
        self.assertIsNone(wrapper._last_task_t)
        alloc.step = lambda *args: {"p_ref": [0., 0., 0.]}
        with self.assertRaises(KeyError):
            wrapper.step(0., .02, s, self.target(s, wrapper.model))
        with self.assertRaises(ValueError):
            ControllerAdapter(alloc, wrapper.model, {"MPC_XY_P": 0., "MPC_Z_P": 1.})
        with self.assertRaises(ValueError):
            ControllerAdapter(alloc, replace(wrapper.model, total_mass=4.), GAINS)

    def test_shared_wrapper_keeps_real_baseline_servo_and_ff_behavior(self):
        m = example_model(elbow_offset_body_m=(0., .04, 0.))
        a = BaselineController(m, GAINS)
        b = BaselineController(m, GAINS)
        wrapper, _, s = self.adapter(model=m, allocator=a)
        b.reset(s.position, s.rotation, s.q)
        d = self.target(s, m)
        raw = b.step(0., .02, s, d)
        out = wrapper.step(0., .02, s, d)
        np.testing.assert_allclose(out.velocity_feedforward, raw["v_ref"], atol=1e-14)
        np.testing.assert_allclose(out.joint_velocity, raw["qd_cmd"], atol=1e-14)
        expected = arm_reaction_feedforward(s.q, s.dq, raw["qdd_ref"], model=m,
            R_world_body=s.rotation, base_acceleration_world=raw["a_ref"], acceleration_limit=1.)
        np.testing.assert_allclose(out.acceleration_feedforward, raw["a_ref"]+expected["acceleration_world"], atol=1e-13)
        with self.assertRaisesRegex(ValueError, "position gains differ"):
            ControllerAdapter(a, m, {"MPC_XY_P": 2., "MPC_Z_P": 1.})

    def test_complete_legacy_wrapper_behavior_with_preserved_prestream_history(self):
        from flyarm_pid_ff import (PidFeedforwardController, ControlState as OldState,
                                   EndpointTarget as OldTarget)
        from flyarm_pid_ff._core.feedforward import ArmParameters
        from flyarm_pid_ff._core.kinematics import tip_world as old_tip_world
        params = ArmParameters()
        model = example_model(mount_body=params.mount_body, total_mass=params.total_mass,
                              angle_offsets=params.angle_offsets, base_com_body=(0., 0., 0.),
                              elbow_offset_body_m=(0., 0., 0.))
        gains = {"MPC_XY_P": 2., "MPC_Z_P": 1.}
        initial = ControlState([0., 0., 2.], np.eye(3), [0.]*3, [0.]*3, [0.]*2, [0.]*2, 9.9)
        convert = lambda s: OldState(s.position, s.rotation, s.velocity, s.omega_body, s.q, s.dq, s.stamp_s)
        old = PidFeedforwardController(convert(initial))
        wrapper = ControllerAdapter(BaselineController(model, gains), model, gains)
        wrapper.reset(initial.position, initial.rotation, initial.q)
        for stamp in (9.94, 9.96, 9.98):
            s = ControlState(initial.position, initial.rotation, initial.velocity,
                             [0., .002*np.sin(stamp), 0.], initial.q, initial.dq, stamp)
            old.hold(convert(s))
            wrapper.observe(s)
        old.reset(convert(initial), preserve_motion_filter=True)
        wrapper.reset(initial.position, initial.rotation, initial.q, preserve_motion_filter=True)
        center = old_tip_world(initial.position, initial.rotation, initial.q)
        for i in range(120):
            t = i*.02
            R = rotation_matrix([np.cos(.001*np.sin(t)), 0., np.sin(.001*np.sin(t)), 0.])
            s = ControlState(initial.position+[.001*np.sin(t), -.001*np.cos(t), .001*np.sin(2*t)], R,
                             [.001*np.cos(t), .001*np.sin(t), .002*np.cos(2*t)],
                             [0., .002*np.cos(t), 0.], [.01*np.sin(t), -.01*np.cos(t)],
                             [.01*np.cos(t), .01*np.sin(t)], 10.+t)
            d = EndpointTarget(center+[.02*np.sin(t), 0., .01*np.sin(2*t)],
                               [.02*np.cos(t), 0., .02*np.cos(2*t)],
                               [-.02*np.sin(t), 0., -.04*np.sin(2*t)])
            a = old.step(t, .02, convert(s), OldTarget(d.position, d.velocity, d.acceleration))
            b = wrapper.step(t, .02, s, d)
            for name in ("position_reference", "velocity_feedforward", "acceleration_feedforward",
                         "base_velocity_target", "joint_velocity"):
                np.testing.assert_allclose(getattr(a, name), getattr(b, name), atol=1e-9, rtol=1e-8, err_msg=name)
            np.testing.assert_allclose(a.diagnostics["filtered_alpha_body"], b.diagnostics["filtered_alpha_body"], atol=1e-14)


if __name__ == "__main__":
    unittest.main()
