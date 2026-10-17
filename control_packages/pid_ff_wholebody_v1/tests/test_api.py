"""Packaging regressions against the archived, validated control implementation.

These tests use deterministic synthetic state inputs, not a new flight and not
reconstructed historical control commands. The oracle is the original core plus
the feedforward/filter/bridge composition from run_case.py. No archived files are
modified. Run from the package directory with ``python -m unittest discover -s tests``.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[1]
LEGACY = PACKAGE / "tests" / "fixtures" / "legacy_v1"
sys.path.insert(0, str(PACKAGE))

from flyarm_pid_ff import (  # noqa: E402
    ArmParameters, ControlState, EndpointTarget, FigureEight,
    PidFeedforwardController, frozen_pid_parameters, tip_world,
)


def load_archived_modules():
    """Load the legacy top-level imports without leaving generic module names."""
    provenance = json.loads((PACKAGE / "flyarm_pid_ff/data/provenance.json").read_text(encoding="utf-8"))
    for entry in provenance["files"]:
        source = LEGACY / Path(entry["original_path"]).name
        if hashlib.sha256(source.read_bytes()).hexdigest() != entry["original_sha256"]:
            raise RuntimeError("Frozen regression reference changed: " + source.name)
    names = ("braking", "kinematics", "trajectory", "feedforward", "control_adapter", "controller")
    previous = {name: sys.modules.get(name) for name in names}
    old_path = sys.path[:]
    loaded = {}
    try:
        for name in names:
            sys.modules.pop(name, None)
        sys.path.insert(0, str(LEGACY))
        for name in names:
            spec = importlib.util.spec_from_file_location(name, LEGACY / (name + ".py"))
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            spec.loader.exec_module(module)
            loaded[name] = module
    finally:
        sys.path[:] = old_path
        for name, value in previous.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value
    return loaded


def rotation_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def state(stamp=100., phase=0., *, q=None, rotation=None, omega=None):
    return ControlState(
        position=np.array([.1 + .014*np.sin(phase), -.2 + .006*np.sin(.8*phase), 2.7 + .01*np.sin(.6*phase)]),
        rotation=rotation_y(.018*np.sin(phase)) if rotation is None else rotation,
        velocity=np.array([.024*np.cos(phase), .005*np.sin(phase), -.015*np.sin(.7*phase)]),
        omega_body=np.array([.02*np.sin(phase), .04*np.cos(phase), .01*np.sin(.3*phase)]) if omega is None else omega,
        q=np.array([.025*np.sin(.4*phase), -.02*np.sin(.5*phase)]) if q is None else q,
        dq=np.array([.01*np.cos(.4*phase), -.01*np.cos(.5*phase)]),
        stamp_s=stamp,
    )


def initial_state():
    return ControlState(np.array([.1, -.2, 2.7]), np.eye(3), np.zeros(3),
                        np.zeros(3), np.zeros(2), np.zeros(2), 100.)


class LegacyMotionFilter:
    """Literal timestamp and low-pass update used by archived run_case.py."""
    def __init__(self):
        self.previous_stamp = None
        self.previous_w = None
        self.alpha = np.zeros(3)

    def observe(self, measurement):
        stamp = measurement.stamp_s
        wb = np.asarray(measurement.omega_body)
        if self.previous_stamp is not None and stamp < self.previous_stamp - 1e-9:
            raise ValueError("Truth timestamp regressed")
        if self.previous_stamp is not None and stamp > self.previous_stamp + 1e-9:
            sample_dt = stamp - self.previous_stamp
            raw_alpha = (wb - self.previous_w) / sample_dt
            weight = sample_dt / (.12 + sample_dt)
            self.alpha = (1-weight)*self.alpha + weight*raw_alpha
        if self.previous_stamp is None or stamp > self.previous_stamp + 1e-9:
            self.previous_w = wb.copy()
            self.previous_stamp = stamp
        return self.alpha.copy()


def command_arrays(command):
    return {name: np.asarray(getattr(command, name)).copy() for name in (
        "position_reference", "velocity_feedforward", "acceleration_feedforward",
        "base_velocity_target", "joint_velocity", "nominal_acceleration",
        "arm_acceleration", "tip_position", "tip_target",
    )}


class ApiBehaviorTests(unittest.TestCase):
    def assert_commands_equal(self, actual, expected):
        for name, value in command_arrays(actual).items():
            np.testing.assert_allclose(value, command_arrays(expected)[name], rtol=0, atol=1e-12, err_msg=name)

    def test_instances_are_independent_and_reset_restores_fresh_sequence(self):
        s0 = initial_state()
        a, b, fresh = (PidFeedforwardController(s0) for _ in range(3))
        trajectory = FigureEight(center=tip_world(s0.position, s0.rotation, s0.q), period_s=8.)
        for i in range(36):
            t = i*.02
            sample = state(100.+t, .04*i)
            target = EndpointTarget(*trajectory.sample(t))
            expected = fresh.step(t, .02, sample, target)
            actual = a.step(t, .02, sample, target)
            b.step(t, .02, state(100.+t, -.08*i), target)
            self.assert_commands_equal(actual, expected)
        a.reset(s0)
        fresh = PidFeedforwardController(s0)
        for i in range(10):
            sample = state(100.+i*.02, .08*i)
            target = EndpointTarget(*trajectory.sample(i*.02))
            self.assert_commands_equal(a.step(i*.02,.02,sample,target), fresh.step(i*.02,.02,sample,target))

    def test_motion_filter_repeated_stamps_and_preserved_reset(self):
        control = PidFeedforwardController(initial_state())
        oracle = LegacyMotionFilter()
        samples = [state(100.,0), state(100.02,.3), state(100.02,1.7), state(100.06,.8)]
        for sample in samples:
            np.testing.assert_allclose(control.observe(sample),oracle.observe(sample),rtol=0,atol=1e-12)
        last = samples[-1]
        control.reset(last, preserve_motion_filter=True)
        sample = state(100.08,1.2)
        np.testing.assert_allclose(control.observe(sample),oracle.observe(sample),rtol=0,atol=1e-12)
        with self.assertRaises(ValueError):
            control.observe(state(100.01,1.3))
        control.reset(initial_state())
        np.testing.assert_allclose(control.observe(state(100.,2.)),np.zeros(3),atol=0)

    def test_quaternion_and_matrix_input_produce_same_commands(self):
        a, b = (PidFeedforwardController(initial_state()) for _ in range(2))
        target = EndpointTarget(tip_world([.1,-.2,2.7],np.eye(3),[0,0]),np.zeros(3),np.zeros(3))
        theta = .035
        qa = state(100.,.2,rotation=rotation_y(theta))
        qb = state(100.,.2,rotation=[np.cos(theta/2),0,np.sin(theta/2),0])
        self.assert_commands_equal(a.step(0,.02,qa,target),b.step(0,.02,qb,target))

    def test_invalid_inputs_are_rejected(self):
        s0 = initial_state()
        target = EndpointTarget(tip_world(s0.position,s0.rotation,s0.q),np.zeros(3),np.zeros(3))
        for dt in (0., -.02, .051, float("nan")):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                PidFeedforwardController(s0).step(0,dt,s0,target)
        with self.assertRaises(ValueError):
            PidFeedforwardController(s0,mode="typo")
        for kwargs in ({"rotation":np.ones((3,3))},{"rotation":[0,0,0,0]},{"q":[float("nan"),0]}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                PidFeedforwardController(s0).step(0,.02,state(**kwargs),target)
        with self.assertRaises(ValueError):
            PidFeedforwardController(s0).step(0,.02,state(q=[.46,0]),target)
        with self.assertRaises(ValueError):
            PidFeedforwardController(s0).step(0,.02,s0,EndpointTarget([float("inf"),0,0],[0,0,0],[0,0,0]))

    def test_frozen_pid_returns_an_independent_copy(self):
        a = frozen_pid_parameters()
        self.assertEqual(len(a),26)
        self.assertEqual(a["MPC_XY_P"],2.)
        a["MPC_XY_P"] = -999.
        a["INJECTED"] = 1.
        b = frozen_pid_parameters()
        self.assertEqual(b["MPC_XY_P"],2.)
        self.assertNotIn("INJECTED",b)

    def test_import_does_not_hijack_generic_modules_or_search_path(self):
        # A clean interpreter isolates this claim from the legacy oracle loader.
        code = """
import json, sys, types
sys.path.insert(0, sys.argv[1])
names = ('controller','trajectory','braking','kinematics','feedforward','control_adapter')
sentinels = {name:types.ModuleType(name) for name in names}
sys.modules.update(sentinels)
before = sys.path[:]
import flyarm_pid_ff
assert sys.path == before, (before,sys.path)
assert all(sys.modules[name] is value for name,value in sentinels.items())
print(json.dumps({'generic_module_names_preserved':True,'sys_path_unchanged':True}))
"""
        result = subprocess.run([sys.executable,"-I","-c",code,str(PACKAGE)],text=True,capture_output=True,check=True)
        self.assertTrue(json.loads(result.stdout)["generic_module_names_preserved"])


class ArchivedImplementationRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old = load_archived_modules()

    def compare_composition(self, new, old_ref, measurement, alpha, params):
        ff = self.old["feedforward"].arm_reaction_feedforward(
            measurement.q,measurement.dq,old_ref["qdd_ref"],
            R_world_body=measurement.rotation,omega_body=measurement.omega_body,
            alpha_body=alpha,base_acceleration_world=old_ref["a_ref"],params=params,acceleration_limit=1.)
        acceleration,_ = self.old["control_adapter"].clip_norm(old_ref["a_ref"]+ff["acceleration_world"],3.)
        expected = {
            "position_reference":old_ref["p_ref"],"velocity_feedforward":old_ref["v_ref"],
            "acceleration_feedforward":acceleration,"joint_velocity":old_ref["qd_cmd"],
            "nominal_acceleration":old_ref["a_ref"],"arm_acceleration":ff["acceleration_world"],
            "base_velocity_target":old_ref["v_ref"]+np.array([2.,2.,1.])*(old_ref["p_ref"]-measurement.position),
            "tip_position":self.old["kinematics"].tip_world(measurement.position,measurement.rotation,measurement.q),
            "tip_target":old_ref["tip_ref"],
        }
        for name,value in expected.items():
            np.testing.assert_allclose(getattr(new,name),value,rtol=0,atol=5e-12,err_msg=name)
        ep,ev=np.array([1.2,-2.1,-2.4]),np.array([.1,-.15,.07])
        old_sp=self.old["control_adapter"].bridge(ep,ev,old_ref["p_ref"],old_ref["v_ref"],acceleration,measurement.position,measurement.velocity)
        sp=new.px4_setpoint(ep,ev)
        for name,value in zip(("position","velocity","acceleration"),old_sp):
            np.testing.assert_allclose(getattr(sp,name),value,rtol=0,atol=5e-12,err_msg="PX4 "+name)
        self.assertEqual(sp.type_mask,2048)
        # PX4 position P must be present exactly once in the resulting velocity error.
        ned_gain=np.array([2.,2.,1.])
        ned_error=ned_gain*(sp.position-ep)+(sp.velocity-ev)
        expected_error=self.old["control_adapter"].enu_to_ned(new.base_velocity_target-measurement.velocity)
        np.testing.assert_allclose(ned_error,expected_error,rtol=0,atol=5e-12)

    def test_all_26_pid_values_match_archived_selection(self):
        source=LEGACY/"selected_parameters.json"
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(),"935428d891b59e64b6a0190f0eac888fbf66cbb989578745adefd25eda395dd2")
        self.assertEqual(frozen_pid_parameters(),json.loads(source.read_text())["parameters"])

    def test_sequential_controller_feedforward_and_bridge_match_archived(self):
        s0=initial_state()
        for mode in ("baseline","compensated"):
            with self.subTest(mode=mode):
                trajectory=self.old["trajectory"].FigureEight(center=tip_world(s0.position,s0.rotation,s0.q),period_s=8.)
                oracle=self.old["controller"].WholeBodyClosedLoop(s0.position,s0.q,s0.rotation,trajectory,mode=mode)
                new=PidFeedforwardController(s0,mode=mode)
                params=self.old["feedforward"].ArmParameters()
                motion=LegacyMotionFilter()
                t=0.;stamp=100.
                for i in range(320):
                    dt=(.016,.02,.024,.04)[i%4]
                    if i%7:stamp+=dt
                    measurement=state(stamp,.027*i)
                    alpha=motion.observe(measurement)
                    ref=oracle.step(t,dt,measurement.position,measurement.rotation,measurement.velocity,
                        measurement.rotation@measurement.omega_body,measurement.q,measurement.dq)
                    actual=new.step(t,dt,measurement,EndpointTarget(*trajectory.sample(t)))
                    for key in ("p_ref","v_ref","a_ref","q_ref","qd_ref","qdd_ref","qd_cmd","base_velocity_cmd"):
                        np.testing.assert_allclose(actual.reference[key],ref[key],rtol=0,atol=5e-12,err_msg=key)
                    self.compare_composition(actual,ref,measurement,alpha,params)
                    t+=dt

    def test_hold_to_task_preserves_ff_history_but_does_not_advance_core(self):
        s0=initial_state();new=PidFeedforwardController(s0)
        motion=LegacyMotionFilter();params=self.old["feedforward"].ArmParameters()
        for i in range(12):
            sample=state(100.+i*.02,.1*i)
            alpha=motion.observe(sample)
            ref=dict(p_ref=s0.position,v_ref=np.zeros(3),a_ref=np.zeros(3),q_ref=s0.q,
                qd_ref=np.zeros(2),qdd_ref=np.zeros(2),
                tip_ref=tip_world(s0.position,s0.rotation,s0.q),
                qd_cmd=np.clip(3*(s0.q-sample.q),[-.45,-.35],[.45,.35]))
            self.compare_composition(new.hold(sample),ref,sample,alpha,params)
        trajectory=self.old["trajectory"].FigureEight(center=ref["tip_ref"],period_s=8.)
        oracle=self.old["controller"].WholeBodyClosedLoop(s0.position,s0.q,s0.rotation,trajectory)
        sample=state(100.24,1.4);alpha=motion.observe(sample)
        ref=oracle.step(0,.02,sample.position,sample.rotation,sample.velocity,sample.rotation@sample.omega_body,sample.q,sample.dq)
        actual=new.step(0,.02,sample,EndpointTarget(*trajectory.sample(0)))
        self.compare_composition(actual,ref,sample,alpha,params)


if __name__=="__main__":
    unittest.main()
