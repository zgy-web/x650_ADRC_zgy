"""Callable, simulation-independent facade for the validated PID/FF profile.

Positions and linear velocities are world ENU; rotations map body FLU to ENU.
Angular rates/accelerations are body FLU, joint units are rad and rad/s, and
timestamps are simulation seconds. PX4 executes the PID; this API supplies
its frozen gains and servo-consistent references, not another PID integrator.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np

from ._core.controller import WholeBodyClosedLoop
from ._core.control_adapter import bridge, clip_norm, POSITION_TARGET_MASK
from ._core.feedforward import ArmParameters, arm_reaction_feedforward
from ._core.kinematics import rotation_matrix, tip_world, _vector

VERSION = "1.0.0"
PID_SHA256 = "935428d891b59e64b6a0190f0eac888fbf66cbb989578745adefd25eda395dd2"
DATA = Path(__file__).resolve().parent / "data"


def _array(value, size, name):
    result = _vector(value, size, name).copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class ControlState:
    position: np.ndarray
    rotation: np.ndarray
    velocity: np.ndarray
    omega_body: np.ndarray
    q: np.ndarray
    dq: np.ndarray
    stamp_s: float

    def __post_init__(self):
        for name, size in (("position", 3), ("velocity", 3), ("omega_body", 3), ("q", 2), ("dq", 2)):
            object.__setattr__(self, name, _array(getattr(self, name), size, name))
        matrix = rotation_matrix(self.rotation).copy()
        if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6) or np.linalg.det(matrix) < 0:
            raise ValueError("rotation must be a proper body-to-world rotation")
        matrix.setflags(write=False)
        object.__setattr__(self, "rotation", matrix)
        if not np.isfinite(self.stamp_s) or self.stamp_s < 0:
            raise ValueError("stamp_s must be a finite nonnegative simulation timestamp")


@dataclass(frozen=True)
class EndpointTarget:
    position: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self):
        for name in ("position", "velocity", "acceleration"):
            object.__setattr__(self, name, _array(getattr(self, name), 3, name))


@dataclass(frozen=True)
class Px4Setpoint:
    """NED references for SET_POSITION_TARGET_LOCAL_NED (yaw supplied by caller)."""
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    type_mask: int = POSITION_TARGET_MASK


@dataclass(frozen=True)
class ControlCommand:
    position_reference: np.ndarray
    velocity_feedforward: np.ndarray
    acceleration_feedforward: np.ndarray
    base_velocity_target: np.ndarray
    joint_velocity: np.ndarray
    nominal_acceleration: np.ndarray
    arm_acceleration: np.ndarray
    tip_position: np.ndarray
    tip_target: np.ndarray
    diagnostics: dict
    reference: dict
    _feedback_position: np.ndarray = field(repr=False)
    _feedback_velocity: np.ndarray = field(repr=False)

    def px4_setpoint(self, px4_position_ned, px4_velocity_ned):
        """Encode the exact feedback errors of this command's state snapshot.

        Send these three references together with type_mask=2048. Sending
        base_velocity_target together with position_reference directly would
        add PX4's position feedback a second time. Do not add joint P to
        joint_velocity either: it is already the executed velocity target.
        """
        p, v, a = bridge(px4_position_ned, px4_velocity_ned,
                         self.position_reference, self.velocity_feedforward,
                         self.acceleration_feedforward,
                         self._feedback_position, self._feedback_velocity)
        return Px4Setpoint(p, v, a)


def frozen_pid_parameters():
    """Return a fresh copy of all 26 gains, after checking the frozen resource."""
    content = (DATA / "selected_parameters.json").read_bytes()
    if hashlib.sha256(content).hexdigest() != PID_SHA256:
        raise RuntimeError("Frozen PID resource changed")
    gains = json.loads(content)["parameters"]
    if len(gains) != 26 or not all(np.isfinite(v) for v in gains.values()):
        raise RuntimeError("Invalid frozen PID profile")
    return dict(gains)


def provenance():
    """Describe original numerical sources; no claim that packaging is a new flight."""
    return json.loads((DATA / "provenance.json").read_text(encoding="utf-8"))


class _TargetSource:
    def __init__(self, target):
        self.target = target

    def sample(self, _t):
        return self.target.position, self.target.velocity, self.target.acceleration


class PidFeedforwardController:
    """Stateful instance of the frozen x500_underfold PID+FF whole-body profile.

    One instance per vehicle/run. ``initial_state`` anchors the nominal base
    pose and joint posture. It does not prime the angular acceleration filter.
    Use hold()/observe() during prestream; then step() preserves that filter
    history while the allocator starts from its original zero command.
    Default mode enables endpoint feedback; baseline retains the same PID/FF.
    """
    control_period_s = .02
    nominal_arm_share = .2
    angular_acceleration_filter_tau_s = .12
    arm_acceleration_limit_m_s2 = 1.
    total_acceleration_limit_m_s2 = 3.

    def __init__(self, initial_state, *, arm_parameters=None, mode="compensated"):
        if mode not in ("baseline", "compensated"):
            raise ValueError("mode must be baseline or compensated")
        self.mode = mode
        self.arm_parameters = arm_parameters or ArmParameters()
        # Kinematics/PID are frozen for this particular model, not a generic arm.
        expected = ArmParameters()
        for name in expected.__dataclass_fields__:
            actual = np.asarray(getattr(self.arm_parameters, name), float)
            reference = np.asarray(getattr(expected, name), float)
            if actual.shape != reference.shape or not np.isfinite(actual).all() or not np.allclose(actual, reference, rtol=1e-8, atol=1e-10):
                raise ValueError("Model differs from the validated x500_underfold profile: " + name)
        frozen_pid_parameters()
        self._previous_stamp = None
        self._previous_omega = None
        self._alpha_body = np.zeros(3)
        self.reset(initial_state)

    def reset(self, initial_state, *, preserve_motion_filter=False):
        """Reset allocator/task time; clear observer history unless explicitly retained.

        After a simulator clock reset use the default (False). Retaining the
        motion filter is useful when restarting a task after prestream in the
        same simulation clock; it must not be used across different vehicles.
        """
        if not isinstance(initial_state, ControlState):
            raise TypeError("initial_state must be ControlState")
        source = _TargetSource(EndpointTarget(tip_world(initial_state.position, initial_state.rotation, initial_state.q)))
        core = WholeBodyClosedLoop(initial_state.position, initial_state.q, initial_state.rotation,
                                  source, mode=self.mode, config={"arm_share": .2})
        self._anchor, self._source, self._core = initial_state, source, core
        self._last_task_t = None
        if not preserve_motion_filter:
            self._previous_stamp = None
            self._previous_omega = None
            self._alpha_body = np.zeros(3)

    def observe(self, state):
        """Update the original .12 s body angular-acceleration filter once per stamp."""
        if not isinstance(state, ControlState):
            raise TypeError("state must be ControlState")
        if self._previous_stamp is not None and state.stamp_s < self._previous_stamp - 1e-9:
            raise ValueError("State timestamp regressed; reset after a simulation clock reset")
        if self._previous_stamp is not None and state.stamp_s > self._previous_stamp + 1e-9:
            sample_dt = state.stamp_s - self._previous_stamp
            raw_alpha = (state.omega_body - self._previous_omega) / sample_dt
            weight = sample_dt / (self.angular_acceleration_filter_tau_s + sample_dt)
            self._alpha_body = (1 - weight) * self._alpha_body + weight * raw_alpha
        if self._previous_stamp is None or state.stamp_s > self._previous_stamp + 1e-9:
            self._previous_stamp = float(state.stamp_s)
            self._previous_omega = state.omega_body.copy()
        return self._alpha_body.copy()

    def _command(self, state, ref, alpha, *, phase):
        prediction = arm_reaction_feedforward(
            state.q, state.dq, ref["qdd_ref"], R_world_body=state.rotation,
            omega_body=state.omega_body, alpha_body=alpha,
            base_acceleration_world=ref["a_ref"], params=self.arm_parameters,
            acceleration_limit=self.arm_acceleration_limit_m_s2)
        acceleration, saturated = clip_norm(ref["a_ref"] + prediction["acceleration_world"],
                                             self.total_acceleration_limit_m_s2)
        joint_velocity = np.asarray(ref["qd_cmd"])
        if np.any(np.abs(joint_velocity) > np.array([.45, .35]) + 1e-9):
            raise RuntimeError("Allocator exceeded frozen joint velocity limits")
        base_target = ref["v_ref"] + np.array([2., 2., 1.]) * (ref["p_ref"] - state.position)
        diagnostics = deepcopy(ref.get("diagnostics", {}))
        diagnostics.update(phase=phase, filtered_alpha_body=alpha.copy(),
                           arm_feedforward=prediction, acceleration_saturated=saturated,
                           state_stamp_s=float(state.stamp_s), mode=self.mode)
        return ControlCommand(
            ref["p_ref"].copy(), ref["v_ref"].copy(), acceleration.copy(), base_target,
            joint_velocity.copy(), ref["a_ref"].copy(), prediction["acceleration_world"].copy(),
            tip_world(state.position, state.rotation, state.q), ref["tip_ref"].copy(),
            diagnostics, deepcopy(ref), state.position.copy(), state.velocity.copy())

    def hold(self, state):
        """Original prestream/washout hold; keeps FF history but not allocator history.

        Call only before the first task step (or after reset). This initialization
        hold uses the original joint position P and speed cap, not task-time
        acceleration/stopping bounds. Task tracking must use step().
        """
        if self._last_task_t is not None:
            raise RuntimeError("hold is for initialization; reset before a new task")
        alpha = self.observe(state)
        ref = dict(p_ref=self._anchor.position.copy(), v_ref=np.zeros(3), a_ref=np.zeros(3),
                   q_ref=self._anchor.q.copy(), qd_ref=np.zeros(2), qdd_ref=np.zeros(2),
                   tip_ref=tip_world(self._anchor.position, self._anchor.rotation, self._anchor.q),
                   qd_cmd=np.clip(3 * (self._anchor.q - state.q), [-.45, -.35], [.45, .35]))
        return self._command(state, ref, alpha, phase="hold")

    def step(self, t, dt, state, target):
        """Compute one task command from a measured state and world-frame target.

        t is task-relative simulation time; dt is the actual controller interval
        (0 < dt <= .05 s). Targets must remain within the original nominal arm
        workspace and rate limits; an infeasible request raises an exception.
        """
        if not isinstance(state, ControlState) or not isinstance(target, EndpointTarget):
            raise TypeError("step requires ControlState and EndpointTarget")
        if not np.isfinite(t) or t < 0 or not np.isfinite(dt) or not 0 < dt <= .05 + 1e-10:
            raise ValueError("Require t >= 0 and 0 < dt <= .05 simulation seconds")
        if self._last_task_t is not None and t <= self._last_task_t:
            raise ValueError("Task time must increase; reset before starting a new task")
        if self._previous_stamp is not None and state.stamp_s < self._previous_stamp - 1e-9:
            raise ValueError("State timestamp regressed")
        self._source.target = target
        ref = self._core.step(t, dt, state.position, state.rotation, state.velocity,
                              state.rotation @ state.omega_body, state.q, state.dq)
        alpha = self.observe(state)
        command = self._command(state, ref, alpha, phase="tracking")
        self._last_task_t = float(t)
        return command

    def metadata(self):
        return dict(version=VERSION, pid_sha256=PID_SHA256, model="x500_underfold",
                    controller=self._core.metadata(), control_period_s=self.control_period_s,
                    motion_filter_tau_s=self.angular_acceleration_filter_tau_s,
                    feedforward="Relative arm translational inertia; reaction moment diagnostic only",
                    feedback="Simulation truth-assisted PX4 encoding; caller supplies measured states",
                    pid_execution="PX4 position/velocity/attitude/rate loops with frozen 26 gains")
