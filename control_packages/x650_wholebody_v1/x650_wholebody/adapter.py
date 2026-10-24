"""Common measured-state feedforward and PX4 encoding for PID / upper MPC.

Allocators return executed base/joint velocities and a bounded joint-command
acceleration. This adapter applies the same physical FF and saturation in both
groups; it never converts a solver failure into a successful PID command.
"""
from __future__ import annotations

from copy import deepcopy
import numpy as np

from .api import ControlCommand
from .feedforward import arm_reaction_feedforward
from .model import vector, rotation_matrix


_DIAGNOSTIC_ATOMS = frozenset((type(None), bool, int, float, complex, str, bytes))
_DIAGNOSTIC_MISSING = object()


def _copy_numeric_diagnostics(value, memo=None):
    """Detach a diagnostic graph using its common numeric/container types.

    This is the private owned-result path, not a replacement for public
    deepcopy behavior. Memoization retains repeated references and cycles.
    Object arrays, container subclasses and any unfamiliar type still use
    deepcopy with the same memo, preserving their own copy protocol.
    """
    kind = type(value)
    if kind in _DIAGNOSTIC_ATOMS:
        return value
    if memo is None:
        memo = {}
    identity = id(value)
    saved = memo.get(identity, _DIAGNOSTIC_MISSING)
    if saved is not _DIAGNOSTIC_MISSING:
        return saved
    if kind is dict:
        result = {}
        memo[identity] = result
        for key, item in value.items():
            result[_copy_numeric_diagnostics(key, memo)] = _copy_numeric_diagnostics(item, memo)
    elif kind is list:
        result = []
        memo[identity] = result
        result.extend(_copy_numeric_diagnostics(item, memo) for item in value)
    elif kind is np.ndarray and not value.dtype.hasobject:
        result = value.copy(order='K')
        memo[identity] = result
    else:
        result = deepcopy(value, memo)
    return result


def clip_norm(value, maximum):
    value = vector(value, 3, "acceleration")
    norm = float(np.linalg.norm(value))
    return value*min(1., maximum/max(norm, 1e-12)), bool(norm > maximum)


class ControllerAdapter:
    """One adapter per model / task, sharing dynamics FF across allocators.

    Call ``observe(state)`` while prestreaming to accumulate the angular
    acceleration filter. ``reset(..., preserve_motion_filter=True)`` then
    resets task/allocator history while retaining that physical observation
    history within the same simulation clock. A clock reset requires the
    default reset, which clears the filter. Initial hold commands and flight
    lifecycle remain the scenario's responsibility.
    """
    def __init__(self, allocator, model, pid_gains, *,
                 angular_acceleration_filter_tau_s=.12,
                 arm_acceleration_limit=1., total_acceleration_limit=3.,
                 acceleration_feedforward_policy="legacy"):
        self.allocator, self.model = allocator, model
        if acceleration_feedforward_policy not in ("legacy", "zero"):
            raise ValueError("acceleration_feedforward_policy must be legacy or zero")
        self.acceleration_feedforward_policy = acceleration_feedforward_policy
        self.pid_gains = {str(k): float(v) for k, v in pid_gains.items()}
        if not all(np.isfinite(v) for v in self.pid_gains.values()):
            raise ValueError("PID readback values must be finite")
        self.position_gains = np.array([self.pid_gains["MPC_XY_P"], self.pid_gains["MPC_XY_P"], self.pid_gains["MPC_Z_P"]])
        if np.any(self.position_gains <= 0):
            raise ValueError("Positive actual PX4 position-gain readback is required")
        self.angular_acceleration_filter_tau_s = float(angular_acceleration_filter_tau_s)
        self.arm_acceleration_limit = float(arm_acceleration_limit)
        self.total_acceleration_limit = float(total_acceleration_limit)
        if not all(np.isfinite(v) and v > 0 for v in (self.angular_acceleration_filter_tau_s,
                         self.arm_acceleration_limit, self.total_acceleration_limit)):
            raise ValueError("Filter time constant and acceleration bounds must be positive")
        if hasattr(allocator, "model") and allocator.model != model:
            raise ValueError("Allocator and common FF adapter refer to different physical models")
        if hasattr(allocator, "base_position_gain") and not np.allclose(
                vector(allocator.base_position_gain, 3, "allocator position gains"), self.position_gains,
                rtol=0, atol=1e-10):
            raise ValueError("Allocator position gains differ from actual PX4 readback")
        self._previous_stamp = None
        self._previous_omega = None
        self._alpha_body = np.zeros(3)
        self._last_task_t = None
        self._initialized = False
        self._owned_diagnostic_snapshot = False

    def use_owned_diagnostic_snapshot(self):
        """Opt in before stepping to the private live recorder ownership path.

        Public/default commands keep their original independent nested copies.
        The live worker only reads its diagnostics and serializes them before
        the next step. It may share subtrees *within one owned result*, never
        with the allocator or another step; callers must not mutate that live
        result before the synchronous binary handoff completes.
        """
        if self._last_task_t is not None:
            raise RuntimeError("Diagnostic ownership must be selected before the first step")
        self._owned_diagnostic_snapshot = True

    def _snapshot_diagnostics(self, allocation):
        if self._owned_diagnostic_snapshot:
            owned = _copy_numeric_diagnostics(allocation)
            diagnostic = owned.get("diagnostics", {})
            if not isinstance(diagnostic, dict):
                # Do not turn an invalid list-of-pairs into a valid mapping
                # merely because the private path uses dict() below.
                raise TypeError("Allocator diagnostics must be a dictionary")
            # Two distinct top-level dicts are essential: using the same dict
            # here would create a cycle when 'allocation' is attached below.
            # The entire graph was detached once; neither the controller nor
            # future steps can modify its shared per-result descendants.
            return dict(diagnostic), owned
        return deepcopy(allocation.get("diagnostics", {})), deepcopy(allocation)

    def reset(self, p0, R0, q0, *, preserve_motion_filter=False):
        p0, R0, q0 = vector(p0, 3, "p0"), rotation_matrix(R0), vector(q0, 2, "q0")
        # The allocator owns its command and prediction history. Do not retain
        # those states implicitly when restarting a task.
        self.allocator.reset(p0.copy(), R0.copy(), q0.copy())
        self._last_task_t, self._initialized = None, True
        self._anchor = (p0.copy(), R0.copy(), q0.copy())
        if not preserve_motion_filter:
            self._previous_stamp = None
            self._previous_omega = None
            self._alpha_body = np.zeros(3)

    def _motion_sample(self, state):
        stamp = float(state.stamp_s)
        omega = vector(state.omega_body, 3, "omega_body")
        if not np.isfinite(stamp) or stamp < 0:
            raise ValueError("State stamp must be finite and nonnegative")
        if self._previous_stamp is not None and stamp < self._previous_stamp-1e-9:
            raise ValueError("State timestamp regressed; reset after a simulation clock reset")
        return stamp, omega

    def observe(self, state):
        """Filter physical angular acceleration once per new measurement stamp.

        A duplicate stamp neither differentiates nor replaces its saved omega;
        successive control calls using one physical sample remain identical.
        """
        stamp, omega = self._motion_sample(state)
        if self._previous_stamp is not None and stamp > self._previous_stamp+1e-9:
            sample_dt = stamp-self._previous_stamp
            raw_alpha = (omega-self._previous_omega)/sample_dt
            weight = sample_dt/(self.angular_acceleration_filter_tau_s+sample_dt)
            self._alpha_body = (1-weight)*self._alpha_body+weight*raw_alpha
        if self._previous_stamp is None or stamp > self._previous_stamp+1e-9:
            self._previous_stamp = stamp
            self._previous_omega = omega.copy()
        return self._alpha_body.copy()

    def step(self, t, dt, state, target):
        if not self._initialized:
            raise RuntimeError("reset(p0,R0,q0) is required before task step")
        if (not np.isfinite(t) or t < 0 or not np.isfinite(dt) or dt <= 0
                or (self._last_task_t is not None and t <= self._last_task_t)):
            raise ValueError("Task time must increase and dt must be positive")
        # Reject stale/regressing motion before advancing a stateful allocator.
        stamp, _ = self._motion_sample(state)
        R = rotation_matrix(state.rotation)
        q, dq = vector(state.q, 2, "q"), vector(state.dq, 2, "dq")
        allocation = self.allocator.step(t, dt, state, target)
        if not isinstance(allocation, dict):
            raise ValueError("Allocator must return the common allocation dictionary")
        p_ref = vector(allocation["p_ref"], 3, "allocation.p_ref")
        base_velocity = vector(allocation["base_velocity_cmd"], 3, "allocation.base_velocity_cmd")
        joint_velocity = vector(allocation["qd_cmd"], 2, "allocation.qd_cmd")
        nominal_acceleration = vector(allocation["a_ref"], 3, "allocation.a_ref")
        qdd_command = vector(allocation["qdd_ref"], 2, "allocation.qdd_ref")
        alpha = self.observe(state)
        ff = arm_reaction_feedforward(
            q, dq, qdd_command, model=self.model, R_world_body=R,
            omega_body=state.omega_body, alpha_body=alpha,
            base_acceleration_world=nominal_acceleration,
            acceleration_limit=self.arm_acceleration_limit)
        combined = nominal_acceleration+ff["acceleration_world"]
        candidate_acceleration, candidate_saturated = clip_norm(combined, self.total_acceleration_limit)
        acceleration, saturated = candidate_acceleration, candidate_saturated
        if self.acceleration_feedforward_policy == "zero":
            # Retain the physical calculation as diagnostics, but send neither
            # nominal trajectory acceleration nor arm reaction compensation.
            acceleration = np.zeros(3)
            saturated = False
        diagnostic, recorded_allocation = self._snapshot_diagnostics(allocation)
        diagnostic.update(
            allocation=recorded_allocation, filtered_alpha_body=alpha.copy(),
            state_stamp_s=stamp, arm_feedforward=ff,
            nominal_acceleration_world=nominal_acceleration.copy(),
            arm_acceleration_world=ff["acceleration_world"].copy(),
            combined_acceleration_before_limit_world=combined,
            candidate_acceleration_before_policy_world=candidate_acceleration.copy(),
            candidate_acceleration_saturated=candidate_saturated,
            command_acceleration_world=acceleration.copy(),
            acceleration_feedforward_policy=self.acceleration_feedforward_policy,
            acceleration_feedforward_applied=self.acceleration_feedforward_policy == "legacy",
            acceleration_saturated=saturated,
            feedforward_joint_acceleration_source="allocation.qdd_ref: bounded difference of executed joint velocity commands",
            feedforward_angular_acceleration_source="same measured body omega at distinct state.stamp_s; first-order .12 s default filter",
            feedforward_scope="Relative arm inertia plus fixed-body COM angular transport; acceleration force feedforward only",
            reaction_moment_applied=False)
        command = ControlCommand.from_executed_velocity(
            state, base_velocity, joint_velocity, nominal_position=p_ref,
            acceleration_feedforward=acceleration, position_gains=self.position_gains,
            diagnostics=diagnostic)
        self._last_task_t = float(t)
        return command

    def metadata(self):
        allocator = self.allocator.metadata() if hasattr(self.allocator, "metadata") else {
            "class": type(self.allocator).__module__+"."+type(self.allocator).__qualname__}
        return {"name": "x650_common_pid_mpc_force_feedforward_adapter_v1",
                "allocator": allocator, "model": self.model.metadata(),
                "pid_gains": deepcopy(self.pid_gains), "position_gain_readback": self.position_gains.tolist(),
                "angular_acceleration_filter_tau_s": self.angular_acceleration_filter_tau_s,
                "arm_acceleration_norm_limit_m_s2": self.arm_acceleration_limit,
                "total_acceleration_norm_limit_m_s2": self.total_acceleration_limit,
                "acceleration_feedforward_policy": self.acceleration_feedforward_policy,
                "acceleration_feedforward_applied": self.acceleration_feedforward_policy == "legacy",
                "joint_acceleration_source": "allocator's bounded executed-command difference; never measured dq derivative",
                "force_feedforward": "relative arm inertia and fixed COM angular transport",
                "reaction_moment": "diagnostic only; not sent as aircraft or joint torque",
                "failure_behavior": "allocator/input exceptions propagate; no silent controller substitution",
                "frame": "base_link origin; measured world ENU / body FLU"}
