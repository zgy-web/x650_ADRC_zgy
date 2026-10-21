"""State-feedback PID reference coordination for the new X650 model.

This ports the archived servo-consistent five-variable allocator.  PID loops
still execute in PX4; their actual position gains are passed by the caller so
reference encoding does not silently assume the old aircraft's tuning.
Mechanical-arm acceleration feedforward is applied by the common outer API.
"""
from __future__ import annotations
from dataclasses import dataclass
from copy import deepcopy
import numpy as np
from .model import (vector, rotation_matrix, tip_body, tip_world,
                    joint_jacobian_body, joint_centripetal_body, inverse_tip_body)


@dataclass
class BoxLeastSquaresResult:
    x: np.ndarray
    active_mask: np.ndarray
    nit: int


def bounded_least_squares(matrix, rhs, lower, upper, tol=1e-10, max_iter=128):
    """Convex box LSQ active-set solve; no clipping of the final optimum."""
    A, b, lo, hi = (np.asarray(v, dtype=float) for v in (matrix, rhs, lower, upper))
    if (A.ndim != 2 or b.shape != (A.shape[0],) or lo.shape != (A.shape[1],)
            or hi.shape != lo.shape or not all(np.isfinite(v).all() for v in (A, b, lo, hi))
            or np.any(lo > hi)):
        raise ValueError("Invalid finite bounded LSQ problem")
    fixed = lo == hi
    if np.any(fixed):
        result, active = lo.copy(), np.full(len(lo), 2, dtype=int)
        if np.all(fixed):
            return BoxLeastSquaresResult(result, active, 0)
        solved = bounded_least_squares(A[:, ~fixed], b-A[:, fixed]@lo[fixed], lo[~fixed], hi[~fixed], tol, max_iter)
        result[~fixed], active[~fixed] = solved.x, solved.active_mask
        return BoxLeastSquaresResult(result, active, solved.nit)
    x0, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    if rank != A.shape[1]:
        raise ValueError("Bounded allocator requires full column rank")
    x = np.clip(x0, lo, hi)
    active = np.zeros(len(lo), dtype=int)
    active[x0 <= lo], active[x0 >= hi] = -1, 1
    threshold = tol*max(1., float(np.max(np.abs(A.T@b))))
    for iteration in range(1, max_iter+1):
        free = active == 0
        z = x.copy()
        if np.any(free):
            z[free] = np.linalg.lstsq(A[:, free], b-A[:, ~free]@x[~free], rcond=None)[0]
        if np.any(z[free] < lo[free]) or np.any(z[free] > hi[free]):
            direction, ratios = z-x, np.full(len(x), np.inf)
            lower_hit, upper_hit = free & (z < lo), free & (z > hi)
            ratios[lower_hit] = (lo[lower_hit]-x[lower_hit])/direction[lower_hit]
            ratios[upper_hit] = (hi[upper_hit]-x[upper_hit])/direction[upper_hit]
            blocker = int(np.argmin(ratios))
            x += float(np.clip(ratios[blocker], 0., 1.))*direction
            active[blocker] = -1 if lower_hit[blocker] else 1
            x[blocker] = lo[blocker] if lower_hit[blocker] else hi[blocker]
            x = np.clip(x, lo, hi)
            continue
        x = z
        gradient = A.T@(A@x-b)
        violation = np.where(active == -1, -gradient, np.where(active == 1, gradient, -np.inf))
        release = int(np.argmax(violation))
        if violation[release] <= threshold:
            return BoxLeastSquaresResult(x.copy(), active.copy(), iteration)
        active[release] = 0
    raise RuntimeError("Bounded LSQ active set did not converge")


def nominal_reference(p0, R0, q0, target, model, arm_share=.2):
    """Analytic common nominal base/joint p/v/a reference for PID and MPC."""
    p0, q0, R0 = vector(p0, 3, "p0"), vector(q0, 2, "q0"), rotation_matrix(R0)
    p, v, a = (vector(getattr(target, name), 3, "target."+name)
               for name in ("position", "velocity", "acceleration"))
    if not np.isfinite(arm_share) or not 0 <= arm_share <= 1:
        raise ValueError("Nominal arm_share must lie in [0,1]")
    delta = R0.T@(p-tip_world(p0, R0, q0, model))
    delta[1] = 0.
    q = inverse_tip_body(tip_body(q0, model)+arm_share*delta, near=q0, model=model)
    if np.any(q < model.joint_lower) or np.any(q > model.joint_upper):
        raise ValueError("Nominal allocation violates physical joint limits")
    J = joint_jacobian_body(q, model)
    dq = np.linalg.solve(J[[0, 2]], (arm_share*(R0.T@v))[[0, 2]])
    ddq = np.linalg.solve(J[[0, 2]], (arm_share*(R0.T@a)-joint_centripetal_body(q, dq, model))[[0, 2]])
    return {"p_nom": p-R0@tip_body(q, model), "v_nom": v-R0@(J@dq),
            "a_nom": a-R0@(J@ddq+joint_centripetal_body(q, dq, model)),
            "q_nom": q, "qd_nom": dq, "qdd_nom": ddq}


class BaselineController:
    """Actual-state whole-body LSQ reference layer with external PX4 gains.

    ``compensated`` enables measured endpoint/body-velocity compensation;
    ``baseline`` retains independent nominal servo tracking for OFF ablation.
    Neither mode disables the flight PID or the common arm inertia FF.
    """
    def __init__(self, model, pid_gains, *, mode="compensated", config=None):
        if mode not in ("baseline", "compensated"):
            raise ValueError("mode must be baseline or compensated")
        self.model, self.mode = model, mode
        self.pid_gains = {str(k): float(v) for k, v in pid_gains.items()}
        if not all(np.isfinite(v) for v in self.pid_gains.values()):
            raise ValueError("PID gains must be finite")
        self.base_position_gain = np.array([self.pid_gains["MPC_XY_P"], self.pid_gains["MPC_XY_P"], self.pid_gains["MPC_Z_P"]])
        if np.any(self.base_position_gain <= 0):
            raise ValueError("Positive explicit PX4 position gains are required")
        c = dict(config or {})
        self.arm_share = float(c.pop("arm_share", .2))
        self.velocity_limit = vector(c.pop("velocity_limit", [.55, .35, .45, .45, .35]), 5, "velocity_limit").copy()
        self.acceleration_limit = vector(c.pop("acceleration_limit", [.45, .30, .45, .60, .60]), 5, "acceleration_limit").copy()
        self.q_lower = vector(c.pop("q_lower", [-.75, -.35]), 2, "q_lower").copy()
        self.q_upper = vector(c.pop("q_upper", [.45, .85]), 2, "q_upper").copy()
        self.joint_position_gain = float(c.pop("joint_position_gain", 3.))
        self.tip_gain = float(c.pop("tip_gain", 2.))
        self.max_tip_correction = float(c.pop("max_tip_correction_m_s", .20))
        self.task_weight = float(c.pop("task_weight", 1.))
        self.measured_base_weight = float(c.pop("measured_base_weight", 4.))
        self.base_nominal_weight = float(c.pop("base_nominal_weight", 4.))
        self.arm_regularizer_length = float(c.pop("arm_regularizer_length_m_rad", .02))
        self.max_dt = float(c.pop("max_dt_s", .05))
        if c:
            raise ValueError("Unknown baseline options: "+str(sorted(c)))
        scalars = (self.joint_position_gain, self.tip_gain, self.max_tip_correction, self.task_weight,
                   self.measured_base_weight, self.base_nominal_weight, self.arm_regularizer_length, self.max_dt)
        if (not all(np.isfinite(x) and x > 0 for x in scalars) or not 0 <= self.arm_share <= 1
                or np.any(self.velocity_limit <= 0) or np.any(self.acceleration_limit <= 0)
                or np.any(self.q_lower < model.joint_lower) or np.any(self.q_upper > model.joint_upper)
                or np.any(self.q_lower >= self.q_upper)):
            raise ValueError("Invalid positive gains/limits or soft bounds outside physical model")
        self.previous_command = np.zeros(5)
        self.last_t, self.initialized, self.step_count = None, False, 0

    def reset(self, p0, R0, q0):
        self.p0, self.R0, self.q0 = vector(p0, 3, "p0").copy(), rotation_matrix(R0).copy(), vector(q0, 2, "q0").copy()
        if np.any(self.q0 < self.q_lower) or np.any(self.q0 > self.q_upper):
            raise ValueError("Home joints exceed soft bounds")
        self.previous_command = np.zeros(5)
        self.last_t, self.initialized, self.step_count = None, True, 0

    def _bounds(self, dt, q):
        if np.any(q < self.q_lower-1e-8) or np.any(q > self.q_upper+1e-8):
            raise ValueError("Measured joints outside soft position envelope")
        lower = np.maximum(-self.velocity_limit, self.previous_command-self.acceleration_limit*dt)
        upper = np.minimum(self.velocity_limit, self.previous_command+self.acceleration_limit*dt)
        a = self.acceleration_limit[3:]
        reserve = a*self.max_dt
        lower[3:] = np.maximum(lower[3:], -(np.sqrt(reserve**2+2*a*np.maximum(q-self.q_lower, 0.))-reserve))
        upper[3:] = np.minimum(upper[3:], np.sqrt(reserve**2+2*a*np.maximum(self.q_upper-q, 0.))-reserve)
        if np.any(lower > upper+1e-10):
            raise ValueError("Measured q/previous command cannot satisfy acceleration and stopping bounds")
        bad, mid = lower > upper, .5*(lower+upper)
        lower[bad], upper[bad] = mid[bad], mid[bad]
        return lower, upper

    def step(self, t, dt, state, target):
        if not self.initialized:
            raise RuntimeError("reset(p0,R0,q0) is required before step")
        if (not np.isfinite(t) or t < 0 or not np.isfinite(dt) or not 0 < dt <= self.max_dt+1e-10
                or (self.last_t is not None and t <= self.last_t)):
            raise ValueError("Require increasing finite t and 0 < dt <= max_dt")
        p, v, q, dq = (vector(getattr(state, name), size, name)
                       for name, size in (("position", 3), ("velocity", 3), ("q", 2), ("dq", 2)))
        R, omega = rotation_matrix(state.rotation), vector(state.omega_body, 3, "omega_body")
        omega_world = R@omega
        target_p, vd, ad = (vector(getattr(target, name), 3, "target."+name)
                            for name in ("position", "velocity", "acceleration"))
        nominal = nominal_reference(self.p0, self.R0, self.q0, target, self.model, self.arm_share)
        p_nom, q_nom = nominal["p_nom"], nominal["q_nom"]
        nominal_v = np.r_[nominal["v_nom"], nominal["qd_nom"]]
        nominal_a = np.r_[nominal["a_nom"], nominal["qdd_nom"]]
        if (np.any(q_nom < self.q_lower) or np.any(q_nom > self.q_upper)
                or np.any(abs(nominal_v) > self.velocity_limit+1e-9)
                or np.any(abs(nominal_a) > self.acceleration_limit+1e-9)):
            raise ValueError("Nominal trajectory exceeds common soft/rate/acceleration limits")
        base_feedback = self.base_position_gain*(p_nom-p)
        joint_feedback = self.joint_position_gain*(q_nom-q)
        u_nom = nominal_v+np.r_[base_feedback, joint_feedback]
        tip = tip_world(p, R, q, self.model)
        error = target_p-tip
        correction = self.tip_gain*error
        correction *= min(1., self.max_tip_correction/max(float(np.linalg.norm(correction)), 1e-12))
        J = R@joint_jacobian_body(q, self.model)
        rotational_velocity = np.cross(omega_world, R@tip_body(q, self.model))
        desired_tip_velocity = vd+correction
        r = desired_tip_velocity-rotational_velocity
        lower, upper = self._bounds(dt, q)
        if self.mode == "baseline":
            command, iterations = np.clip(u_nom, lower, upper), 0
        else:
            w1, w2, w3 = np.sqrt([self.task_weight, self.measured_base_weight, self.base_nominal_weight])
            A = np.vstack((w1*np.column_stack((np.eye(3), J)),
                           w2*np.column_stack((np.zeros((3, 3)), J)),
                           w3*np.column_stack((np.eye(3), np.zeros((3, 2)))),
                           np.column_stack((np.zeros((2, 3)), self.arm_regularizer_length*np.eye(2)))))
            b = np.r_[w1*r, w2*(r-v), w3*u_nom[:3], self.arm_regularizer_length*u_nom[3:]]
            solved = bounded_least_squares(A, b, lower, upper)
            command, iterations = solved.x, solved.nit
        command_acc = (command-self.previous_command)/dt
        if np.any(abs(command) > self.velocity_limit+1e-8) or np.any(abs(command_acc) > self.acceleration_limit+1e-8):
            raise RuntimeError("Generalized command bounds failed")
        self.previous_command, self.last_t = command.copy(), float(t)
        self.step_count += 1
        instantaneous_error = v+J@command[3:]+rotational_velocity-desired_tip_velocity
        planned_error = command[:3]+J@command[3:]+rotational_velocity-desired_tip_velocity
        projector = J@np.linalg.pinv(J, rcond=1e-8)
        diagnostics = {"active_bounds": int(np.count_nonzero(np.isclose(command, lower, atol=1e-9)|np.isclose(command, upper, atol=1e-9))),
                       "solver_iterations": int(iterations), "task_velocity_residual": float(np.linalg.norm(instantaneous_error)),
                       "instantaneous_task_velocity_residual_world": instantaneous_error,
                       "planned_task_velocity_residual_world": planned_error,
                       "desired_tip_velocity_world": desired_tip_velocity,
                       "actual_tip_velocity_world": v+J@dq+rotational_velocity,
                       "measured_base_velocity_world": v.copy(), "base_executed_velocity_target_world": command[:3].copy(),
                       "nominal_executed_velocity_target": u_nom.copy(), "nominal_q_ref": q_nom.copy(),
                       "nominal_qd_ref": nominal["qd_nom"].copy(), "nominal_qdd_ref": nominal["qdd_nom"].copy(),
                       "rotation_velocity_world": rotational_velocity, "tip_error_world": error,
                       "arm_reachable_task_velocity_world": projector@(r-v),
                       "arm_unreachable_task_velocity_world": (np.eye(3)-projector)@(r-v),
                       "command_acceleration": command_acc, "command_lower": lower, "command_upper": upper,
                       "servo_reconstructed_base_velocity": base_feedback+(command[:3]-base_feedback),
                       "servo_reconstructed_joint_velocity": joint_feedback+(command[3:]-joint_feedback)}
        return {"p_ref": p_nom.copy(), "v_ref": command[:3]-base_feedback, "a_ref": nominal["a_nom"].copy(),
                "q_ref": q_nom.copy(), "qd_ref": command[3:]-joint_feedback, "qdd_ref": command_acc[3:].copy(),
                "qd_cmd": command[3:].copy(), "base_velocity_cmd": command[:3].copy(),
                "tip_ref": target_p.copy(), "tip_v_ref": vd.copy(), "tip_a_ref": ad.copy(),
                "tip_actual": tip, "diagnostics": diagnostics}

    def metadata(self):
        return {"name": "x650_servo_consistent_pid_ff_reference_v1", "mode": self.mode,
                "profile": self.model.metadata(), "pid_gains": deepcopy(self.pid_gains),
                "pid_status": "External X650 profile; requires separate base/arm commissioning, not old x500 optimum",
                "nominal_arm_share": self.arm_share, "base_position_gain_s_inv": self.base_position_gain.tolist(),
                "joint_position_gain_s_inv": self.joint_position_gain, "tip_gain_s_inv": self.tip_gain,
                "velocity_limit": self.velocity_limit.tolist(), "acceleration_limit": self.acceleration_limit.tolist(),
                "q_lower": self.q_lower.tolist(), "q_upper": self.q_upper.tolist(), "max_dt_s": self.max_dt,
                "feedforward_scope": "Common wrapper applies relative-arm inertia acceleration; moment diagnostic only",
                "scope": "Measured endpoint/body velocity constrained LSQ; no prediction horizon"}
