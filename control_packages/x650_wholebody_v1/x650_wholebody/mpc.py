"""Finite-horizon whole-body velocity MPC above the common PX4 PID loops.

One linear time-varying convex QP is formed per update. Its prediction includes
an explicit closed-loop base response, ideal joint-velocity execution and a
short measured-angular-rate attitude extrapolation. This is not floating-base
rigid-body, rotor-thrust/arm-torque NMPC. Default response constants are unfit
engineering seeds; native identification is a prerequisite for performance claims.
"""
from __future__ import annotations

from copy import deepcopy
from time import perf_counter
from types import SimpleNamespace

import numpy as np
from scipy.linalg import cholesky, cho_solve, expm, solve_triangular
from scipy.optimize import LinearConstraint, minimize, nnls

from .baseline import BaselineController, nominal_reference
from .model import (vector, rotation_matrix, tip_body, tip_world, joint_jacobian_body,
                    joint_centripetal_body, inverse_tip_body)
from . import prediction_chain as chain


class MpcSolveError(RuntimeError):
    """No acceptable MPC command; caller must stop the task safely."""

    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation_increment(v):
    angle = float(np.linalg.norm(v))
    K = skew(v)
    if angle < 1e-8:
        return np.eye(3)+K+.5*K@K
    return np.eye(3)+(np.sin(angle)/angle)*K+((1-np.cos(angle))/angle**2)*K@K


def response_matrices(h, base_tau):
    """Exact first-order base ZOH, ideal joint velocity, x=[p,v,q,dq]."""
    A, B = np.eye(10), np.zeros((10, 5))
    for i, tau in enumerate(base_tau):
        alpha = np.exp(-h/tau)
        integrated = tau*(1-alpha)
        A[i, 3+i], A[3+i, 3+i] = integrated, alpha
        B[i, i], B[3+i, i] = h-integrated, 1-alpha
    A[6:8, 8:10] = 0.
    A[8:10, 8:10] = 0.
    B[6:8, 3:5], B[8:10, 3:5] = h*np.eye(2), np.eye(2)
    return A, B


MIXED_RESPONSE_KIND = "xy_second_order_z_first_order_v1"
MIXED_STATE_ORDER = ("px", "py", "pz", "vx", "vy", "vz", "q1", "q2", "dq1", "dq2", "ax", "ay")


def normalize_response_model(value):
    """Validate numerical structure only; flight admission owns evidence checks.

    Positive frequency, damping and time constant give stable velocity poles.
    No fit, inferred gain/delay or conversion from legacy time constants occurs.
    """
    if (not isinstance(value, dict) or set(value) != {"kind", "parameters"}
            or value["kind"] != MIXED_RESPONSE_KIND):
        raise ValueError("response_model must select the explicit mixed response kind and parameters")
    parameters = value["parameters"]
    if not isinstance(parameters, dict) or set(parameters) != {"x", "y", "z"}:
        raise ValueError("Mixed response requires exactly x, y and z parameters")
    result = {}
    for axis, names in (("x", ("wn_rad_s", "zeta")), ("y", ("wn_rad_s", "zeta")), ("z", ("tau_s",))):
        part = parameters[axis]
        if not isinstance(part, dict) or set(part) != set(names):
            raise ValueError("Invalid mixed response parameters for "+axis)
        result[axis] = {}
        for name in names:
            number = part[name]
            if (isinstance(number, (bool, np.bool_))
                    or not isinstance(number, (int, float, np.integer, np.floating))):
                raise ValueError("Mixed response parameter must be finite, real and positive: "+axis+"."+name)
            try:
                number = float(number)
            except (ValueError, OverflowError) as exc:
                raise ValueError("Mixed response parameter is outside finite float range") from exc
            if not np.isfinite(number) or number <= 0:
                raise ValueError("Mixed response parameter must be finite, real and positive: "+axis+"."+name)
            result[axis][name] = number
    return {"kind": MIXED_RESPONSE_KIND, "parameters": result}


def mixed_response_matrices(h, response_model):
    """Exact ZOH for [p3,v3,q2,dq2,ax,ay] and [v_cmd3,dq_cmd2]."""
    response = normalize_response_model(response_model)["parameters"]
    if (isinstance(h, (bool, np.bool_)) or not isinstance(h, (int, float, np.integer, np.floating))
            or not np.isfinite(h) or h <= 0):
        raise ValueError("Mixed response interval must be finite and positive")
    A, B = np.eye(12), np.zeros((12, 5))
    for axis, name in enumerate(("x", "y")):
        wn, zeta = response[name]["wn_rad_s"], response[name]["zeta"]
        # Augment constant input into [p,v,a,u] before exponentiation: p'=v,
        # v'=a, a'=wn^2*(u-v)-2*zeta*wn*a. No acceleration-state reset.
        generator = np.array([[0., 1., 0., 0.], [0., 0., 1., 0.],
                              [0., -wn*wn, -2*zeta*wn, wn*wn], [0., 0., 0., 0.]])
        transition = expm(float(h)*generator)
        indices = [axis, 3+axis, 10+axis]
        A[np.ix_(indices, indices)] = transition[:3, :3]
        B[indices, axis] = transition[:3, 3]
    tau = response["z"]["tau_s"]
    alpha = np.exp(-h/tau)
    integrated = tau*(1-alpha)
    A[2, 5], A[5, 5] = integrated, alpha
    B[2, 2], B[5, 2] = h-integrated, 1-alpha
    A[6:8, 8:10] = 0.
    A[8:10, 8:10] = 0.
    B[6:8, 3:5], B[8:10, 3:5] = h*np.eye(2), np.eye(2)
    return A, B


def integer_setting(value, name):
    if isinstance(value, (bool, str)) or not np.isscalar(value) or not np.isfinite(value):
        raise ValueError(name+" must be an integer")
    result = int(value)
    if result != value:
        raise ValueError(name+" must be an integer; fractional settings are not truncated")
    return result


class MpcController(BaselineController):
    """Shared-model/limits MPC; ``trajectory`` provides causal reference preview.

    ``sample(t) -> (p,v,a)``, ``preview(t) -> EndpointTarget`` and a callable
    returning either are supported. Without a source, the supplied target is
    extrapolated at constant acceleration and that limitation is reported.
    ``t`` is task-relative simulation time; reset is mandatory before a task.
    """

    def __init__(self, model, pid_gains, *, trajectory=None, config=None):
        c = dict(config or {})
        self.prediction_chain = chain.normalize(c.pop("prediction_chain")) if "prediction_chain" in c else None
        self.nodes = integer_setting(c.pop("nodes", 15), "nodes")
        self.node_dt = float(c.pop("node_dt_s", .04))
        self.mixed_response = "response_model" in c
        if self.mixed_response:
            if "base_time_constant_s" in c:
                raise ValueError("response_model cannot be combined with base_time_constant_s")
            self.response_model = normalize_response_model(c.pop("response_model"))
            self.base_tau = None
        else:
            self.response_model = None
            self.base_tau = vector(c.pop("base_time_constant_s", [.2, .2, .25]), 3, "base_time_constant_s").copy()
        self.state_size = 12 if self.mixed_response else 10
        if self.prediction_chain is not None and not self.mixed_response:
            raise ValueError("Prediction chain requires the accepted mixed base-response model")
        self.response_identified = bool(c.pop("response_identified", False))
        self.attitude_tau = float(c.pop("attitude_rate_decay_s", .15))
        self.max_iterations = integer_setting(c.pop("max_iterations", 100), "max_iterations")
        self.ftol = float(c.pop("solver_ftol", 1e-9))
        self.feasibility_tolerance = float(c.pop("feasibility_tolerance", 2e-7))
        self.kkt_tolerance = float(c.pop("kkt_tolerance", 2e-3))
        self.solve_budget_ms = float(c.pop("solve_budget_ms", 20.))
        self.failure_policy = c.pop("failure_policy", "raise")
        self.weights = {
            "tip_position": float(c.pop("tip_position_weight", 400.)),
            "tip_velocity": float(c.pop("tip_velocity_weight", 4.)),
            "base_position": float(c.pop("base_position_weight", .25)),
            "joint_posture": float(c.pop("joint_posture_weight", .02)),
            "input_nominal": float(c.pop("input_nominal_weight", .01)),
            "input_delta": float(c.pop("input_delta_weight", .01)),
        }
        if (self.nodes < 2 or self.nodes > 60 or self.max_iterations < 1
                or any(not np.isfinite(v) or v <= 0 for v in (
                    self.node_dt, self.attitude_tau, self.ftol, self.feasibility_tolerance,
                    self.kkt_tolerance, self.solve_budget_ms))
                or (self.base_tau is not None and np.any(self.base_tau <= 0)) or self.failure_policy not in ("raise", "shift")
                or any(not np.isfinite(v) or v < 0 for v in self.weights.values())
                or self.weights["tip_position"] <= 0 or self.weights["input_delta"] <= 0):
            raise ValueError("Invalid MPC horizon, response, weights or solver settings")
        super().__init__(model, pid_gains, config=c)
        self.trajectory = trajectory
        self._plan, self._plan_t, self._plan_h = None, None, None
        self._previous_stamp = None
        self._horizon_cache = {}
        self._joint_history, self._chain_clock_origin = [], None
        self._history_primed = False

    def reset(self, p0, R0, q0):
        super().reset(p0, R0, q0)
        self._plan, self._plan_t, self._plan_h = None, None, None
        self._previous_stamp = None
        self._joint_history, self._chain_clock_origin = [], None
        self._history_primed = False
        # reset validates and copies these immutable task anchors once.
        self._nominal_tip_body0 = tip_body(self.q0, self.model)
        self._nominal_tip_world0 = self.p0+self.R0@self._nominal_tip_body0

    def prime_command_history(self, history, start):
        """Seed the new chain with commands actually sent before task enable.

        ``history`` contains (absolute pre-solve simulation time, u5) pairs,
        strictly ordered, and ``start`` is this task's absolute time origin.
        This is called once after reset, never to overwrite a running plan.
        The caller records a pair only after both command interfaces return.
        """
        if (self.prediction_chain is None or not self.initialized or self.step_count
                or self._history_primed):
            raise ValueError("Command history may only prime a reset prediction-chain controller once")
        if (isinstance(start, (bool, np.bool_)) or not np.isscalar(start)
                or not np.isfinite(start) or start < 0):
            raise ValueError("Command history requires a finite nonnegative task origin")
        checked = []
        previous_time = -np.inf
        for when, value in history:
            if (isinstance(when, (bool, np.bool_)) or not np.isscalar(when)
                    or not np.isfinite(when) or when < 0 or when <= previous_time
                    or when > start+1e-9):
                raise ValueError("Command history times must be finite, ordered and no later than task enable")
            command = vector(value, 5, "primed command").copy()
            if np.any(abs(command) > self.velocity_limit+1e-9):
                raise ValueError("Primed command exceeds unchanged velocity limits")
            checked.append((float(when)-float(start), command))
            previous_time = float(when)
        if not checked or checked[-1][0] < -.25:
            raise ValueError("Command history must contain a recent actually sent command")
        self._joint_history = [(when, command[3:].copy()) for when, command in checked if when >= -.75]
        self.previous_command = checked[-1][1].copy()
        self.last_t = checked[-1][0]
        self._chain_clock_origin = float(start)
        self._history_primed = True

    def _nominal(self, target, *, derivatives=True):
        """Same nominal geometry; preview omits acceleration terms it never uses."""
        p, v, a = (vector(getattr(target, name), 3, "target."+name)
                   for name in ("position", "velocity", "acceleration"))
        delta = self.R0.T@(p-self._nominal_tip_world0)
        delta[1] = 0.
        q = inverse_tip_body(self._nominal_tip_body0+self.arm_share*delta, near=self.q0, model=self.model)
        if np.any(q < self.model.joint_lower) or np.any(q > self.model.joint_upper):
            raise ValueError("Nominal allocation violates physical joint limits")
        J = joint_jacobian_body(q, self.model)
        dq = np.linalg.solve(J[[0, 2]], (self.arm_share*(self.R0.T@v))[[0, 2]])
        nominal = {"p_nom": p-self.R0@tip_body(q, self.model), "v_nom": v-self.R0@(J@dq),
                   "q_nom": q, "qd_nom": dq}
        if derivatives:
            centripetal = joint_centripetal_body(q, dq, self.model)
            ddq = np.linalg.solve(J[[0, 2]], (self.arm_share*(self.R0.T@a)-centripetal)[[0, 2]])
            nominal.update(a_nom=a-self.R0@(J@ddq+centripetal), qdd_nom=ddq)
        return nominal

    def _preview_nominals(self, targets):
        """Batch the identical two-branch planar IK and velocity solves.

        Only task-anchor geometry and independent target samples are shared;
        the QP, bounds, weights, horizon and every target remain unchanged.
        All target p/v/a are still validated even though preview acceleration
        is not used by this velocity/position objective.
        """
        p, v, a = (np.asarray([getattr(item, name) for item in targets], dtype=float)
                   for name in ("position", "velocity", "acceleration"))
        expected = (len(targets), 3)
        if any(x.shape != expected or not np.isfinite(x).all() for x in (p, v, a)):
            raise ValueError("Preview targets must contain finite three-vector p/v/a")
        delta = (p-self._nominal_tip_world0)@self.R0
        delta[:, 1] = 0.
        r = self._nominal_tip_body0+self.arm_share*delta-np.array(self.model.mount_body)-np.array(self.model.elbow_offset_body_m)
        if np.any(abs(r[:, 1]) > 1e-8):
            raise ValueError("Arm endpoint lies outside its body XZ plane")
        x, y = r[:, 0], -r[:, 2]
        l1, l2 = self.model.lengths
        cosine = (x*x+y*y-l1*l1-l2*l2)/(2*l1*l2)
        if np.any(abs(cosine) > 1+1e-10):
            raise ValueError("Arm target is unreachable")
        beta = np.arccos(np.clip(cosine, -1., 1.))[:, None]*np.array([-1., 1.])
        theta = np.arctan2(y, x)[:, None]-np.arctan2(l2*np.sin(beta), l1+l2*np.cos(beta))
        candidates = np.stack((theta-self.model.angle_offsets[0],
            beta+self.model.angle_offsets[0]-self.model.angle_offsets[1]), axis=2)
        candidates = self.q0+(candidates-self.q0+np.pi) % (2*np.pi)-np.pi
        branch = np.argmin(np.linalg.norm(candidates-self.q0, axis=2), axis=1)
        q = candidates[np.arange(len(targets)), branch]
        if np.any(q < self.model.joint_lower) or np.any(q > self.model.joint_upper):
            raise ValueError("Nominal allocation violates physical joint limits")
        angles = np.array(self.model.angle_offsets)+np.column_stack((q[:, 0], q[:, 0]+q[:, 1]))
        sine, cosine = np.sin(angles), np.cos(angles)
        j1 = l1*np.column_stack((-sine[:, 0], np.zeros(len(q)), -cosine[:, 0]))
        j2 = l2*np.column_stack((-sine[:, 1], np.zeros(len(q)), -cosine[:, 1]))
        J = np.stack((j1+j2, j2), axis=2)
        body_velocity = self.arm_share*(v@self.R0)
        dq = np.linalg.solve(J[:, [0, 2], :], body_velocity[:, [0, 2], None])[:, :, 0]
        # Preserve the original FK addition order: mount + link1 + elbow + link2.
        arm = (np.array(self.model.mount_body)+l1*np.column_stack((cosine[:, 0], np.zeros(len(q)), -sine[:, 0]))
               +np.array(self.model.elbow_offset_body_m)+l2*np.column_stack((cosine[:, 1], np.zeros(len(q)), -sine[:, 1])))
        return {"p_nom": p-arm@self.R0.T, "v_nom": v-np.einsum("nij,nj->ni", J, dq)@self.R0.T,
                "q_nom": q, "qd_nom": dq, "target_p": p, "target_v": v}

    def _horizon(self, dt):
        """Exact-key cache: changing first dt or parameters never reuses old data."""
        N, n = self.nodes, self.nodes*5
        h = np.r_[dt, np.full(N-1, self.node_dt)]
        if self.mixed_response:
            response = normalize_response_model(self.response_model)
            p = response["parameters"]
            response_key = (MIXED_RESPONSE_KIND, p["x"]["wn_rad_s"], p["x"]["zeta"],
                            p["y"]["wn_rad_s"], p["y"]["zeta"], p["z"]["tau_s"])
        else:
            response_key = ("legacy_first_order", self.base_tau.tobytes())
        key = (N, h.tobytes(), response_key, self.velocity_limit.tobytes(),
               self.acceleration_limit.tobytes(), self.max_dt, self.attitude_tau)
        cached = self._horizon_cache.get(key)
        if cached is not None:
            return cached
        scale = np.tile(self.velocity_limit, N)
        transition, jacobian, jacobians, inputs, deltas = [], np.zeros((self.state_size, n)), [], [], []
        diagonal = np.diag(self.velocity_limit)
        mixed_steps = {}
        for i, step in enumerate(h):
            if self.mixed_response:
                if step not in mixed_steps:
                    mixed_steps[step] = mixed_response_matrices(step, response)
                A, B = mixed_steps[step]
            else:
                A, B = response_matrices(step, self.base_tau)
            transition.append(A)
            jacobian = A@jacobian
            jacobian[:, i*5:(i+1)*5] += B
            jacobians.append(jacobian.copy()*scale)
            U = np.zeros((5, n))
            U[:, i*5:(i+1)*5] = diagonal
            D = U.copy()
            if i:
                D[:, (i-1)*5:i*5] -= diagonal
            inputs.append(U)
            deltas.append(D)
        cached = dict(h=h, times=np.cumsum(h), scale=scale, transitions=transition,
                      jacobians=np.asarray(jacobians), inputs=inputs, deltas=deltas,
                      attitude_decay=np.exp(-np.cumsum(h)/self.attitude_tau), identity=np.eye(n))
        if len(self._horizon_cache) >= 8:
            self._horizon_cache.pop(next(iter(self._horizon_cache)))
        self._horizon_cache[key] = cached
        return cached

    def _target(self, t, current, offset):
        source = self.trajectory
        if source is None:
            p, v, a = (vector(getattr(current, n), 3, n) for n in ("position", "velocity", "acceleration"))
            return SimpleNamespace(position=p+offset*v+.5*offset**2*a, velocity=v+offset*a, acceleration=a)
        if hasattr(source, "sample"):
            value = source.sample(t)
        elif hasattr(source, "preview"):
            value = source.preview(t)
        else:
            value = source(t)
        if all(hasattr(value, n) for n in ("position", "velocity", "acceleration")):
            return value
        p, v, a = value
        return SimpleNamespace(position=p, velocity=v, acceleration=a)

    def _warm_start(self, t, h):
        if self._plan is None:
            return np.tile(self.previous_command, (self.nodes, 1))
        old_ends = np.cumsum(self._plan_h)
        sample_offsets = t-self._plan_t+np.r_[0., np.cumsum(h)[:-1]]
        indices = np.minimum(np.searchsorted(old_ends, sample_offsets, side="right"), self.nodes-1)
        return self._plan[indices].copy()

    def _problem(self, t, dt, state, target):
        N, n = self.nodes, self.nodes*5
        cached = self._horizon(dt)
        h, times, scale = cached["h"], cached["times"], cached["scale"]
        x0 = np.r_[state.position, state.velocity, state.q, state.dq]
        if self.mixed_response:
            acceleration = getattr(state, "acceleration_enu", None)
            if acceleration is None:
                raise ValueError("Mixed MPC requires measured acceleration_enu; no zero or differentiated fallback")
            x0 = np.r_[x0, vector(acceleration, 3, "measured acceleration_enu")[:2]]
        offset, offsets = x0.copy(), []
        for A in cached["transitions"]:
            offset = A@offset
            offsets.append(offset.copy())
        offsets, jacobians = np.asarray(offsets), cached["jacobians"]
        joint_prediction, chain_context, chain_outputs, guard = None, None, None, None
        if self.prediction_chain is not None:
            if state.control_time_s is None or state.joint_stamp_s is None:
                raise ValueError("Prediction chain requires measured control/joint source timestamps")
            clock_origin = state.control_time_s-t
            if self._chain_clock_origin is not None and abs(clock_origin-self._chain_clock_origin) > 1e-8:
                raise ValueError("Prediction-chain task/control time origin changed")
            joint_prediction = chain.joint_prediction(t, times, h, state, self._joint_history,
                                                       self.prediction_chain, self.velocity_limit[3:])
            jacobians = jacobians.copy()
            for i, (_, q, dq, Q, D) in enumerate(joint_prediction["nodes"]):
                offsets[i, 6:8], offsets[i, 8:10] = q, dq
                jacobians[i, 6:8], jacobians[i, 8:10] = Q, D
            x0[6:8], x0[8:10] = joint_prediction["q_current"], joint_prediction["dq_current"]
        warm = self._warm_start(t, h)
        zwarm = warm.reshape(-1)/scale
        linearization = offsets+np.einsum("nij,j->ni", jacobians, zwarm)
        if self.prediction_chain is not None:
            chain_context = chain.attitude_context(state, times, self.prediction_chain)
            chain_outputs = chain.endpoint_outputs(linearization, warm, chain_context, self.model,
                                                   self.response_model["parameters"], jacobians=True)
        rows, rhs, constraints, lower, upper = [], [], [], [], []

        def cost(A, b, weight, step=1.):
            if weight:
                w = np.sqrt(weight*step)
                rows.append(w*A)
                rhs.append(w*np.asarray(b))

        def constrain(C, lo, hi):
            constraints.append(C)
            lower.append(np.atleast_1d(lo))
            upper.append(np.atleast_1d(hi))

        rotations = []
        future_targets = [self._target(t+elapsed, target, elapsed) for elapsed in times]
        preview = self._preview_nominals(future_targets)
        R0, omega = rotation_matrix(state.rotation), np.asarray(state.omega_body)
        for i, (elapsed, step) in enumerate(zip(times, h)):
            predicted = future_targets[i]
            nominal = {key: preview[key][i] for key in ("p_nom", "v_nom", "q_nom", "qd_nom")}
            S, o = jacobians[i], offsets[i]
            if chain_outputs is not None:
                R = chain_outputs["rotations"][i]
                U = cached["inputs"][i]
                px, pu, vx, vu = (chain_outputs[key][i] for key in ("px", "pu", "vx", "vu"))
                P, V = px@S+pu@U, vx@S+vu@U
                po = chain_outputs["position"][i]+px@(o-linearization[i])-pu@warm[i]
                vo = chain_outputs["velocity"][i]+vx@(o-linearization[i])-vu@warm[i]
            else:
                qlin = linearization[i, 6:8]
                R = R0@rotation_increment(omega*self.attitude_tau*(1-cached["attitude_decay"][i]))
                wworld = R@(omega*cached["attitude_decay"][i])
                arm, J = tip_body(qlin, self.model), R@joint_jacobian_body(qlin, self.model)
                P = S[:3]+J@S[6:8]
                po = o[:3]+R@arm+J@(o[6:8]-qlin)
                V = S[3:6]+J@S[8:10]+skew(wworld)@J@S[6:8]
                vo = o[3:6]+J@o[8:10]+skew(wworld)@(R@arm+J@(o[6:8]-qlin))
            rotations.append(R)
            cost(P, preview["target_p"][i]-po, self.weights["tip_position"], step)
            cost(V, preview["target_v"][i]-vo, self.weights["tip_velocity"], step)
            cost(S[:3], nominal["p_nom"]-o[:3], self.weights["base_position"], step)
            cost(S[6:8], nominal["q_nom"]-o[6:8], self.weights["joint_posture"], step)
            U = cached["inputs"][i]
            nominal_u = np.r_[nominal["v_nom"], nominal["qd_nom"]]
            cost(U/self.velocity_limit[:, None], nominal_u/self.velocity_limit,
                 self.weights["input_nominal"], step)
            D = cached["deltas"][i]
            previous = self.previous_command if i == 0 else np.zeros(5)
            constrain(D, previous-self.acceleration_limit*step, previous+self.acceleration_limit*step)
            cost(D/(self.acceleration_limit*step)[:, None], previous/(self.acceleration_limit*step),
                 self.weights["input_delta"], step)
            if joint_prediction is None:
                constrain(S[6:8], self.q_lower-o[6:8], self.q_upper-o[6:8])
        # This linear terminal reserve overbounds u^2/(2a)+max_dt*|u|
        # over the known speed interval. It is conservative, not a proof of
        # robust recursive feasibility against servo error or external forces.
        reserve = self.velocity_limit[3:]/(2*self.acceleration_limit[3:])+self.max_dt
        if joint_prediction is None:
            S, o = jacobians[-1], offsets[-1]
            constrain(S[6:8]+reserve[:, None]*S[8:10],
                      self.q_lower-o[6:8]-reserve*o[8:10],
                      self.q_upper-o[6:8]-reserve*o[8:10])
            first_lo, first_hi = self._bounds(dt, np.asarray(state.q))
        else:
            first_lo, first_hi, guard = chain.conservative_first_bounds(
                self, dt, state, joint_prediction, self._joint_history, t, self.prediction_chain)
            # Candidate protection for pending velocity commands. Bounds at
            # every switch and node cover piecewise-linear q, and reserve uses
            # the unchanged command braking schedule plus pipeline/age margin.
            reserve = reserve+self.prediction_chain["joint_delay_guard_s"]+joint_prediction["measurement_age_s"]
            for _, q, dq, Q, D in joint_prediction["events"]:
                constrain(Q, self.q_lower-q, self.q_upper-q)
                # Future dq is exactly one speed-box-bounded input; adding
                # another identical bound only duplicates solver work.
                # Before its switch the measured/pending dq is a constant.
                if not np.any(D) and np.any(abs(dq) > self.velocity_limit[3:]):
                    raise ValueError("Measured/pending joint velocity exceeds unchanged speed limits")
                constrain(Q+reserve[:, None]*D,
                          self.q_lower-q-reserve*dq, self.q_upper-q-reserve*dq)
        first = cached["inputs"][0]
        constrain(first, first_lo, first_hi)
        C, lo, hi = np.vstack(constraints), np.concatenate(lower), np.concatenate(upper)
        if joint_prediction is not None:
            variable = np.any(C != 0., axis=1)
            # These rows concern measured/pending motion before any new input
            # can act. Check constants before removing them from the solver.
            if np.any(lo[~variable] > 0.) or np.any(hi[~variable] < 0.):
                raise ValueError("Measured/pending joint motion violates a constant prediction constraint")
            C, lo, hi = C[variable], lo[variable], hi[variable]
        A, b = np.vstack(rows), np.concatenate(rhs)
        return {"A": A, "b": b, "H": 2*A.T@A, "g": -2*A.T@b,
                "C": C, "lo": lo, "hi": hi, "scale": scale, "warm": zwarm,
                "h": h, "initial_state": x0.copy(), "offsets": offsets, "jacobians": jacobians,
                "linearization": linearization, "rotations": rotations,
                "targets": future_targets, "first_lower": first_lo, "first_upper": first_hi, "identity": cached["identity"],
                **({"chain_context": chain_context, "joint_prediction": joint_prediction, "joint_guard": guard}
                   if self.prediction_chain is not None else {})}

    @staticmethod
    def _strictly_feasible_center(problem, center):
        """A positive interior margin admits the exact unconstrained optimum."""
        if not np.isfinite(center).all() or np.any(abs(center) > 1.-1e-10):
            return False
        value = problem["C"]@center
        return bool(np.all(value >= problem["lo"]+1e-10)
                    and np.all(value <= problem["hi"]-1e-10))

    @staticmethod
    def _quality(problem, z):
        if not np.isfinite(z).all():
            return {"primal_violation": float("inf"), "kkt_stationarity_inf": float("inf"),
                    "kkt_complementarity_inf": float("inf"), "active_constraints": 0}
        C, lo, hi = problem["C"], problem["lo"], problem["hi"]
        value = C@z
        violation = max(0., float(np.max(lo-value)), float(np.max(value-hi)), float(np.max(abs(z)-1)))
        normals = np.vstack((C, -C, np.eye(len(z)), -np.eye(len(z))))
        limits = np.r_[hi, -lo, np.ones(len(z)*2)]
        slack = limits-normals@z
        active = slack < 1e-6
        gradient = problem["H"]@z+problem["g"]
        if np.any(active):
            try:
                multipliers, _ = nnls(normals[active].T, -gradient, maxiter=10*int(active.sum()))
                residual = gradient+normals[active].T@multipliers
                complementarity = float(np.max(abs(multipliers*slack[active])))
            except RuntimeError:
                residual, complementarity = np.full_like(gradient, np.inf), float("inf")
        else:
            residual, complementarity = gradient, 0.
        return {"primal_violation": violation, "kkt_stationarity_inf": float(np.max(abs(residual))),
                "kkt_complementarity_inf": complementarity, "active_constraints": int(active.sum())}

    def step(self, t, dt, state, target):
        begin = perf_counter()
        if not self.initialized:
            raise RuntimeError("reset(p0,R0,q0) is required before step")
        if (not np.isfinite(t) or t < 0 or not np.isfinite(dt) or not 0 < dt <= self.max_dt+1e-10
                or (self.last_t is not None and t <= self.last_t)):
            raise ValueError("Require increasing finite t and 0 < dt <= max_dt")
        for name, size in (("position",3),("velocity",3),("omega_body",3),("q",2),("dq",2)):
            vector(getattr(state, name), size, name)
        rotation_matrix(state.rotation)
        stamp = float(state.stamp_s)
        if (not np.isfinite(stamp) or stamp < 0
                or (self._previous_stamp is not None and stamp < self._previous_stamp-1e-9)):
            raise ValueError("State timestamp regressed or is invalid; reset after clock changes")
        nominal = self._nominal(target)
        if (np.any(nominal["q_nom"] < self.q_lower) or np.any(nominal["q_nom"] > self.q_upper)
                or np.any(abs(np.r_[nominal["v_nom"],nominal["qd_nom"]]) > self.velocity_limit+1e-9)
                or np.any(abs(np.r_[nominal["a_nom"],nominal["qdd_nom"]]) > self.acceleration_limit+1e-9)):
            raise ValueError("Nominal trajectory exceeds common soft/rate/acceleration limits")
        problem = self._problem(t, dt, state, target)
        H, g = problem["H"], problem["g"]
        solve_begin = perf_counter()
        # Exact change of variables, not an approximate diagonal scaling:
        # H=L L^T, z=center+L^-T y makes the objective .5*y^T*y.
        # SLSQP's initial identity Hessian is then exact for this convex QP,
        # avoiding dozens of BFGS iterations on the poorly conditioned horizon.
        L = cholesky(H, lower=True, check_finite=False)
        center = -cho_solve((L,True),g,check_finite=False)
        exact_center = self.prediction_chain is not None and self._strictly_feasible_center(problem, center)
        if exact_center:
            # H is positive definite and H*center+g=0: an interior feasible
            # center is also the constrained global optimum. Quality/KKT and
            # the unchanged time budget still apply below.
            result = SimpleNamespace(success=True, status=0, nit=0,
                                     message="Exact unconstrained Cholesky optimum is strictly feasible")
        else:
            transform = solve_triangular(L.T,problem["identity"],lower=False,check_finite=False)
            C = np.vstack((problem["C"],problem["identity"]))
            lo, hi = np.r_[problem["lo"],-np.ones(len(g))], np.r_[problem["hi"],np.ones(len(g))]
            shifted, matrix = C@center, C@transform
            row_scale = np.maximum(np.linalg.norm(matrix,axis=1),1e-12)
            linear = LinearConstraint(matrix/row_scale[:,None],(lo-shifted)/row_scale,(hi-shifted)/row_scale)
            initial = L.T@(np.clip(problem["warm"],-1.,1.)-center)
            result = minimize(lambda y: .5*y@y, initial, jac=lambda y: y,
                              method="SLSQP", constraints=[linear],
                              options={"maxiter": self.max_iterations, "ftol": self.ftol, "disp": False})
        solve_ms = (perf_counter()-solve_begin)*1000
        z = center.copy() if exact_center else center+transform@np.asarray(result.x,float)
        quality = self._quality(problem, z)
        accepted = (bool(result.success) and np.isfinite(z).all()
                    and quality["primal_violation"] <= self.feasibility_tolerance
                    and quality["kkt_stationarity_inf"] <= self.kkt_tolerance)
        fallback = False
        if not accepted and self.failure_policy == "shift" and self._plan is not None:
            candidate = problem["warm"]
            candidate_quality = self._quality(problem, candidate)
            if candidate_quality["primal_violation"] <= self.feasibility_tolerance:
                z, quality, fallback = candidate.copy(), candidate_quality, True
        diagnostics = {"solver": ("scipy_cholesky_exact_feasible_convex_qp" if exact_center else
                                  "scipy_slsqp_exact_hessian_whitened_convex_qp"), "solver_success": bool(result.success),
                       "solver_status": int(result.status), "solver_message": str(result.message),
                       "solver_iterations": int(result.nit), "solve_ms": solve_ms,
                       "solve_budget_ms": self.solve_budget_ms, "solve_budget_exceeded": solve_ms > self.solve_budget_ms,
                       "accepted_optimum": accepted, "previous_plan_fallback": fallback, **quality}
        if not accepted and not fallback:
            diagnostics["compute_ms"] = (perf_counter()-begin)*1000
            raise MpcSolveError("MPC solve failed quality/feasibility checks", diagnostics)
        plan = (z*problem["scale"]).reshape(self.nodes,5)
        command = plan[0].copy()
        acceleration = (command-self.previous_command)/dt
        predicted = problem["offsets"]+np.einsum("nij,j->ni", problem["jacobians"], z)
        if self.prediction_chain is not None:
            chain_result = chain.endpoint_outputs(predicted, plan, problem["chain_context"], self.model,
                                                  self.response_model["parameters"])
            exact_tip = chain_result["position"]
        else:
            exact_tip = np.array([x[:3]+R@tip_body(x[6:8], self.model)
                                  for x,R in zip(predicted, problem["rotations"])])
        predicted_errors = exact_tip-np.array([v.position for v in problem["targets"]])
        tip = tip_world(state.position, state.rotation, state.q, self.model)
        diagnostics.update(predicted_horizon_s=float(np.sum(problem["h"])), node_intervals_s=problem["h"].copy(),
                           prediction_time_constant_identified=self.response_identified,
                           predicted_states=predicted.copy(), predicted_commands=plan.copy(),
                           predicted_tip_positions=exact_tip, predicted_tip_error_norm=np.linalg.norm(predicted_errors,axis=1),
                           objective=float(np.linalg.norm(problem["A"]@z-problem["b"])**2),
                           command_acceleration=acceleration.copy(), command_lower=problem["first_lower"].copy(),
                           command_upper=problem["first_upper"].copy(), tip_error_world=np.asarray(target.position)-tip,
                           base_executed_velocity_target_world=command[:3].copy(),
                           command_delay_compensated=False, state_delay_compensated=False,
                           reference_preview="trajectory" if self.trajectory is not None else "current_target_constant_acceleration")
        if self.mixed_response:
            diagnostics.pop("prediction_time_constant_identified")
            diagnostics.update(prediction_response_identified=self.response_identified,
                               prediction_response_kind=MIXED_RESPONSE_KIND,
                               prediction_state_order=list(MIXED_STATE_ORDER),
                               measured_acceleration_enu=vector(state.acceleration_enu, 3, "acceleration_enu"),
                               initial_prediction_acceleration_xy=problem["initial_state"][10:12].copy())
        if self.prediction_chain is not None:
            diagnostics.update(prediction_chain=deepcopy(self.prediction_chain),
                solve_method="exact_feasible_center" if exact_center else "slsqp",
                exact_center_interior_margin=1e-10,
                prediction_control_time_s=state.control_time_s, prediction_joint_stamp_s=state.joint_stamp_s,
                prediction_joint_measurement_age_s=problem["joint_prediction"]["measurement_age_s"],
                prediction_measured_joint_position=np.asarray(state.q).copy(),
                prediction_measured_joint_velocity=np.asarray(state.dq).copy(),
                prediction_initial_joint_position=problem["initial_state"][6:8].copy(),
                prediction_initial_joint_velocity=problem["initial_state"][8:10].copy(),
                prediction_joint_guard=problem["joint_guard"],
                predicted_rotations_flu_to_enu=chain_result["rotations"],
                prediction_attitude_initial_residual_rotvec=problem["chain_context"]["initial_residual_rotvec"],
                joint_command_delay_compensated=True, joint_measurement_age_propagated=True,
                command_delay_compensated=True, state_delay_compensated=True,
                compensation_scope="joint_execution_and_joint_measurement_age_only",
                base_command_delay_compensated=False)
            self._joint_history.append((float(t), command[3:].copy()))
            self._joint_history = [(when, value) for when, value in self._joint_history if when >= t-.75]
            self._chain_clock_origin = state.control_time_s-t
        self._plan, self._plan_t, self._plan_h = plan.copy(), float(t), problem["h"].copy()
        self.previous_command, self.last_t, self._previous_stamp = command.copy(), float(t), stamp
        self.step_count += 1
        diagnostics["compute_ms"] = (perf_counter()-begin)*1000
        base_feedback = self.base_position_gain*(nominal["p_nom"]-state.position)
        joint_feedback = self.joint_position_gain*(nominal["q_nom"]-state.q)
        return {"p_ref": nominal["p_nom"].copy(), "v_ref": command[:3]-base_feedback,
                "a_ref": nominal["a_nom"].copy(), "q_ref": nominal["q_nom"].copy(),
                "qd_ref": command[3:]-joint_feedback, "qdd_ref": acceleration[3:].copy(),
                "qd_cmd": command[3:].copy(), "base_velocity_cmd": command[:3].copy(),
                "tip_ref": np.asarray(target.position).copy(), "tip_v_ref": np.asarray(target.velocity).copy(),
                "tip_a_ref": np.asarray(target.acceleration).copy(), "tip_actual": tip, "diagnostics": diagnostics}

    def metadata(self):
        common = super().metadata()
        common.update(name="x650_hierarchical_wholebody_kinematic_mpc_v1", mode="mpc",
                      nodes=self.nodes, subsequent_node_dt_s=self.node_dt,
                      first_node_dt="Actual control interval; no doubled first-command acceleration allowance",
                      joint_time_constant_s=[0.,0.],
                      response_identified=self.response_identified,
                      prediction_model="Exact ZOH closed-loop base first order + ideal joint velocity; not full rigid-body dynamics",
                      attitude_model="Measured body omega exponentially decays with attitude_rate_decay_s; not an optimized free pose",
                      attitude_rate_decay_s=self.attitude_tau, weights=deepcopy(self.weights),
                      solver="SciPy SLSQP, exact Cholesky Hessian whitening, analytic gradient/linear constraints; one linearization per update",
                      max_iterations=self.max_iterations, solver_ftol=self.ftol,
                      feasibility_tolerance=self.feasibility_tolerance, kkt_tolerance=self.kkt_tolerance,
                      solve_budget_ms=self.solve_budget_ms, failure_policy=self.failure_policy,
                      constraint_scope="Command speeds/changes and predicted q; not a guarantee of actual accelerations or robust collision avoidance",
                      realtime_verified=False, command_delay_compensated=False, state_delay_compensated=False,
                      time_alignment="Caller must supply coherent measured state; no invented delay identification",
                      terminal_reserve="q_N + (vmax_joint/(2*amax_joint)+max_dt)*dq_N inside soft limits; conservative linear reserve",
                      scope="Finite-horizon endpoint-position MPC above PX4 PID; translational arm FF supplied by shared wrapper")
        if self.mixed_response:
            common.update(response_model=deepcopy(self.response_model), prediction_state_order=list(MIXED_STATE_ORDER),
                          name="x650_hierarchical_wholebody_mixed_response_mpc_v1",
                          prediction_model="Exact ZOH XY second-order velocity response + Z first order + ideal joint velocity; not full rigid-body dynamics",
                          acceleration_initialization="Measured world ENU acceleration at every solve; no differencing or zero fallback")
        else:
            common["base_time_constant_s"] = self.base_tau.tolist()
        if self.prediction_chain is not None:
            common.update(prediction_chain=deepcopy(self.prediction_chain),
                solver="Exact Cholesky unconstrained optimum when strictly feasible; otherwise original SLSQP exact-Hessian-whitened QP; same quality gates",
                exact_center_interior_margin=1e-10,
                prediction_model="Unchanged accepted base response + timestamped delayed joint velocity + thrust-direction attitude output",
                joint_time_constant_s=[0., 0.], joint_delay_identified=False,
                joint_delay_origin="Pre-solve control consumption; development candidate, not actual dispatch/apply timestamp",
                attitude_model="Predicted acceleration plus gravity defines thrust axis; measured rotation residual decays",
                joint_command_delay_compensated=True, joint_measurement_age_propagated=True,
                command_delay_compensated=True, state_delay_compensated=True,
                compensation_scope="joint_execution_and_joint_measurement_age_only",
                base_command_delay_compensated=False,
                time_alignment="Measured joint source time propagated with recorded command history; independent native truth never used",
                constraint_scope="Unchanged speed/slew/soft limits plus conservative pending-motion envelope and all switch/node reserves; no robust feasibility proof")
        return common
