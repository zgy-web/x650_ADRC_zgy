"""Position outer loops for the same PX4 velocity-command interface.

All vectors are ENU, metres and seconds.  Controllers only consume estimated
position/velocity.  The common runner owns final speed/slew limits and must call
``accept_applied`` after sending a command, including a limited command.

The identified plant is p_dot=v, v_dot=(gain*u-v)/tau + disturbance.
``delay_s`` is retained as identification evidence; this first comparison does
not add a transport-delay state to either LADRC or MPC.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np


DEFAULT_CONFIG = {
    "limits": {"speed": 0.4, "slew": 1.0},
    "pid": {"kp": 1.2, "ki": 0.12, "kd": 0.15,
            "derivative_tau": 0.08, "antiwindup_gain": 2.0,
            "integral_limit": 0.3},
    "ladrc": {"wc": 2.0, "wo": 8.0},
    "mpc": {"dt": 0.02, "horizon": 50, "q_position": 25.0,
            "q_velocity": 1.0, "r_input": 0.2, "r_delta": 1.0,
            "terminal_factor": 2.0, "time_limit_s": 0.015,
            "max_iter": 4000},
}


class ControllerError(RuntimeError):
    """An invalid estimate, command, or optimization ends this trial."""


def vector(value: Any, name: str, *, scalar: bool = False) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if scalar and result.ndim == 0:
        result = np.full(3, float(result))
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite ENU vector of length 3")
    return result.copy()


def positive(value: Any, name: str, *, allow_zero: bool = False) -> np.ndarray:
    result = vector(value, name, scalar=True)
    if np.any(result < 0 if allow_zero else result <= 0):
        raise ValueError(f"{name} must be {'nonnegative' if allow_zero else 'positive'}")
    return result


def check_dt(dt: float) -> float:
    dt = float(dt)
    if not math.isfinite(dt) or not 0 < dt <= 0.15:
        raise ValueError(f"Control dt must be in (0, 0.15] s, got {dt}")
    return dt


class PositionController:
    def __init__(self, config: dict, model: dict):
        limits = DEFAULT_CONFIG["limits"] | config.get("limits", {})
        self.speed = positive(limits["speed"], "limits.speed")
        self.slew = positive(limits["slew"], "limits.slew")
        self.gain = positive(model["gain"], "model.gain")
        self.tau = positive(model["tau"], "model.tau")
        self.delay_s = positive(model.get("delay_s", 0.0), "model.delay_s", allow_zero=True)
        self.diagnostics: dict[str, Any] = {}
        self.initialized = False
        self.applied = np.zeros(3)
        self.requested = np.zeros(3)

    def reset(self, position, velocity, applied_command):
        position = vector(position, "position")
        velocity = vector(velocity, "velocity")
        self.applied = vector(applied_command, "applied_command")
        if np.any(np.abs(self.applied) > self.speed + 1e-9):
            raise ValueError("Initial applied command exceeds the common speed limit")
        self.requested = self.applied.copy()
        self.diagnostics = {}
        self.initialized = True
        return position, velocity

    def _inputs(self, dt, position, velocity, target):
        if not self.initialized:
            raise ControllerError("Call reset(position, velocity, applied_command) before step")
        return (check_dt(dt), vector(position, "position"),
                vector(velocity, "velocity"), vector(target, "target"))

    def accept_applied(self, command, dt):
        check_dt(dt)
        command = vector(command, "applied_command")
        if np.any(np.abs(command) > self.speed + 1e-8):
            raise ControllerError("Sent command exceeds the common speed limit")
        self.applied = command


class PID(PositionController):
    """Derivative on measured velocity; integral stores its output contribution."""

    def __init__(self, config, model):
        super().__init__(config, model)
        cfg = DEFAULT_CONFIG["pid"] | config.get("pid", {})
        self.kp = positive(cfg["kp"], "pid.kp", allow_zero=True)
        self.ki = positive(cfg["ki"], "pid.ki", allow_zero=True)
        self.kd = positive(cfg["kd"], "pid.kd", allow_zero=True)
        self.filter_tau = positive(cfg["derivative_tau"], "pid.derivative_tau")
        self.kaw = positive(cfg["antiwindup_gain"], "pid.antiwindup_gain", allow_zero=True)
        self.integral_limit = positive(cfg["integral_limit"], "pid.integral_limit")

    def reset(self, position, velocity, applied_command):
        _, velocity = super().reset(position, velocity, applied_command)
        self.integral = np.zeros(3)
        self.filtered_velocity = velocity.copy()

    def step(self, dt, position, velocity, target):
        dt, position, velocity, target = self._inputs(dt, position, velocity, target)
        error = target - position
        alpha = np.exp(-dt / self.filter_tau)
        self.filtered_velocity = alpha*self.filtered_velocity + (1-alpha)*velocity
        self.integral = np.clip(self.integral + self.ki*error*dt,
                                -self.integral_limit, self.integral_limit)
        self.requested = self.kp*error + self.integral - self.kd*self.filtered_velocity
        self.diagnostics = {"integral": self.integral.tolist(),
                            "filtered_velocity": self.filtered_velocity.tolist()}
        return self.requested.copy()

    def accept_applied(self, command, dt):
        super().accept_applied(command, dt)
        # Back calculation uses what was actually sent after both speed and slew
        # clipping, not just the controller's own requested output.
        active = self.ki > 0
        self.integral = np.where(active, np.clip(
            self.integral + self.kaw*(self.applied-self.requested)*dt,
            -self.integral_limit, self.integral_limit), 0.0)
        self.diagnostics["integral"] = self.integral.tolist()


class LADRC(PositionController):
    """Second-order LADRC with a third-order linear extended-state observer.

    ``z[:,2]`` estimates total equivalent position acceleration (m/s^2), which
    includes the inner-loop damping and other unmodelled dynamics.  It is not a
    measurement of physical external force.  The observer is exactly discretized
    for held measured position and the last actually transmitted command.
    """

    def __init__(self, config, model):
        super().__init__(config, model)
        cfg = DEFAULT_CONFIG["ladrc"] | config.get("ladrc", {})
        self.wc = positive(cfg["wc"], "ladrc.wc")
        self.wo = positive(cfg["wo"], "ladrc.wo")
        self.b0 = self.gain / self.tau
        self._observer_cache: dict[float, tuple[np.ndarray, np.ndarray]] = {}

    def reset(self, position, velocity, applied_command):
        position, velocity = super().reset(position, velocity, applied_command)
        self.z = np.column_stack((position, velocity, np.zeros(3)))

    def _observer_matrices(self, dt):
        # A_o has the repeated pole -wo. (A_o + wo I)^3 == 0, so this
        # polynomial gives exp(A_o dt) without an extra SciPy dependency.
        key = round(dt, 12)
        if key not in self._observer_cache:
            transitions, inputs = [], []
            for w, b0 in zip(self.wo, self.b0):
                observer = np.array([[-3*w, 1., 0.], [-3*w*w, 0., 1.],
                                     [-w**3, 0., 0.]])
                nilpotent = observer + w*np.eye(3)
                transition = math.exp(-w*dt)*(np.eye(3)+dt*nilpotent
                                               +0.5*dt*dt*(nilpotent@nilpotent))
                forcing = np.array([[3*w, 0.], [3*w*w, b0], [w**3, 0.]])
                discrete_input = np.linalg.solve(observer, (transition-np.eye(3))@forcing)
                transitions.append(transition)
                inputs.append(discrete_input)
            if len(self._observer_cache) > 32:
                self._observer_cache.clear()
            self._observer_cache[key] = (np.array(transitions), np.array(inputs))
        return self._observer_cache[key]

    def step(self, dt, position, velocity, target):
        dt, position, _, target = self._inputs(dt, position, velocity, target)
        transition, forcing = self._observer_matrices(dt)
        self.z = (np.einsum("aij,aj->ai", transition, self.z)
                  + np.einsum("aij,aj->ai", forcing,
                              np.column_stack((position, self.applied))))
        self.requested = (self.wc**2*(target-self.z[:, 0])
                          -2*self.wc*self.z[:, 1]-self.z[:, 2]) / self.b0
        if not np.all(np.isfinite(self.z)) or not np.all(np.isfinite(self.requested)):
            raise ControllerError("LADRC observer produced a nonfinite state")
        self.diagnostics = {"position_estimate": self.z[:, 0].tolist(),
                            "velocity_estimate": self.z[:, 1].tolist(),
                            "disturbance_acceleration": self.z[:, 2].tolist()}
        return self.requested.copy()


class LinearMPC(PositionController):
    """Sparse constrained QP for six states and three speed-command inputs."""

    def __init__(self, config, model):
        super().__init__(config, model)
        try:
            import osqp
            from scipy import sparse
        except ImportError as exc:
            raise ControllerError("MPC requires the comparison environment with OSQP and SciPy") from exc
        self.sparse = sparse
        cfg = DEFAULT_CONFIG["mpc"] | config.get("mpc", {})
        self.prediction_dt = check_dt(cfg["dt"])
        self.horizon = int(cfg["horizon"])
        if self.horizon != cfg["horizon"] or not 2 <= self.horizon <= 500:
            raise ValueError("mpc.horizon must be an integer in [2, 500]")
        qp = positive(cfg["q_position"], "mpc.q_position")
        qv = positive(cfg["q_velocity"], "mpc.q_velocity", allow_zero=True)
        ru = positive(cfg["r_input"], "mpc.r_input")
        self.rd = positive(cfg["r_delta"], "mpc.r_delta", allow_zero=True)
        terminal = float(cfg["terminal_factor"])
        time_limit = float(cfg["time_limit_s"])
        max_iter = int(cfg["max_iter"])
        if not math.isfinite(terminal) or terminal <= 0:
            raise ValueError("mpc.terminal_factor must be finite and positive")
        if not math.isfinite(time_limit) or time_limit <= 0 or max_iter < 1:
            raise ValueError("MPC solver time limit and max_iter must be positive")

        h, n = self.prediction_dt, self.horizon
        decay = np.exp(-h/self.tau)
        integral = self.tau*(1-decay)
        self.A = np.block([[np.eye(3), np.diag(integral)],
                           [np.zeros((3, 3)), np.diag(decay)]])
        self.B = np.vstack((np.diag(self.gain*(h-integral)),
                            np.diag(self.gain*(1-decay))))
        self.nstate = 6*(n+1)
        self.nvar = self.nstate+3*n
        state_cost = sparse.diags(np.r_[qp, qv], format="csc")
        Q = sparse.block_diag([state_cost]*n+[terminal*state_cost], format="csc")
        differences = sparse.eye(n, format="csc") - sparse.diags(np.ones(n-1), -1, shape=(n,n), format="csc")
        self.D = sparse.kron(differences, sparse.eye(3), format="csc")
        Rd = sparse.kron(sparse.eye(n), sparse.diags(self.rd), format="csc")
        R = sparse.kron(sparse.eye(n), sparse.diags(ru), format="csc") + self.D.T@Rd@self.D
        P = sparse.block_diag([2*Q, 2*R], format="csc")
        Ax = (sparse.eye(self.nstate, format="csc")
              + sparse.kron(sparse.diags(-np.ones(n), -1, shape=(n+1,n+1)), self.A, format="csc"))
        selector = sparse.vstack([sparse.csc_matrix((1, n)), sparse.eye(n)], format="csc")
        Bu = sparse.kron(selector, -self.B, format="csc")
        equality = sparse.hstack([Ax, Bu], format="csc")
        command_bounds = sparse.hstack([sparse.csc_matrix((3*n,self.nstate)), sparse.eye(3*n)], format="csc")
        slew_bounds = sparse.hstack([sparse.csc_matrix((3*n,self.nstate)), self.D], format="csc")
        constraints = sparse.vstack([equality, command_bounds, slew_bounds], format="csc")
        self.linear_cost = np.zeros(self.nvar)
        self.lower = np.r_[np.zeros(self.nstate), -np.tile(self.speed,n), -np.tile(self.slew*h,n)]
        self.upper = np.r_[np.zeros(self.nstate), np.tile(self.speed,n), np.tile(self.slew*h,n)]
        self.first_slew = self.nstate+3*n
        self.solver = osqp.OSQP()
        self.solver.setup(P=P, q=self.linear_cost, A=constraints,
                          l=self.lower, u=self.upper, verbose=False,
                          eps_abs=1e-5, eps_rel=1e-5, max_iter=max_iter,
                          time_limit=time_limit, polishing=False,
                          warm_starting=True, adaptive_rho=True)

    def reset(self, position, velocity, applied_command):
        super().reset(position, velocity, applied_command)
        self.solver.warm_start(x=np.zeros(self.nvar), y=np.zeros(len(self.lower)))

    def step(self, dt, position, velocity, target):
        dt, position, velocity, target = self._inputs(dt, position, velocity, target)
        initial = np.r_[position-target, velocity]
        self.lower[:6] = initial
        self.upper[:6] = initial
        first_bounds = slice(self.first_slew,self.first_slew+3)
        self.lower[first_bounds] = self.applied-self.slew*dt
        self.upper[first_bounds] = self.applied+self.slew*dt
        self.linear_cost[self.nstate:self.nstate+3] = -2*self.rd*self.applied
        self.solver.update(q=self.linear_cost, l=self.lower, u=self.upper)
        solution = self.solver.solve(raise_error=False)
        info = solution.info
        self.diagnostics = {"solver_status": str(info.status),
                            "solver_status_val": int(info.status_val),
                            "solver_iterations": int(info.iter),
                            "solver_run_time_s": float(info.run_time),
                            "solver_primal_residual": float(info.prim_res),
                            "solver_dual_residual": float(info.dual_res),
                            "prediction_dt_s": self.prediction_dt}
        if info.status_val != 1 or solution.x is None:
            raise ControllerError(f"MPC solve failed: {info.status} (iterations={info.iter})")
        requested = np.asarray(solution.x[self.nstate:self.nstate+3], dtype=float)
        low = np.maximum(-self.speed, self.applied-self.slew*dt)
        high = np.minimum(self.speed, self.applied+self.slew*dt)
        if (not np.all(np.isfinite(requested)) or np.any(requested < low-1e-4)
                or np.any(requested > high+1e-4)):
            raise ControllerError("MPC first command violates speed/slew constraints")
        # Remove only solver-tolerance-sized constraint violations. The runner
        # independently enforces the exact same limits for all three algorithms.
        self.requested = np.clip(requested, low, high)
        return self.requested.copy()


def make_controller(name: str, config: dict, model: dict) -> PositionController:
    types = {"pid": PID, "ladrc": LADRC, "mpc": LinearMPC}
    if name not in types:
        raise ValueError(f"Unknown controller {name!r}; choose pid, ladrc or mpc")
    return types[name](config, model)
