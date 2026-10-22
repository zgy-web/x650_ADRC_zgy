"""Conditional TS20 joint prediction in the existing upper velocity MPC.

The base response is an explicitly reused prior. This does not identify a
24 V loaded actuator or change the PX4 attitude controller into whole-body NMPC.
"""
from __future__ import annotations

from copy import deepcopy
from time import perf_counter
import numpy as np

from .mpc import MpcController, MpcSolveError
from .ts20_prediction import KIND
from .qp_constraints import solver_constraint_view
from . import prediction_chain as chain


CHAIN_CONFIG = {"kind": KIND, "gravity_m_s2": 9.8, "attitude_residual_decay_s": .15}
CANDIDATE_POLICY = {
    "q_linearization_error_max_rad": .006,
    "dq_linearization_error_max_rad_s": .10,
    "actual_bound_tolerance": 2e-7,
    "pending_guard_s": .05,
    "numerical_q_envelope_padding_rad": .0003,
    "numerical_dq_envelope_padding_rad_s": .03,
}
RELINEARIZATION_POLICY = {"max_subproblems": 3, "total_deadline_ms": 50.,
                         "max_joint_trust_radius_normalized": .02,
                         "radius_basis": ".58 s * .45 rad/s * .02 = 5.22 mrad, within 6 mrad local q budget"}


class Ts20MpcController(MpcController):
    """Receive explicit measured reference/packet history at every solve.

    context_provider(state,t) returns task-relative reference_state,
    packet_history, command_history and bus_origin_s. Histories describe the
    plant's actually consumed commands, never a previous unexecuted QP plan.
    """
    def __init__(self, model, pid_gains, *, predictor, context_provider,
                 trajectory=None, config=None):
        cfg = dict(config or {})
        if "prediction_chain" in cfg and cfg["prediction_chain"] != CHAIN_CONFIG:
            raise ValueError("TS20 MPC requires its explicit actuator prediction kind")
        cfg["prediction_chain"] = deepcopy(CHAIN_CONFIG)
        if "response_model" not in cfg:
            raise ValueError("TS20 upper MPC requires the explicitly bound base-response prior")
        self.actuator_predictor, self.context_provider = predictor, context_provider
        self._actuator_context = None
        self._frozen_context, self._linearization_plan = None, None
        super().__init__(model, pid_gains, trajectory=trajectory, config=cfg)

    @staticmethod
    def _normalize_prediction_chain(value):
        if value != CHAIN_CONFIG:
            raise ValueError("Invalid TS20 thrust/actuator chain configuration")
        return deepcopy(value)

    @staticmethod
    def _solver_constraint_view(C, lo, hi):
        """Opt in to exact row proofs without altering archived MPC solvers."""
        return solver_constraint_view(C, lo, hi)

    def _joint_prediction(self, t, times, h, state, warm):
        context = self.context_provider(state, t) if self._frozen_context is None else self._frozen_context
        required = {"reference_state", "packet_history", "command_history", "bus_origin_s"}
        if not isinstance(context, dict) or not required.issubset(context):
            raise ValueError("Actual TS20 packet/reference/consumption context is incomplete")
        self._actuator_context = context
        return self.actuator_predictor.predict(t, times, h, state,
            context["command_history"], self.velocity_limit[3:], warm,
            reference_state=context["reference_state"],
            packet_history=context["packet_history"], bus_origin_s=context["bus_origin_s"],
            exact_local=self._linearization_plan is not None)

    def _warm_start(self, t, h):
        if self._linearization_plan is not None:
            return self._linearization_plan.copy()
        return super()._warm_start(t, h)

    def _joint_constraints(self, constrain, prediction, dt, state, t, warm):
        # Preserve the old stopping/pending guard as a conservative protection;
        # it is not a robust stopping theorem for an unidentified loaded servo.
        guard_cfg = {"joint_delay_guard_s": CANDIDATE_POLICY["pending_guard_s"]}
        lo, hi, guard = chain.conservative_first_bounds(self, dt, state, prediction,
            self._actuator_context["command_history"], t, guard_cfg)
        reserve = (self.velocity_limit[3:]/(2*self.acceleration_limit[3:])+self.max_dt
                   +guard_cfg["joint_delay_guard_s"]+prediction["measurement_age_s"])
        for _, q, dq, Q, D in prediction["nodes"]:
            constrain(Q, self.q_lower-q, self.q_upper-q)
            constrain(D, -self.velocity_limit[3:]-dq, self.velocity_limit[3:]-dq)
            constrain(Q+reserve[:, None]*D,
                      self.q_lower-q-reserve*dq, self.q_upper-q-reserve*dq)
        for _, r, R in prediction["reference_nodes"]:
            constrain(R, self.q_lower-r, self.q_upper-r)
        indices = np.array([5*i+j for i in range(self.nodes) for j in (3, 4)])
        identity = np.eye(self.nodes*5)[indices]
        warm_z = warm.reshape(-1)/np.tile(self.velocity_limit, self.nodes)
        radius = min(self.actuator_predictor.trust_radius_normalized,
                     RELINEARIZATION_POLICY["max_joint_trust_radius_normalized"])
        constrain(identity, warm_z[indices]-radius, warm_z[indices]+radius)
        guard.update(trust_radius_normalized=radius, pending_guard_is_robust_proof=False,
                     actual_continuous_bounds_checked_before_publish=True)
        return lo, hi, guard

    def _validate_candidate(self, t, state, problem, plan, predicted):
        prediction = problem["joint_prediction"]
        actual = self.actuator_predictor.candidate_rollout(prediction, plan)
        node_indices = [int(np.argmin(abs(actual["time_s"]-(t+x))))
                        for x in np.cumsum(problem["h"])]
        q, dq = actual["q"][node_indices], actual["dq"][node_indices]
        q_error = float(np.max(abs(q-predicted[:, 6:8])))
        dq_error = float(np.max(abs(dq-predicted[:, 8:10])))
        tolerance = CANDIDATE_POLICY["actual_bound_tolerance"]
        envelope = actual.get("envelope")
        if envelope is None or not envelope.get("collected"):
            raise MpcSolveError("TS20 candidate lacks full-substep safety envelope",
                                {"accepted_optimum": False, "ts20_nonlinear_check_passed": False})
        qp = CANDIDATE_POLICY["numerical_q_envelope_padding_rad"]
        vp = CANDIDATE_POLICY["numerical_dq_envelope_padding_rad_s"]
        bounds_ok = (np.all(np.asarray(envelope["q_lower"])-qp >= self.q_lower-tolerance)
                     and np.all(np.asarray(envelope["q_upper"])+qp <= self.q_upper+tolerance)
                     and np.all(np.asarray(envelope["dq_lower"])-vp >= -self.velocity_limit[3:]-tolerance)
                     and np.all(np.asarray(envelope["dq_upper"])+vp <= self.velocity_limit[3:]+tolerance)
                     and np.all(actual["reference_nodes"] >= self.q_lower-tolerance)
                     and np.all(actual["reference_nodes"] <= self.q_upper+tolerance))
        passed = (bounds_ok and q_error <= CANDIDATE_POLICY["q_linearization_error_max_rad"]
                  and dq_error <= CANDIDATE_POLICY["dq_linearization_error_max_rad_s"])
        diagnostic = {"ts20_nonlinear_check_passed": bool(passed),
            "ts20_candidate_policy": deepcopy(CANDIDATE_POLICY),
            "ts20_candidate_q_linearization_error_rad": q_error,
            "ts20_candidate_dq_linearization_error_rad_s": dq_error,
            "ts20_candidate_envelope": deepcopy(envelope),
            "ts20_candidate_actual_bounds_passed": bool(bounds_ok),
            "ts20_bus_period_s": self.actuator_predictor.bus_period_s,
            "ts20_combined_interface_delay_s": self.actuator_predictor.delay_s,
            "ts20_reference_state_rad": prediction["r_current"].copy(),
            "ts20_prediction_model": self.actuator_predictor.metadata(),
            "base_response_is_prior": True, "supply_24v_qualified": False,
            "aerial_arm_load_qualified": False}
        if not passed:
            diagnostic["ts20_rejected_candidate_plan"] = plan.copy()
            raise MpcSolveError("TS20 nonlinear candidate rejected before command publication", diagnostic)
        # Report the nonlinear endpoint forecast used for the actual check.
        predicted = predicted.copy()
        predicted[:, 6:8], predicted[:, 8:10] = q, dq
        return predicted, diagnostic

    def step(self, t, dt, state, target):
        """Bounded sequential QPs, with one frozen observation and no dispatch.

        A rejected-but-bounded candidate may seed the next linearization. Its
        commands never become previous_command or actually sent history.
        All quality/bounds/nonlinear checks still apply to the final candidate.
        State is rolled back on rejection or total prepublication deadline.
        """
        begin = perf_counter()
        committed_names = ("_plan", "_plan_t", "_plan_h", "previous_command", "last_t", "_previous_stamp",
                           "step_count", "_joint_history", "_chain_clock_origin")
        saved = {name: deepcopy(getattr(self, name)) for name in committed_names}
        self._frozen_context = deepcopy(self.context_provider(state, t))
        self._linearization_plan = None
        attempts = []
        deadline = RELINEARIZATION_POLICY["total_deadline_ms"]
        try:
            for iteration in range(RELINEARIZATION_POLICY["max_subproblems"]):
                try:
                    out = super().step(t, dt, state, target)
                except MpcSolveError as exc:
                    diagnostic = exc.diagnostics
                    attempts.append({"subproblem": iteration+1,
                        "q_linearization_error_rad": diagnostic.get("ts20_candidate_q_linearization_error_rad"),
                        "dq_linearization_error_rad_s": diagnostic.get("ts20_candidate_dq_linearization_error_rad_s"),
                        "actual_bounds_passed": diagnostic.get("ts20_candidate_actual_bounds_passed"),
                        "elapsed_ms": (perf_counter()-begin)*1000, "passed": False})
                    retry = (diagnostic.get("ts20_candidate_actual_bounds_passed") is True
                             and "ts20_rejected_candidate_plan" in diagnostic
                             and iteration+1 < RELINEARIZATION_POLICY["max_subproblems"]
                             and (perf_counter()-begin)*1000 < deadline)
                    if not retry:
                        diagnostic.update(ts20_relinearization_attempts=attempts,
                                          ts20_total_compute_ms=(perf_counter()-begin)*1000)
                        raise
                    self._linearization_plan = diagnostic["ts20_rejected_candidate_plan"].copy()
                    continue
                elapsed = (perf_counter()-begin)*1000
                attempts.append({"subproblem": iteration+1, "passed": True,
                    "q_linearization_error_rad": out["diagnostics"]["ts20_candidate_q_linearization_error_rad"],
                    "dq_linearization_error_rad_s": out["diagnostics"]["ts20_candidate_dq_linearization_error_rad_s"],
                    "actual_bounds_passed": True, "elapsed_ms": elapsed})
                if elapsed > deadline:
                    raise MpcSolveError("TS20 total prepublication computation deadline exceeded",
                        {"accepted_optimum": False, "ts20_nonlinear_check_passed": True,
                         "ts20_command_accepted": False, "ts20_total_compute_ms": elapsed,
                         "ts20_relinearization_attempts": attempts,
                         "ts20_relinearization_policy": deepcopy(RELINEARIZATION_POLICY)})
                out["diagnostics"].update(compensation_scope="ts20_reference_bus_event_response_and_source_age",
                    joint_delay_origin="Single published combined interface delay after actual MIT send",
                    prediction_response_identified=False, base_response_is_prior=True,
                    ts20_command_accepted=True, ts20_relinearization_attempts=attempts,
                    ts20_relinearization_policy=deepcopy(RELINEARIZATION_POLICY),
                    compute_ms=elapsed, ts20_total_compute_ms=elapsed)
                return out
            raise RuntimeError("Unreachable TS20 relinearization exit")
        except BaseException:
            for name, value in saved.items():
                setattr(self, name, value)
            raise
        finally:
            self._frozen_context, self._linearization_plan = None, None

    def metadata(self):
        out = super().metadata()
        out.update(name="x650_upper_mpc_conditional_ts20_v1", response_identified=False,
            base_response_is_prior=True, actuator_prediction=self.actuator_predictor.metadata(),
            candidate_policy=deepcopy(CANDIDATE_POLICY), joint_time_constant_s=None,
            relinearization_policy=deepcopy(RELINEARIZATION_POLICY),
            prediction_model="Base response prior plus conditional MIT reference/bus/stiction joint model",
            joint_delay_origin="Published combined interface delay, applied once after actual MIT bus send",
            joint_delay_identified=False, supply_24v_qualified=False, aerial_arm_load_qualified=False,
            realtime_verified=False, physical_shaft_torque_model=False)
        return out
