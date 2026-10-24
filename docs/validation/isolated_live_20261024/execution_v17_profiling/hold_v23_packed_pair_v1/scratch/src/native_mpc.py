"""Explicit native numerical backend for the conditional TS20 upper MPC.

The archived Python implementation remains untouched and is the numerical
reference. Step lifecycle is deliberately identical; TS20's outer transaction,
relinearization limit and 50 ms rejection deadline are inherited unchanged.
Observation/packet preparation and lifecycle still run in Python. This is not
an independent C++ ROS executor or a hard-real-time certification.
"""
from copy import deepcopy
from time import perf_counter
from types import SimpleNamespace
import numpy as np
from .mpc import MpcController, MpcSolveError, MIXED_RESPONSE_KIND, MIXED_STATE_ORDER
from .model import vector, rotation_matrix, tip_world, tip_body
from .ts20_mpc import Ts20MpcController, CANDIDATE_POLICY, RELINEARIZATION_POLICY, CHAIN_CONFIG
from .native_optimization import NativeOptimization
from .native_joint_prediction import NativeJointMatrices


class NativeMpcStep(MpcController):
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
        g = problem["g"]
        solve_begin = perf_counter()
        try:
            z, quality, exact_center = self.native_optimization.solve(problem, self.max_iterations)
        except ValueError as exc:
            raise MpcSolveError(str(exc), {"accepted_optimum": False,
                "compute_ms": (perf_counter()-begin)*1000}) from exc
        solve_ms = (perf_counter()-solve_begin)*1000
        result = SimpleNamespace(success=True, status=0, nit=-1,
                                 message="Native full-constraint convex QP")
        constraint_view = {"original_rows":len(problem["C"])+len(g),
                           "solver_rows":len(problem["C"])+len(g),
                           "path":"native_full_rows"}
        accepted = (bool(result.success) and np.isfinite(z).all()
                    and quality["primal_violation"] <= self.feasibility_tolerance
                    and quality["kkt_stationarity_inf"] <= self.kkt_tolerance)
        fallback = False
        diagnostics = {"solver": ("cpp_cholesky_exact_feasible_convex_qp" if exact_center else
                                  "cpp_quadprogpp_exact_hessian_whitened_convex_qp"), "solver_success": bool(result.success),
                       "solver_status": int(result.status), "solver_message": str(result.message),
                       "solver_iterations": int(result.nit), "solve_ms": solve_ms,
                       "solve_budget_ms": self.solve_budget_ms, "solve_budget_exceeded": solve_ms > self.solve_budget_ms,
                       "solver_constraint_view": constraint_view,
                       "accepted_optimum": accepted, "previous_plan_fallback": fallback, **quality}
        if not accepted and not fallback:
            diagnostics["compute_ms"] = (perf_counter()-begin)*1000
            raise MpcSolveError("MPC solve failed quality/feasibility checks", diagnostics)
        plan = (z*problem["scale"]).reshape(self.nodes,5)
        command = plan[0].copy()
        acceleration = (command-self.previous_command)/dt
        predicted = problem["offsets"]+np.einsum("nij,j->ni", problem["jacobians"], z)
        predicted, actuator_diagnostics = self._validate_candidate(t, state, problem, plan, predicted)
        diagnostics.update(actuator_diagnostics)
        if self.prediction_chain is not None:
            chain_result = self.native_optimization.endpoint(self, problem, predicted, plan)
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
                solve_method="exact_feasible_center" if exact_center else "quadprogpp",
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



class NativeTs20MpcController(Ts20MpcController, NativeMpcStep):
    def __init__(self, *args, native_library, native_sha256, **kwargs):
        self.native_optimization = NativeOptimization(native_library,native_sha256)
        super().__init__(*args, **kwargs)
        self.actuator_predictor = NativeJointMatrices(self.actuator_predictor, self.native_optimization)
        if self.failure_policy != "raise":
            raise ValueError("Native TS20 requires reject-without-fallback")
        if (CANDIDATE_POLICY != {"q_linearization_error_max_rad":.006,
                "dq_linearization_error_max_rad_s":.10,"actual_bound_tolerance":2e-7,
                "pending_guard_s":.05,"numerical_q_envelope_padding_rad":.0003,
                "numerical_dq_envelope_padding_rad_s":.03}
                or RELINEARIZATION_POLICY["max_joint_trust_radius_normalized"] != .02
                or CHAIN_CONFIG["gravity_m_s2"] != 9.8
                or CHAIN_CONFIG["attitude_residual_decay_s"] != .15):
            raise ValueError("Native kernel requires its frozen candidate/attitude policy")

    def _problem(self,t,dt,state,target):
        return self.native_optimization.assemble(self,t,dt,state,target)

    def _joint_prediction(self,t,times,h,state,warm):
        if not isinstance(self.actuator_predictor,NativeJointMatrices):
            return super()._joint_prediction(t,times,h,state,warm)
        context = self.context_provider(state,t) if self._frozen_context is None else self._frozen_context
        required = {"reference_state","packet_history","command_history","bus_origin_s"}
        if not isinstance(context,dict) or not required.issubset(context):
            raise ValueError("Actual TS20 packet/reference/consumption context is incomplete")
        self._actuator_context = context
        return self.actuator_predictor._predict_packed(t,times,h,state,
            context["command_history"],self.velocity_limit[3:],warm,
            reference_state=context["reference_state"],packet_history=context["packet_history"],
            bus_origin_s=context["bus_origin_s"],exact_local=self._linearization_plan is not None)

    def _validate_candidate(self, t, state, problem, plan, predicted):
        prediction = problem["joint_prediction"]
        if isinstance(self.actuator_predictor,NativeJointMatrices):
            actual = self.actuator_predictor._candidate_packed(prediction,plan)
        else:
            actual = self.actuator_predictor.candidate_rollout(prediction,plan)
        node_indices = [int(np.argmin(abs(actual["time_s"]-(t+x))))
                        for x in np.cumsum(problem["h"])]
        q, dq = actual["q"][node_indices], actual["dq"][node_indices]
        envelope = actual.get("envelope")
        try:
            q_error,dq_error,bounds_ok,passed = self.native_optimization.candidate(
                self,predicted,q,dq,actual)
        except ValueError as exc:
            raise MpcSolveError(str(exc), {"accepted_optimum":False,
                                         "ts20_nonlinear_check_passed":False}) from exc
        diagnostic = {"ts20_nonlinear_check_passed": bool(passed),
            "ts20_candidate_policy": deepcopy(CANDIDATE_POLICY),
            "ts20_candidate_q_linearization_error_rad": q_error,
            "ts20_candidate_dq_linearization_error_rad_s": dq_error,
            # candidate_rollout owns this new envelope; it has no future
            # writers. Transfer it instead of copying every extremum dict.
            "ts20_candidate_envelope": envelope,
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


    def metadata(self):
        out = super().metadata()
        out.update(name="x650_upper_mpc_conditional_ts20_cpp_numerics_v1",
                   solver="C++ Eigen Cholesky and bounded QuadProg++ active-set convex QP",
                   solver_ftol=None,
                   native_optimization=dict(self.native_optimization.receipt),
                   native_scope=["matrix_assembly","qp_solve","full_row_kkt","candidate_gates","endpoint_forecast"],
                   orchestration="Python; unchanged TS20 transactional deadline and relinearization",
                   realtime_verified=False)
        return out
