"""Offline qualification of one explicit nonlinear OCP with IPOPT and acados.

The model layer owns all physics, objective expressions, and constraints. This
module never calls the legacy local-QP proposal/recovery controller. Both solver
backends consume the same CasADi stage expressions. Python is used for generation
and numerical qualification; generated acados C is the intended online solver.
No result here is authorization to publish an actuator command.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, is_dataclass
from pathlib import Path
import hashlib
import json
import time
import copy
import subprocess
from typing import Any

import numpy as np


@dataclass(frozen=True)
class SolverOptions:
    max_iterations: int = 100
    primal_tolerance: float = 2e-7
    stationarity_tolerance: float = 2e-3
    complementarity_tolerance: float = 2e-5
    active_tolerance: float = 2e-6
    original_issue_budget_ms: float = 50.
    reference_tolerance: float = 1e-8

    def validate(self):
        if self.max_iterations < 1 or any(not np.isfinite(value) or value <= 0 for value in
                (self.primal_tolerance, self.stationarity_tolerance,
                 self.complementarity_tolerance, self.active_tolerance,
                 self.reference_tolerance)) or not 0 <= self.original_issue_budget_ms <= 50.:
            raise ValueError("Invalid explicit numerical/deadline options")


def _array(value, shape=None):
    result = np.asarray(value, dtype=float)
    if shape is not None:
        result = result.reshape(shape)
    if not np.isfinite(result).all():
        raise ValueError("Nonfinite numerical OCP input/output")
    return result


def _max(value):
    value = np.asarray(value)
    return float(np.max(value)) if value.size else 0.


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _hash(value):
    return hashlib.sha256(_canonical(value)).hexdigest()


def _json(value):
    if is_dataclass(value):
        return _json(asdict(value))
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key:_json(item) for key,item in value.items()}
    if isinstance(value, (list,tuple)):
        return [_json(item) for item in value]
    return value


def independent_residuals(nlp, plan, *, options=SolverOptions(), supplied_duals=None):
    """Evaluate primal/KKT conditions independently of either solver's status.

    The condensed decision contains controls only; all states are propagated
    through the exact same discrete nonlinear model. At an acados solution its
    multiple-shooting states are separately checked against this rollout.
    Without supplied duals, nonnegative active-set multipliers are reconstructed
    by bounded least squares. With supplied duals, the full multipliers are
    independently checked against every bound (not trusted from solver status).
    This avoids discarding near-active interior-point constraints solely because
    their slack exceeds the active-set reconstruction threshold. Both checks
    test first-order KKT conditions, not sufficiency or global optimality.
    """
    from scipy.optimize import lsq_linear
    options.validate()
    z = _array(plan).reshape(-1)
    f, g, gradient, jacobian = nlp.evaluate(z)
    g, gradient, jacobian = _array(g).reshape(-1), _array(gradient).reshape(-1), _array(jacobian)
    lower, upper = nlp.lower_g, nlp.upper_g
    lbz, ubz = nlp.lower_z, nlp.upper_z
    primal = max(0.,_max(lower-g),_max(g-upper),_max(lbz-z),_max(z-ubz))
    eq = np.isfinite(lower) & np.isfinite(upper) & (lower == upper)
    columns, slacks, names = [], [], []
    if np.any(eq):
        columns.extend(jacobian[eq]);slacks.extend(np.zeros(int(eq.sum())))
        names.extend(["equality"]*int(eq.sum()))
    equality_count = len(columns)
    for derivative, slack, finite, label in (
            (-jacobian,g-lower,np.isfinite(lower)&~eq,"constraint_lower"),
            (jacobian,upper-g,np.isfinite(upper)&~eq,"constraint_upper"),
            (-np.eye(len(z)),z-lbz,np.isfinite(lbz),"control_lower"),
            (np.eye(len(z)),ubz-z,np.isfinite(ubz),"control_upper")):
        chosen = finite & (slack <= options.active_tolerance)
        columns.extend(derivative[chosen]);slacks.extend(slack[chosen])
        names.extend([label]*int(chosen.sum()))
    if columns:
        matrix = np.asarray(columns).T
        lower_dual = np.r_[np.full(equality_count,-np.inf),np.zeros(len(columns)-equality_count)]
        fit = lsq_linear(matrix,-gradient,bounds=(lower_dual,np.full(len(columns),np.inf)),
                         tol=1e-12,max_iter=500,lsq_solver="exact")
        multipliers = fit.x
        stationarity = _max(abs(gradient+matrix@multipliers))
        dual_fit_success = bool(fit.success)
        complementarity = _max(abs(multipliers[equality_count:]*np.asarray(slacks[equality_count:])))
        dual_violation = max(0.,_max(-multipliers[equality_count:]))
    else:
        multipliers = np.empty(0)
        stationarity = _max(abs(gradient));complementarity=dual_violation=0.;dual_fit_success=True
    result = dict(objective=float(f),primal_violation=primal,
        primal_residual_units="OCP constraint coordinates; joint/slew rows normalized; not one common physical unit",
        stationarity_inf=stationarity,complementarity_inf=complementarity,
        dual_nonnegativity_violation=dual_violation,active_constraint_count=len(columns),
        equality_count=equality_count,multiplier_reconstruction_succeeded=dual_fit_success,
        reconstructed_multipliers=multipliers.tolist(),active_constraint_types=names,
        feasible=primal<=options.primal_tolerance,
        first_order_kkt_passed=bool(dual_fit_success and primal<=options.primal_tolerance
            and stationarity<=options.stationarity_tolerance
            and complementarity<=options.complementarity_tolerance and dual_violation<=1e-10),
        scope="Independent condensed nonlinear OCP residuals; active-set multiplier fit; no global optimum claim")
    if supplied_duals is not None:
        lam_g = _array(supplied_duals["lam_g"]).reshape(g.shape)
        lam_z = _array(supplied_duals["lam_z"]).reshape(z.shape)
        residual = gradient+jacobian.T@lam_g+lam_z
        direct_stationarity = _max(abs(residual))
        direct_complementarity = direct_dual_violation = 0.
        # Signed interval multipliers: positive belongs to the upper bound,
        # negative to the lower bound. Equality multipliers are unrestricted.
        for value, lo, hi, signed in ((g,lower,upper,lam_g),(z,lbz,ubz,lam_z)):
            equality = np.isfinite(lo)&np.isfinite(hi)&(lo==hi)
            lower_dual, upper_dual = np.maximum(-signed,0.),np.maximum(signed,0.)
            for finite, dual, slack in ((np.isfinite(lo),lower_dual,value-lo),
                                         (np.isfinite(hi),upper_dual,hi-value)):
                bounded = finite&~equality
                direct_complementarity=max(direct_complementarity,
                    _max(abs(dual[bounded]*slack[bounded])))
                direct_dual_violation=max(direct_dual_violation,
                    _max(dual[~finite&~equality]))
        direct_pass=bool(primal<=options.primal_tolerance
            and direct_stationarity<=options.stationarity_tolerance
            and direct_complementarity<=options.complementarity_tolerance
            and direct_dual_violation<=1e-10)
        result["active_set_reconstruction"]={key:result[key] for key in
            ("stationarity_inf","complementarity_inf","dual_nonnegativity_violation",
             "first_order_kkt_passed","multiplier_reconstruction_succeeded")}
        result.update(stationarity_inf=direct_stationarity,
            complementarity_inf=direct_complementarity,
            dual_nonnegativity_violation=direct_dual_violation,
            first_order_kkt_passed=direct_pass,
            solver_dual_stationarity_inf=direct_stationarity,
            kkt_multiplier_source="supplied full signed multipliers; independently evaluated",
            supplied_multipliers=dict(lam_g=lam_g.tolist(),lam_z=lam_z.tolist()),
            scope="Independent full-bound primal, stationarity, complementarity and dual-sign residuals; "
                  "active-set reconstruction retained as diagnostic; no global optimum claim")
    else:
        result["kkt_multiplier_source"]="reconstructed nonnegative active-set multipliers"
    return result


def check_derivatives(nlp, plan, *, seed=650, directions=6, steps=(2e-5,1e-5)):
    """Compare graph AD gradients/Jacobians/Hessian with central differences."""
    z = _array(plan).reshape(-1)
    f,g,gradient,jacobian = nlp.evaluate(z)
    gradient,jacobian = _array(gradient).reshape(-1),_array(jacobian)
    rng = np.random.default_rng(seed)
    rows=[]
    multipliers = rng.normal(size=len(nlp.lower_g))*.01
    hessian = _array(nlp.lagrangian_hessian(z,multipliers))
    for _ in range(directions):
        direction = rng.normal(size=len(z));direction /= np.linalg.norm(direction)
        for step in steps:
            fp,gp,dfp,jgp = nlp.evaluate(z+step*direction)
            fm,gm,dfm,jgm = nlp.evaluate(z-step*direction)
            fd_f=(float(fp)-float(fm))/(2*step)
            fd_g=(_array(gp).reshape(-1)-_array(gm).reshape(-1))/(2*step)
            fd_h=(_array(dfp).reshape(-1)+_array(jgp).T@multipliers
                 -_array(dfm).reshape(-1)-_array(jgm).T@multipliers)/(2*step)
            rows.append(dict(step=step,
                objective_scaled_error=abs(float(gradient@direction)-fd_f)/(1+abs(fd_f)),
                constraint_scaled_error=_max(abs(jacobian@direction-fd_g))/(1+_max(abs(fd_g))),
                hessian_scaled_error=_max(abs(hessian@direction-fd_h))/(1+_max(abs(fd_h)))))
    maxima={key:max(row[key] for row in rows) for key in
            ("objective_scaled_error","constraint_scaled_error","hessian_scaled_error")}
    passed=maxima["objective_scaled_error"]<=2e-5 and maxima["constraint_scaled_error"]<=2e-5 \
           and maxima["hessian_scaled_error"]<=2e-4
    return dict(kind="x650_standard_nmpc_derivative_check_v1",passed=bool(passed),
        seed=seed,directions=directions,rows=rows,maxima=maxima,
        scope="Same symbolic objective/constraints/Lagrangian AD versus central finite differences")


def deadline_flags(original_issue_ns, completed_ns, budget_ms):
    """Never refresh an original issue stamp or confuse offline time with WCET."""
    if original_issue_ns is None:
        return dict(original_issue_available=False,issue_age_ms=None,deadline_passed=False,
                    hard_realtime_qualified=False)
    if not isinstance(original_issue_ns,int) or original_issue_ns<=0 or completed_ns<original_issue_ns:
        raise ValueError("Original issue must use the current host's monotonic clock")
    age=(completed_ns-original_issue_ns)/1e6
    return dict(original_issue_available=True,issue_age_ms=age,deadline_passed=age<=budget_ms,
                hard_realtime_qualified=False)


class CondensedNlp:
    """One single-shooting graph used as an independent audit and IPOPT NLP.

    Flattened controls are stage-major physical units, not normalized QP steps.
    Numeric fixture history has already been encoded by ``build_case``; nothing
    here fabricates a packet receipt or advances committed controller memory.
    """

    def __init__(self, model, numerical_case):
        import casadi as ca
        self.model, self.case = model, numerical_case
        self.nx, self.nu, self.np = int(model.x.numel()), int(model.u.numel()), int(model.p.numel())
        self.horizon = _array(numerical_case.horizon).reshape(-1)
        self.N = len(self.horizon)
        if (self.nx,self.nu,self.N)!=(23,5,15) or np.any(self.horizon<=0):
            raise ValueError("This qualification interface requires 15 stages, 23 states and 5 controls")
        self.x0 = _array(numerical_case.x0,(self.nx,))
        self.parameters = _array(numerical_case.parameters,(self.N+1,self.np))
        self.initial_plan = _array(numerical_case.initial_plan,(self.N,self.nu))
        self.lower_z = np.tile(np.asarray(model.control_lower,dtype=float).reshape(self.nu),self.N)
        self.upper_z = np.tile(np.asarray(model.control_upper,dtype=float).reshape(self.nu),self.N)
        self.nh=int(model.constraints.numel())
        self.lower_g=np.tile(np.asarray(model.constraint_lower,dtype=float).reshape(self.nh),self.N)
        self.upper_g=np.tile(np.asarray(model.constraint_upper,dtype=float).reshape(self.nh),self.N)
        for lower,upper in ((self.lower_z,self.upper_z),(self.lower_g,self.upper_g)):
            if np.any(np.isnan(lower)) or np.any(np.isnan(upper)) or np.any(lower>upper):
                raise ValueError("Malformed hard constraint bounds")
        self.stage = ca.Function("standard_stage",[model.x,model.u,model.p],
            [model.discrete,model.cost,model.constraints])
        self.z=ca.MX.sym("U",self.N*self.nu)
        self.P=ca.MX.sym("P",self.nx+(self.N+1)*self.np)
        self.parameter_values=np.r_[self.x0,self.parameters.reshape(-1)]
        state=self.P[:self.nx]
        states=[state]; costs=[]; constraints=[]
        for stage in range(self.N):
            control=self.z[stage*self.nu:(stage+1)*self.nu]
            parameter=self.P[self.nx+stage*self.np:self.nx+(stage+1)*self.np]
            state,cost,con=self.stage(state,control,parameter)
            states.append(state); costs.append(cost); constraints.append(con)
        self.f=ca.sum1(ca.vertcat(*costs))
        self.g=ca.vertcat(*constraints)
        self.X=ca.horzcat(*states)
        self._evaluation=ca.Function("standard_evaluation",[self.z,self.P],
            [self.f,self.g,ca.gradient(self.f,self.z),ca.jacobian(self.g,self.z)])
        self._rollout=ca.Function("standard_rollout",[self.z,self.P],[self.X])
        lam=ca.MX.sym("lambda",int(self.g.numel()))
        self._hessian=ca.Function("standard_lagrangian_hessian",[self.z,self.P,lam],
            [ca.hessian(self.f+ca.dot(lam,self.g),self.z)[0]])
        self.graph_sha256=hashlib.sha256(self.stage.serialize().encode()).hexdigest()
        self.binding_sha256=_hash(dict(graph=self.graph_sha256,x0=self.x0.tolist(),
            parameters=self.parameters.tolist(),horizon=self.horizon.tolist()))

    def evaluate(self, plan):
        values=self._evaluation(_array(plan,(self.N*self.nu,)),self.parameter_values)
        return float(values[0]),_array(values[1]).reshape(-1),_array(values[2]).reshape(-1),_array(values[3])

    def with_case(self,numerical_case):
        """Reuse graph construction only; bind a new independent numeric case."""
        result=copy.copy(self)
        result.case=numerical_case
        result.horizon=_array(numerical_case.horizon,(self.N,))
        if np.any(result.horizon<=0): raise ValueError("Nonpositive stage duration")
        result.x0=_array(numerical_case.x0,(self.nx,))
        result.parameters=_array(numerical_case.parameters,(self.N+1,self.np))
        result.initial_plan=_array(numerical_case.initial_plan,(self.N,self.nu))
        result.parameter_values=np.r_[result.x0,result.parameters.reshape(-1)]
        result.binding_sha256=_hash(dict(graph=result.graph_sha256,x0=result.x0.tolist(),
            parameters=result.parameters.tolist(),horizon=result.horizon.tolist()))
        return result

    def rollout(self, plan):
        return _array(self._rollout(_array(plan,(self.N*self.nu,)),self.parameter_values)).T

    def lagrangian_hessian(self,plan,multipliers):
        return _array(self._hessian(_array(plan,(self.N*self.nu,)),self.parameter_values,
                                   _array(multipliers,(self.N*self.nh,))))

    def dump_case_json(self,path,*,source_identity=None,config_sha256=None):
        """Export all numeric solver data for a Python-free C/C++ reproduction."""
        payload=dict(kind="x650_standard_nmpc_numeric_case_v1",model_name="x650_standard",
            N=self.N,nx=self.nx,nu=self.nu,np=self.np,nh=self.nh,
            graph_sha256=self.graph_sha256,binding_sha256=self.binding_sha256,
            source_identity=source_identity,config_sha256=config_sha256,
            x0=self.x0,p=self.parameters,guess_x=self.rollout(self.initial_plan),
            guess_u=self.initial_plan,time_steps=self.horizon,cost_scaling=np.ones(self.N+1),
            lbu=self.lower_z.reshape(self.N,self.nu),ubu=self.upper_z.reshape(self.N,self.nu),
            lh=self.lower_g.reshape(self.N,self.nh),uh=self.upper_g.reshape(self.N,self.nh),
            metadata=self.case.metadata,
            terminal=dict(cost=0.,constraints=[],control=None),
            scope="Independent snapshot; guess is numerical initialization, never an executed command")
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(_json(payload),indent=2,allow_nan=False)+"\n",encoding="utf-8")
        return dict(path=str(path.resolve()),bytes=path.stat().st_size,
                    sha256=hashlib.sha256(path.read_bytes()).hexdigest())

    def result(self, plan, *, backend, raw_status, solver_success, elapsed_ms,
               options, original_issue_ns=None, solver_states=None, supplied_duals=None,
               details=None):
        validation_begin=time.monotonic_ns()
        plan=_array(plan,(self.N,self.nu))
        states=self.rollout(plan)
        residuals=independent_residuals(self,plan,options=options,supplied_duals=supplied_duals)
        dynamics_residual=0.
        if solver_states is not None:
            solver_states=_array(solver_states,(self.N+1,self.nx))
            # Local defects and accumulated disagreement are both recorded.
            defects=[_max(abs(solver_states[0]-self.x0))]
            for k in range(self.N):
                next_state,_,_=self.stage(solver_states[k],plan[k],self.parameters[k])
                defects.append(_max(abs(solver_states[k+1]-_array(next_state).reshape(-1))))
            dynamics_residual=max(defects)
        completed=time.monotonic_ns()
        deadline=deadline_flags(original_issue_ns,completed,options.original_issue_budget_ms)
        feasible=residuals["feasible"] and dynamics_residual<=options.primal_tolerance
        result=dict(kind="x650_standard_nmpc_solver_result_v1",backend=backend,
            graph_sha256=self.graph_sha256,binding_sha256=self.binding_sha256,
            raw_status=raw_status,solver_success=bool(solver_success),solve_elapsed_ms=float(elapsed_ms),
            postsolve_validation_elapsed_ms=(completed-validation_begin)/1e6,
            plan=plan.tolist(),plan_flat=plan.reshape(-1).tolist(),states=states.tolist(),
            objective=residuals["objective"],residuals=residuals,
            multiple_shooting_dynamics_residual_inf=dynamics_residual,
            rollout_disagreement_inf=0. if solver_states is None else _max(abs(solver_states-states)),
            numerically_feasible=bool(feasible),
            first_order_kkt_passed=bool(feasible and residuals["first_order_kkt_passed"]),
            deadline=deadline,accepted_for_publication=False,native_event_validation_required=True,
            rejection_reasons=[],details={} if details is None else _json(details),
            scope="Offline smooth upper-layer OCP; no global optimality or live actuator qualification")
        if not solver_success: result["rejection_reasons"].append("solver_status_not_success")
        if not feasible: result["rejection_reasons"].append("independent_hard_constraint_or_dynamics_residual")
        if not result["first_order_kkt_passed"]: result["rejection_reasons"].append("first_order_kkt_residual")
        if not deadline["deadline_passed"]: result["rejection_reasons"].append("original_issue_deadline_unqualified")
        result["rejection_reasons"].append("native_event_and_live_interface_not_qualified")
        return result


class IpoptReference:
    """High-accuracy local reference for exactly the condensed nonlinear OCP."""

    def __init__(self,nlp,options=SolverOptions()):
        import casadi as ca
        options.validate()
        self.nlp,self.options=nlp,options
        self.solver=ca.nlpsol("standard_ipopt_reference","ipopt",
            {"x":nlp.z,"p":nlp.P,"f":nlp.f,"g":nlp.g},
            {"print_time":False,"error_on_fail":False,
             "ipopt.print_level":0,"ipopt.sb":"yes","ipopt.max_iter":options.max_iterations,
             "ipopt.tol":options.reference_tolerance,"ipopt.constr_viol_tol":options.reference_tolerance,
             "ipopt.acceptable_iter":0,"ipopt.nlp_scaling_method":"none",
             "ipopt.bound_relax_factor":0.})

    def bind(self,nlp):
        if nlp.graph_sha256!=self.nlp.graph_sha256:
            raise ValueError("Cannot reuse IPOPT solver with a changed symbolic OCP")
        self.nlp=nlp

    def solve(self,initial_plan=None,*,original_issue_ns=None):
        nlp=self.nlp
        initial=nlp.initial_plan if initial_plan is None else _array(initial_plan,(nlp.N,nlp.nu))
        start=time.monotonic_ns()
        solution=self.solver(x0=initial.reshape(-1),p=nlp.parameter_values,
            lbx=nlp.lower_z,ubx=nlp.upper_z,lbg=nlp.lower_g,ubg=nlp.upper_g)
        elapsed=(time.monotonic_ns()-start)/1e6
        stats=self.solver.stats()
        return nlp.result(_array(solution["x"]),backend="ipopt_reference",
            raw_status=str(stats.get("return_status","unknown")),solver_success=stats.get("success",False),
            elapsed_ms=elapsed,options=self.options,original_issue_ns=original_issue_ns,
            supplied_duals={"lam_g":_array(solution["lam_g"]).reshape(-1),
                            "lam_z":_array(solution["lam_x"]).reshape(-1)},
            details=dict(iterations=int(stats.get("iter_count",-1)),
                         same_symbolic_graph=True,global_optimum_claim=False))


class AcadosSolver:
    """Generate the production C solver, exercise it offline through its Python wrapper.

    Stage objective already includes h. Cost scaling is explicitly one, avoiding
    an accidental second multiplication by acados' default time-step scaling.
    No local trust-radius box or hidden slacks are added to the model's OCP.
    """

    def __init__(self,nlp,export_directory,*,mode="SQP",options=SolverOptions(),verbose=False):
        import casadi as ca
        from acados_template import AcadosModel,AcadosOcp,AcadosOcpSolver
        options.validate()
        if mode not in ("SQP","SQP_RTI"):
            raise ValueError("Explicit mode must be SQP or SQP_RTI")
        self.nlp,self.options,self.mode=nlp,options,mode
        export=Path(export_directory).resolve()
        export.mkdir(parents=True,exist_ok=True)
        binding=dict(kind="x650_standard_acados_generation_v1",graph_sha256=nlp.graph_sha256,
            mode=mode,options=asdict(options),casadi_version=ca.__version__,acados_version="v0.6.0",
            N=nlp.N,nx=nlp.nx,nu=nlp.nu,np=nlp.np,
            hard_constraints_per_stage=nlp.nh,cost_scaling="one; h already in symbolic stage objective",
            hessian_approximation="GAUSS_NEWTON",cost_type="NONLINEAR_LS",
            ocp_name="x650_standard",cold_residual_hessian_compile_optimization="-O0; all hot paths -O2",
            nls_expression="sqrt(2*h)*residual; W=I; zero reference; same h*residual^T*residual objective")
        existing=export/"generation_binding.json"
        if existing.exists() and json.loads(existing.read_text(encoding="utf-8"))!=binding:
            raise ValueError("Refusing to reuse generated C directory with a different OCP binding")
        model=AcadosModel()
        model.name="x650_standard"
        model.x,model.u,model.p=nlp.model.x,nlp.model.u,nlp.model.p
        model.disc_dyn_expr=nlp.model.discrete
        # acados NLS is 0.5*y'W*y. This preserves the exact nonlinear
        # objective while using its standard positive-semidefinite SQP
        # Hessian approximation. IPOPT and independent Hessian audits stay
        # exact. Dynamics and constraint first derivatives remain exact AD.
        h=nlp.model.p[nlp.model.parameter_layout["h"]][0]
        y=ca.sqrt(2*h)*nlp.model.residual
        model.cost_y_expr=model.cost_y_expr_0=y
        model.cost_y_expr_e=ca.SX.zeros(1,1)
        model.con_h_expr=model.con_h_expr_0=nlp.model.constraints
        ocp=AcadosOcp();ocp.model=model;ocp.name="x650_standard"
        ocp.cost.cost_type=ocp.cost.cost_type_0=ocp.cost.cost_type_e="NONLINEAR_LS"
        ny=int(y.numel())
        ocp.cost.W=ocp.cost.W_0=np.eye(ny)
        ocp.cost.yref=ocp.cost.yref_0=np.zeros(ny)
        ocp.cost.W_e=np.eye(1);ocp.cost.yref_e=np.zeros(1)
        ocp.constraints.idxbu=np.arange(nlp.nu,dtype=np.int64)
        ocp.constraints.lbu=nlp.lower_z[:nlp.nu].copy()
        ocp.constraints.ubu=nlp.upper_z[:nlp.nu].copy()
        ocp.constraints.lh=ocp.constraints.lh_0=nlp.lower_g[:nlp.nh].copy()
        ocp.constraints.uh=ocp.constraints.uh_0=nlp.upper_g[:nlp.nh].copy()
        ocp.constraints.x0=nlp.x0.copy()
        ocp.parameter_values=nlp.parameters[0].copy()
        opt=ocp.solver_options
        opt.N_horizon=nlp.N;opt.tf=float(nlp.horizon.sum());opt.time_steps=nlp.horizon.copy()
        opt.cost_scaling=np.ones(nlp.N+1)
        opt.integrator_type="DISCRETE";opt.nlp_solver_type=mode
        opt.hessian_approx="GAUSS_NEWTON"
        opt.exact_hess_cost=opt.exact_hess_dyn=opt.exact_hess_constr=0
        opt.regularize_method="PROJECT";opt.reg_epsilon=1e-8
        opt.qp_solver="PARTIAL_CONDENSING_HPIPM";opt.qp_solver_cond_N=5
        opt.qp_solver_ric_alg=0;opt.qp_solver_cond_ric_alg=0
        opt.globalization="MERIT_BACKTRACKING" if mode=="SQP" else "FIXED_STEP"
        opt.nlp_solver_max_iter=options.max_iterations
        opt.nlp_solver_tol_stat=opt.nlp_solver_tol_eq=options.reference_tolerance
        opt.nlp_solver_tol_ineq=opt.nlp_solver_tol_comp=options.reference_tolerance
        opt.qp_solver_iter_max=100
        opt.qp_solver_tol_stat=opt.qp_solver_tol_eq=options.reference_tolerance/10
        opt.qp_solver_tol_ineq=opt.qp_solver_tol_comp=options.reference_tolerance/10
        opt.print_level=0
        ocp.code_gen_options.code_export_directory=str(export/"c_generated_code")
        ocp.code_gen_options.json_file="acados_ocp.json"
        ocp.code_gen_options.ext_fun_compile_flags="-O2"
        self.ocp=ocp
        AcadosOcpSolver.generate(ocp)
        code=Path(ocp.code_gen_options.code_export_directory)
        generated_options=json.loads((code/"acados_ocp.json").read_text(encoding="utf-8"))["solver_options"]
        if generated_options["hessian_approx"]!="GAUSS_NEWTON" or generated_options["exact_hess_cost"]!=0:
            raise ValueError("Cold residual Hessian compilation is allowed only with explicit Gauss Newton")
        cold=["x650_standard_cost/x650_standard_cost_y_0_hess.o",
              "x650_standard_cost/x650_standard_cost_y_hess.o"]
        for obj in cold:
            if not (code/Path(obj).with_suffix(".c")).is_file():
                raise ValueError("Expected generated residual Hessian source missing")
        flags=code/"cold_hessian_flags.mk"
        flags.write_text("# These exact residual-Hessian expressions are unused by the chosen GN branch.\n"+
            "\n".join(f"{obj}: CFLAGS += -O0" for obj in cold)+"\n",encoding="utf-8")
        command=["make","-f","Makefile","-f",flags.name,"-j2","ocp_shared_lib"]
        with (export/"compilation.log").open("w",encoding="utf-8") as log:
            subprocess.run(command,cwd=code,stdout=log,stderr=subprocess.STDOUT,check=True)
        build_flags=dict(kind="standard_acados_object_compilation_v1",command=command,
            default_flags="-fPIC -std=c99 -O2",overrides={obj:"-fPIC -std=c99 -O2 -O0" for obj in cold},
            rationale="acados v0.6.0 NLS GN branch uses J^T W J; exact residual Hessian expressions retained but unused",
            hessian_approximation="GAUSS_NEWTON",exact_hess_cost=0,mathematical_expressions_unchanged=True)
        (export/"compilation_flags.json").write_text(json.dumps(build_flags,indent=2)+"\n",encoding="utf-8")
        physical=_json(nlp.model.config.as_dict());surrogate=_json(nlp.model.surrogate.as_dict())
        header="#pragma once\nnamespace x650_standard_generated {\n"
        for key,value in (("graph_sha256",nlp.graph_sha256),("physical_config_sha256",_hash(physical)),
                          ("solver_mode",mode)):
            header+=f'inline constexpr char {key}[]="{value}";\n'
        for key,value in (("physical_config_json",physical),("surrogate_json",surrogate)):
            header+=f'inline constexpr char {key}[]=R"X650JSON('+_canonical(value).decode()+')X650JSON";\n'
        header+="}\n"
        (code/"x650_model_identity.hpp").write_text(header,encoding="utf-8")
        self.solver=AcadosOcpSolver(ocp,generate=False,build=False,verbose=verbose)
        existing.write_text(json.dumps(binding,indent=2,allow_nan=False)+"\n",encoding="utf-8")
        generated=[]
        for artifact in sorted(export.rglob("*")):
            if artifact.is_file() and artifact.suffix in (".c",".h",".hpp",".json",".so",".dll",".dylib",".mk") \
                    and artifact.name!="generated_solver_manifest.json":
                generated.append(dict(path=str(artifact),bytes=artifact.stat().st_size,
                    sha256=hashlib.sha256(artifact.read_bytes()).hexdigest()))
        (export/"generated_solver_manifest.json").write_text(json.dumps(dict(
            kind="x650_standard_acados_generated_manifest_v1",binding=binding,files=generated),
            indent=2,allow_nan=False)+"\n",encoding="utf-8")
        self.export_directory=export

    def bind(self,nlp):
        if nlp.graph_sha256!=self.nlp.graph_sha256:
            raise ValueError("Cannot reuse generated solver with a changed symbolic OCP")
        if not np.array_equal(nlp.horizon,self.nlp.horizon):
            self.solver.set_new_time_steps(nlp.horizon)
        self.nlp=nlp

    def solve(self,initial_plan=None,*,original_issue_ns=None):
        nlp=self.nlp
        controls=nlp.initial_plan if initial_plan is None else _array(initial_plan,(nlp.N,nlp.nu))
        states=nlp.rollout(controls)
        # Cases are independent snapshots. No prior fixture's numerical state,
        # duals, or simulated actuator history may leak into this test.
        self.solver.reset(reset_qp_solver_mem=True)
        for stage in range(nlp.N+1):
            self.solver.set(stage,"x",states[stage])
            self.solver.set(stage,"p",nlp.parameters[stage])
            self.solver.cost_set(stage,"scaling",np.array([1.]))
            if stage<nlp.N: self.solver.set(stage,"u",controls[stage])
        self.solver.constraints_set(0,"lbx",nlp.x0)
        self.solver.constraints_set(0,"ubx",nlp.x0)
        if self.mode=="SQP_RTI": self.solver.options_set("rti_phase",0)
        start=time.monotonic_ns();status=int(self.solver.solve());elapsed=(time.monotonic_ns()-start)/1e6
        plan=np.vstack([self.solver.get(stage,"u") for stage in range(nlp.N)])
        returned_states=np.vstack([self.solver.get(stage,"x") for stage in range(nlp.N+1)])
        returned_state_objective=sum(float(nlp.stage(returned_states[stage],plan[stage],nlp.parameters[stage])[1])
                                     for stage in range(nlp.N))
        details=dict(mode=self.mode,generated_c=True,iterations=int(self.solver.get_stats("sqp_iter")),
            hessian_approximation="GAUSS_NEWTON",cost_type="NONLINEAR_LS",
            internal_total_time_ms=1000.*float(self.solver.get_stats("time_tot")),
            reported_cost=float(self.solver.get_cost()),
            solver_reported_residuals=_array(self.solver.get_residuals(recompute=True)).tolist(),
            rti_status_is_not_convergence=True,global_optimum_claim=False)
        result=nlp.result(plan,backend="acados_"+self.mode.lower(),raw_status=status,
            solver_success=status==0,elapsed_ms=elapsed,options=self.options,
            original_issue_ns=original_issue_ns,solver_states=returned_states,details=details)
        details=result["details"]
        details["reported_vs_condensed_cost_abs_difference"]=abs(details["reported_cost"]-result["objective"])
        details["returned_state_symbolic_objective"]=returned_state_objective
        details["reported_vs_returned_state_symbolic_cost_abs_difference"]=abs(details["reported_cost"]-returned_state_objective)
        details["objective_expression_consistent"]=bool(
            details["reported_vs_returned_state_symbolic_cost_abs_difference"]
            <= self.options.reference_tolerance*max(1.,abs(returned_state_objective)))
        if not details["objective_expression_consistent"]:
            result["first_order_kkt_passed"]=False
            result["rejection_reasons"].append("solver_and_independent_symbolic_objective_disagree")
        return result
