"""C2 figure-eight references and bounded body/joint velocity coordination.

This is a kinematic whole-system reference allocator, not dynamic whole-body
MPC. Base attitude is a measured underactuated response, never a decision
coordinate. Fixed mode assigns 20% of the nominal body-plane task to the arm;
coordinated mode allocates reference rates online using actual tip feedback.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .braking import tighten_velocity_bounds
from .kinematics import (tip_body, tip_world, joint_jacobian_body,
    joint_centripetal_body, inverse_tip_body, rotation_matrix, _vector)


@dataclass
class BoxLeastSquaresResult:
    x: np.ndarray
    active_mask: np.ndarray
    nit: int


def bounded_least_squares(matrix, rhs, lower, upper, tol=1e-10, max_iter=128):
    """NumPy-only full-column-rank BVLS, for the five-variable allocator.

    A feasible active set is maintained. Each free-variable least-squares
    minimizer is reached by a bound-limited line search; active coordinates
    are released only for a violated KKT gradient sign. This solves the
    original convex bounded problem, unlike clipping an unconstrained result.
    A small Tikhonov block in the caller guarantees unique free solutions.
    """
    A,b = np.asarray(matrix,float),np.asarray(rhs,float)
    lo,hi = np.asarray(lower,float),np.asarray(upper,float)
    if (A.ndim != 2 or b.shape != (A.shape[0],) or lo.shape != (A.shape[1],)
            or hi.shape != lo.shape or not all(np.isfinite(v).all() for v in (A,b,lo,hi))
            or np.any(lo>hi)):
        raise ValueError("Invalid finite bounded least-squares problem")
    # A valid braking domain can collapse to one exact value. Eliminate those
    # fixed coordinates instead of widening a limit or rejecting the solution.
    fixed = lo == hi
    if np.any(fixed):
        result = lo.copy()
        active = np.full(len(lo),2,dtype=int)
        if np.all(fixed):
            return BoxLeastSquaresResult(result,active,0)
        free_result = bounded_least_squares(A[:,~fixed],b-A[:,fixed]@lo[fixed],
                                           lo[~fixed],hi[~fixed],tol,max_iter)
        result[~fixed] = free_result.x
        active[~fixed] = free_result.active_mask
        return BoxLeastSquaresResult(result,active,free_result.nit)
    x0,_,rank,_ = np.linalg.lstsq(A,b,rcond=None)
    if rank != A.shape[1]:
        raise ValueError("Bounded allocator requires full column rank")
    x = np.clip(x0,lo,hi)
    active = np.zeros(len(lo),dtype=int)
    active[x0<=lo] = -1
    active[x0>=hi] = 1
    g_scale = max(1.,float(np.max(np.abs(A.T@b))))
    threshold = tol*g_scale
    for iteration in range(1,max_iter+1):
        free = active == 0
        z = x.copy()
        if np.any(free):
            residual_rhs = b-A[:,~free]@x[~free]
            z[free] = np.linalg.lstsq(A[:,free],residual_rhs,rcond=None)[0]
        if np.any(z[free]<lo[free]) or np.any(z[free]>hi[free]):
            direction = z-x
            ratios = np.full(len(x),np.inf)
            lower_hit = free & (z<lo)
            upper_hit = free & (z>hi)
            ratios[lower_hit] = (lo[lower_hit]-x[lower_hit])/direction[lower_hit]
            ratios[upper_hit] = (hi[upper_hit]-x[upper_hit])/direction[upper_hit]
            blocker = int(np.argmin(ratios))
            step = float(np.clip(ratios[blocker],0.,1.))
            x += step*direction
            # Explicitly pin the first blocking bound; feasibility tolerances
            # do not propagate into subsequent physical-reference integration.
            active[blocker] = -1 if lower_hit[blocker] else 1
            x[blocker] = lo[blocker] if lower_hit[blocker] else hi[blocker]
            x = np.clip(x,lo,hi)
            continue
        x = z
        gradient = A.T@(A@x-b)
        violation = np.where(active==-1,-gradient,np.where(active==1,gradient,-np.inf))
        release = int(np.argmax(violation))
        if violation[release] <= threshold:
            return BoxLeastSquaresResult(x.copy(),active.copy(),iteration)
        active[release] = 0
    raise RuntimeError("Bounded least-squares active set did not converge")


@dataclass
class FigureEight:
    amplitude_x: float = .25
    amplitude_z: float = .10
    period_s: float = 12.
    cycles: int = 2
    start_s: float = 2.
    ramp_s: float = 2.
    center: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self):
        self.center = _vector(self.center, 3, "center").copy()
        if (not all(np.isfinite(x) for x in (self.amplitude_x,self.amplitude_z,
                 self.period_s,self.start_s,self.ramp_s)) or self.period_s <= 0
                 or self.ramp_s <= 0 or self.start_s < 0 or self.cycles < 1
                 or int(self.cycles) != self.cycles or self.ramp_s > self.cycles*self.period_s):
            raise ValueError("Invalid figure-eight timing")

    @property
    def duration_s(self):
        return self.cycles*self.period_s + self.ramp_s

    @property
    def end_s(self):
        return self.start_s + self.duration_s

    def phase(self, t):
        """Phase, angular speed, acceleration, with quintic speed ramps.

        A ramp integrates to half its duration, hence total movement time is
        cycles*period+ramp_s. The middle period is the specified period_s.
        """
        s = float(t)-self.start_s
        omega = 2*np.pi/self.period_s
        total = 2*np.pi*self.cycles
        if s <= 0:
            return 0., 0., 0.
        if s >= self.duration_s:
            return total, 0., 0.

        def ramp(u):
            integral = 2.5*u**4-3*u**5+u**6
            speed = 10*u**3-15*u**4+6*u**5
            accel = 30*u*u*(1-u)**2
            return integral, speed, accel

        if s < self.ramp_s:
            integral, speed, accel = ramp(s/self.ramp_s)
            return omega*self.ramp_s*integral, omega*speed, omega*accel/self.ramp_s
        if s > self.duration_s-self.ramp_s:
            integral, speed, accel = ramp((self.duration_s-s)/self.ramp_s)
            return total-omega*self.ramp_s*integral, omega*speed, -omega*accel/self.ramp_s
        return omega*(s-.5*self.ramp_s), omega, 0.

    def sample(self, t):
        phi, dphi, ddphi = self.phase(t)
        a, b = self.amplitude_x, self.amplitude_z
        p = self.center + [a*np.sin(phi), 0., b*np.sin(2*phi)]
        d = np.array([a*np.cos(phi), 0., 2*b*np.cos(2*phi)])
        dd = np.array([-a*np.sin(phi), 0., -4*b*np.sin(2*phi)])
        return p, d*dphi, dd*dphi*dphi+d*ddphi

    def metadata(self):
        samples = [self.sample(t) for t in np.linspace(self.start_s,self.end_s,10001)]
        return {"plane": "world XZ", "amplitude_x_m": self.amplitude_x,
            "amplitude_z_m": self.amplitude_z, "nominal_period_s": self.period_s,
            "cycles": self.cycles, "start_s": self.start_s, "end_s": self.end_s,
            "ramp_s": self.ramp_s, "movement_duration_s": self.duration_s,
            "center_world_m": self.center.tolist(),
            "sampled_max_speed_m_s": float(max(np.linalg.norm(x[1]) for x in samples)),
            "sampled_max_acceleration_m_s2": float(max(np.linalg.norm(x[2]) for x in samples))}


class WholeBodyCoordinator:
    """Five reference coordinates [base XYZ,q1,q2], same low-level controllers.

    Bounds apply to generated references. Actual joint controller bounds are
    additional and must be logged by the runner. A/B fixed references are
    analytic; C reference acceleration is the finite change of its bounded
    velocity, so no numerical derivative of noisy measured attitude is taken.
    """
    def __init__(self, p0, q0, R0, trajectory, mode="fixed", config=None):
        if mode not in ("fixed", "coordinated"):
            raise ValueError("mode must be fixed or coordinated")
        cfg = dict(config or {})
        self.p0, self.q0 = _vector(p0,3,"p0").copy(), _vector(q0,2,"q0").copy()
        self.R0 = rotation_matrix(R0).copy()
        self.trajectory, self.mode = trajectory, mode
        self.initial_tip = tip_world(self.p0,self.R0,self.q0)
        self.ref = np.r_[self.p0,self.q0]
        self.dref = np.zeros(5)
        self.aref = np.zeros(5)
        self.filtered_omega = None
        self.arm_share = float(cfg.get("arm_share",.20))
        self.velocity_limit = np.asarray(cfg.get("velocity_limit",[.55,.35,.45,.45,.35]),float)
        self.acceleration_limit = np.asarray(cfg.get("acceleration_limit",[.45,.30,.45,.60,.60]),float)
        self.jerk_limit = np.asarray(cfg.get("jerk_limit",[1.0,1.0,1.0,2.0,2.0]),float)
        self.omega_filter_tau_s = float(cfg.get("omega_filter_tau_s",.10))
        self.velocity_filter_tau_s = float(cfg.get("velocity_filter_tau_s",.25))
        self.velocity_scale = np.asarray(cfg.get("velocity_scale",[.30,.20,.25,.45,.35]),float)
        # Strictly inside SDF physical limits q1[-60,+30]deg, q2[-25,+60]deg.
        self.q_lower = np.asarray(cfg.get("q_lower",[-.75,-.35]),float)
        self.q_upper = np.asarray(cfg.get("q_upper",[.45,.85]),float)
        self.tip_gain = float(cfg.get("tip_gain",2.0))
        self.reference_pull_gain = float(cfg.get("reference_pull_gain",1.5))
        self.posture_gain = float(cfg.get("posture_gain",.80))
        self.damping = float(cfg.get("damping",.025))
        self.max_correction = float(cfg.get("max_tip_correction_m_s",.20))
        if np.any(self.q0 < self.q_lower) or np.any(self.q0 > self.q_upper):
            raise ValueError("Initial joints exceed coordinator limits")

    def metadata(self):
        return {"mode":self.mode,"solver":"numpy_bvls_active_set",
            "arm_share":self.arm_share,"velocity_limit":self.velocity_limit.tolist(),
            "acceleration_limit":self.acceleration_limit.tolist(),
            "jerk_limit":self.jerk_limit.tolist(),"velocity_scale":self.velocity_scale.tolist(),
            "q_lower":self.q_lower.tolist(),"q_upper":self.q_upper.tolist(),
            "tip_gain":self.tip_gain,"reference_pull_gain":self.reference_pull_gain,
            "posture_gain":self.posture_gain,"damping":self.damping,
            "max_tip_correction_m_s":self.max_correction,
            "omega_filter_tau_s":self.omega_filter_tau_s,
            "velocity_filter_tau_s":self.velocity_filter_tau_s,
            "posture_target":"same analytic fixed-allocation joint reference",
            "acceleration_semantics":"analytic in fixed mode; finite difference of jerk-bounded reference velocity in coordinated mode",
            "smoothing_scope":"coordinated mode only; fixed mode retains analytic trajectory derivatives",
            "braking_domain":"v2: signed jerk-aware stopping envelope and velocity viability; right-end position margin max_step*peak_speed",
            "braking_max_step_s":.05,
            "initial_base_world_m":self.p0.tolist(),"initial_joints_rad":self.q0.tolist(),
            "initial_R_world_body":self.R0.tolist()}

    def _fixed_reference(self, target, velocity, accel):
        # Only the arm's own XZ plane is assigned to joints. Base translation
        # supplies the rest, making nominal 3D FK exact even for nonzero yaw.
        delta = self.R0.T @ (target-self.initial_tip)
        delta[1] = 0.
        local_target = tip_body(self.q0)+self.arm_share*delta
        q = inverse_tip_body(local_target,near=self.q0)
        J = joint_jacobian_body(q)
        arm_v = self.arm_share*(self.R0.T@velocity)
        arm_a = self.arm_share*(self.R0.T@accel)
        dq = np.linalg.solve(J[[0,2]],arm_v[[0,2]])
        ddq = np.linalg.solve(J[[0,2]],(arm_a-joint_centripetal_body(q,dq))[[0,2]])
        if np.any(q < self.q_lower) or np.any(q > self.q_upper):
            raise ValueError("Fixed allocation violates joint position envelope")
        p = target-self.R0@tip_body(q)
        v = velocity-self.R0@(J@dq)
        a = accel-self.R0@(J@ddq+joint_centripetal_body(q,dq))
        vr, ar = np.r_[v,dq],np.r_[a,ddq]
        if np.any(abs(vr)>self.velocity_limit+1e-9) or np.any(abs(ar)>self.acceleration_limit+1e-9):
            raise ValueError("Fixed allocation violates reference speed/acceleration envelope")
        return np.r_[p,q],vr,ar

    def _fixed(self, target, velocity, accel):
        self.ref,self.dref,ar = self._fixed_reference(target,velocity,accel)
        self.aref = ar.copy()
        return ar,{"active_bounds":0,"task_velocity_residual":0.,"solver_iterations":0}

    def _coordinated(self,dt,p,R,omega,q,dq,target,velocity,accel):
        measured = np.r_[p,q]
        J = np.column_stack((np.eye(3),R@joint_jacobian_body(q)))
        nominal_ref,nominal_velocity,nominal_acceleration = self._fixed_reference(target,velocity,accel)
        error = target-tip_world(p,R,q)
        correction = self.tip_gain*error
        norm = np.linalg.norm(correction)
        if norm > self.max_correction:
            correction *= self.max_correction/norm
        pull = self.reference_pull_gain*(measured-self.ref)
        if self.filtered_omega is None:
            self.filtered_omega = omega.copy()
        else:
            weight = dt/(self.omega_filter_tau_s+dt)
            self.filtered_omega += weight*(omega-self.filtered_omega)
        rotation_velocity = np.cross(self.filtered_omega,R@tip_body(q))
        task_velocity = velocity+correction-rotation_velocity+J@pull
        # Preferred joint posture follows the SAME nominal fixed-allocation
        # trajectory as A/B. Its paired base velocity keeps J*preferred equal
        # to the nominal tip velocity: posture correction is in J's nullspace.
        preferred_qd = nominal_velocity[3:]+self.posture_gain*(nominal_ref[3:]-q)
        preferred = np.r_[velocity-rotation_velocity-J[:,3:]@preferred_qd,preferred_qd]+pull
        regularizer = self.damping*np.diag(1/self.velocity_scale)
        matrix = np.vstack((J,regularizer))
        rhs = np.r_[task_velocity,regularizer@preferred]
        # Smooth the feedback-driven velocity correction, while retaining
        # analytic nominal trajectory acceleration as the reference-filter
        # feedforward. A jerk bound alone is a rate limiter and can oscillate
        # when asked to track an abrupt desired velocity without this damping.
        velocity_weight = dt/(self.velocity_filter_tau_s+dt)
        rhs = matrix@(self.dref+dt*nominal_acceleration)+velocity_weight*(rhs-matrix@self.dref)
        # Preserve the original v/a/jerk/position limits while keeping each
        # candidate next reference state inside a conservative braking domain.
        # The old acceleration-only stopping cap could require an instantaneous
        # acceleration reversal that its simultaneous jerk bound prohibited.
        lower,upper = tighten_velocity_bounds(
            self.ref,self.dref,self.aref,dt,self.velocity_limit,
            self.acceleration_limit,self.jerk_limit,
            position_lower=np.r_[np.full(3,-np.inf),self.q_lower],
            position_upper=np.r_[np.full(3,np.inf),self.q_upper],
            max_step_s=.05)
        solved = bounded_least_squares(matrix,rhs,lower,upper)
        acceleration = (solved.x-self.dref)/dt
        jerk = (acceleration-self.aref)/dt
        self.dref = solved.x.copy()
        self.aref = acceleration.copy()
        self.ref += dt*self.dref
        return acceleration,{"active_bounds":int(np.count_nonzero(solved.active_mask)),
            "task_velocity_residual":float(np.linalg.norm(J@self.dref-task_velocity)),
            "solver_iterations":int(solved.nit),"rotation_velocity_world":rotation_velocity,
            "omega_raw_world":omega.copy(),"omega_filtered_world":self.filtered_omega.copy(),
            "nominal_q_ref":nominal_ref[3:].copy(),"reference_jerk":jerk}

    def step(self,t,dt,p,R,omega_world,q,dq):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive")
        p,q,dq = _vector(p,3,"p"),_vector(q,2,"q"),_vector(dq,2,"dq")
        R,omega = rotation_matrix(R),_vector(omega_world,3,"omega_world")
        target,velocity,accel = self.trajectory.sample(t)
        if self.mode == "fixed":
            ar,diag = self._fixed(target,velocity,accel)
        else:
            ar,diag = self._coordinated(dt,p,R,omega,q,dq,target,velocity,accel)
        return {"p_ref":self.ref[:3].copy(),"v_ref":self.dref[:3].copy(),"a_ref":ar[:3].copy(),
            "q_ref":self.ref[3:].copy(),"qd_ref":self.dref[3:].copy(),"qdd_ref":ar[3:].copy(),
            "tip_ref":target,"tip_v_ref":velocity,"tip_a_ref":accel,
            "tip_actual":tip_world(p,R,q),"diagnostics":diag}
