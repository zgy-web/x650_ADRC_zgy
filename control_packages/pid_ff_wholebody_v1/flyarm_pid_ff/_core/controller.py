"""Servo-consistent, constrained kinematic whole-body endpoint controller.

This is an engineering weighted velocity controller, not dynamic MPC.  Aircraft
attitude is measured, not an independently commandable decision coordinate.
"""
from __future__ import annotations

from pathlib import Path
import numpy as np

MATH = Path(__file__).resolve().parent
from .trajectory import WholeBodyCoordinator, bounded_least_squares
from .kinematics import tip_body, tip_world, joint_jacobian_body, rotation_matrix, _vector


class WholeBodyClosedLoop:
    def __init__(self, p0, q0, R0, trajectory, mode="compensated", config=None):
        if mode not in ("baseline", "compensated"):
            raise ValueError("mode must be baseline or compensated")
        c = dict(config or {})
        self.mode, self.trajectory = mode, trajectory
        self.p0, self.q0 = _vector(p0,3,"p0").copy(), _vector(q0,2,"q0").copy()
        self.R0 = rotation_matrix(R0).copy()
        self.arm_share = float(c.get("arm_share", .2))
        if not 0 <= self.arm_share <= 1:
            raise ValueError("arm_share outside [0,1]")
        self.nominal = WholeBodyCoordinator(p0,q0,R0,trajectory,mode="fixed",config={"arm_share":self.arm_share})
        self.velocity_limit = np.array([.55,.35,.45,.45,.35])
        self.acceleration_limit = np.array([.45,.30,.45,.60,.60])
        self.q_lower, self.q_upper = np.array([-.75,-.35]),np.array([.45,.85])
        self.base_position_gain = np.array([2.,2.,1.])
        self.joint_position_gain = 3.
        self.tip_gain = float(c.get("tip_gain", 2.))
        self.max_tip_correction = float(c.get("max_tip_correction_m_s", .20))
        self.task_weight = float(c.get("task_weight", 1.))
        self.measured_base_weight = float(c.get("measured_base_weight", 4.))
        self.base_nominal_weight = float(c.get("base_nominal_weight", 4.))
        self.arm_regularizer_length = float(c.get("arm_regularizer_length_m_rad", .02))
        self.max_dt = .05
        if not all(np.isfinite(x) and x>0 for x in (self.tip_gain,self.max_tip_correction,self.task_weight,
                self.measured_base_weight,self.base_nominal_weight,self.arm_regularizer_length)):
            raise ValueError("All gains and weights must be finite and positive")
        self.previous_command = np.zeros(5)
        self.step_count = 0

    def metadata(self):
        return {"name":"servo_consistent_kinematic_whole_body_v1","mode":self.mode,
            "decision_variables":"desired executed base XYZ velocity [m/s], desired executed q1/q2 velocity [rad/s]",
            "objective":"||ub+Jw*uj-r||^2 + 4||v_measured+Jw*uj-r||^2 + 4||ub-ub_nom||^2 + ||0.02*(uj-uj_nom)||^2",
            "r":"tip_v_target + clipped(2*(tip_target-tip_actual)) - omega_world cross (R*tip_body(q))",
            "objective_weights":{"whole_body_task":self.task_weight,"current_measured_base_task":self.measured_base_weight,
                "base_nominal_restoration":self.base_nominal_weight,"arm_regularizer_length_m_per_rad":self.arm_regularizer_length},
            "weights_status":"Engineering choices; no optimum or closed-loop stability proof claimed",
            "nominal_arm_share":self.arm_share,"base_position_gain_s_inv":self.base_position_gain.tolist(),
            "joint_position_gain_s_inv":self.joint_position_gain,"tip_gain_s_inv":self.tip_gain,
            "max_tip_correction_m_s":self.max_tip_correction,
            "base_setpoint_encoding":"p_ref=p_nom; v_ref=ub - Kpos*(p_nom-p_measured); PX4 outer position P restores exactly ub",
            "joint_setpoint_encoding":"send qd_cmd=uj directly; q_ref=q_nom, qd_ref=uj-3*(q_nom-q) are diagnostic servo-equivalent fields",
            "aircraft_acceleration_feedforward":"Only analytic nominal base acceleration, never differentiated feedback command",
            "arm_qdd_for_force_feedforward":"Bounded difference of joint velocity COMMANDS, not derivative of noisy measured qd",
            "velocity_limit":self.velocity_limit.tolist(),"acceleration_limit":self.acceleration_limit.tolist(),
            "q_lower":self.q_lower.tolist(),"q_upper":self.q_upper.tolist(),"max_dt_s":self.max_dt,
            "position_bound_semantics":"Measured joint position; conservative discrete stopping-distance envelope for velocity command",
            "acceleration_bound_semantics":"Per-step commanded generalized velocity difference; not guarantee of actual physical acceleration",
            "jerk_constraint":"Not applied; old constraint was on an integrated reference, not actual executed servo velocity",
            "joint_position_limit_scope":"Command viability for measured-state servo; cannot guarantee against external impacts or model/servo tracking error",
            "rank_scope":"3D endpoint position; 2-DOF arm spans body XZ only, aircraft supplies uncontrollable residual directions",
            "feedforward_scope":"Same external arm-force feedforward model in both baseline and compensated modes",
            "math_source_directory":str(MATH)}

    def _bounds(self, dt, q):
        if np.any(q < self.q_lower-1e-8) or np.any(q > self.q_upper+1e-8):
            raise ValueError("Measured joints outside soft position envelope")
        lower = np.maximum(-self.velocity_limit,self.previous_command-self.acceleration_limit*dt)
        upper = np.minimum(self.velocity_limit,self.previous_command+self.acceleration_limit*dt)
        # For outward command u, reserve hmax*u + u^2/(2*a) before the
        # position boundary.  hmax covers one full command hold interval.
        a = self.acceleration_limit[3:]
        margin_lower, margin_upper = np.maximum(q-self.q_lower,0.), np.maximum(self.q_upper-q,0.)
        slow_lower = np.sqrt((a*self.max_dt)**2+2*a*margin_lower)-a*self.max_dt
        slow_upper = np.sqrt((a*self.max_dt)**2+2*a*margin_upper)-a*self.max_dt
        lower[3:] = np.maximum(lower[3:],-slow_lower)
        upper[3:] = np.minimum(upper[3:],slow_upper)
        if np.any(lower > upper+1e-10):
            raise ValueError("Measured joint/previous command state cannot satisfy acceleration and stopping bounds simultaneously")
        # Remove floating-point-only reversed singleton intervals.
        mid = .5*(lower+upper)
        bad = lower>upper
        lower[bad],upper[bad] = mid[bad],mid[bad]
        return lower,upper

    def step(self,t,dt,p,R,v,omega_world,q,dq):
        if not np.isfinite(t) or not np.isfinite(dt) or not 0 < dt <= self.max_dt+1e-10:
            raise ValueError("Require finite time and 0 < dt <= 0.05 s")
        p,v,q,dq = _vector(p,3,"p"),_vector(v,3,"v"),_vector(q,2,"q"),_vector(dq,2,"dq")
        omega = _vector(omega_world,3,"omega_world")
        R = rotation_matrix(R)
        if not np.allclose(R.T@R,np.eye(3),atol=1e-6) or np.linalg.det(R)<0:
            raise ValueError("Invalid body-to-world rotation")
        target,vd,ad = self.trajectory.sample(t)
        nominal_ref,nominal_v,nominal_a = self.nominal._fixed_reference(target,vd,ad)
        p_nom,q_nom = nominal_ref[:3],nominal_ref[3:]
        base_feedback = self.base_position_gain*(p_nom-p)
        joint_feedback = self.joint_position_gain*(q_nom-q)
        u_nom = nominal_v+np.r_[base_feedback,joint_feedback]
        tip = tip_world(p,R,q)
        e = target-tip
        correction = self.tip_gain*e
        correction *= min(1.,self.max_tip_correction/max(float(np.linalg.norm(correction)),1e-12))
        J = R@joint_jacobian_body(q)
        rotational_velocity = np.cross(omega,R@tip_body(q))
        desired_tip_velocity = vd+correction
        r = desired_tip_velocity-rotational_velocity
        lower,upper = self._bounds(dt,q)
        if self.mode == "baseline":
            command = np.clip(u_nom,lower,upper)
            iterations = 0
        else:
            w1,w2,w3 = np.sqrt([self.task_weight,self.measured_base_weight,self.base_nominal_weight])
            A = np.vstack((w1*np.column_stack((np.eye(3),J)),
                           w2*np.column_stack((np.zeros((3,3)),J)),
                           w3*np.column_stack((np.eye(3),np.zeros((3,2)))),
                           np.column_stack((np.zeros((2,3)),self.arm_regularizer_length*np.eye(2)))))
            b = np.r_[w1*r,w2*(r-v),w3*u_nom[:3],self.arm_regularizer_length*u_nom[3:]]
            solved = bounded_least_squares(A,b,lower,upper)
            command,iterations = solved.x,solved.nit
        command_acceleration = (command-self.previous_command)/dt
        if np.any(np.abs(command)>self.velocity_limit+1e-8) or np.any(np.abs(command_acceleration)>self.acceleration_limit+1e-8):
            raise AssertionError("Command bounds failed")
        self.previous_command = command.copy()
        self.step_count += 1
        active = np.isclose(command,lower,atol=1e-9)|np.isclose(command,upper,atol=1e-9)
        instantaneous_error = v+J@command[3:]+rotational_velocity-desired_tip_velocity
        planned_error = command[:3]+J@command[3:]+rotational_velocity-desired_tip_velocity
        actual_velocity = v+J@dq+rotational_velocity
        projector = J@np.linalg.pinv(J,rcond=1e-8)
        diagnostics = {"active_bounds":int(active.sum()),"solver_iterations":int(iterations),
            "task_velocity_residual":float(np.linalg.norm(instantaneous_error)),
            "instantaneous_task_velocity_residual_world":instantaneous_error,
            "planned_task_velocity_residual_world":planned_error,
            "desired_tip_velocity_world":desired_tip_velocity,"actual_tip_velocity_world":actual_velocity,
            "measured_base_velocity_world":v.copy(),"base_executed_velocity_target_world":command[:3].copy(),
            "nominal_executed_velocity_target":u_nom.copy(),"nominal_q_ref":q_nom.copy(),
            "nominal_qd_ref":nominal_v[3:].copy(),"nominal_qdd_ref":nominal_a[3:].copy(),
            "rotation_velocity_world":rotational_velocity,"omega_raw_world":omega.copy(),"omega_filtered_world":omega.copy(),
            "tip_error_world":e,"arm_reachable_task_velocity_world":projector@(r-v),
            "arm_unreachable_task_velocity_world":(np.eye(3)-projector)@(r-v),
            "command_acceleration":command_acceleration,"command_lower":lower,"command_upper":upper,
            "reference_jerk":np.zeros(5),"reference_jerk_valid":False,
            "servo_reconstructed_base_velocity":base_feedback+(command[:3]-base_feedback),
            "servo_reconstructed_joint_velocity":joint_feedback+(command[3:]-joint_feedback)}
        return {"p_ref":p_nom.copy(),"v_ref":command[:3]-base_feedback,"a_ref":nominal_a[:3].copy(),
            "q_ref":q_nom.copy(),"qd_ref":command[3:]-joint_feedback,"qdd_ref":command_acceleration[3:].copy(),
            "qd_cmd":command[3:].copy(),"base_velocity_cmd":command[:3].copy(),
            "tip_ref":target,"tip_v_ref":vd,"tip_a_ref":ad,"tip_actual":tip,"diagnostics":diagnostics}
