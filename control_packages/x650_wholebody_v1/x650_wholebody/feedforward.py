"""Relative-arm inertia acceleration FF; reaction moment is diagnostic only.

Nominal gravity uses the *complete* model mass.  This module therefore never
adds arm weight a second time.  Nonzero base-link COM offsets are supported:
all lever arms and angular transport terms refer to the declared body origin.
"""
from __future__ import annotations
import numpy as np
from .model import arm_com_kinematics, rotation_matrix, vector


def arm_reaction_feedforward(q, dq, ddq, *, model, R_world_body=None,
                             omega_body=(0., 0., 0.), alpha_body=(0., 0., 0.),
                             base_acceleration_world=(0., 0., 0.),
                             gravity_world=(0., 0., -9.80665), acceleration_limit=1.):
    q, dq, ddq = (vector(v, 2, n) for v, n in ((q, "q"), (dq, "dq"), (ddq, "ddq")))
    omega = vector(omega_body, 3, "omega_body")
    alpha = vector(alpha_body, 3, "alpha_body")
    base_acc = vector(base_acceleration_world, 3, "base_acceleration_world")
    gravity = vector(gravity_world, 3, "gravity_world")
    R = rotation_matrix(np.eye(3) if R_world_body is None else R_world_body)
    if acceleration_limit is not None and (not np.isfinite(acceleration_limit) or acceleration_limit <= 0):
        raise ValueError("acceleration_limit must be positive or None")
    position, velocity, relative_acc = arm_com_kinematics(q, dq, ddq, model)
    transported_acc = (relative_acc+2*np.cross(omega, velocity)+np.cross(alpha, position)
                       +np.cross(omega, np.cross(omega, position)))
    inertial_relative_world = transported_acc@R.T
    absolute_acc = base_acc+inertial_relative_world
    masses = np.array(model.masses)
    required_force = masses[:, None]*(absolute_acc-gravity)
    # The imported body origin need not coincide with its fixed-part COM.
    # Force balance is expressed at that same observed origin, so include the
    # fixed-body angular transport as well as the arm's relative motion.
    base_com = np.array(model.base_com_body)
    fixed_transport_body = np.cross(alpha, base_com)+np.cross(omega, np.cross(omega, base_com))
    fixed_transport_force = model.fixed_mass*(R@fixed_transport_body)
    arm_relative_force = masses@inertial_relative_world
    relative_force = arm_relative_force+fixed_transport_force
    raw = relative_force/model.total_mass
    norm = np.linalg.norm(raw)
    scale = 1. if acceleration_limit is None or norm <= acceleration_limit else acceleration_limit/norm
    moment = np.cross(position@R.T, required_force).sum(axis=0)
    theta = np.array(model.angle_offsets)+[q[0], q.sum()]
    axis = np.array([0., 1., 0.])
    for i, (rate, accel) in enumerate(zip((dq[0], dq.sum()), (ddq[0], ddq.sum()))):
        c, s = np.cos(theta[i]), np.sin(theta[i])
        link_R = R@np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
        inertia_world = link_R@np.diag(model.inertia_diagonal[i])@link_R.T
        link_omega = R@(omega+axis*rate)
        link_alpha = R@(alpha+axis*accel+np.cross(omega, axis*rate))
        moment += inertia_world@link_alpha+np.cross(link_omega, inertia_world@link_omega)
    body_moment = R.T@moment
    return {"acceleration_world": raw*scale, "raw_acceleration_world": raw,
            "relative_force_world": relative_force,
            "arm_relative_force_world": arm_relative_force,
            "fixed_body_origin_transport_force_world": fixed_transport_force,
            "predicted_reaction_force_world": -required_force.sum(axis=0),
            "predicted_reaction_moment_body": -body_moment,
            "required_arm_moment_body": body_moment, "saturated": bool(scale < 1.),
            "saturation_scale": float(scale), "total_mass_kg": model.total_mass,
            "arm_com_position_body": position, "arm_com_acceleration_world": absolute_acc,
            "moment_origin": "base_link body origin; moment diagnostic only",
            "profile_name": model.profile_name, "source_report_sha256": model.source_report_sha256}
