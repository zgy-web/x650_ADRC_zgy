"""Measured-state 3D kinematics for the Gazebo x500_underfold two-link arm.

World coordinates are ENU and body coordinates are FLU. Both revolute axes
are +body Y: positive angle rotates +X toward -Z. The distal endpoint is
arm_link_2 local [0.25, 0, 0], not the link origin or centre of mass.
"""
from __future__ import annotations

import numpy as np

MOUNT = np.array([0.0, 0.0, -0.10])
L1, L2 = 0.30, 0.25
FOLD = np.deg2rad(150.0)


def _vector(value, size, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f"{name} must be a finite {size}-vector")
    return a


def quaternion_matrix(wxyz):
    """Normalized active body-to-world rotation, quaternion order w,x,y,z."""
    q = _vector(wxyz, 4, "wxyz")
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError("Zero quaternion")
    w, x, y, z = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def rotation_matrix(value):
    a = np.asarray(value, dtype=float)
    if a.shape == (4,):
        return quaternion_matrix(a)
    if a.shape != (3, 3) or not np.isfinite(a).all():
        raise ValueError("Rotation must be 3x3 or a wxyz quaternion")
    return a


def endpoint_positions_body(q):
    q1, q2 = _vector(q, 2, "q")
    a1, a2 = FOLD + q1, q1 + q2
    elbow = MOUNT + L1 * np.array([np.cos(a1), 0., -np.sin(a1)])
    tip = elbow + L2 * np.array([np.cos(a2), 0., -np.sin(a2)])
    return np.array([MOUNT, elbow, tip])


def tip_body(q):
    return endpoint_positions_body(q)[-1]


def tip_world(p, R, q):
    return _vector(p, 3, "p") + rotation_matrix(R) @ tip_body(q)


def joint_jacobian_body(q):
    q1, q2 = _vector(q, 2, "q")
    a1, a2 = FOLD + q1, q1 + q2
    j1 = L1 * np.array([-np.sin(a1), 0., -np.cos(a1)])
    j2 = L2 * np.array([-np.sin(a2), 0., -np.cos(a2)])
    return np.column_stack((j1+j2, j2))


def joint_centripetal_body(q, dq):
    """Jdot(q,dq)*dq, excluding base angular motion."""
    q1, q2 = _vector(q, 2, "q")
    d1, d2 = _vector(dq, 2, "dq")
    a1, a2 = FOLD + q1, q1 + q2
    return (-L1*np.array([np.cos(a1), 0., -np.sin(a1)])*d1*d1
            -L2*np.array([np.cos(a2), 0., -np.sin(a2)])*(d1+d2)**2)


def inverse_tip_body(target, near=None):
    """Negative-elbow IK branch continuous around folded q=[0,0]."""
    r = _vector(target, 3, "target") - MOUNT
    if abs(r[1]) > 1e-8:
        raise ValueError("The arm endpoint must lie in the body XZ plane")
    x, y = r[0], -r[2]
    c = (x*x+y*y-L1*L1-L2*L2)/(2*L1*L2)
    if abs(c) > 1+1e-10:
        raise ValueError("Prescribed arm target is unreachable")
    elbow = -np.arccos(np.clip(c, -1., 1.))
    theta = np.arctan2(y, x) - np.arctan2(L2*np.sin(elbow), L1+L2*np.cos(elbow))
    q = np.array([theta-FOLD, elbow+FOLD])
    reference = np.zeros(2) if near is None else _vector(near, 2, "near")
    return reference + (q-reference+np.pi) % (2*np.pi) - np.pi
