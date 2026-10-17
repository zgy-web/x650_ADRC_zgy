"""Explicit ENU truth-feedback adapter for PX4's NED position/velocity PID."""
import numpy as np

# Enable position, velocity, acceleration, yaw; ignore only yaw rate. Not force mode.
POSITION_TARGET_MASK = 2048


def quaternion_rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid wxyz quaternion')
    w, x, y, z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def enu_to_ned(v):
    x, y, z = np.asarray(v, dtype=float)
    return np.array([y, x, -z])


def bridge(px4_p, px4_v, p_ref, v_ref, a_ref, p_true, v_true):
    """Preserve exact world feedback errors inside PX4; acceleration is feedforward."""
    arrays = [np.asarray(v, dtype=float) for v in
              (px4_p, px4_v, p_ref, v_ref, a_ref, p_true, v_true)]
    if not all(a.shape == (3,) and np.isfinite(a).all() for a in arrays):
        raise ValueError('Invalid controller vector')
    ep, ev, pr, vr, ar, pt, vt = arrays
    return ep+enu_to_ned(pr-pt), ev+enu_to_ned(vr-vt), enu_to_ned(ar)


def clip_norm(v, maximum):
    v = np.asarray(v, dtype=float)
    norm = np.linalg.norm(v)
    return v * min(1., maximum/max(norm, 1e-12)), bool(norm > maximum)
