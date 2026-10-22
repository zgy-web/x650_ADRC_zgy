"""Optional timing/attitude output extension; no new base-response identification.

Joint delay is relative to the logged pre-solve consumption time. It represents
the observed command pipeline, not motor inertia or an identified dispatch time.
"""
from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation
from .model import vector, rotation_matrix

KIND = "delayed_joint_thrust_attitude_v1"
KEYS = {"kind", "joint_delay_s", "joint_delay_guard_s", "gravity_m_s2", "attitude_residual_decay_s"}


def normalize(value):
    if not isinstance(value, dict) or set(value) != KEYS or value.get("kind") != KIND:
        raise ValueError("Unknown prediction_chain structure")
    result = {"kind": KIND}
    for name in KEYS-{"kind"}:
        x = value[name]
        if isinstance(x, (bool, np.bool_)) or not np.isscalar(x) or not np.isfinite(x):
            raise ValueError("Prediction-chain settings must be finite numbers")
        result[name] = float(x)
    if not (0 < result["joint_delay_s"] <= result["joint_delay_guard_s"] <= .25
            and result["gravity_m_s2"] > 0 and result["attitude_residual_decay_s"] > 0):
        raise ValueError("Invalid prediction-chain delay, guard or gravity")
    return result


def skew_batch(v):
    out = np.zeros((*v.shape[:-1], 3, 3))
    out[..., 0, 1], out[..., 0, 2] = -v[..., 2], v[..., 1]
    out[..., 1, 0], out[..., 1, 2] = v[..., 2], -v[..., 0]
    out[..., 2, 0], out[..., 2, 1] = -v[..., 1], v[..., 0]
    return out


def thrust_rotation(acceleration, jerk, yaw, yaw_rate, gravity):
    """Exact normalized thrust direction and its analytic time derivative."""
    force = np.asarray(acceleration, float)+[0., 0., gravity]
    norm = np.linalg.norm(force, axis=1)
    if np.any(norm < gravity*.25) or np.any(force[:, 2] <= gravity*.25):
        raise ValueError("Thrust-direction prediction outside upright flight domain")
    z = force/norm[:, None]
    zd = (jerk-z*np.sum(z*jerk, axis=1)[:, None])/norm[:, None]
    xc = np.column_stack((np.cos(yaw), np.sin(yaw), np.zeros(len(yaw))))
    xcd = np.column_stack((-np.sin(yaw), np.cos(yaw), np.zeros(len(yaw))))*yaw_rate[:, None]
    cross = np.cross(z, xc)
    cross_norm = np.linalg.norm(cross, axis=1)
    if np.any(cross_norm < .25):
        raise ValueError("Thrust direction is singular against heading")
    y = cross/cross_norm[:, None]
    cd = np.cross(zd, xc)+np.cross(z, xcd)
    yd = (cd-y*np.sum(y*cd, axis=1)[:, None])/cross_norm[:, None]
    x, xd = np.cross(y, z), np.cross(yd, z)+np.cross(y, zd)
    return np.stack((x, y, z), axis=2), np.stack((xd, yd, zd), axis=2)


def attitude_context(state, times, config):
    R = rotation_matrix(state.rotation)
    acceleration = vector(state.acceleration_enu, 3, "measured acceleration")
    yaw = np.arctan2(R[1, 0], R[0, 0])
    roll, pitch = np.arctan2(R[2, 1], R[2, 2]), np.arcsin(np.clip(-R[2, 0], -1., 1.))
    omega = vector(state.omega_body, 3, "measured omega")
    yaw_rate = (np.sin(roll)*omega[1]+np.cos(roll)*omega[2])/np.cos(pitch)
    tau = config["attitude_residual_decay_s"]
    decay = np.exp(-np.asarray(times)/tau)
    nominal, _ = thrust_rotation(acceleration[None], np.zeros((1, 3)), np.array([yaw]), np.zeros(1), config["gravity_m_s2"])
    residual = Rotation.from_matrix(nominal[0].T@R).as_rotvec()
    residual_R = Rotation.from_rotvec(decay[:, None]*residual).as_matrix()
    residual_Rdot = residual_R@skew_batch(-decay[:, None]*residual/tau)
    return {"yaw": yaw+tau*(1-decay)*yaw_rate, "yaw_rate": decay*yaw_rate,
            "residual_R": residual_R, "residual_Rdot": residual_Rdot,
            "initial_residual_rotvec": residual, "gravity": config["gravity_m_s2"]}


def _geometry(q, a, dq, jerk, context, model):
    nominal, nominal_dot = thrust_rotation(a, jerk, context["yaw"], context["yaw_rate"], context["gravity"])
    R = nominal@context["residual_R"]
    Rd = nominal_dot@context["residual_R"]+nominal@context["residual_Rdot"]
    angle = np.array(model.angle_offsets)+np.column_stack((q[:, 0], q.sum(axis=1)))
    direction = np.stack((np.cos(angle), np.zeros_like(angle), -np.sin(angle)), axis=2)
    tangent = np.stack((-np.sin(angle), np.zeros_like(angle), -np.cos(angle)), axis=2)
    l1, l2 = model.lengths
    r = np.array(model.mount_body)+np.array(model.elbow_offset_body_m)+l1*direction[:, 0]+l2*direction[:, 1]
    J = np.stack((l1*tangent[:, 0]+l2*tangent[:, 1], l2*tangent[:, 1]), axis=2)
    RJ = R@J
    rate = np.einsum("nij,nj->ni", RJ, dq)+np.einsum("nij,nj->ni", Rd, r)
    position = np.einsum("nij,nj->ni", R, r)
    curvature = np.stack((-l1*direction[:, 0]*dq[:, :1]-l2*direction[:, 1]*dq.sum(axis=1)[:, None],
                          -l2*direction[:, 1]*dq.sum(axis=1)[:, None]), axis=2)
    return position, rate, R, RJ, R@curvature+Rd@J


def endpoint_outputs(states, commands, context, model, response, *, jacobians=False):
    """State/input output Jacobians; local FD only over three accelerations.

Everything else (q/dq, v, jerk, input feedthrough) uses analytic derivatives.
This never perturbs the 75 QP decision variables or finite-differences velocity.
"""
    states, commands = np.asarray(states), np.asarray(commands)
    n = len(states)
    tau = response["z"]["tau_s"]
    a = np.column_stack((states[:, 10:12], (commands[:, 2]-states[:, 5])/tau))
    jerk = np.empty_like(a)
    ax, au, jx, ju = (np.zeros((n, 3, size)) for size in (12, 5, 12, 5))
    for i, name in enumerate(("x", "y")):
        wn, zeta = response[name]["wn_rad_s"], response[name]["zeta"]
        jerk[:, i] = wn*wn*(commands[:, i]-states[:, i+3])-2*zeta*wn*a[:, i]
        ax[:, i, 10+i] = 1.
        jx[:, i, i+3], jx[:, i, i+10], ju[:, i, i] = -wn*wn, -2*zeta*wn, wn*wn
    jerk[:, 2] = -a[:, 2]/tau
    ax[:, 2, 5], au[:, 2, 2], jx[:, 2, 5], ju[:, 2, 2] = -1/tau, 1/tau, 1/tau**2, -1/tau**2
    q, dq = states[:, 6:8], states[:, 8:10]
    arm, rate, R, RJ, rate_q = _geometry(q, a, dq, jerk, context, model)
    out = {"position": states[:, :3]+arm, "velocity": states[:, 3:6]+rate, "rotations": R,
           "acceleration": a}
    if not jacobians:
        return out
    pa, va = np.empty((n, 3, 3)), np.empty((n, 3, 3))
    epsilon = 1e-5
    for axis in range(3):
        delta = np.zeros_like(a)
        delta[:, axis] = epsilon
        plus = _geometry(q, a+delta, dq, jerk, context, model)
        minus = _geometry(q, a-delta, dq, jerk, context, model)
        pa[:, :, axis] = (plus[0]-minus[0])/(2*epsilon)
        va[:, :, axis] = (plus[1]-minus[1])/(2*epsilon)
    px, pu, vx, vu = pa@ax, pa@au, va@ax+pa@jx, va@au+pa@ju
    px[:, :, :3] += np.eye(3)
    px[:, :, 6:8] += RJ
    vx[:, :, 3:6] += np.eye(3)
    vx[:, :, 6:8] += rate_q
    vx[:, :, 8:10] += RJ
    out.update(px=px, pu=pu, vx=vx, vu=vu)
    return out


def joint_prediction(t, node_times, intervals, state, history, config, velocity_limits):
    """Affine exact delayed velocity execution, preserving measured initial dq."""
    now, stamp = state.control_time_s, state.joint_stamp_s
    if now is None or stamp is None or not np.isfinite([now, stamp]).all() or not 0 <= now-stamp <= .25+1e-9:
        raise ValueError("Prediction chain requires explicit current control/joint source times")
    age, delay = float(now-stamp), config["joint_delay_s"]
    start = t-age
    n = len(node_times)*5
    q, dq = np.asarray(state.q).copy(), np.asarray(state.dq).copy()
    Q, D = np.zeros((2, n)), np.zeros((2, n))
    events = [(when+delay, np.asarray(command), None) for when, command in history if when+delay > start+1e-10]
    for i, relative in enumerate(np.r_[0., np.cumsum(intervals)[:-1]]):
        matrix = np.zeros((2, n))
        matrix[:, i*5+3:i*5+5] = np.diag(velocity_limits)
        events.append((t+relative+delay, np.zeros(2), matrix))
    events.sort(key=lambda item: item[0])
    # All command switch instants and nodes matter for piecewise-linear q.
    queries = sorted(set([t, *[t+x for x in node_times], *[x[0] for x in events if t <= x[0] <= t+node_times[-1]]]))
    at, event_index, records = start, 0, []
    for end in queries:
        while event_index < len(events) and events[event_index][0] <= end+1e-10:
            when, value, matrix = events[event_index]
            if when < at-1e-9:
                raise ValueError("Command history regressed")
            step = max(0., when-at)
            q, Q = q+step*dq, Q+step*D
            dq, D = value.copy(), np.zeros_like(D) if matrix is None else matrix.copy()
            at = when
            event_index += 1
        q, Q = q+(end-at)*dq, Q+(end-at)*D
        at = end
        records.append((end-t, q.copy(), dq.copy(), Q.copy(), D.copy()))
    def at_time(relative):
        return next(record for record in records if abs(record[0]-relative) <= 1e-9)
    initial = at_time(0.)
    return {"nodes": [at_time(x) for x in node_times], "events": records,
            "q_current": initial[1], "dq_current": initial[2], "measurement_age_s": age}


def conservative_first_bounds(controller, dt, state, prediction, history, t, config):
    """Tighten the existing command bounds using pending-motion envelopes."""
    age = prediction["measurement_age_s"]
    recent = [u for when, u in history if when >= t-config["joint_delay_guard_s"]-age-controller.max_dt]
    slew = controller.acceleration_limit[3:]*dt
    velocities = np.asarray([state.dq, prediction["dq_current"], controller.previous_command[3:]-slew,
                             controller.previous_command[3:]+slew, *recent])
    guard = config["joint_delay_guard_s"]+age
    qlo = prediction["q_current"]+guard*np.minimum(velocities.min(axis=0), 0.)
    qhi = prediction["q_current"]+guard*np.maximum(velocities.max(axis=0), 0.)
    lo, _ = controller._bounds(dt, qlo)
    _, hi = controller._bounds(dt, qhi)
    if np.any(lo > hi+1e-10):
        raise ValueError("Pending joint motion cannot satisfy unchanged stopping/slew bounds")
    return lo, hi, {"q_guard_lower": qlo, "q_guard_upper": qhi, "guard_interval_s": guard}
