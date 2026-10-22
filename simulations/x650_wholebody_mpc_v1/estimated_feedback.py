"""PX4-estimated upper feedback in the bound raw-EV world frame.

This is an opt-in, fail-closed adapter. Gazebo truth is never a state or clock
source here. The inputs are the latest asynchronous callbacks held by ApiNode,
with separate /clock receipt ages. PX4 DDS source timestamps are preserved for
audit, not treated as a shared simulation clock or synchronized sample times.
The angular callback receipt time owns ControlState.stamp_s so a held angular
sample cannot be differentiated a second time just because another topic moves.
"""
from __future__ import annotations

from copy import deepcopy
import math
import time
import numpy as np

if __package__:
    from .hover_reference import PINNED_PX4, position_snapshot
else:  # run_case also supports direct script loading via its scenario path.
    from hover_reference import PINNED_PX4, position_snapshot


NED_TO_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_TO_FLU = np.diag([1., -1., -1.])
JOINTS = ("arm_joint_1", "arm_joint_2")
TOPICS = ("position", "attitude", "angular_velocity", "estimator_flags", "joints")
EV_FLAGS = {"cs_ev_pos": True, "cs_ev_vel": True, "cs_ev_hgt": True,
            "cs_gnss_pos": False, "cs_gnss_vel": False,
            "cs_gps_hgt": False, "cs_baro_hgt": False}
EXTRA_RESETS = ("vxy_reset_counter", "vz_reset_counter", "quat_reset_counter")
ACCELERATION_PROFILE = "px4_estimated_acceleration_raw_ev_world_v2"
ACCELERATION_CONTRACT = "px4_estimator_acceleration_receipt_enu_v2"


def _finite(value, name):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(name+" must be a finite number")
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(name+" must be a finite number") from exc
    if not math.isfinite(value):
        raise ValueError(name+" must be finite")
    return value


def _integer(value, name, *, maximum=None, minimum=0):
    result = _finite(value, name)
    if not result.is_integer() or result < minimum or (maximum is not None and result > maximum):
        raise ValueError("Invalid "+name)
    return int(result)


def _field(message, name):
    if not hasattr(message, name):
        raise RuntimeError("Estimator feedback lacks field "+name)
    return getattr(message, name)


def _boolean(message, name):
    value = _field(message, name)
    if not isinstance(value, (bool, np.bool_)):
        raise ValueError(name+" must be a boolean")
    return bool(value)


def _vector(value, n, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (n,) or not np.isfinite(array).all():
        raise ValueError(name+" must be a finite "+str(n)+"-vector")
    return array.copy()


def _reference_document(reference):
    document = deepcopy(reference if isinstance(reference, dict) else reference.document)
    if document.get("frame_mode") != "raw_ev_world":
        raise ValueError("Estimated feedback requires a bound raw_ev_world reference")
    if document.get("px4_source_contract_commit") != PINNED_PX4:
        raise ValueError("Estimated feedback requires the pinned PX4 frame contract")
    if document.get("target_frozen") is not True or document.get("fit_estimator_translation") is not False:
        raise ValueError("Estimated feedback requires a frozen, unfitted reference")
    bound = document.get("px4_reference_at_bind", {})
    for key in ("ref_lat", "ref_lon", "ref_alt", "ref_timestamp"):
        _finite(bound.get(key), "bound "+key)
    if not bound.get("z_global") or bound["ref_timestamp"] <= 0:
        raise ValueError("raw_ev_world requires an established altitude origin")
    for key in ("xy_reset_counter", "z_reset_counter", "heading_reset_counter"):
        _integer(bound.get(key), "bound "+key, maximum=255)
    return document


def encode_world_command(command, frozen_reference):
    """Encode world p/vFF/a directly, without reading any current estimator.

    With this frame transform, the same logged estimated-state snapshot allows
    reconstruction of PX4's requested velocity K*(p_ref-p_est)+vFF. Actual PX4
    execution still occurs asynchronously and is verified separately in ULog.
    """
    from x650_wholebody.api import Px4Setpoint
    document = _reference_document(frozen_reference)
    position = NED_TO_ENU @ _vector(command.position_reference, 3, "position_reference")
    position[2] += document["px4_reference_at_bind"]["ref_alt"]
    return Px4Setpoint(position,
        NED_TO_ENU @ _vector(command.velocity_feedforward, 3, "velocity_feedforward"),
        NED_TO_ENU @ _vector(command.acceleration_feedforward, 3, "acceleration_feedforward"))


class Px4EstimatedFeedback:
    """Latest held PX4 pose/rates plus measured joints, with frozen frame guards.

    Construct after reference binding and before issuing upper-control commands.
    Passing node freezes velocity/quaternion reset counters immediately; without
    it they freeze at the first successful read. A failed read produces no state.
    All read calls must run on ApiNode's single-threaded executor's caller thread.
    """
    def __init__(self, reference, node=None, *, max_state_hold_s=.10,
                 max_flags_hold_s=1.25, max_joint_hold_s=.25, max_wall_age_s=4.,
                 max_flags_wall_age_s=10., require_acceleration=False, require_prediction_timing=False):
        # Pinned PX4 EKF2.cpp::PublishStatusFlags publishes every 1_s or
        # immediately on status change. DDS rate_limit=5 Hz is a ceiling,
        # not a 5 Hz source guarantee. 1.25 s = 1 s source period + 0.2 s
        # DDS rate-limit interval + 0.05 s check margin. The separate 10 s
        # wall bound permits slow SITL while all faster streams retain 4 s.
        self._document = _reference_document(reference)
        if type(require_acceleration) is not bool:
            raise ValueError("require_acceleration must be a boolean")
        self._require_acceleration = require_acceleration
        if type(require_prediction_timing) is not bool or (require_prediction_timing and not require_acceleration):
            raise ValueError("Prediction timing requires acceleration-enabled feedback")
        self._require_prediction_timing = require_prediction_timing
        self._limits = {key: _finite(value, key) for key, value in (
            ("state", max_state_hold_s), ("flags", max_flags_hold_s),
            ("joint", max_joint_hold_s), ("wall", max_wall_age_s),
            ("flags_wall", max_flags_wall_age_s))}
        if any(value <= 0 for value in self._limits.values()):
            raise ValueError("Feedback age limits must be positive")
        self._extra_resets = self._reset_snapshot(node.data) if node is not None else None
        self._last_callbacks = {}
        self._last_sim_s = None
        self._last_joint_stamp = None

    @property
    def document(self):
        result = {"kind": "x650_px4_estimated_feedback", "schema_version": 1,
            "profile": "px4_estimated_raw_ev_world_v1", "frame_mode": "raw_ev_world",
            "state_source": "PX4 vehicle_local_position/vehicle_attitude/vehicle_angular_velocity and measured JointState",
            "clock_source": "ROS /clock receipt time; never Gazebo odometry time",
            "snapshot_protocol": "latest asynchronous held callbacks; receipt-age guarded, not sample-time synchronized",
            "angular_derivative_stamp": "angular_velocity callback received_sim; same receipt time merges samples",
            "raw_px4_timestamps": "diagnostic only; DDS epochs are not simulation timestamps",
            "truth_feedback": False, "truth_clock": False,
            "reference": deepcopy(self._document), "max_age_s": dict(self._limits),
            "extra_reset_counters_at_bind": deepcopy(self._extra_resets)}
        if self._require_acceleration:
            result.update(schema_version=2, profile=ACCELERATION_PROFILE,
                feedback_contract=ACCELERATION_CONTRACT, acceleration_required=True,
                acceleration_source="PX4 vehicle_local_position ax/ay/az: NED earth-fixed velocity derivative, mapped directly to ENU",
                acceleration_stamp="position callback received_sim; independent of angular derivative stamp",
                acceleration_fallback=False, acceleration_numerical_differentiation=False)
        if self._require_prediction_timing:
            result["prediction_timing"] = "control_time_s=/clock consumption; joint_stamp_s=actual JointState header"
        return result

    @staticmethod
    def _reset_snapshot(data):
        if "position" not in data or "attitude" not in data:
            raise RuntimeError("Cannot freeze estimator resets without position and attitude")
        return {name: _integer(_field(data["attitude" if name == "quat_reset_counter" else "position"], name),
                              name, maximum=255) for name in EXTRA_RESETS}

    def _check_position(self, message):
        snapshot = position_snapshot(message)
        bound = self._document["px4_reference_at_bind"]
        for name, tolerance in (("ref_lat", 1e-11), ("ref_lon", 1e-11), ("ref_alt", 1e-6)):
            if abs(snapshot[name]-bound[name]) > tolerance:
                raise RuntimeError("PX4 origin changed after estimated feedback binding: "+name)
        for name in ("xy_global", "z_global", "xy_reset_counter", "z_reset_counter", "heading_reset_counter"):
            if snapshot[name] != bound[name]:
                raise RuntimeError("PX4 origin/reset changed after estimated feedback binding: "+name)
        for name in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "heading_good_for_control"):
            if not _boolean(message, name):
                raise RuntimeError("PX4 estimated feedback is invalid: "+name)
        if _boolean(message, "dead_reckoning"):
            raise RuntimeError("PX4 estimated feedback is dead reckoning")
        return snapshot

    def read(self, node, sim_s, *, wall_now=None):
        """Return (ControlState, joint header stamp, immutable-value diagnostics)."""
        from x650_wholebody.api import ControlState
        from x650_wholebody.model import rotation_matrix
        if not getattr(node, "estimator_feedback_enabled", False):
            raise RuntimeError("ApiNode.enable_estimator_feedback() must be called first")
        if getattr(node, "estimator_feedback_error", None):
            raise RuntimeError("Estimator feedback clock error: "+str(node.estimator_feedback_error))
        sim_s = _finite(sim_s, "simulation time")
        clock_s = _finite(getattr(node, "estimator_clock_s", None), "estimator /clock time")
        wall_now = _finite(time.monotonic() if wall_now is None else wall_now, "wall time")
        if sim_s < 0 or abs(sim_s-clock_s) > 1e-8:
            raise RuntimeError("Feedback simulation time must be the current estimator /clock time")
        if self._last_sim_s is not None and sim_s < self._last_sim_s-1e-9:
            raise RuntimeError("Estimator feedback simulation clock regressed")
        missing = [key for key in TOPICS if key not in node.data]
        if missing:
            raise RuntimeError("Missing PX4 estimated feedback streams: "+", ".join(missing))

        # SingleThreadedExecutor cannot replace a callback while this read runs.
        callbacks = {}
        for key in (*TOPICS, "clock"):
            seq = _integer(node.callback_sequences.get(key), key+" callback sequence", minimum=1)
            received_sim = _finite(node.received_sim.get(key), key+" received_sim")
            received_wall = _finite(node.received.get(key), key+" received_wall")
            sim_age, wall_age = sim_s-received_sim, wall_now-received_wall
            limit = self._limits["flags" if key == "estimator_flags" else "joint" if key == "joints" else "state"]
            if sim_age < -1e-9 or sim_age > limit+1e-9:
                raise RuntimeError(key+" feedback is stale or has a future /clock receipt")
            wall_limit = self._limits["flags_wall" if key == "estimator_flags" else "wall"]
            if wall_age < -1e-6 or wall_age > wall_limit:
                raise RuntimeError(key+" feedback is stale or has a future wall receipt")
            prior = self._last_callbacks.get(key)
            if prior:
                if seq < prior["sequence"] or received_sim < prior["received_sim_s"]-1e-9 or received_wall < prior["received_wall_s"]-1e-9:
                    raise RuntimeError(key+" callback sequence/receipt regressed")
                if seq == prior["sequence"] and (received_sim != prior["received_sim_s"] or received_wall != prior["received_wall_s"]):
                    raise RuntimeError(key+" held callback receipt changed without a new sequence")
            callbacks[key] = {"sequence": seq, "received_sim_s": received_sim,
                "received_wall_s": received_wall, "sim_age_s": sim_age, "wall_age_s": wall_age,
                "new_callback": prior is None or seq > prior["sequence"]}

        p, attitude, angular, flags, joints = (node.data[key] for key in TOPICS)
        origin = self._check_position(p)
        extra_resets = self._reset_snapshot(node.data)
        if self._extra_resets is not None and extra_resets != self._extra_resets:
            raise RuntimeError("PX4 velocity/quaternion reset changed after estimated feedback binding")
        flag_snapshot = {name: _boolean(flags, name) for name in EV_FLAGS}
        if flag_snapshot != EV_FLAGS:
            raise RuntimeError("PX4 EV fusion flags changed or are not ready")
        for name in ("cs_inertial_dead_reckoning", "cs_wind_dead_reckoning"):
            flag_snapshot[name] = _boolean(flags, name)
            if flag_snapshot[name]:
                raise RuntimeError("PX4 estimator flags indicate dead reckoning: "+name)
        pn = _vector([p.x, p.y, p.z], 3, "PX4 NED position")
        vn = _vector([_field(p, name) for name in ("vx", "vy", "vz")], 3, "PX4 NED velocity")
        # ax/ay/az are already earth-fixed velocity derivatives. They are not
        # body accelerometer specific force: neither rotate by attitude nor add
        # gravity. This optional input has no truth/differentiation fallback.
        acceleration_ned = None
        if self._require_acceleration:
            acceleration_ned = _vector([_finite(_field(p, name), "PX4 "+name)
                                        for name in ("ax", "ay", "az")],
                                       3, "PX4 NED earth-fixed acceleration")
        quaternion = _vector(_field(attitude, "q"), 4, "PX4 attitude wxyz")
        if abs(np.linalg.norm(quaternion)-1.) > 1e-3:
            raise RuntimeError("PX4 attitude quaternion is not unit length")
        rate = _vector(_field(angular, "xyz"), 3, "PX4 FRD angular velocity")
        position = NED_TO_ENU @ pn
        position[2] += self._document["px4_reference_at_bind"]["ref_alt"]
        rotation = NED_TO_ENU @ rotation_matrix(quaternion) @ FRD_TO_FLU
        names = list(_field(joints, "name"))
        if len(names) != len(set(names)) or any(name not in names for name in JOINTS):
            raise RuntimeError("Joint feedback names are missing or duplicated")
        q, dq = [], []
        for name in JOINTS:
            index = names.index(name)
            if index >= len(joints.position) or index >= len(joints.velocity):
                raise RuntimeError("Incomplete measured joint feedback: "+name)
            q.append(_finite(joints.position[index], name+" position"))
            dq.append(_finite(joints.velocity[index], name+" velocity"))
        stamp = joints.header.stamp
        sec = _integer(stamp.sec, "joint header sec")
        nanosec = _integer(stamp.nanosec, "joint header nanosec", maximum=999999999)
        joint_stamp = sec+nanosec*1e-9
        if self._last_joint_stamp is not None and joint_stamp < self._last_joint_stamp-1e-9:
            raise RuntimeError("Joint source timestamp regressed")
        # JointState carries a real Gazebo simulation timestamp. Cross-topic
        # delivery may put it slightly ahead of the latest /clock callback;
        # do not claim receipt-time synchronization. Bound both lag and lead.
        if abs(sim_s-joint_stamp) > self._limits["joint"]+1e-9:
            raise RuntimeError("Measured joint source timestamp is stale or too far ahead")
        raw_times = {key: {name: _integer(_field(node.data[key], name), key+" "+name)
                          for name in ("timestamp", "timestamp_sample")}
                     for key in TOPICS if key != "joints"}
        acceleration_args = ({"acceleration_enu": NED_TO_ENU @ acceleration_ned}
                             if self._require_acceleration else {})
        if self._require_prediction_timing:
            if joint_stamp > sim_s+1e-9:
                raise RuntimeError("Prediction chain cannot propagate a future joint measurement backwards")
            acceleration_args.update(control_time_s=sim_s, joint_stamp_s=joint_stamp)
        state = ControlState(position, rotation, NED_TO_ENU @ vn, FRD_TO_FLU @ rate,
                             q, dq, callbacks["angular_velocity"]["received_sim_s"], **acceleration_args)
        diagnostics = {"feedback_profile": "px4_estimated_raw_ev_world_v1",
            "snapshot_protocol": "latest_asynchronous_receipt_age_guarded",
            "truth_feedback": False, "truth_clock": False, "sim_time_s": sim_s,
            "state_stamp_s": state.stamp_s, "joint_stamp_s": joint_stamp,
            "joint_source_age_s": sim_s-joint_stamp, "callbacks": callbacks,
            "px4_raw_timestamps": raw_times,
            "px4_snapshot": {"position_ned_m": pn.tolist(), "velocity_ned_m_s": vn.tolist(),
                "attitude_wxyz": quaternion.tolist(), "angular_velocity_frd_rad_s": rate.tolist(),
                "origin": origin, "extra_reset_counters": extra_resets,
                "valid": {name: _boolean(p, name) for name in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "heading_good_for_control")},
                "dead_reckoning": False, "estimator_flags": flag_snapshot},
            "control_state": {"position_enu_m": state.position.tolist(),
                "velocity_enu_m_s": state.velocity.tolist(), "rotation_flu_to_enu": state.rotation.tolist(),
                "omega_flu_rad_s": state.omega_body.tolist(), "q_rad": q, "dq_rad_s": dq}}
        if self._require_acceleration:
            position_callback = callbacks["position"]
            diagnostics.update(feedback_profile=ACCELERATION_PROFILE,
                feedback_contract=ACCELERATION_CONTRACT,
                acceleration_stamp_s=position_callback["received_sim_s"],
                acceleration_source={"topic": "vehicle_local_position",
                    "callback_sequence": position_callback["sequence"],
                    "received_sim_s": position_callback["received_sim_s"],
                    "received_wall_s": position_callback["received_wall_s"],
                    **raw_times["position"], "quantity": "earth_fixed_velocity_derivative",
                    "input_frame": "NED", "output_frame": "ENU",
                    "numerical_differentiation": False, "truth_feedback": False})
            diagnostics["px4_snapshot"]["acceleration_ned_m_s2"] = acceleration_ned.tolist()
            diagnostics["control_state"]["acceleration_enu_m_s2"] = state.acceleration_enu.tolist()
        # Commit guard history only after the complete snapshot has passed.
        self._extra_resets = extra_resets
        self._last_callbacks = deepcopy(callbacks)
        self._last_sim_s, self._last_joint_stamp = sim_s, joint_stamp
        return state, joint_stamp, diagnostics
