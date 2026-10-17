#!/usr/bin/env python3
"""AMSIM adapter for the reusable frozen PID + arm feedforward controller.

Only this adapter knows about ROS, MAVLink, flight lifecycle and Gazebo state.
The controller package itself is callable without a running simulator.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PACKAGE_ROOT = ROOT / "control_packages/pid_ff_wholebody_v1"
for path in (HERE, PACKAGE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from flyarm_pid_ff import (ArmParameters, ControlState, EndpointTarget, FigureEight,
                           PidFeedforwardController, frozen_pid_parameters, tip_world)
from launch import atomic_json, selected_task_config

JOINTS = ("arm_joint_1", "arm_joint_2")


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(v) for v in value]
    return value


def seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def rotation_from_quaternion(quaternion):
    w, x, y, z = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm([w, x, y, z])
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError("Invalid truth quaternion")
    w, x, y, z = np.array([w, x, y, z]) / norm
    return np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)],
                     [2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x)],
                     [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)]])


def read_state(ctx):
    """The Gazebo Odometry twist is in child FLU; position is world ENU."""
    truth = ctx.node.data["truth"]
    point, quat = truth.pose.pose.position, truth.pose.pose.orientation
    rotation = rotation_from_quaternion([quat.w, quat.x, quat.y, quat.z])
    linear, angular = truth.twist.twist.linear, truth.twist.twist.angular
    joints = ctx.node.joint_values()
    state = ControlState(
        position=np.array([point.x, point.y, point.z]), rotation=rotation,
        velocity=rotation @ np.array([linear.x, linear.y, linear.z]),
        omega_body=np.array([angular.x, angular.y, angular.z]),
        q=np.array([joints[j].position_rad for j in JOINTS]),
        dq=np.array([joints[j].velocity_rad_s for j in JOINTS]),
        stamp_s=seconds(truth.header.stamp),
    )
    return state, seconds(ctx.node.data["joints"].header.stamp)


def set_parameter(ctx, name, value, parameter_type):
    """Do not arm until all frozen parameters have been acknowledged."""
    for _attempt in range(3):
        ctx.vehicle.link.mav.param_set_send(*ctx.vehicle.target, name.encode(), float(value), parameter_type)
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            msg = ctx.vehicle.link.recv_match(type="PARAM_VALUE", blocking=False)
            if msg is not None:
                key = msg.param_id.decode() if isinstance(msg.param_id, bytes) else msg.param_id
                if key.rstrip("\x00") == name and abs(float(msg.param_value) - value) < 1e-6:
                    return float(msg.param_value)
            ctx.node.spin_once(timeout_sec=.01)
    raise RuntimeError("No verified parameter acknowledgement: " + name)


def check_flight_state(ctx, state, initial_position):
    status = ctx.node.data["status"]
    if status.failsafe or status.nav_state != 14:
        raise RuntimeError("PX4 failsafe or offboard exit")
    if np.linalg.norm(state.position - initial_position) > 1.5:
        raise RuntimeError("Flight position envelope exceeded")
    if math.acos(float(np.clip(state.rotation[2, 2], -1., 1.))) > .65:
        raise RuntimeError("Flight tilt envelope exceeded")
    if (np.any(state.q < np.array([-1.0472, -.43634]) - .01)
            or np.any(state.q > np.array([.5236, 1.0472]) + .01)):
        raise RuntimeError("Joint physical limit exceeded")


def scenario():
    from amsim import ScenarioContext
    from pymavlink import mavutil
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

    out = Path(os.environ["AMSIM_RUN_DIR"]).resolve()
    cfg = selected_task_config()
    atomic_json(out / "controller_task.json", cfg)
    model_path = out / "model_snapshot.sdf"
    # Parsing and model compatibility failures are fatal; never use defaults
    # when the simulator's actual model cannot be validated.
    params = ArmParameters.from_sdf(model_path)
    gains = frozen_pid_parameters()
    if len(gains) != 26:
        raise RuntimeError("Expected the 26 frozen PID gains")
    atomic_json(out / "controller_pid_parameters.json", gains)
    sources = {str(p.relative_to(ROOT)).replace("\\", "/"): hashlib.sha256(p.read_bytes()).hexdigest()
               for folder in (PACKAGE_ROOT / "flyarm_pid_ff", HERE)
               for p in sorted(folder.rglob("*"))
               if p.is_file() and p.suffix in (".py", ".json") and "__pycache__" not in p.parts}
    meta = {
        "schema_version": 1, "complete": False, "phase": "initializing", "task": cfg,
        "controller_package": "flyarm_pid_ff", "source_hashes": sources,
        "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "requested_parameters": gains, "simulation_only": True,
        "feedback": "Gazebo truth-assisted ENU pose/velocity and measured joints; PX4 NED setpoint bridge",
        "feedforward": "Relative arm translational inertia plus analytic trajectory acceleration; moment diagnostic only",
        "joint_actuation": "Existing Gazebo joint velocity controller",
        "external_disturbance": False, "control_period_s": cfg["control_period_s"],
        "native_recorder": False, "truth_max_age_s": .25,
    }

    def save():
        atomic_json(out / "controller_metadata.json", json_value(meta))

    save()
    fields = ["sim_s", "t_s", "in_task", "truth_stamp_s", "truth_age_s", "joint_stamp_s",
              "joint_age_s", "control_dt_s", "compute_ms", "px4_timestamp_us"]
    for prefix in ("base_p", "base_v", "body_w", "tip_", "tip_ref_", "tip_error_", "base_ref_",
                   "base_v_ref_", "base_velocity_target_", "nominal_a_", "arm_ff_a_", "cmd_a_",
                   "ned_sp_", "ned_sv_", "ned_sa_"):
        fields += [prefix + axis for axis in "xyz"]
    fields += ["q1", "q2", "qd1", "qd2", "qd1_cmd", "qd2_cmd", "roll", "pitch", "yaw"]
    with ((out / "control_trace.csv").open("w", newline="", encoding="utf-8") as trace,
          (out / "controller_diagnostics.jsonl").open("w", encoding="utf-8") as diagnostic_file,
          ScenarioContext() as ctx):
        writer = csv.DictWriter(trace, fieldnames=fields)
        writer.writeheader()
        clock = [0.]

        def receive_clock(msg):
            clock[0] = seconds(msg.clock)

        clock_sub = ctx.node.create_subscription(
            Clock, "/clock", receive_clock,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))

        def now():
            joint = ctx.node.data.get("joints")
            joint_time = 0. if joint is None else seconds(joint.header.stamp)
            return max(clock[0], ctx.node.sim_time_s(), joint_time)

        original_pump = ctx.vehicle._pump
        flight_started = False
        start, last, next_control = [None], [None], [-math.inf]
        previous_truth, previous_joint = [None], [None]
        rows = [0]
        try:
            ctx.vehicle.wait_ready()
            ctx.arm.stop()
            preflight_state, _ = read_state(ctx)
            # Validate the full frozen model before arming. Construction does
            # not observe motion or advance the allocation state.
            controller = PidFeedforwardController(preflight_state, arm_parameters=params, mode=cfg["mode"])
            meta["model_compatibility_verified_before_takeoff"] = True
            save()
            meta["calibration"] = {
                "SENS_BAR_AUTOCAL": set_parameter(ctx, "SENS_BAR_AUTOCAL", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
                "CAL_BARO0_OFF": set_parameter(ctx, "CAL_BARO0_OFF", 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            }
            meta["verified_parameters"] = {
                name: set_parameter(ctx, name, float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                for name, value in gains.items()}
            meta["parameters_verified"] = True
            save()
            flight_started = True
            ctx.vehicle.takeoff(cfg["hover_height_m"], stable_s=2.5)
            ctx.wait(3.)
            measured, _ = read_state(ctx)
            p0, R0, q0 = measured.position.copy(), measured.rotation.copy(), np.zeros(2)
            # The validated home reference is q0=0; subsequent states use measured q.
            initial = ControlState(position=p0, rotation=R0, velocity=measured.velocity,
                                   omega_body=measured.omega_body, q=q0, dq=measured.dq,
                                   stamp_s=measured.stamp_s)
            controller.reset(initial)
            meta["controller_details"] = controller.metadata()
            center = tip_world(p0, R0, q0)
            trajectory = None if cfg["trajectory"] == "hover" else FigureEight(center=center, **cfg["trajectory_parameters"])
            yaw_ned = float(ctx.node.data["position"].heading)
            meta.update(reference_base_enu_m=p0, reference_rotation=R0,
                        reference_joint_position_rad=q0, reference_center_enu_m=center, yaw_ned_rad=yaw_ned)
            save()

            def pump():
                original_pump()
                for _ in range(6):
                    ctx.node.spin_once(timeout_sec=0.)
                sim = now()
                if sim + 1e-9 < next_control[0]:
                    return
                computation_start = time.perf_counter()
                state, joint_stamp = read_state(ctx)
                age, joint_age = sim - state.stamp_s, sim - joint_stamp
                if not -.001 <= age <= .25:
                    raise RuntimeError(f"Truth age {age:.4f} outside bounds")
                if not -.001 <= joint_age <= .25:
                    raise RuntimeError(f"Joint age {joint_age:.4f} outside bounds")
                if previous_truth[0] is not None and state.stamp_s < previous_truth[0] - 1e-9:
                    raise RuntimeError("Truth timestamp regressed")
                if previous_joint[0] is not None and joint_stamp < previous_joint[0] - 1e-9:
                    raise RuntimeError("Joint timestamp regressed")
                previous_truth[0], previous_joint[0] = state.stamp_s, joint_stamp
                dt = cfg["control_period_s"] if last[0] is None else sim - last[0]
                if dt <= 0.:
                    return
                if dt > .05 + 1e-10:
                    raise RuntimeError(f"Control interval {dt:.4f} s exceeds 0.05 s")
                t = -1. if start[0] is None else sim - start[0]
                if start[0] is None:
                    command = controller.hold(state)
                else:
                    target = (EndpointTarget(center, np.zeros(3), np.zeros(3)) if trajectory is None
                              else EndpointTarget(*trajectory.sample(t)))
                    command = controller.step(t, dt, state, target)
                ep = ctx.node.data["position"]
                px4 = command.px4_setpoint([ep.x, ep.y, ep.z], [ep.vx, ep.vy, ep.vz])
                ctx.vehicle.link.mav.set_position_target_local_ned_send(
                    int(sim * 1000) & 0xffffffff, *ctx.vehicle.target,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, px4.type_mask,
                    *px4.position, *px4.velocity, *px4.acceleration, yaw_ned, 0.)
                joints = np.asarray(command.joint_velocity)
                if np.any(np.abs(joints) > np.array([.45, .35]) + 1e-9):
                    raise RuntimeError("Controller joint command exceeded frozen speed cap")
                for name, value in zip(JOINTS, joints):
                    ctx.arm.set_velocity(name, float(value))
                R = state.rotation
                row = dict(sim_s=sim, t_s=t, in_task=int(start[0] is not None),
                           truth_stamp_s=state.stamp_s, truth_age_s=age, joint_stamp_s=joint_stamp,
                           joint_age_s=joint_age, control_dt_s=dt,
                           px4_timestamp_us=int(ep.timestamp),
                           q1=state.q[0], q2=state.q[1], qd1=state.dq[0], qd2=state.dq[1],
                           qd1_cmd=joints[0], qd2_cmd=joints[1],
                           roll=math.atan2(R[2, 1], R[2, 2]),
                           pitch=math.asin(float(np.clip(-R[2, 0], -1., 1.))),
                           yaw=math.atan2(R[1, 0], R[0, 0]))
                for prefix, values in (
                    ("base_p", state.position), ("base_v", state.velocity), ("body_w", state.omega_body),
                    ("tip_", command.tip_position), ("tip_ref_", command.tip_target),
                    ("tip_error_", command.tip_target - command.tip_position),
                    ("base_ref_", command.position_reference), ("base_v_ref_", command.velocity_feedforward),
                    ("base_velocity_target_", command.base_velocity_target),
                    ("nominal_a_", command.nominal_acceleration), ("arm_ff_a_", command.arm_acceleration),
                    ("cmd_a_", command.acceleration_feedforward),
                    ("ned_sp_", px4.position), ("ned_sv_", px4.velocity), ("ned_sa_", px4.acceleration)):
                    row.update({prefix + axis: float(value) for axis, value in zip("xyz", values)})
                row["compute_ms"] = (time.perf_counter() - computation_start) * 1000.
                writer.writerow(row)
                diagnostic_file.write(json.dumps(json_value({"sim_s": sim, "t_s": t,
                    "diagnostics": command.diagnostics, "reference": command.reference}), allow_nan=False) + "\n")
                rows[0] += 1
                if rows[0] % 50 == 0:
                    trace.flush()
                    diagnostic_file.flush()
                last[0], next_control[0] = sim, sim + cfg["control_period_s"]

            ctx.vehicle._pump = pump
            prestream_start = time.monotonic()
            while time.monotonic() - prestream_start < 1.25:
                pump()
            ctx.vehicle._command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, [1, 6, 0])
            ctx.vehicle.wait(lambda: ctx.node.data["status"].nav_state == 14, 8., "PID + feedforward offboard")
            meta["phase"] = "washout"
            save()
            washout_start, stable, deadline = now(), None, time.monotonic() + 150.
            while True:
                pump()
                state, _ = read_state(ctx)
                sim = now()
                check_flight_state(ctx, state, p0)
                good = (sim - washout_start >= 5. and np.linalg.norm(state.velocity) < .015
                        and np.linalg.norm(state.position - p0) < .015 and np.max(np.abs(state.q)) < .006)
                stable = (sim if stable is None else stable) if good else None
                if stable is not None and sim - stable >= .5:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError("Initial stable-hover gate timed out")
            start[0], last[0] = now(), None
            next_control[0] = start[0]
            meta.update(phase="tracking", t0_sim_s=start[0], duration_s=cfg["duration_s"],
                        initial_speed_m_s=float(np.linalg.norm(state.velocity)),
                        initial_joint_error_rad=float(np.max(np.abs(state.q))))
            save()
            print("PID_FF_START", cfg["trajectory"], cfg["mode"], cfg["duration_s"], flush=True)
            deadline = time.monotonic() + max(180., cfg["duration_s"] * 8.)
            while now() - start[0] < cfg["duration_s"] + .3:
                pump()
                state, _ = read_state(ctx)
                check_flight_state(ctx, state, p0)
                if time.monotonic() > deadline:
                    raise RuntimeError("Tracking wall timeout")
            trace.flush()
            diagnostic_file.flush()
            meta.update(trajectory_finished=True, phase="landing", control_rows=rows[0])
            save()
        except BaseException as exc:
            meta["error"] = repr(exc)
            save()
            raise
        finally:
            active_exception = sys.exc_info()[0] is not None
            ctx.vehicle._pump = original_pump
            try:
                ctx.arm.stop()
            finally:
                if flight_started:
                    try:
                        ctx.vehicle.land()
                        meta["landed_and_disarmed"] = True
                    except BaseException as exc:
                        meta["landing_error"] = repr(exc)
                        save()
                        if not active_exception:
                            raise
                save()
    meta.update(complete=True, phase="complete")
    save()
    print("PID_FF_COMPLETE", out, flush=True)


if __name__ == "__main__":
    if "AMSIM_RUN_DIR" not in os.environ:
        raise SystemExit("Use launch.py, or scripts/amsim run pid_ff_wholebody_v1 --headless")
    scenario()
