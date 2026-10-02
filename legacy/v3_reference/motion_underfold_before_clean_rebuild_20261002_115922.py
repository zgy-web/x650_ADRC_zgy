"""Reviewed x500_underfold stationary-arm SITL baseline.

Protocol:
  fresh disarmed model
  q1 ~= 0, q2 ~= 0
  takeoff 2.5 m
  stable for 3 simulation seconds
  hold 10 s
  stationary-arm interval 20 s
  hold 10 s
  land
  confirm automatic disarm

No ground_stow.
No arm trajectory.
Arm command is zero velocity only.
"""

import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from pymavlink import mavutil
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


MODEL = "x500_underfold"

JOINT_TOPIC = "/arm/joint_states"
CMD1_TOPIC = "/arm/arm_joint_1/cmd_vel"
CMD2_TOPIC = "/arm/arm_joint_2/cmd_vel"

PX4_POSITION = "/fmu/out/vehicle_local_position_v1"
PX4_STATUS = "/fmu/out/vehicle_status_v1"
PX4_LAND = "/fmu/out/vehicle_land_detected"
TRUTH = "/ground_truth/odometry"

TOPICS = [
    JOINT_TOPIC,
    CMD1_TOPIC,
    CMD2_TOPIC,
    PX4_POSITION,
    "/fmu/out/vehicle_attitude",
    "/fmu/out/vehicle_odometry",
    PX4_STATUS,
    "/fmu/out/vehicle_control_mode",
    PX4_LAND,
    TRUTH,
    "/clock",
]

TAKEOFF_HEIGHT_M = 2.5

PRE_HOLD_S = 10.0
BASELINE_HOLD_S = 20.0

# ============================================================
# Motion V1 parameters
# q1: 0 -> -5 deg -> 0 deg
# q2: fixed at zero
# ============================================================

MOTION_Q1_TARGET_DEG = -5.0
MOTION_SPEED_DEG_S = 0.5
MOTION_HOLD_S = 5.0

MOTION_SPEED_RAD_S = (
    MOTION_SPEED_DEG_S
    * 3.141592653589793
    / 180.0
)

MOTION_TARGET_RAD = (
    MOTION_Q1_TARGET_DEG
    * 3.141592653589793
    / 180.0
)

POST_HOLD_S = 10.0

INITIAL_JOINT_TOL_DEG = 0.4
FLIGHT_JOINT_TOL_DEG = 1.0

INITIAL_JOINT_VEL_MAX = 0.01
FLIGHT_JOINT_VEL_MAX = 0.05


class Telemetry(Node):
    def __init__(self):
        super().__init__("underfold_baseline_supervisor")

        self.data = {}
        self.received = {}

        for key, typ, topic in (
            ("position", VehicleLocalPosition, PX4_POSITION),
            ("status", VehicleStatus, PX4_STATUS),
            ("land", VehicleLandDetected, PX4_LAND),
            ("joints", JointState, JOINT_TOPIC),
            ("truth", Odometry, TRUTH),
        ):
            self.create_subscription(
                typ,
                topic,
                lambda msg, key=key: self.receive(key, msg),
                qos_profile_sensor_data,
            )

        self.pub1 = self.create_publisher(
            Float64,
            CMD1_TOPIC,
            10,
        )

        self.pub2 = self.create_publisher(
            Float64,
            CMD2_TOPIC,
            10,
        )

    def receive(self, key, msg):
        self.data[key] = msg
        self.received[key] = time.monotonic()

    def fresh(self, require_truth=True):
        keys = (
            "position",
            "status",
            "land",
            "joints",
        )

        if require_truth:
            keys += ("truth",)

        now = time.monotonic()

        return all(
            now - self.received.get(key, 0)
            < (3 if key in ("status", "land") else 2)
            for key in keys
        )

    def zero(self):
        m = Float64()
        m.data = 0.0

        self.pub1.publish(m)
        self.pub2.publish(m)


class Run:
    def __init__(self, folder):
        # Do not silently share the GCS UDP port.
        with socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        ) as probe:
            probe.bind(("0.0.0.0", 14550))

        self.link = mavutil.mavlink_connection(
            "udpin:127.0.0.1:14550",
            source_system=255,
            source_component=190,
        )

        self.target = None

        self.last_heartbeat = 0.0
        self.heartbeat_time = 0.0

        self.global_position = None
        self.global_time = 0.0

        self.ack = {}

        self.node = Telemetry()

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self.folder = folder
        self.recorder = None

        self.attempted_arm = False
        self.airborne = False
        self.monitor_flight = False

        self.origin = None
        self.reset_counters = None

        self.events = []
        self.last_sim_stamp = None

    def event(self, text):
        print(text, flush=True)

        truth = self.node.data.get("truth")

        if truth is None:
            sim_time = None
        else:
            sim_time = (
                truth.header.stamp.sec
                + truth.header.stamp.nanosec / 1e9
            )

        self.events.append(
            {
                "wall_time": time.time(),
                "sim_time_latest_truth": sim_time,
                "event": text,
            }
        )

    def joint_values(self):
        msg = self.node.data["joints"]

        values = {}

        for name in (
            "arm_joint_1",
            "arm_joint_2",
        ):
            if name not in msg.name:
                raise RuntimeError(
                    "Missing joint feedback: " + name
                )

            i = msg.name.index(name)

            if (
                i >= len(msg.position)
                or i >= len(msg.velocity)
            ):
                raise RuntimeError(
                    "Incomplete JointState arrays: " + name
                )

            q = msg.position[i]
            v = msg.velocity[i]

            if not (
                math.isfinite(q)
                and math.isfinite(v)
            ):
                raise RuntimeError(
                    "Nonfinite joint state: " + name
                )

            values[name] = (q, v)

        return values

    def check_initial_joints(self):
        for name, (q, v) in self.joint_values().items():
            if abs(q) > math.radians(
                INITIAL_JOINT_TOL_DEG
            ):
                raise RuntimeError(
                    f"{name} initial q outside "
                    f"+/-{INITIAL_JOINT_TOL_DEG} deg: "
                    f"{math.degrees(q):.6f} deg"
                )

            if abs(v) > INITIAL_JOINT_VEL_MAX:
                raise RuntimeError(
                    f"{name} initial velocity too high: "
                    f"{v:.7f} rad/s"
                )

    def check_flight_joints(self):
        for name, (q, v) in self.joint_values().items():
            if abs(q) > math.radians(
                FLIGHT_JOINT_TOL_DEG
            ):
                raise RuntimeError(
                    f"{name} moved outside "
                    f"+/-{FLIGHT_JOINT_TOL_DEG} deg: "
                    f"{math.degrees(q):.6f} deg"
                )

            if abs(v) > FLIGHT_JOINT_VEL_MAX:
                raise RuntimeError(
                    f"{name} velocity too high: "
                    f"{v:.7f} rad/s"
                )

    def arm_velocity(self, q1_vel, q2_vel=0.0):
        msg1 = Float64()
        msg2 = Float64()

        msg1.data = float(q1_vel)
        msg2.data = float(q2_vel)

        self.node.pub1.publish(msg1)
        self.node.pub2.publish(msg2)


    def arm_stop(self):
        self.arm_velocity(0.0, 0.0)


    def pump(self):
        self.executor.spin_once(timeout_sec=0.02)

        now = time.monotonic()

        for _ in range(100):
            msg = self.link.recv_match(
                blocking=False
            )

            if msg is None:
                break

            if (
                msg.get_type() == "HEARTBEAT"
                and
                msg.autopilot
                == mavutil.mavlink.MAV_AUTOPILOT_PX4
            ):
                identity = (
                    msg.get_srcSystem(),
                    msg.get_srcComponent(),
                )

                if (
                    self.target is not None
                    and self.target != identity
                ):
                    raise RuntimeError(
                        "Multiple PX4 systems detected."
                    )

                self.target = identity
                self.last_heartbeat = now

            if (
                self.target
                and
                (
                    msg.get_srcSystem(),
                    msg.get_srcComponent(),
                ) == self.target
            ):
                if msg.get_type() == "COMMAND_ACK":
                    self.ack[msg.command] = (
                        now,
                        msg.result,
                    )

                elif (
                    msg.get_type()
                    == "GLOBAL_POSITION_INT"
                ):
                    self.global_position = msg
                    self.global_time = now

                elif (
                    msg.get_type() == "STATUSTEXT"
                    and msg.severity <= 4
                ):
                    self.event(
                        "PX4: " + msg.text
                    )

        if (
            self.target
            and
            now - self.heartbeat_time >= 0.5
        ):
            self.link.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )

            self.heartbeat_time = now

        # Stationary-arm baseline:
        # always command zero joint velocity.
        self.node.zero()

        if (
            self.recorder
            and self.recorder.poll() is not None
        ):
            raise RuntimeError(
                "Recorder exited unexpectedly; "
                "inspect recorder.log."
            )

        if not self.monitor_flight:
            return

        if (
            not self.node.fresh()
            or now - self.last_heartbeat > 3
        ):
            raise RuntimeError(
                "Flight telemetry stale."
            )

        p = self.node.data["position"]
        st = self.node.data["status"]

        if not all(
            math.isfinite(x)
            for x in (
                p.x,
                p.y,
                p.z,
                p.vx,
                p.vy,
                p.vz,
            )
        ):
            raise RuntimeError(
                "Nonfinite position or velocity."
            )

        if (
            self.reset_counters
            != (
                p.xy_reset_counter,
                p.z_reset_counter,
            )
        ):
            raise RuntimeError(
                "Estimator position reference "
                "reset during experiment."
            )

        if not (
            p.xy_valid
            and p.z_valid
            and p.v_xy_valid
            and p.v_z_valid
        ):
            raise RuntimeError(
                "Invalid position estimate."
            )

        if st.failsafe:
            raise RuntimeError(
                "PX4 failsafe active."
            )

        self.check_flight_joints()

        if (
            self.node.count_publishers(CMD1_TOPIC) != 1
            or
            self.node.count_publishers(CMD2_TOPIC) != 1
        ):
            raise RuntimeError(
                "Competing arm command publisher."
            )

        if self.origin:
            horizontal = math.hypot(
                p.x - self.origin[0],
                p.y - self.origin[1],
            )

            height = (
                -p.z
                + self.origin[2]
            )

            if (
                horizontal > 2.0
                or height > 4.0
            ):
                raise RuntimeError(
                    "Experiment position "
                    "envelope exceeded."
                )

        if (
            self.airborne
            and (
                self.node.data["land"].landed
                or
                st.arming_state
                != VehicleStatus.ARMING_STATE_ARMED
            )
        ):
            raise RuntimeError(
                "Unexpected landing or disarm."
            )

    def wait(
        self,
        predicate,
        timeout,
        description,
    ):
        self.event(description)

        deadline = (
            time.monotonic()
            + timeout
        )

        while (
            time.monotonic()
            < deadline
        ):
            self.pump()

            if predicate():
                return

        raise RuntimeError(
            "Timeout: " + description
        )

    def command(self, command, params):
        self.ack.pop(command, None)

        self.link.mav.command_long_send(
            *self.target,
            command,
            0,
            *(params + [0.0] * 7)[:7],
        )

        self.wait(
            lambda: command in self.ack,
            8,
            f"Awaiting command {command} acknowledgement",
        )

        result = self.ack[command][1]

        if (
            result
            != mavutil.mavlink.MAV_RESULT_ACCEPTED
        ):
            raise RuntimeError(
                f"Command {command} result={result}; "
                "not accepted. No force arming."
            )

    def sim_seconds(self):
        stamp = (
            self.node.data["truth"]
            .header.stamp
        )

        value = (
            stamp.sec
            + stamp.nanosec * 1e-9
        )

        if not math.isfinite(value):
            raise RuntimeError(
                "Invalid simulation time."
            )

        if (
            self.last_sim_stamp is not None
            and
            value
            < self.last_sim_stamp - 1e-9
        ):
            raise RuntimeError(
                "Simulation clock moved backwards."
            )

        self.last_sim_stamp = value
        return value

    def hover_ok(self, target_z):
        p = self.node.data["position"]

        return (
            abs(p.z - target_z) < 0.30
            and
            math.sqrt(
                p.vx * p.vx
                + p.vy * p.vy
                + p.vz * p.vz
            ) < 0.25
        )

    def hold(self, seconds, target_z, name):
        start = self.sim_seconds()

        self.wait(
            lambda:
                self.sim_seconds() - start
                >= seconds,
            seconds * 10 + 30,
            name,
        )

        actual = (
            self.sim_seconds()
            - start
        )

        self.event(
            f"{name} finished: "
            f"requested={seconds:.1f}s, "
            f"actual_sim={actual:.3f}s"
        )

        if not self.hover_ok(target_z):
            raise RuntimeError(
                "Hover drifted outside "
                "starting tolerance."
            )

    def execute_arm_motion(self):

        self.event(
            "MOTION BEGIN: q1 0deg -> -5deg -> 0deg"
        )

        # Move q1 negative direction
        t_end = (
            time.monotonic()
            +
            abs(MOTION_TARGET_RAD)
            /
            MOTION_SPEED_RAD_S
        )

        while time.monotonic() < t_end:
            self.arm_velocity(
                -MOTION_SPEED_RAD_S,
                0.0,
            )
            self.pump()
            time.sleep(0.02)

        self.arm_stop()

        self.event(
            "MOTION TARGET HOLD q1=-5deg"
        )

        hold_end = (
            time.monotonic()
            +
            MOTION_HOLD_S
        )

        while time.monotonic() < hold_end:
            self.arm_stop()
            self.pump()
            time.sleep(0.02)


        self.event(
            "MOTION RETURN q1 -> 0deg"
        )

        t_end = (
            time.monotonic()
            +
            abs(MOTION_TARGET_RAD)
            /
            MOTION_SPEED_RAD_S
        )

        while time.monotonic() < t_end:
            self.arm_velocity(
                MOTION_SPEED_RAD_S,
                0.0,
            )
            self.pump()
            time.sleep(0.02)

        self.arm_stop()

        self.event(
            "MOTION END"
        )

        # Verify motion returned to reviewed zero pose.
        self.check_initial_joints()


    def land(self):
        self.monitor_flight = False

        self.node.zero()

        self.command(
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            [
                0,
                0,
                0,
                math.nan,
                math.nan,
                math.nan,
                math.nan,
            ],
        )

        self.wait(
            lambda:
                self.node.fresh(
                    require_truth=False
                )
                and
                self.node.data["land"].landed,
            180,
            "Waiting for confirmed landing",
        )

        self.wait(
            lambda:
                self.node.fresh(
                    require_truth=False
                )
                and
                self.node.data["status"].arming_state
                != VehicleStatus.ARMING_STATE_ARMED,
            30,
            "Waiting for automatic disarm",
        )

    def start_recorder(self):
        output = open(
            self.folder / "recorder.log",
            "w",
        )

        self.recorder_output = output

        self.recorder = subprocess.Popen(
            [
                "ros2",
                "bag",
                "record",
                "-o",
                str(self.folder / "bag"),
                *TOPICS,
            ],
            stdout=output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        def recording():
            log = (
                self.folder
                / "recorder.log"
            ).read_text(
                errors="replace"
            )

            return all(
                any(
                    "Subscribed to topic"
                    in line
                    and topic in line
                    for line
                    in log.splitlines()
                )
                for topic in TOPICS
            )

        self.wait(
            recording,
            30,
            "Waiting for recorder subscriptions "
            "to all baseline topics",
        )

    def execute(self):
        self.wait(
            lambda: self.target is not None,
            45,
            "Waiting for PX4 MAVLink heartbeat",
        )

        self.wait(
            lambda: self.node.fresh(),
            45,
            "Waiting for PX4, joint and truth telemetry",
        )

        self.wait(
            lambda:
                self.node.data["status"]
                .pre_flight_checks_pass,
            60,
            "Waiting for normal PX4 preflight checks",
        )

        if (
            self.node.data["status"].arming_state
            == VehicleStatus.ARMING_STATE_ARMED
            or
            not self.node.data["land"].landed
        ):
            raise RuntimeError(
                "Start requires disarmed "
                "aircraft on ground."
            )

        self.check_initial_joints()

        self.wait(
            lambda:
                self.node.pub1.get_subscription_count() > 0
                and
                self.node.pub2.get_subscription_count() > 0,
            10,
            "Checking ROS-to-Gazebo "
            "zero-velocity command subscribers",
        )

        # Establish explicit zero command before arming.
        zero_until = (
            time.monotonic() + 1.0
        )

        while (
            time.monotonic()
            < zero_until
        ):
            self.pump()

        self.check_initial_joints()

        self.command(
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            [
                mavutil.mavlink
                .MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                200000,
            ],
        )

        self.wait(
            lambda:
                self.global_position is not None
                and
                time.monotonic()
                - self.global_time
                < 2,
            15,
            "Waiting for global altitude reference",
        )

        p = self.node.data["position"]

        self.origin = (
            p.x,
            p.y,
            p.z,
        )

        self.reset_counters = (
            p.xy_reset_counter,
            p.z_reset_counter,
        )

        target_z = (
            p.z - TAKEOFF_HEIGHT_M
        )

        altitude_amsl = (
            self.global_position.alt / 1000
            + TAKEOFF_HEIGHT_M
        )

        self.start_recorder()

        self.event(
            "Takeoff reference: "
            f"origin={self.origin}, "
            f"target_z={target_z}, "
            f"altitude_amsl={altitude_amsl}"
        )

        self.monitor_flight = True
        self.attempted_arm = True

        self.command(
            mavutil.mavlink
            .MAV_CMD_COMPONENT_ARM_DISARM,
            [1, 0],
        )

        self.wait(
            lambda:
                self.node.data["status"]
                .arming_state
                == VehicleStatus.ARMING_STATE_ARMED,
            8,
            "Confirming armed state",
        )

        self.command(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            [
                0,
                0,
                0,
                math.nan,
                math.nan,
                math.nan,
                altitude_amsl,
            ],
        )

        stable_since = [None]

        def stable():
            stamp = self.sim_seconds()

            if (
                self.hover_ok(target_z)
                and
                not self.node.data["land"].landed
            ):
                if (
                    stable_since[0]
                    is None
                ):
                    stable_since[0] = stamp

                return (
                    stamp - stable_since[0]
                    >= 3.0
                )

            stable_since[0] = None
            return False

        self.wait(
            stable,
            180,
            "Waiting for 2.5 m hover "
            "stable for 3 simulation seconds",
        )

        self.airborne = True

        self.hold(
            PRE_HOLD_S,
            target_z,
            "Pre-baseline stabilization hold",
        )

        self.event(
            "STATIONARY UNDERFOLD BASELINE "
            "INTERVAL BEGIN"
        )

        self.hold(
            BASELINE_HOLD_S,
            target_z,
            "Stationary underfold baseline interval",
        )

        self.event(
            "STATIONARY UNDERFOLD BASELINE "
            "INTERVAL END"
        )

        self.execute_arm_motion()

        self.hold(
            POST_HOLD_S,
            target_z,
            "Post-baseline stabilization hold",
        )

        self.land()

        self.event(
            "COMPLETE: stationary underfold "
            "baseline, landing and disarm confirmed."
        )

    def close(self):
        if self.recorder:
            try:
                os.killpg(
                    self.recorder.pid,
                    signal.SIGINT,
                )
            except ProcessLookupError:
                pass

            try:
                self.recorder.wait(
                    timeout=20
                )
            except subprocess.TimeoutExpired:
                self.event(
                    "Recorder did not finish; "
                    "inspect bag before reuse."
                )

        if hasattr(
            self,
            "recorder_output",
        ):
            self.recorder_output.close()

        self.node.zero()

        self.executor.shutdown()
        self.node.destroy_node()
        self.link.close()


def existing_ulogs(px4):
    root = (
        px4
        / "build/px4_sitl_default/logs"
    )

    if not root.exists():
        return set()

    return {
        str(p.resolve())
        for p in root.rglob("*.ulg")
    }


def main():
    base = Path("/root/autodl-tmp")

    here = (
        Path(__file__)
        .resolve()
        .parent
    )

    px4 = Path(
        os.environ.get(
            "PX4_DIR",
            str(base / "PX4-Autopilot"),
        )
    )

    model = (
        px4
        / "Tools/simulation/gz/models/"
          "x500_underfold/model.sdf"
    )

    expected_sha = (
        here / "MODEL_SHA256"
    ).read_text().strip()

    raw = model.read_bytes()

    actual_sha = hashlib.sha256(
        raw
    ).hexdigest()

    if actual_sha != expected_sha:
        raise RuntimeError(
            "Installed model hash does not "
            "match reviewed underfold model."
        )

    folder = (
        Path(
            os.environ.get(
                "RESULTS_DIR",
                str(base / "px4_baseline_bags"),
            )
        )
        / (
            "baseline_underfold_"
            + time.strftime("%Y%m%d_%H%M%S")
            + f"_{os.getpid()}"
        )
    )

    folder.mkdir(
        parents=True,
        exist_ok=False,
    )

    (folder / "model_snapshot.sdf").write_bytes(
        raw
    )

    protocol = {
        "model": MODEL,
        "takeoff_height_m": TAKEOFF_HEIGHT_M,
        "stable_required_sim_s": 3.0,
        "pre_hold_sim_s": PRE_HOLD_S,
        "baseline_interval_sim_s": BASELINE_HOLD_S,
        "post_hold_sim_s": POST_HOLD_S,
        "initial_joint_tolerance_deg":
            INITIAL_JOINT_TOL_DEG,
        "flight_joint_tolerance_deg":
            FLIGHT_JOINT_TOL_DEG,
        "arm_command":
            "zero velocity only",
    }

    (folder / "protocol.json").write_text(
        json.dumps(
            protocol,
            indent=2,
        )
    )

    result = {
        "success": False,
        "experiment":
            "motion_underfold_v1",
        "model_sha256": actual_sha,
        "actuator_model":
            "unchanged direct velocity control",
        "ground_stow": False,
        "arm_motion": True,
        "protocol": protocol,
    }

    before_ulogs = existing_ulogs(px4)

    rclpy.init()

    run = None

    try:
        run = Run(folder)
        run.execute()
        result["success"] = True

    except (
        Exception,
        KeyboardInterrupt,
    ) as exc:
        result["error"] = (
            str(exc)
            or "Interrupted by user"
        )

        print(
            "EXPERIMENT STOPPED: "
            + result["error"],
            flush=True,
        )

        if (
            run
            and run.attempted_arm
        ):
            recorder = run.recorder
            run.recorder = None

            try:
                run.land()
                result[
                    "recovery_landed"
                ] = True

            except (
                Exception,
                KeyboardInterrupt,
            ) as recovery:
                result[
                    "recovery_error"
                ] = str(recovery)

                print(
                    "Landing NOT confirmed. "
                    "Keep simulator running; "
                    "inspect flight state.",
                    flush=True,
                )

            finally:
                run.recorder = recorder

    finally:
        if run:
            try:
                run.close()

            except Exception as cleanup:
                result[
                    "cleanup_error"
                ] = str(cleanup)

                result[
                    "success"
                ] = False

            finally:
                result["events"] = run.events

        if rclpy.ok():
            rclpy.shutdown()

        time.sleep(2)

        after_ulogs = existing_ulogs(
            px4
        )

        new_ulogs = [
            Path(p)
            for p in (
                after_ulogs
                - before_ulogs
            )
        ]

        new_ulogs.sort(
            key=lambda p:
                p.stat().st_mtime,
            reverse=True,
        )

        if new_ulogs:
            src = new_ulogs[0]

            dst = (
                folder
                / "px4.ulg"
            )

            try:
                shutil.copy2(
                    src,
                    dst,
                )

                result[
                    "ulog_source"
                ] = str(src)

                result[
                    "ulog_copied"
                ] = True

            except Exception as exc:
                result[
                    "ulog_copied"
                ] = False

                result[
                    "ulog_copy_error"
                ] = str(exc)
        else:
            result[
                "ulog_copied"
            ] = False

        (
            folder
            / "result.json"
        ).write_text(
            json.dumps(
                result,
                indent=2,
            )
        )

        print(
            f"Results: {folder}",
            flush=True,
        )

    raise SystemExit(
        0
        if result["success"]
        else 1
    )


if __name__ == "__main__":
    main()
