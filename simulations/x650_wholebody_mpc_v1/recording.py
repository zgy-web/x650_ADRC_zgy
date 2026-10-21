"""Real ROS flight samples and independent same-physics-step tip evidence.

No kinematic tip estimate is labelled native truth here. The world recorder
plugin measures the distal-link pose and tip in Gazebo; its complete original
CSV rows are frozen before hashing because its live stream keeps appending
until the managed runtime shuts down.
"""
from __future__ import annotations
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import time
from xml.sax.saxutils import escape
import numpy as np


JOINTS = ("arm_joint_1", "arm_joint_2")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_value(v) for v in value]
    return value


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(json_value(value), indent=2, allow_nan=False)+"\n", encoding="utf-8")
    temporary.replace(path)


def seconds(stamp):
    return float(stamp.sec)+float(stamp.nanosec)*1e-9


def state_from_node(node):
    """Pose ENU, Odometry child-FLU twist rotated once, measured joint q/dq."""
    from x650_wholebody.api import ControlState
    from x650_wholebody.model import rotation_matrix
    truth = node.data["truth"]
    p, quat = truth.pose.pose.position, truth.pose.pose.orientation
    R = rotation_matrix([quat.w, quat.x, quat.y, quat.z])
    linear, angular = truth.twist.twist.linear, truth.twist.twist.angular
    joints = node.joint_values()
    state = ControlState([p.x, p.y, p.z], R, R@np.array([linear.x, linear.y, linear.z]),
                         [angular.x, angular.y, angular.z],
                         [joints[j].position_rad for j in JOINTS],
                         [joints[j].velocity_rad_s for j in JOINTS], seconds(truth.header.stamp))
    return state, seconds(node.data["joints"].header.stamp)


class FlightRecorder:
    """Capture every flight phase on the API's existing ROS executor."""
    columns = ["sim_time_s", "phase", "world_x_m", "world_y_m", "world_z_m",
               "world_qx", "world_qy", "world_qz", "world_qw", "q1_rad", "q2_rad",
               "q1_velocity_rad_s", "q2_velocity_rad_s", "joint_stamp_s", "joint_age_s",
               "frame_id", "child_frame_id", "armed", "landed", "failsafe", "nav_state",
               "target_world_x_m", "target_world_y_m", "target_world_z_m",
               "tip_target_world_x_m", "tip_target_world_y_m", "tip_target_world_z_m"]

    def __init__(self, folder, *, model_sha256, source_urdf_sha256, rate_hz=20.):
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("Capture rate must be positive")
        self.folder, self.rate_hz = Path(folder), float(rate_hz)
        self.model_sha256, self.source_urdf_sha256 = model_sha256, source_urdf_sha256
        self.phase, self.rows, self.error = "ground_ready", [], None
        self.target_body, self.tip_target = None, None
        self.clock_s, self.last_stamp, self.next_sample = 0., None, None
        self.context, self.subscriptions = None, []
        self.stream = (self.folder/"flight_samples.csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=self.columns)
        self.writer.writeheader()
        self.last_flush = time.monotonic()

    def attach(self, ctx):
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from rclpy.qos import qos_profile_sensor_data
        self.context = ctx
        self.subscriptions = [
            ctx.node.create_subscription(Odometry, "/ground_truth/odometry", self.receive, qos_profile_sensor_data),
            ctx.node.create_subscription(Clock, "/clock", self.receive_clock, qos_profile_sensor_data),
        ]
        self.receive(ctx.node.data["truth"])

    def receive_clock(self, message):
        try:
            stamp = seconds(message.clock)
            if stamp < self.clock_s-1e-9:
                raise RuntimeError("Simulation clock regressed during capture")
            self.clock_s = stamp
        except Exception as exc:
            self.error = self.error or str(exc)

    def now(self):
        node = self.context.node
        joint = node.data.get("joints")
        return max(self.clock_s, node.sim_time_s(), 0. if joint is None else seconds(joint.header.stamp))

    def receive(self, message):
        try:
            node = self.context.node
            if not all(k in node.data for k in ("position", "status", "land", "joints")):
                return
            stamp = seconds(message.header.stamp)
            if self.last_stamp is not None and stamp < self.last_stamp-1e-9:
                raise RuntimeError("Truth stamp regressed during capture")
            if self.last_stamp is not None and stamp <= self.last_stamp+1e-9:
                return
            self.last_stamp = stamp
            if self.next_sample is not None and stamp < self.next_sample-1e-9:
                return
            pos, quat = message.pose.pose.position, message.pose.pose.orientation
            joints, status, land = node.joint_values(), node.data["status"], node.data["land"]
            joint_stamp = seconds(node.data["joints"].header.stamp)
            values = [stamp, pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w,
                      *[joints[j].position_rad for j in JOINTS], *[joints[j].velocity_rad_s for j in JOINTS], joint_stamp]
            if not np.isfinite(values).all() or np.linalg.norm(values[4:8]) < 1e-9:
                raise ValueError("Nonfinite/invalid measured flight pose or joint state")
            row = dict(sim_time_s=stamp, phase=self.phase, world_x_m=pos.x, world_y_m=pos.y, world_z_m=pos.z,
                       world_qx=quat.x, world_qy=quat.y, world_qz=quat.z, world_qw=quat.w,
                       q1_rad=joints[JOINTS[0]].position_rad, q2_rad=joints[JOINTS[1]].position_rad,
                       q1_velocity_rad_s=joints[JOINTS[0]].velocity_rad_s, q2_velocity_rad_s=joints[JOINTS[1]].velocity_rad_s,
                       joint_stamp_s=joint_stamp, joint_age_s=stamp-joint_stamp,
                       frame_id=message.header.frame_id, child_frame_id=message.child_frame_id,
                       armed=int(status.arming_state == 2), landed=int(land.landed), failsafe=int(status.failsafe),
                       nav_state=int(status.nav_state))
            for prefix, target in (("target_world_", self.target_body), ("tip_target_world_", self.tip_target)):
                row.update({prefix+axis+"_m": "" if target is None else float(value)
                            for axis, value in zip("xyz", [None]*3 if target is None else target)})
            self.writer.writerow(row)
            self.rows.append(row)
            period = 1/self.rate_hz
            self.next_sample = stamp+period if self.next_sample is None else self.next_sample+max(1, math.floor((stamp-self.next_sample)/period+1e-9)+1)*period
            if time.monotonic()-self.last_flush > 1.:
                self.stream.flush()
                self.last_flush = time.monotonic()
        except Exception as exc:
            self.error = self.error or str(exc)

    def check(self):
        if self.error:
            raise RuntimeError("Flight recording failed: "+self.error)

    def close(self, *, complete):
        for sub in self.subscriptions:
            self.context.node.destroy_subscription(sub)
        self.subscriptions = []
        self.stream.close()
        intervals = np.diff([r["sim_time_s"] for r in self.rows])
        manifest = dict(schema_version=1, complete=bool(complete and not self.error and len(self.rows) > 1),
                        file="flight_samples.csv", sha256=sha(self.folder/"flight_samples.csv"),
                        model_sha256=self.model_sha256, source_urdf_sha256=self.source_urdf_sha256,
                        requested_rate_hz=self.rate_hz, sample_count=len(self.rows), capture_error=self.error,
                        phases={p: sum(r["phase"] == p for r in self.rows) for p in dict.fromkeys(r["phase"] for r in self.rows)},
                        time_semantics="Original truth header stamp; measured joints have their own asynchronous stamp; no inserted/interpolated samples",
                        tip_semantics="Only desired tip_target_world is present; actual independent tip is native_trace.csv",
                        pose_semantics="Original Gazebo ENU base_link pose; ROS xyzw quaternion; frame IDs retained",
                        interval_min_s=None if not len(intervals) else float(intervals.min()),
                        interval_max_s=None if not len(intervals) else float(intervals.max()))
        atomic_json(self.folder/"flight_samples_manifest.json", manifest)
        return manifest


class NativeRecorder:
    """Load the existing native observer into the world, without editing SDF."""
    def __init__(self, folder, library, instance):
        self.folder, self.library = Path(folder), Path(library).resolve()
        if not self.library.is_file() or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_-]*", instance):
            raise ValueError("Native recorder requires an existing library and explicit model instance")
        self.instance, self.library_sha256 = instance, sha(self.library)
        self.live = self.folder/"native_stream.csv"

    def load(self, pump):
        config = self.folder/"native_recorder_config.txt"
        config.write_text(f"model={self.instance}\noutput={self.live}\nduration=300\n", encoding="utf-8")
        request = ('entity { id: 1 } plugins { name: "wbbench::WholeBodyRecorder" filename: '
                   +json.dumps(str(self.library))+' innerxml: '
                   +json.dumps("<config_path>"+escape(str(config))+"</config_path>")+" }")
        log_path = self.folder/"native_recorder_load.log"
        with log_path.open("w", encoding="utf-8") as log:
            proc = subprocess.Popen(["gz", "service", "--timeout", "8000", "-s", "/world/default/entity/system/add",
                "--reptype", "gz.msgs.Boolean", "--reqtype", "gz.msgs.EntityPlugin_V", "--req", request],
                stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic()+12.
                while proc.poll() is None:
                    pump()
                    if time.monotonic() > deadline:
                        raise RuntimeError("Native recorder service load timed out")
            finally:
                if proc.poll() is None:
                    proc.terminate()
                    proc.wait(timeout=3.)
        if proc.returncode != 0 or "data: true" not in log_path.read_text(encoding="utf-8"):
            raise RuntimeError("Native recorder service did not confirm loading")
        deadline = time.monotonic()+12.
        while not self.live.is_file() or self.live.stat().st_size < 1000:
            pump()
            if time.monotonic() > deadline:
                raise RuntimeError("Loaded native recorder did not produce physics samples")

    def freeze(self, end_sim_s, *, model_sha256, source_urdf_sha256, complete, task_window=None, filename="native_trace.csv"):
        if not self.live.is_file():
            raise RuntimeError("Missing independent native physics stream")
        # An appending source may end halfway through a row. Keep only complete
        # lines, and preserve their exact text rather than parsing/reformatting.
        data = self.live.read_bytes()
        lines = data.splitlines(keepends=True)
        if not lines or not lines[0].endswith(b"\n"):
            raise RuntimeError("Native CSV has no complete header")
        fields = next(csv.reader([lines[0].decode("utf-8")]))
        required = {"sim_s", "valid_base", "valid_joint_state", "valid_tip", "tip_x", "tip_y", "tip_z"}
        if not required.issubset(fields):
            raise RuntimeError("Native observer lacks independent pose/joint/tip fields")
        frozen, records = [lines[0]], []
        for line in lines[1:]:
            if not line.endswith(b"\n"):
                break
            values = next(csv.reader([line.decode("utf-8")]))
            if len(values) != len(fields):
                raise RuntimeError("Native observer produced a malformed complete row")
            row = dict(zip(fields, values))
            if float(row["sim_s"]) <= end_sim_s+1e-9:
                frozen.append(line)
                records.append(row)
        if len(records) < 2:
            raise RuntimeError("Insufficient complete native physics samples")
        stamps = np.array([float(r["sim_s"]) for r in records])
        if not np.isfinite(stamps).all() or np.any(np.diff(stamps) <= 0):
            raise RuntimeError("Native physics stamps are invalid")
        selected = records if task_window is None else [r for r in records if task_window[0] <= float(r["sim_s"]) <= task_window[1]]
        if task_window is not None and (not selected or float(selected[0]["sim_s"]) > task_window[0]+.02
                                      or float(selected[-1]["sim_s"]) < task_window[1]-.02):
            raise RuntimeError("Native physics trace does not cover the complete task interval")
        if any(any(int(r[k]) != 1 for k in ("valid_base", "valid_joint_state", "valid_tip"))
               or not np.isfinite([float(r[k]) for k in ("tip_x", "tip_y", "tip_z")]).all() for r in selected):
            raise RuntimeError("Invalid native pose/joint/tip within the audited interval")
        if Path(filename).name != filename or not filename.endswith(".csv"):
            raise ValueError("Frozen native filename must be a local CSV basename")
        destination = self.folder/filename
        destination.write_bytes(b"".join(frozen))
        manifest = dict(schema_version=1, complete=bool(complete), file=destination.name, sha256=sha(destination),
                        model_sha256=model_sha256, source_urdf_sha256=source_urdf_sha256,
                        sample_count=len(records), time_first_s=float(stamps[0]), time_last_s=float(stamps[-1]),
                        task_window_sim_s=task_window, max_physics_interval_s=float(np.max(np.diff(stamps))),
                        library=str(self.library), library_sha256=self.library_sha256, instance=self.instance,
                        source_stream="native_stream.csv", source_snapshot_bytes=len(data),
                        semantics="Unmodified complete original Gazebo PostUpdate rows frozen from the append-only stream; tip=arm_link_2 WorldPose*[.25,0,0] independently measured by native plugin",
                        model_snapshot_unmodified=True)
        atomic_json(self.folder/(destination.stem+"_manifest.json"), manifest)
        return manifest


def native_ground_metrics(path, interval):
    """Ground stability from independent physics states over a full 5 s."""
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = [r for r in csv.DictReader(stream) if interval[0] <= float(r["sim_s"]) <= interval[1]]
    if (len(rows) < 100 or float(rows[0]["sim_s"]) > interval[0]+.02
            or float(rows[-1]["sim_s"]) < interval[1]-.02 or interval[1]-interval[0] < 5.-1e-9):
        raise RuntimeError("Native ground gate requires complete 5 s physical coverage")
    fields = ["sim_s", "base_qw", "base_qx", "base_qy", "base_qz", "base_vx", "base_vy", "base_vz",
              "arm_joint_1_q", "arm_joint_2_q", "arm_joint_1_qd", "arm_joint_2_qd"]
    if any(any(int(r[k]) != 1 for k in ("valid_base", "valid_joint_state", "valid_tip")) for r in rows):
        raise RuntimeError("Ground native observer has invalid state flags")
    a = np.array([[float(r[k]) for k in fields] for r in rows])
    if not np.isfinite(a).all() or np.any(np.diff(a[:, 0]) <= 0):
        raise RuntimeError("Nonfinite/nonmonotone independent ground state")
    quat = a[:, 1:5]
    norms = np.linalg.norm(quat, axis=1)
    if np.any(norms < 1e-9):
        raise RuntimeError("Invalid independent ground quaternion")
    quat = quat/norms[:, None]
    tilt = np.rad2deg(np.arccos(np.clip(1-2*(quat[:, 1]**2+quat[:, 2]**2), -1., 1.)))
    metrics = dict(duration_s=float(a[-1, 0]-a[0, 0]), sample_count=len(rows),
                   tilt_max_deg=float(tilt.max()), speed_max_m_s=float(np.linalg.norm(a[:, 5:8], axis=1).max()),
                   folded_q_abs_max_rad=np.max(abs(a[:, 8:10]), axis=0).tolist(),
                   joint_speed_abs_max_rad_s=np.max(abs(a[:, 10:12]), axis=0).tolist(),
                   max_physics_interval_s=float(np.diff(a[:, 0]).max()))
    criteria = dict(tilt_at_most_2deg=metrics["tilt_max_deg"] <= 2., speed_at_most_003m_s=metrics["speed_max_m_s"] <= .03,
                    folded_q_at_most_001rad=max(metrics["folded_q_abs_max_rad"]) <= .01,
                    joint_speed_at_most_002rad_s=max(metrics["joint_speed_abs_max_rad_s"]) <= .02,
                    native_sampling_complete=metrics["max_physics_interval_s"] <= .008+1e-9)
    return dict(metrics=metrics, criteria=criteria, passed=all(criteria.values()))


def native_hover_metrics(path, target, interval, *, expected_duration_s=20.):
    """Hover acceptance uses native base pose, with complete physics coverage."""
    if (len(interval) != 2 or not np.isfinite(interval).all()
            or not np.isfinite(expected_duration_s) or expected_duration_s <= 0
            or interval[1]-interval[0] < expected_duration_s-.008):
        raise RuntimeError("Native hover acceptance requires the full declared measurement interval")
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = [r for r in csv.DictReader(stream) if interval[0]-1e-9 <= float(r["sim_s"]) <= interval[1]+1e-9]
    if (len(rows) < 2 or float(rows[0]["sim_s"]) > interval[0]+.008+1e-9
            or float(rows[-1]["sim_s"]) < interval[1]-.008-1e-9):
        raise RuntimeError("Native hover samples do not cover the full measurement window")
    t = np.array([float(r["sim_s"]) for r in rows])
    position = np.array([[float(r["base_p"+axis]) for axis in "xyz"] for r in rows])
    quat = np.array([[float(r["base_q"+axis]) for axis in "wxyz"] for r in rows])
    target = np.asarray(target, dtype=float)
    if (target.shape != (3,) or not all(np.isfinite(v).all() for v in (t, position, quat, target))
            or np.any(np.diff(t) <= 0) or np.max(np.diff(t)) > .008+1e-9
            or any(r["valid_base"] != "1" for r in rows)):
        raise RuntimeError("Invalid, sparse or nonmonotone independent hover physics samples")
    norms = np.linalg.norm(quat, axis=1)
    if np.any(norms < 1e-9):
        raise RuntimeError("Invalid independent hover quaternion")
    quat /= norms[:, None]
    tilt = np.rad2deg(np.arccos(np.clip(1-2*(quat[:, 1]**2+quat[:, 2]**2), -1., 1.)))
    error = position-target
    error2 = np.sum(error**2, axis=1)
    integrate = getattr(np, "trapezoid", np.trapz)
    result = dict(source="native_trace.csv independent same-physics-step base_link world pose",
                  fixed_target_enu_m=target.tolist(), window_sim_s=list(interval),
                  requested_duration_s=expected_duration_s, duration_s=float(t[-1]-t[0]), sample_count=len(t),
                  max_physics_interval_s=float(np.diff(t).max()), no_phase_alignment=True,
                  position_rmse_m=float(np.sqrt(integrate(error2, t)/(t[-1]-t[0]))),
                  position_max_m=float(np.sqrt(error2.max())), tilt_max_deg=float(tilt.max()),
                  axis_rmse_m=np.sqrt(integrate(error**2, t, axis=0)/(t[-1]-t[0])).tolist())
    result["criteria"] = {"position_rmse_at_most_025m": result["position_rmse_m"] <= .25,
                          "position_peak_at_most_05m": result["position_max_m"] <= .50,
                          "tilt_at_most_15deg": result["tilt_max_deg"] <= 15.}
    result["accepted"] = all(result["criteria"].values())
    return result


def native_tip_metrics(folder, target_sample, task_start_s, interval):
    """Same-time, time-weighted 3D error using independent native tip only."""
    data = []
    with (Path(folder)/"native_trace.csv").open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            t = float(row["sim_s"])-task_start_s
            if interval[0]-1e-9 <= t <= interval[1]+1e-9:
                if row["valid_tip"] != "1":
                    raise RuntimeError("Invalid independently measured tip")
                position = np.array([float(row["tip_"+a]) for a in "xyz"])
                desired = np.asarray(target_sample(t)[0], dtype=float)
                data.append((t, position-desired))
    if len(data) < 2 or data[-1][0]-data[0][0] < interval[1]-interval[0]-.04:
        raise RuntimeError("Insufficient native samples for complete primary metric")
    stamps, error = np.array([r[0] for r in data]), np.array([r[1] for r in data])
    integrate = getattr(np, "trapezoid", np.trapz)
    norm2 = np.sum(error**2, axis=1)
    return dict(source="native_trace.csv independent physical tip", same_time=True, no_phase_alignment=True,
                interval_task_s=interval, sample_count=len(data), duration_s=float(stamps[-1]-stamps[0]),
                position_rmse_m=float(np.sqrt(integrate(norm2, stamps)/(stamps[-1]-stamps[0]))),
                position_max_m=float(np.sqrt(norm2.max())),
                axis_rmse_m=np.sqrt(integrate(error**2, stamps, axis=0)/(stamps[-1]-stamps[0])).tolist())


def native_joint_probe_metrics(folder, reference, task_start_s, duration, base_anchor):
    """Excitation/sign/tracking checks from independently measured joints."""
    with (Path(folder)/"native_trace.csv").open(newline="", encoding="utf-8") as stream:
        rows = [r for r in csv.DictReader(stream) if -.001 <= float(r["sim_s"])-task_start_s <= duration+.001]
    if (len(rows) < 100 or float(rows[0]["sim_s"])-task_start_s > .02
            or float(rows[-1]["sim_s"])-task_start_s < duration-.02):
        raise RuntimeError("Joint probe lacks full native task coverage")
    t = np.array([float(r["sim_s"])-task_start_s for r in rows])
    actual = np.array([[float(r["arm_joint_"+str(i)+"_q"]) for i in (1, 2)] for r in rows])
    velocity = np.array([[float(r["arm_joint_"+str(i)+"_qd"]) for i in (1, 2)] for r in rows])
    desired = np.array([reference(v, duration)[0] for v in t])
    position = np.array([[float(r["base_p"+a]) for a in "xyz"] for r in rows])
    quat = np.array([[float(r["base_q"+a]) for a in "wxyz"] for r in rows])
    if not all(np.isfinite(a).all() for a in (t, actual, velocity, desired, position, quat)) or np.any(np.diff(t) <= 0):
        raise RuntimeError("Invalid native joint-probe physical state")
    norms = np.linalg.norm(quat, axis=1)
    if np.any(norms < 1e-9) or any(any(r[k] != "1" for k in ("valid_base", "valid_joint_state", "valid_tip")) for r in rows):
        raise RuntimeError("Invalid native joint-probe measurement flags/quaternion")
    quat /= norms[:, None]
    tilt = np.rad2deg(np.arccos(np.clip(1-2*(quat[:, 1]**2+quat[:, 2]**2), -1., 1.)))
    integrate = getattr(np, "trapezoid", np.trapz)
    rmse = np.sqrt(integrate((actual-desired)**2, t, axis=0)/(t[-1]-t[0]))
    correlation = np.sum(actual*desired, axis=0)/np.sqrt(np.maximum(1e-24, np.sum(actual**2, axis=0)*np.sum(desired**2, axis=0)))
    metrics = dict(sample_count=len(rows), duration_s=float(t[-1]-t[0]),
        q_reference_positive_peak_rad=np.max(desired, axis=0).tolist(), q_reference_negative_peak_rad=np.min(desired, axis=0).tolist(),
        q_measured_positive_peak_rad=np.max(actual, axis=0).tolist(), q_measured_negative_peak_rad=np.min(actual, axis=0).tolist(),
        q_tracking_rmse_rad=rmse.tolist(), q_tracking_peak_rad=np.max(abs(actual-desired), axis=0).tolist(),
        reference_measured_sign_correlation=correlation.tolist(), joint_velocity_abs_max_rad_s=np.max(abs(velocity), axis=0).tolist(),
        base_anchor_displacement_max_m=float(np.linalg.norm(position-np.array(base_anchor), axis=1).max()), tilt_max_deg=float(tilt.max()))
    criteria = dict(both_joints_move_both_directions=bool(np.all(np.max(actual, axis=0) >= .035) and np.all(np.min(actual, axis=0) <= -.035)),
        command_sign_is_correct=bool(np.all(correlation >= .8)), q_tracking_rmse_at_most_002rad=bool(np.all(rmse <= .02)),
        q_tracking_peak_at_most_005rad=max(metrics["q_tracking_peak_rad"]) <= .05,
        q_amplitude_within_small_probe=bool(np.max(abs(actual)) <= .12),
        joint_speeds_at_most_02rad_s=max(metrics["joint_velocity_abs_max_rad_s"]) <= .2,
        body_displacement_at_most_03m=metrics["base_anchor_displacement_max_m"] <= .3,
        tilt_at_most_15deg=metrics["tilt_max_deg"] <= 15.)
    return dict(metrics=metrics, criteria=criteria, passed=all(criteria.values()),
                source="Independent native physics q/dq/base pose; no fitted phase or amplitude")
