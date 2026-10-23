"""Synthetic unit fixtures only; these checks do not represent native flights."""
from __future__ import annotations
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
for p in (HERE, ROOT/"control_packages/x650_wholebody_v1"):
    sys.path.insert(0, str(p))
from recording import (NativeRecorder, FlightRecorder, native_ground_metrics, native_hover_metrics,
                       native_tip_metrics, native_joint_probe_metrics, sha, state_from_node,
                       JOINTS, seconds, atomic_json)


FIELDS = ["sim_s", "step_dt_s", "valid_base", "valid_joint_state", "valid_tip",
          "base_px", "base_py", "base_pz", "base_qw", "base_qx", "base_qy", "base_qz",
          "base_vx", "base_vy", "base_vz", "arm_joint_1_q", "arm_joint_2_q",
          "arm_joint_1_qd", "arm_joint_2_qd", "tip_x", "tip_y", "tip_z"]


def native_fixture(path, *, end=5., sample=None):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for t in np.arange(0., end+.001, .004):
            row = {k: 0. for k in FIELDS}
            row.update(sim_s=float(t), step_dt_s=.004, valid_base=1, valid_joint_state=1,
                       valid_tip=1, base_qw=1., base_pz=2., tip_x=.01)
            if sample is not None:
                row.update(sample(float(t)))
            writer.writerow(row)


def ros_message(stamp=1., yaw=np.pi/2):
    xyz = lambda x, y, z: NS(x=x, y=y, z=z)
    return NS(header=NS(stamp=NS(sec=int(stamp), nanosec=int(round((stamp-int(stamp))*1e9))), frame_id="world"),
        child_frame_id="base_link", pose=NS(pose=NS(position=xyz(1., 2., 3.),
            orientation=NS(x=0., y=0., z=np.sin(yaw/2), w=np.cos(yaw/2)))),
        twist=NS(twist=NS(linear=xyz(1., 0., 0.), angular=xyz(0., .2, 0.))))


class RecordingTests(unittest.TestCase):
    def test_native_freeze_preserves_complete_rows_and_is_immutable(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            library = folder/"observer.so"
            library.write_bytes(b"unit placeholder never loaded")
            observer = NativeRecorder(folder, library, "unit_fixture_0")
            native_fixture(observer.live, end=5.1)
            with observer.live.open("ab") as stream:
                stream.write(b"incomplete trailing row")
            manifest = observer.freeze(5., model_sha256="a"*64, source_urdf_sha256="b"*64,
                                       complete=True, task_window=(0., 5.))
            frozen = (folder/"native_trace.csv").read_bytes()
            self.assertEqual(manifest["sample_count"], 1251)
            self.assertNotIn(b"incomplete", frozen)
            self.assertEqual(sha(folder/"native_trace.csv"), manifest["sha256"])
            with observer.live.open("ab") as stream:
                stream.write(b"later mutation of live stream")
            self.assertEqual((folder/"native_trace.csv").read_bytes(), frozen)
            metric = native_tip_metrics(folder, lambda t: (np.zeros(3), None, None), 0., (0., 5.))
            self.assertAlmostEqual(metric["position_rmse_m"], .01)
            self.assertAlmostEqual(metric["position_max_m"], .01)

    def test_ground_gate_rejects_tilt_motion_bad_q_and_short_coverage(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"native.csv"
            native_fixture(path)
            self.assertTrue(native_ground_metrics(path, (0., 5.))["passed"])
            for field, value in (("base_vx", .1), ("arm_joint_1_q", .02), ("arm_joint_2_qd", .04),
                                 ("base_qx", .1)):
                native_fixture(path, sample=lambda t: {field: value})
                self.assertFalse(native_ground_metrics(path, (0., 5.))["passed"], field)
            with self.assertRaises(RuntimeError):
                native_ground_metrics(path, (0., 6.))

    def test_hover_acceptance_rejects_two_sparse_zero_error_points_and_partial_window(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"native.csv"
            native_fixture(path, end=20.)
            result = native_hover_metrics(path, [0., 0., 2.], (0., 20.))
            self.assertTrue(result["accepted"])
            self.assertEqual(result["sample_count"], 5001)
            self.assertIn("native_trace.csv", result["source"])
            all_lines = path.read_text().splitlines()
            path.write_text("\n".join([all_lines[0], all_lines[1], all_lines[-1]])+"\n")
            with self.assertRaisesRegex(RuntimeError, "sparse"):
                native_hover_metrics(path, [0., 0., 2.], (0., 20.))
            native_fixture(path, end=19.)
            with self.assertRaises(RuntimeError):
                native_hover_metrics(path, [0., 0., 2.], (0., 20.))
            with self.assertRaises(RuntimeError):
                native_hover_metrics(path, [0., 0., 2.], (0., 19.))

    def test_native_joint_probe_requires_both_signs_and_measured_tracking(self):
        from run_case import joint_probe_reference
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            def values(t, sign=1.):
                q, dq = joint_probe_reference(t, 30.)
                return {"arm_joint_1_q": sign*q[0], "arm_joint_2_q": sign*q[1],
                        "arm_joint_1_qd": sign*dq[0], "arm_joint_2_qd": sign*dq[1]}
            native_fixture(folder/"native_trace.csv", end=30., sample=values)
            report = native_joint_probe_metrics(folder, joint_probe_reference, 0., 30., [0., 0., 2.])
            self.assertTrue(report["passed"])
            native_fixture(folder/"native_trace.csv", end=30., sample=lambda t: values(t, -1.))
            self.assertFalse(native_joint_probe_metrics(folder, joint_probe_reference, 0., 30., [0., 0., 2.])["passed"])
            native_fixture(folder/"native_trace.csv", end=30.)
            self.assertFalse(native_joint_probe_metrics(folder, joint_probe_reference, 0., 30., [0., 0., 2.])["passed"])

    def test_state_rotates_child_twist_exactly_once(self):
        truth = ros_message()
        node = NS(data={"truth": truth, "joints": NS(header=truth.header)},
            joint_values=lambda: {j: NS(position_rad=.1*i, velocity_rad_s=.2*i) for i, j in enumerate(("arm_joint_1", "arm_joint_2"))})
        state, joint_stamp = state_from_node(node)
        np.testing.assert_allclose(state.velocity, [0., 1., 0.], atol=1e-14)
        np.testing.assert_array_equal(state.omega_body, [0., .2, 0.])
        self.assertEqual(joint_stamp, 1.)

    def test_flight_samples_keep_real_pose_and_no_native_tip_estimate(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            message = ros_message()
            node = NS(data={"position": object(), "status": NS(arming_state=2, failsafe=False, nav_state=14),
                            "land": NS(landed=False), "joints": NS(header=message.header)},
                      joint_values=lambda: {j: NS(position_rad=.01, velocity_rad_s=.02) for j in ("arm_joint_1", "arm_joint_2")})
            recorder = FlightRecorder(folder, model_sha256="a"*64, source_urdf_sha256="b"*64)
            recorder.context = NS(node=node)
            recorder.phase = "takeoff"
            recorder.receive(message)
            recorder.phase = "measure"
            recorder.receive(ros_message(1.05))
            recorder.receive(ros_message(1.05))
            meta = recorder.close(complete=True)
            self.assertTrue(meta["complete"])
            self.assertEqual(meta["sample_count"], 2)
            self.assertNotIn("tip_world_x_m", FlightRecorder.columns)
            self.assertIn("tip_target_world_x_m", FlightRecorder.columns)
            with (folder/"flight_samples.csv").open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["phase"], "takeoff")
            self.assertEqual(rows[1]["phase"], "measure")
            self.assertEqual(float(rows[0]["world_x_m"]), 1.)




# Independent comparison oracle copied from FlightRecorder immediately before
# this candidate. It retains rows and performs the original close-time np.diff
# and phase reduction; it does not call the new online-statistics implementation.
class FrozenRowRetentionReference:
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



class OnlineFlightAggregationTests(unittest.TestCase):
    def pair(self, folder, *, rate_hz=20.):
        recorders = []
        for name, cls in (("online", FlightRecorder), ("row_reference", FrozenRowRetentionReference)):
            directory = folder/name
            directory.mkdir()
            recorder = cls(directory, model_sha256="a"*64, source_urdf_sha256="b"*64, rate_hz=rate_hz)
            destroyed = []
            node = NS(data={"position": object(), "status": NS(arming_state=2, failsafe=False, nav_state=14),
                            "land": NS(landed=False), "joints": NS(header=ros_message().header)},
                      joint_values=lambda: {j: NS(position_rad=.01*i, velocity_rad_s=.02*i)
                                             for i, j in enumerate(JOINTS)},
                      destroy_subscription=destroyed.append)
            recorder.context = NS(node=node)
            recorder.subscriptions = ["truth_subscription", "clock_subscription"]
            recorder.destroyed_for_test = destroyed
            recorders.append(recorder)
        return recorders

    def feed(self, recorders, stamp, phase, *, target=True, nonfinite=False):
        for recorder in recorders:
            recorder.phase = phase
            recorder.target_body = np.array([.1, .2, .3]) if target else None
            recorder.tip_target = np.array([.4, .5, .6]) if target else None
            recorder.context.node.data["joints"] = NS(header=ros_message(stamp-.004).header)
            message = ros_message(stamp)
            if nonfinite:
                message.pose.pose.position.x = float("nan")
            recorder.receive(message)

    def compare_closed(self, recorders, *, complete=True):
        actual, reference = [recorder.close(complete=complete) for recorder in recorders]
        self.assertEqual(actual, reference)
        self.assertEqual(list(actual["phases"]), list(reference["phases"]))
        for name in ("flight_samples.csv", "flight_samples_manifest.json"):
            self.assertEqual((recorders[0].folder/name).read_bytes(),
                             (recorders[1].folder/name).read_bytes(), name)
        for recorder in recorders:
            self.assertEqual(recorder.destroyed_for_test, ["truth_subscription", "clock_subscription"])
            self.assertEqual(recorder.subscriptions, [])
        return actual

    def test_irregular_cadence_duplicates_and_phase_reentry_match_original_bytes_and_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorders = self.pair(Path(tmp))
            events = [(1., "takeoff"), (1.011, "not_sampled"), (1.05, "measure"),
                      (1.05, "duplicate"), (1.071, "not_sampled"),
                      (1.111, "takeoff"), (1.168, "measure"), (1.251, "landing"),
                      (1.305, "measure"), (1.49, "landing"), (1.501, "landed")]
            for index, (stamp, phase) in enumerate(events):
                self.feed(recorders, stamp, phase, target=bool(index % 2))
            result = self.compare_closed(recorders)
            self.assertEqual(list(result["phases"]), ["takeoff", "measure", "landing", "landed"])
            self.assertNotIn("not_sampled", result["phases"])
            self.assertNotIn("duplicate", result["phases"])
            self.assertTrue(result["complete"])
            self.assertGreater(result["interval_max_s"], result["interval_min_s"])

    def test_zero_one_and_incomplete_recordings_match_all_manifest_fields(self):
        for count, complete in ((0, True), (1, True), (2, False)):
            with self.subTest(count=count, complete=complete), tempfile.TemporaryDirectory() as tmp:
                recorders = self.pair(Path(tmp))
                for index in range(count):
                    self.feed(recorders, 1.+index*.1, "measure")
                result = self.compare_closed(recorders, complete=complete)
                self.assertFalse(result["complete"])
                self.assertEqual(result["sample_count"], count)
                if count < 2:
                    self.assertIsNone(result["interval_min_s"])
                    self.assertIsNone(result["interval_max_s"])

    def test_regression_and_nonfinite_sample_errors_remain_sticky_and_keep_later_valid_rows(self):
        for fault in ("regression", "nonfinite"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as tmp:
                recorders = self.pair(Path(tmp))
                self.feed(recorders, 1., "takeoff")
                self.feed(recorders, 1.1, "measure")
                self.feed(recorders, 1.05 if fault == "regression" else 1.15, "bad",
                          nonfinite=fault == "nonfinite")
                errors = [recorder.error for recorder in recorders]
                self.assertEqual(errors[0], errors[1])
                for recorder in recorders:
                    with self.assertRaisesRegex(RuntimeError, "Flight recording failed"):
                        recorder.check()
                self.feed(recorders, 1.3, "landing")
                self.assertEqual([recorder.error for recorder in recorders], errors)
                result = self.compare_closed(recorders)
                self.assertFalse(result["complete"])
                self.assertEqual(result["sample_count"], 3)
                self.assertNotIn("bad", result["phases"])

    def test_missing_telemetry_and_failed_target_conversion_preserve_acceptance_order(self):
        for fault in ("missing", "target"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as tmp:
                recorders = self.pair(Path(tmp))
                self.feed(recorders, 1., "takeoff")
                for recorder in recorders:
                    if fault == "missing":
                        del recorder.context.node.data["position"]
                    else:
                        recorder.target_body = ["not a number", 0., 0.]
                    recorder.phase = "omitted"
                    recorder.receive(ros_message(1.1))
                    recorder.context.node.data["position"] = object()
                self.feed(recorders, 1.2, "measure")
                result = self.compare_closed(recorders)
                self.assertEqual(result["sample_count"], 2)
                self.assertNotIn("omitted", result["phases"])
                self.assertEqual(result["capture_error"] is None, fault == "missing")

    def test_clock_error_does_not_change_csv_or_original_first_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorders = self.pair(Path(tmp))
            self.feed(recorders, 1., "takeoff")
            for recorder in recorders:
                recorder.receive_clock(NS(clock=NS(sec=5, nanosec=0)))
                recorder.receive_clock(NS(clock=NS(sec=4, nanosec=0)))
            self.feed(recorders, 1.1, "bad", nonfinite=True)
            self.feed(recorders, 1.2, "landing")
            result = self.compare_closed(recorders)
            self.assertEqual(result["capture_error"], "Simulation clock regressed during capture")
            self.assertEqual(result["sample_count"], 2)

    def test_writer_failure_is_not_counted_and_flush_failure_after_write_is_counted(self):
        class FailOnceWriter:
            def __init__(self, delegate):
                self.delegate, self.first = delegate, True
            def writerow(self, row):
                if self.first:
                    self.first = False
                    raise OSError("synthetic write failure")
                return self.delegate.writerow(row)
        class FailOnceFlush:
            def __init__(self, delegate):
                self.delegate, self.first = delegate, True
            def flush(self):
                if self.first:
                    self.first = False
                    raise OSError("synthetic flush failure")
                return self.delegate.flush()
            def close(self):
                return self.delegate.close()
        for fault in ("write", "flush"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as tmp:
                recorders = self.pair(Path(tmp))
                for recorder in recorders:
                    if fault == "write":
                        recorder.writer = FailOnceWriter(recorder.writer)
                    else:
                        recorder.stream = FailOnceFlush(recorder.stream)
                        recorder.last_flush = 0.
                with patch("recording.time.monotonic", return_value=2.):
                    self.feed(recorders, 1., "takeoff")
                    self.feed(recorders, 1.1, "measure")
                result = self.compare_closed(recorders)
                self.assertFalse(result["complete"])
                self.assertEqual(result["sample_count"], 1 if fault == "write" else 2)
                self.assertEqual(result["capture_error"], "synthetic "+fault+" failure")

    def test_close_can_repeat_and_post_close_receive_has_original_error_behavior(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorders = self.pair(Path(tmp))
            self.feed(recorders, 1., "measure")
            self.feed(recorders, 1.1, "landing")
            first = self.compare_closed(recorders)
            self.assertEqual(self.compare_closed(recorders), first)
            self.feed(recorders, 1.2, "after_close")
            final = self.compare_closed(recorders)
            self.assertFalse(final["complete"])
            self.assertEqual(final["sample_count"], 2)
            self.assertIsNotNone(final["capture_error"])
            self.assertNotIn("after_close", final["phases"])

    def test_retained_statistics_do_not_accumulate_rows_for_fixed_phase_vocabulary(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorders = self.pair(Path(tmp))
            vocabulary = ("ground_ready", "measure", "landing")
            for index in range(1000):
                self.feed(recorders, 1.+index*.06, vocabulary[index % len(vocabulary)], target=False)
            online = recorders[0]
            self.assertFalse(hasattr(online, "rows"))
            self.assertEqual(online._sample_count, 1000)
            self.assertEqual(len(online._phase_counts), len(vocabulary))
            self.assertTrue(all(isinstance(value, int) for value in online._phase_counts.values()))
            self.assertIsInstance(online._last_recorded_stamp, float)
            self.assertIsInstance(online._interval_min_s, float)
            self.assertIsInstance(online._interval_max_s, float)
            result = self.compare_closed(recorders)
            self.assertEqual(result["sample_count"], 1000)


if __name__ == "__main__":
    unittest.main()
