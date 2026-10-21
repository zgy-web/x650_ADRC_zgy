"""Synthetic unit fixtures only; these checks do not represent native flights."""
from __future__ import annotations
import csv
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
import numpy as np

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
for p in (HERE, ROOT/"control_packages/x650_wholebody_v1"):
    sys.path.insert(0, str(p))
from recording import (NativeRecorder, FlightRecorder, native_ground_metrics, native_hover_metrics,
                       native_tip_metrics, native_joint_probe_metrics, sha, state_from_node)


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


if __name__ == "__main__":
    unittest.main()
