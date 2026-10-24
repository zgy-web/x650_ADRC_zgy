"""Synthetic evidence-contract tests only; no flight, GPU render or solver."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import uuid
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("isolated_video", ROOT / "scripts/render_x650_isolated_validation.py")
video = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(video)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2), encoding="utf-8")
    return path


class IsolatedVideoTests(unittest.TestCase):
    def directory(self):
        root = (ROOT / "tmp" / ("isolated_video_fixture_" + uuid.uuid4().hex)).resolve()
        root.mkdir(parents=True)
        self.addCleanup(lambda: shutil.rmtree(root))
        return root

    def fixture(self):
        root = self.directory()
        run, prep = root / "run", root / "prepared_ts20"
        run.mkdir(); prep.mkdir(); (root / "launch").mkdir()
        mesh = prep / "synthetic.stl"
        tetra = np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0]], [[0, 0, 0], [0, 0, 1], [1, 0, 0]],
                          [[0, 0, 0], [0, 1, 0], [0, 0, 1]], [[1, 0, 0], [0, 0, 1], [0, 1, 0]]], float) * .01
        video.replay.write_binary_stl(mesh, tetra)
        robot = ET.Element("robot", name="synthetic_two_link")
        for index, name in enumerate(("base_link", "arm_link_1", "arm_link_2")):
            link = ET.SubElement(robot, "link", name=name)
            geometry = ET.SubElement(ET.SubElement(link, "visual"), "geometry")
            if index == 0:
                ET.SubElement(geometry, "mesh", filename=mesh.name)
            else:
                ET.SubElement(geometry, "box", size=".25 .02 .02")
        for name, parent, child, xyz, angle in (
            ("arm_joint_1", "base_link", "arm_link_1", "0 .04 -.18", math.radians(150)),
            ("arm_joint_2", "arm_link_1", "arm_link_2", ".3 0 0", -math.radians(150))):
            joint = ET.SubElement(robot, "joint", name=name, type="revolute")
            ET.SubElement(joint, "parent", link=parent); ET.SubElement(joint, "child", link=child)
            ET.SubElement(joint, "origin", xyz=xyz, rpy=f"0 {angle:.17g} 0")
            ET.SubElement(joint, "axis", xyz="0 1 0")
        source = prep / "robot.urdf"
        ET.ElementTree(robot).write(source)
        model_hash = "a" * 64
        geometry_report = {"model_name": "x650_2link_ts20_damped_v1", "derived_urdf_sha256": video.sha(source),
            "physical_geometry_validated": True, "arm_self_collision_resolved": True,
            "mesh_resources": {mesh.name: video.sha(mesh)},
            "conditional_actuator_plant": {"sdf_sha256": model_hash, "rigid_geometry_reused_unchanged": True},
            "controller_model": {"mount_body": [0, 0, -.18], "elbow_offset_body_m": [0, .04, 0],
                                 "angle_offsets": [math.radians(150), 0], "lengths": [.3, .25]}}
        for name, field in (("clearance_report.json", "clearance_certificate_sha256"),
                            ("arm_self_collision_report.json", "arm_self_collision_report_sha256"),
                            ("kinematics.json", "kinematics_sha256")):
            file = write_json(prep / name, {"passed": True, "elbow_offset_body_m": [0, .04, 0]})
            geometry_report[field] = video.sha(file)
        preparation = write_json(prep / "preparation_report.json", geometry_report)
        params = {"amplitude_x": .25, "amplitude_z": .1, "period_s": 12., "cycles": 2, "start_s": 2., "ramp_s": 2.}
        task = {"scenario": "x650_isolated_figure8_v1", "stage": "normal", "controller": "mpc", "duration_s": 32.,
                "preparation_report": str(preparation), "trajectory_parameters": params}
        write_json(run / "controller_task.json", task)
        center = np.array([.4, .04, 1.2])
        reference = video.accepted.shared.FigureEight(center, **params)
        write_json(run / "isolated_live/live_start.json", {"origin_s": 100., "target_position_enu": center.tolist()})
        metadata = {"complete": True, "task": task, "stage": "normal", "controller": "mpc",
                    "source_urdf_sha256": video.sha(source), "model_sha256": model_hash,
                    "provenance": {"preparation_report_sha256": video.sha(preparation)}}
        write_json(run / "controller_metadata.json", metadata)
        path = run / "native_trace.csv"
        fields = ["sim_s", "valid_base", "valid_joint_state", "valid_tip", *["base_p" + a for a in "xyz"],
                  *["base_q" + a for a in "wxyz"], "arm_joint_1_q", "arm_joint_2_q", *["tip_" + a for a in "xyz"]]
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
            for t in np.linspace(0, 32, 4001):
                sim = 100. + float(t); t = sim - 100.
                joints = np.array([.06 * np.sin(t), -.04 * np.sin(t * .8)])
                relative = video.replay.tip_from_report(np.zeros(3), [1, 0, 0, 0], joints, geometry_report)
                base = reference.sample(t)[0] - relative + [.002, 0, 0]
                # A small independent physical/FK difference tests that displayed
                # native errors are never silently replaced by FK reconstruction.
                tip = base + relative + [0, 0, .00005]
                writer.writerow(dict(zip(fields, [sim, 1, 1, 1, *base, 1, 0, 0, 0, *joints, *tip])))
        write_json(run / "native_trace_manifest.json", {"complete": True, "sha256": video.sha(path),
            "source_urdf_sha256": video.sha(source), "model_sha256": model_hash})
        endpoint = video.accepted.shared.native_tip_metrics(run, reference.sample, 100., (0., 32.))
        cert = {"kind": "x650_isolated_normal_stage_certificate_v1", "stage": "normal", "passed": True,
                "complete": True, "full_window_ulog_verified": True, "parameter_restore_verified": True,
                "native_ground_verified": True, "actuator_log_verified": True,
                "source_run_directory": str(run), "launch_directory": str(root / "launch"),
                "window_sim_s": [100., 132.], "endpoint": endpoint,
                "artifacts": {str(p): video.sha(p) for p in run.rglob("*") if p.is_file()}}
        report = write_json(root / "audit/report.json", cert)
        return root, report

    def test_complete_native_evidence_and_dynamic_ts20_geometry_remain_bound(self):
        root, report = self.fixture()
        with patch("scipy.optimize.minimize", side_effect=AssertionError("No optimization during display")):
            result = video.load_recording(report, video.sha(report))
        self.assertEqual(result["geometry_report"]["model_name"], "x650_2link_ts20_damped_v1")
        self.assertEqual(len(result["data"]["t"]), 4001)
        self.assertEqual(result["data"]["elapsed"][[0, -1]].tolist(), [0., 32.])
        self.assertAlmostEqual(result["fk"]["recorded_vs_fk_maximum_error_m"], .00005, places=12)
        np.testing.assert_allclose(result["data"]["provided_tip"] - result["data"]["tip"],
                                   np.tile([0, 0, .00005], (4001, 1)), atol=1e-14)
        self.assertAlmostEqual(result["verified"]["metrics"]["position_rmse_m"], math.hypot(.002, .00005), places=12)
        self.assertEqual(result["source"], root / "prepared_ts20/robot.urdf")
        self.assertIn("mesh:synthetic.stl", result["inputs"])

    def test_failed_hold_or_changed_certificate_is_not_presented_as_normal(self):
        root = self.directory()
        report = write_json(root / "report.json", {"kind": "x650_isolated_live_stage_certificate_v1", "stage": "tip_hold"})
        with self.assertRaisesRegex(ValueError, "Only a complete"):
            video.load_recording(report)
        with self.assertRaisesRegex(ValueError, "explicit SHA256"):
            video.load_recording(report, "f" * 64)
        write_json(report, {"kind": "x650_isolated_normal_stage_certificate_v1", "stage": "normal", "passed": False})
        with self.assertRaisesRegex(ValueError, "complete passed"):
            video.load_recording(report)

    def test_geometry_changes_are_rejected_through_recorded_metadata(self):
        root, report = self.fixture()
        task = video.read(root / "run/controller_task.json")
        metadata = video.read(root / "run/controller_metadata.json")
        mesh = root / "prepared_ts20/synthetic.stl"
        mesh.write_bytes(mesh.read_bytes() + b"changed")
        with self.assertRaisesRegex(ValueError, "Recorded input changed"):
            video.geometry(task, metadata)
        preparation = root / "prepared_ts20/preparation_report.json"
        preparation.write_text(preparation.read_text() + "\n")
        with self.assertRaisesRegex(ValueError, "Recorded input changed"):
            video.geometry(task, metadata)

    def test_native_flags_quaternions_and_sample_correspondence_are_enforced(self):
        root, report = self.fixture()
        verified = video.accepted.verified_data(report)
        path = root / "run/native_trace.csv"
        with path.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        def replace():
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
        rows[100]["valid_joint_state"] = "0"; replace()
        with self.assertRaisesRegex(ValueError, "Invalid native"):
            video.native_states(verified)
        rows[100]["valid_joint_state"] = "1"; rows[100]["base_qw"] = "2"; replace()
        with self.assertRaisesRegex(ValueError, "unit length"):
            video.native_states(verified)
        rows[100]["base_qw"] = "1"; rows[100]["tip_x"] = "123"; replace()
        with self.assertRaisesRegex(ValueError, "exactly the same samples"):
            video.native_states(verified)

    def test_exact_duration_and_past_only_visual_sampling(self):
        times = video.frame_times(32.)
        self.assertEqual(len(times), 768)
        self.assertEqual(times[0], 0.)
        self.assertEqual(times[-1], 767 / 24)
        self.assertEqual(video.frame_times(32., preview_time=32.).tolist(), [32.])
        for duration, fps, preview in ((30., 24, None), (32., 30, None), (32., True, None),
                                       (32., 24, -1), (32., 24, 33), (32., 24, float("nan"))):
            with self.assertRaises(ValueError):
                video.frame_times(duration, fps, preview)
        native = np.array([.004, .008, .012, .016])
        self.assertEqual(video.past_indices(native, 0., 3).tolist(), [])
        ids = video.past_indices(native, .013, 2)
        self.assertEqual(ids.tolist(), [0, 2])
        self.assertTrue(np.all(native[ids] <= .013))

    def test_full_decode_verifies_dimensions_frame_count_and_timestamps(self):
        text = "#tb 0: 1/24\n#dimensions 0: 1600x1140\n" + "\n".join(
            f"0, {i}, {i}, 1, 2736000, {'a' * 32}" for i in range(768))
        checked = video.decoded_video_contract(text, 768)
        self.assertEqual(checked["duration_s"], 32.)
        self.assertTrue(checked["full_decode_passed"])
        for changed in (text.replace("1600x1140", "800x650"), text.replace("1/24", "1/30"),
                        text.rsplit("\n", 1)[0], text.replace("0, 4, 4,", "0, 4, 5,")):
            with self.assertRaises(ValueError):
                video.decoded_video_contract(changed, 768)

    def test_broken_encoder_pipe_still_reaps_and_closes_graphics(self):
        process, log, renderer = Mock(), Mock(), Mock()
        process.poll.return_value = None
        process.stdin.close.side_effect = BrokenPipeError("Synthetic failed encoder")
        video.close_media(process, log, renderer)
        process.terminate.assert_called_once()
        process.wait.assert_called_once_with(timeout=10)
        log.close.assert_called_once()
        renderer.close.assert_called_once()
        process, log, renderer = Mock(), Mock(), Mock()
        process.poll.return_value = None
        process.terminate.side_effect = OSError("Synthetic process teardown failure")
        with self.assertRaises(OSError):
            video.close_media(process, log, renderer)
        log.close.assert_called_once()
        renderer.close.assert_called_once()

    def test_audit_only_is_read_only_for_originals_and_never_opens_media_runtime(self):
        root, report = self.fixture()
        inputs = {str(p): video.sha(p) for p in root.rglob("*") if p.is_file()}
        args = SimpleNamespace(report=report, report_sha256=video.sha(report), output=root / "audit_only_output",
                               audit_only=True, preview_only=False, preview_time=10., fps=24, ffmpeg=None)
        with patch.object(video, "render_frames", side_effect=AssertionError("No media runtime")), \
             patch.object(video.subprocess, "Popen", side_effect=AssertionError("No subprocess")):
            record = video.render(args)
        self.assertTrue(record["complete"])
        self.assertFalse(record["video_produced"])
        self.assertFalse(record["eligible_as_stage_gate"])
        self.assertEqual(inputs, {file: video.sha(file) for file in inputs})
        self.assertTrue((args.output / "delivery_manifest.json").is_file())
        with self.assertRaisesRegex(ValueError, "new output"):
            video.render(args)


if __name__ == "__main__":
    unittest.main()
