"""Synthetic code checks only: no simulated flight or experiment media claimed."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"scripts"))
SPEC = importlib.util.spec_from_file_location("x650_arm_replay", ROOT/"scripts/render_x650_arm_replay.py")
replay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay)
from prepare_x650_arm import original_arm, add_arm, set_origin, rotation


class X650ArmReplayTests(unittest.TestCase):
    def fixture(self, directory):
        # All fixture states and geometry are SYNTHETIC, used only by tests.
        source = directory/"robot.urdf"
        robot = ET.Element("robot", name="synthetic_x650_2link_v2")
        base = ET.SubElement(robot, "link", name="base_link")
        from prepare_x650_arm import set_properties
        set_properties(base, 3.02, np.zeros(3), np.diag([.1, .1, .15]))
        mesh = directory/"SYNTHETIC_fixture.stl"
        triangles = np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                              [[0, 0, 0], [0, 0, 1], [1, 0, 0]],
                              [[0, 0, 0], [0, 1, 0], [0, 0, 1]],
                              [[1, 0, 0], [0, 0, 1], [0, 1, 0]]], dtype=float)*.01
        replay.write_binary_stl(mesh, triangles)
        for link in [base, *[ET.SubElement(robot, "link", name=f"rotor_{i}") for i in range(4)]]:
            node = ET.SubElement(link, "visual")
            ET.SubElement(ET.SubElement(node, "geometry"), "mesh", filename=mesh.name, scale="1 1 1")
            if link is not base:
                joint = ET.SubElement(robot, "joint", name=link.get("name")+"_joint", type="continuous")
                ET.SubElement(joint, "parent", link="base_link")
                ET.SubElement(joint, "child", link=link.get("name"))
                ET.SubElement(joint, "axis", xyz="0 0 1")
                set_origin(joint, np.array([.2, .2, 0]))
        specs, _ = original_arm(ROOT/"assets/models/x500_underfold/model.sdf")
        add_arm(robot, specs)
        ET.ElementTree(robot).write(source)
        report = {"model_name": "x650_2link_v2", "derived_urdf_sha256": replay.sha(source),
                  "physical_geometry_validated": True, "arm_self_collision_resolved": True,
                  "mesh_resources": {mesh.name: replay.sha(mesh)},
                  "controller_model": {"mount_body": [0, 0, -.18], "elbow_offset_body_m": [0, .04, 0],
                                       "angle_offsets": [math.radians(150), 0], "lengths": [.30, .25]}}
        for name, field in (("clearance_report.json", "clearance_certificate_sha256"),
                            ("arm_self_collision_report.json", "arm_self_collision_report_sha256"),
                            ("kinematics.json", "kinematics_sha256")):
            file = directory/name
            file.write_text(json.dumps({"passed": True, "elbow_offset_body_m": [0, .04, 0]}))
            report[field] = replay.sha(file)
        source.with_name("preparation_report.json").write_text(json.dumps(report))
        return source, robot, report

    def csv(self, path, report, include_tip=True, bias=0):
        fields = [*replay.REQUIRED, "model_sha256", *replay.TARGET]
        if include_tip: fields.extend(replay.TIP)
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for i, t in enumerate([100., 100.1, 100.2]):
                position, quat, q = np.array([.1*i, 0, 2]), np.array([1, 0, 0, 0]), np.array([.01*i, .02*i])
                tip = replay.tip_from_report(position, quat, q, report)
                row = dict(zip(replay.REQUIRED, [t, "track", *position, *quat[1:], quat[0], *q]))
                row["model_sha256"] = "synthetic-model-identity"
                row.update(dict(zip(replay.TARGET, tip)))
                if include_tip: row.update(dict(zip(replay.TIP, tip+[bias, 0, 0])))
                writer.writerow(row)

    def test_quaternion_conversion_including_half_turns(self):
        for angles in ([0, 0, 0], [math.pi, 0, 0], [0, math.pi, 0], [0, 0, math.pi], [.2, -.3, 1.5]):
            expected = rotation(angles)
            quat = replay.rotation_quaternion(expected)
            np.testing.assert_allclose(replay.quaternion_rotation(quat), expected, atol=1e-14)

    def test_every_csv_sample_fk_is_verified_with_lateral_offset(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            source, robot, report = self.fixture(directory)
            path = directory/"SYNTHETIC_states.csv"; self.csv(path, report)
            data = replay.load_samples(path)
            audit = replay.audit_forward_kinematics(data, robot, report)
            self.assertTrue(audit["passed"])
            self.assertTrue(audit["all_samples_audited"])
            self.assertEqual(audit["samples"], 3)
            np.testing.assert_allclose(data["tip"][:, 1], .04, atol=1e-15)
            self.assertLess(audit["report_vs_urdf_maximum_error_m"], 1e-14)
            self.csv(path, report, bias=.001)
            with self.assertRaisesRegex(ValueError, "Logged tip disagrees"):
                replay.audit_forward_kinematics(replay.load_samples(path), robot, report)

    def test_geometry_evidence_checksums_and_pass_flags_required(self):
        with tempfile.TemporaryDirectory() as temp:
            source, _, _ = self.fixture(Path(temp))
            replay.load_composition(source)
            file = source.with_name("arm_self_collision_report.json")
            file.write_text(json.dumps({"passed": False}))
            with self.assertRaisesRegex(ValueError, "checksum mismatch"):
                replay.load_composition(source)

    def test_scene_has_only_mocap_display_and_all_visuals(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            source, robot, _ = self.fixture(directory)
            scene, audit = replay.make_scene(directory/"scene", source, robot)
            mjcf = ET.parse(scene).getroot()
            self.assertEqual(audit["visual_mesh_count"], 5)
            self.assertEqual(audit["primitive_visual_count"], 3)
            self.assertEqual(audit["original_triangle_count"], 20)
            self.assertEqual(audit["mocap_link_count"], 7)
            self.assertFalse(audit["new_dynamics_simulated"])
            self.assertEqual(len(mjcf.findall("worldbody/body[@mocap='true']")), 8)
            self.assertFalse(mjcf.findall(".//joint"))
            self.assertFalse(mjcf.findall(".//actuator"))
            self.assertTrue(all(g.get("contype") == "0" for g in mjcf.findall(".//geom")))
            self.assertEqual(mjcf.find("statistic").get("extent"), "2")
            self.assertEqual(mjcf.find("visual/map").get("znear"), ".002")
            marker = mjcf.find("worldbody/body[@name='replay_tip_reference']")
            self.assertEqual(marker.get("pos"), "0 0 0")
            self.assertTrue(all(g.get("rgba").endswith(" 0") for g in marker.findall("geom")))

    def test_camera_targets_follow_actual_translated_rotated_body_and_arm(self):
        with tempfile.TemporaryDirectory() as temp:
            _, _, report = self.fixture(Path(temp))
            p, quat, q = np.array([1., 2., 3.]), np.array([2**-.5, 0, 0, 2**-.5]), [.06, -.04]
            main, close = replay.camera_targets(p, quat, q, report)
            np.testing.assert_allclose(main, p+[0., 0., -.12], atol=1e-14)
            offset = np.array([4., -3., 2.])
            shifted = replay.camera_targets(p+offset, quat, q, report)
            np.testing.assert_allclose(shifted[0]-main, offset, atol=1e-14)
            np.testing.assert_allclose(shifted[1]-close, offset, atol=1e-14)
            self.assertLess(np.linalg.norm(close-p), .6)

    def test_joint_probe_reference_exists_only_in_actual_measure_window(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            t = np.array([0., 1., 3., 8., 13., 17., 18.])
            data = dict(t=t, phase=np.array(["ground", "measure", "measure", "measure", "measure", "measure", "landing"]),
                        p=np.tile([1., 2., 3.], (len(t), 1)), q=np.tile([1., 0., 0., 0.], (len(t), 1)),
                        joint_q=np.zeros((len(t), 2)))
            metadata = dict(complete=True, stage="joint_probe", model_sha256="actual-model",
                            source_urdf_sha256="actual-source", task_start_sim_s=1., requested_duration_s=16.,
                            reference_base_enu_m=[1., 2., 3.])
            path = directory/"controller_metadata.json"; path.write_text(json.dumps(metadata))
            context = replay.joint_probe_context(data, directory/"flight_samples.csv", {"metadata": metadata})
            self.assertTrue(np.isnan(context["q_reference"][[0, -1]]).all())
            self.assertTrue(np.isfinite(context["q_reference"][1:-1]).all())
            self.assertTrue(np.isnan(context["body_error"][[0, -1]]).all())
            self.assertFalse(context["metadata"]["primary_control_metric"])
            metadata["model_sha256"] = "different-model"; path.write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError, "identity"):
                replay.joint_probe_context(data, directory/"flight_samples.csv", {"metadata": {"model_sha256": "actual-model"}})

    def test_joint_and_base_share_gap_hold_policy(self):
        data = {"elapsed": np.array([0., .1]), "p": np.array([[0., 0, 0], [2., 4, 6]]),
                "q": np.array([[1., 0, 0, 0], [0., 0, 0, 1]]), "joint_q": np.array([[0., 0], [1., 2.]])}
        p, quat, q, _, gap = replay.sample_state(data, .05)
        np.testing.assert_allclose(p, [1, 2, 3])
        np.testing.assert_allclose(q, [.5, 1.])
        np.testing.assert_allclose(quat, [2**-.5, 0, 0, 2**-.5])
        self.assertFalse(gap)
        data["elapsed"][1] = 1.
        p, quat, q, _, gap = replay.sample_state(data, .5)
        self.assertTrue(gap)
        np.testing.assert_array_equal(p, data["p"][0])
        np.testing.assert_array_equal(quat, data["q"][0])
        np.testing.assert_array_equal(q, data["joint_q"][0])

    def test_final_capture_identity_required(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            source, _, report = self.fixture(directory)
            path = directory/"SYNTHETIC_states.csv"; self.csv(path, report)
            data = replay.load_samples(path)
            manifest = {"complete": True, "file": path.name, "sha256": replay.sha(path),
                        "source_urdf_sha256": replay.sha(source), "model_sha256": "synthetic-model-identity"}
            capture = directory/"flight_samples_manifest.json"
            capture.write_text(json.dumps(manifest))
            replay.verify_recording(path, source, data)
            manifest["synthetic"] = True; capture.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, "Synthetic captures"):
                replay.verify_recording(path, source, data)

    def test_ros_recorder_blank_references_and_manifest_only_model_identity(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            source, _, report = self.fixture(directory)
            path = directory/"recorded_ros_schema.csv"; self.csv(path, report)
            with path.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            fields = [name for name in rows[0] if name != "model_sha256"]
            for row in rows:
                row.pop("model_sha256")
            for field in replay.TARGET:
                rows[0][field] = ""
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader(); writer.writerows(rows)
            data = replay.load_samples(path)
            self.assertTrue(np.isnan(data["target"][0]).all())
            self.assertTrue(np.isfinite(data["target"][1:]).all())
            manifest = {"complete": True, "file": path.name, "sha256": replay.sha(path),
                        "source_urdf_sha256": replay.sha(source), "model_sha256": "actual-model-bound-to-csv-sha"}
            (directory/"flight_samples_manifest.json").write_text(json.dumps(manifest))
            self.assertEqual(replay.verify_recording(path, source, data)["model_identity_source"], "finalized manifest")


if __name__ == "__main__":
    unittest.main()
