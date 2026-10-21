from __future__ import annotations

import copy
import hashlib
import json
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import yaml

from amsim.modeling.urdf_flight import bind_flight_profile


class FlightProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / "amsim_urdf_test"
        self.source.mkdir()
        (self.source / "meshes").mkdir()
        (self.source / "meshes" / "test.stl").write_text("fixture", encoding="utf-8")
        self.profile_path = self.root / "profile.yaml"
        sdf = ET.Element("sdf", version="1.9")
        model = ET.SubElement(sdf, "model", name="amsim_urdf_test")
        base = ET.SubElement(model, "link", name="base_link")
        inertial = ET.SubElement(base, "inertial")
        ET.SubElement(inertial, "mass").text = "7.77"
        visual = ET.SubElement(base, "visual", name="mesh")
        geometry = ET.SubElement(visual, "geometry")
        mesh = ET.SubElement(geometry, "mesh")
        ET.SubElement(mesh, "uri").text = "model://amsim_urdf_test/meshes/test.stl"
        self.manifest = {"schema": {"kind": "urdf_import", "version": 1}, "root_link": "base_link", "flight_ready": False,
                         "links": ["base_link"], "joints": []}
        self.profile = {"schema_version": 1, "type": "px4_multicopter", "base_link": "base_link", "body_frame": "FLU",
                        "px4_sys_autostart": 4999, "controlled_joints": [], "rotors": [],
                        "notes": "Synthetic unit fixture only; no real airframe calibration."}
        for number in range(4):
            link, joint_name = f"prop_{number}", f"spin_{number}"
            ET.SubElement(model, "link", name=link)
            joint = ET.SubElement(model, "joint", name=joint_name, type="revolute")
            ET.SubElement(joint, "parent").text = "base_link"
            ET.SubElement(joint, "child").text = link
            axis = ET.SubElement(joint, "axis")
            ET.SubElement(axis, "xyz").text = "0 0 1"
            limit = ET.SubElement(axis, "limit")
            ET.SubElement(limit, "lower").text = "-1e16"
            ET.SubElement(limit, "upper").text = "1e16"
            self.manifest["links"].append(link)
            self.manifest["joints"].append({"name": joint_name, "type": "continuous", "parent": "base_link", "child": link,
                                             "origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}, "axis": [0, 0, 1],
                                             "axis_in_root_at_zero": [0, 0, 1], "limits": {}})
            self.profile["rotors"].append({"motor_number": number, "joint": joint_name, "link": link,
                "turning_direction": "cw" if number % 2 else "ccw", "motor_constant": 1.2e-5 + number*1e-7,
                "moment_constant": .027, "max_rot_velocity": 833., "time_constant_up": .013,
                "time_constant_down": .031, "rotor_drag_coefficient": 9.2e-5,
                "rolling_moment_coefficient": 1.2e-6, "rotor_velocity_slowdown_sim": 7.})
        self.tree = ET.ElementTree(sdf)
        self.save_source()
        self.save_profile()

    def save_source(self):
        self.tree.write(self.source / "model.sdf", encoding="utf-8", xml_declaration=True)
        self.manifest["model_sha256"] = hashlib.sha256((self.source / "model.sdf").read_bytes()).hexdigest()
        (self.source / "import_manifest.json").write_text(json.dumps(self.manifest), encoding="utf-8")

    def save_profile(self):
        self.profile_path.write_text(yaml.safe_dump(self.profile), encoding="utf-8")

    def bind(self):
        self.save_profile()
        with patch("amsim.modeling.urdf_flight.subprocess.run", return_value=subprocess.CompletedProcess([], 0, "Valid.\n", "")) as native:
            result = bind_flight_profile(self.source, self.profile_path)
        self.assertEqual(native.call_args.args[0][:3], ["gz", "sdf", "-k"])
        return result

    def test_binding_copies_only_sensor_settings_and_explicit_physics(self):
        original = {p.relative_to(self.source).as_posix(): p.read_bytes() for p in self.source.rglob("*") if p.is_file()}
        result = self.bind()
        self.assertEqual(result.name, "amsim_urdf_test_flight")
        self.assertEqual(result.joints, [])
        model = ET.parse(result.sdf_path).getroot().find("model")
        self.assertEqual(model.findtext("link[@name='base_link']/inertial/mass"), "7.77")
        self.assertEqual(len(model.findall("link[@name='base_link']/sensor")), 4)
        motors = model.findall("plugin[@name='gz::sim::systems::MulticopterMotorModel']")
        self.assertEqual(len(motors), 4)
        self.assertEqual(float(motors[2].findtext("motorConstant")), self.profile["rotors"][2]["motor_constant"])
        self.assertEqual(motors[2].findtext("jointName"), "spin_2")
        self.assertEqual(motors[2].findtext("turningDirection"), "ccw")
        self.assertEqual(model.findtext(".//mesh/uri"), "model://amsim_urdf_test_flight/meshes/test.stl")
        self.assertEqual((result.output_dir / "meshes/test.stl").read_text(), "fixture")
        manifest = json.loads(result.manifest_path.read_text())
        self.assertTrue(manifest["flight_ready"])
        self.assertFalse(manifest["flight_validated"])
        self.assertEqual(manifest["px4_sys_autostart"], 4999)
        self.assertEqual(manifest["source_manifest_sha256"], hashlib.sha256(original["import_manifest.json"]).hexdigest())
        self.assertIn("meshes/test.stl", manifest["resources"])
        for name, data in original.items():
            self.assertEqual((self.source / name).read_bytes(), data)
        with self.assertRaises(FileExistsError):
            self.bind()

    def test_all_propulsion_fields_required_no_x500_defaults(self):
        fields = ("motor_constant", "moment_constant", "max_rot_velocity", "time_constant_up", "time_constant_down",
                  "rotor_drag_coefficient", "rolling_moment_coefficient", "rotor_velocity_slowdown_sim")
        pristine = copy.deepcopy(self.profile)
        for field in fields:
            with self.subTest(field=field):
                self.profile = copy.deepcopy(pristine)
                del self.profile["rotors"][0][field]
                with self.assertRaisesRegex(ValueError, field):
                    self.bind()
        self.assertFalse((self.root / "amsim_urdf_test_flight").exists())

    def test_source_angle_limit_requires_explicit_conversion(self):
        self.manifest["joints"][0]["type"] = "revolute"
        self.manifest["joints"][0]["limits"] = {"lower": -.5, "upper": .5}
        joint = self.tree.getroot().find("model/joint[@name='spin_0']")
        joint.find("axis/limit/lower").text = "-.5"
        joint.find("axis/limit/upper").text = ".5"
        self.save_source()
        with self.assertRaisesRegex(ValueError, "allow_continuous_conversion"):
            self.bind()
        self.profile["rotors"][0]["allow_continuous_conversion"] = True
        result = self.bind()
        output = ET.parse(result.sdf_path).getroot()
        self.assertEqual(float(output.findtext("model/joint[@name='spin_0']/axis/limit/lower")), -1e16)
        changes = json.loads(result.manifest_path.read_text())["continuous_conversions"]
        self.assertEqual(changes[0]["original_source_limits"], {"lower": -.5, "upper": .5})

    def test_rejects_wrong_frame_axis_and_topology(self):
        for change, message in (
            (lambda: self.profile.update(body_frame="FRD"), "FLU"),
            (lambda: self.profile.update(base_link="body"), "base_link"),
            (lambda: self.manifest["joints"][0].update(parent="other"), "root-attached"),
            (lambda: self.manifest["joints"][0].update(axis_in_root_at_zero=[0, -1, 0]), "source joint axis"),
        ):
            pristine_profile, pristine_manifest = copy.deepcopy(self.profile), copy.deepcopy(self.manifest)
            with self.subTest(message=message):
                change()
                self.save_source()
                with self.assertRaisesRegex(ValueError, message):
                    self.bind()
            self.profile, self.manifest = pristine_profile, pristine_manifest

    def test_thrust_axis_rejected_even_with_world_aligned_joint_axis(self):
        model = self.tree.getroot().find("model")
        ET.SubElement(model.find("link[@name='prop_0']"), "pose").text = "0 0 0 0 0.3 0"
        axis = model.find("joint[@name='spin_0']/axis/xyz")
        axis.set("expressed_in", "base_link")
        self.save_source()
        with self.assertRaisesRegex(ValueError, "thrust axis"):
            self.bind()

    def test_sdf_joint_axis_uses_joint_frame_not_assumed_model_frame(self):
        joint = self.tree.getroot().find("model/joint[@name='spin_0']")
        ET.SubElement(joint, "pose", relative_to="base_link").text = "0 0 0 1.5707963267948966 0 0"
        joint.find("axis/xyz").text = "0 1 0"
        self.save_source()
        self.bind()

    def test_rejects_plugin_and_sensor_duplicates_including_empty_elements(self):
        model = self.tree.getroot().find("model")
        for tag in ("plugin", "sensor"):
            node = ET.SubElement(model, tag)
            self.save_source()
            with self.subTest(tag=tag), self.assertRaisesRegex(ValueError, "already contains"):
                self.bind()
            model.remove(node)

    def test_invalid_motor_numbers_physics_and_missing_provenance(self):
        changes = (
            lambda p: p["rotors"][0].update(motor_number=2),
            lambda p: p["rotors"][0].update(motor_number=True),
            lambda p: p["rotors"][0].update(motor_constant=float("nan")),
            lambda p: p["rotors"][0].update(moment_constant=-.1),
            lambda p: p["rotors"][0].update(time_constant_up=0),
            lambda p: p["rotors"][0].update(max_rot_velocity="833"),
            lambda p: p["rotors"][0].update(turning_direction="unknown"),
            lambda p: p.update(px4_sys_autostart=True),
            lambda p: p.pop("px4_sys_autostart"),
            lambda p: p.update(notes=""),
            lambda p: p.update(controlled_joints=["spin_0"]),
        )
        pristine = copy.deepcopy(self.profile)
        for i, change in enumerate(changes):
            self.profile = copy.deepcopy(pristine)
            change(self.profile)
            with self.subTest(case=i), self.assertRaises(ValueError):
                self.bind()

    def test_controlled_joint_plugins_and_initial_state(self):
        model = self.tree.getroot().find("model")
        ET.SubElement(model, "link", name="arm")
        joint = ET.SubElement(model, "joint", name="arm_joint", type="revolute")
        ET.SubElement(joint, "parent").text = "base_link"
        ET.SubElement(joint, "child").text = "arm"
        self.manifest["joints"].append({"name": "arm_joint", "parent": "base_link", "child": "arm", "type": "revolute"})
        self.profile["controlled_joints"] = ["arm_joint"]
        self.save_source()
        result = self.bind()
        output = ET.parse(result.sdf_path).getroot().find("model")
        self.assertEqual(result.initial_joint_positions, {"arm_joint": 0.})
        self.assertEqual(output.findtext("plugin[@name='gz::sim::systems::JointController']/joint_name"), "arm_joint")
        self.assertIsNotNone(output.find("plugin[@name='gz::sim::systems::JointStatePublisher']"))

    def test_native_validator_failure_and_missing_gazebo_publish_nothing(self):
        for failure in (subprocess.CompletedProcess([], 1, "", "Invalid SDF"), FileNotFoundError("gz")):
            with self.subTest(failure=str(failure)):
                kwargs = {"side_effect": failure} if isinstance(failure, Exception) else {"return_value": failure}
                with patch("amsim.modeling.urdf_flight.subprocess.run", **kwargs), self.assertRaises(RuntimeError):
                    bind_flight_profile(self.source, self.profile_path)
                self.assertFalse((self.root / "amsim_urdf_test_flight").exists())
                self.assertEqual(list(self.root.glob(".amsim-flight-*")), [])

    def test_source_sdf_tampering_is_rejected(self):
        with (self.source / "model.sdf").open("a") as output:
            output.write(" ")
        with self.assertRaisesRegex(ValueError, "hash differs"):
            self.bind()

    def test_output_cannot_be_nested_in_source(self):
        with self.assertRaisesRegex(ValueError, "outside the source"):
            bind_flight_profile(self.source, self.profile_path, output_root=self.source / "derived")
        self.assertFalse((self.source / "derived").exists())

    def test_importer_artifact_mapping_is_verified_including_meshes(self):
        self.manifest["artifacts"] = {p.relative_to(self.source).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                      for p in self.source.rglob("*") if p.is_file() and p.name != "import_manifest.json"}
        self.manifest.pop("model_sha256")
        (self.source / "import_manifest.json").write_text(json.dumps(self.manifest), encoding="utf-8")
        (self.source / "meshes/test.stl").write_text("tampered", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "source artifact"):
            self.bind()

    def test_motor_slowdown_must_fit_source_and_sdf_joint_speed_limits(self):
        source = self.manifest["joints"][0]
        source["limits"]["velocity"] = 5.
        joint = self.tree.getroot().find("model/joint[@name='spin_0']")
        ET.SubElement(joint.find("axis/limit"), "velocity").text = "5"
        self.profile["rotors"][0].update(max_rot_velocity=100., rotor_velocity_slowdown_sim=10.)
        self.save_source()
        with self.assertRaisesRegex(ValueError, "source velocity limit 5.*below"):
            self.bind()
        # Checking only source metadata would miss an incompatible SDF limit.
        source["limits"]["velocity"] = 20.
        self.save_source()
        with self.assertRaisesRegex(ValueError, "SDF velocity limit 5.*below"):
            self.bind()
        self.profile["rotors"][0]["rotor_velocity_slowdown_sim"] = 20.
        result = self.bind()
        check = json.loads(result.manifest_path.read_text())["rotor_velocity_checks"][0]
        self.assertEqual(check["maximum_simulated_joint_speed_rad_s"], 5.)
        self.assertEqual(check["sdf_velocity_limit_rad_s"], 5.)

    def test_continuous_conversion_does_not_override_speed_limit(self):
        source = self.manifest["joints"][0]
        source.update(type="revolute", limits={"lower": -1.57, "upper": 1.57, "velocity": 5.})
        joint = self.tree.getroot().find("model/joint[@name='spin_0']")
        ET.SubElement(joint.find("axis/limit"), "velocity").text = "5"
        self.profile["rotors"][0]["allow_continuous_conversion"] = True
        self.save_source()
        with self.assertRaisesRegex(ValueError, "continuous conversion only changes angle limits"):
            self.bind()

    def test_prismatic_joint_cannot_bind_to_angular_arm_api(self):
        model = self.tree.getroot().find("model")
        ET.SubElement(model, "link", name="slider")
        joint = ET.SubElement(model, "joint", name="slider_joint", type="prismatic")
        ET.SubElement(joint, "parent").text = "base_link"
        ET.SubElement(joint, "child").text = "slider"
        self.manifest["joints"].append({"name": "slider_joint", "parent": "base_link", "child": "slider", "type": "prismatic"})
        self.profile["controlled_joints"] = ["slider_joint"]
        self.save_source()
        with self.assertRaisesRegex(ValueError, "prismatic ArmAPI control is unsupported"):
            self.bind()


if __name__ == "__main__":
    unittest.main()
