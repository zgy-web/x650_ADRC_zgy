"""Independent physical checks of the separate X650+arm PX4 airframe derivation.

Fixtures are small explicit composition contracts. These tests exercise arithmetic
and refusal behavior; they do not establish Gazebo contact or flight stability.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import tempfile
import unittest

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "x650_arm_airframe_under_test", ROOT / "scripts/prepare_x650_arm_airframe.py")
generator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(generator)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parse_parameters(text):
    result = {}
    for name, value in re.findall(r"^param set (\w+) ([^\s#]+)", text, flags=re.MULTILINE):
        if name in result:
            raise AssertionError("Duplicate parameter: " + name)
        result[name] = float(value)
    return result


class X650ArmAirframeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="x650_arm_airframe_")
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.report_path = self.folder / "preparation_report.json"
        # Deliberately nonzero body COM and lateral arm offset: using no-arm
        # allocation coordinates or subtracting the wrong COM must be detectable.
        masses = np.array([3.02, .025, .25, .20])
        positions = np.array([[.001651210684, .002125143399, -.063341221526],
                              [0., 0., -.1646],
                              [-.15*math.sqrt(3)/2, 0., -.18-.15/2],
                              [-.30*math.sqrt(3)/2+.125, .04, -.18-.30/2]])
        self.mass = float(masses.sum())
        self.com = masses @ positions / self.mass
        self.profile = yaml.safe_load((ROOT / "configs/vehicles/x650_no_arm/flight_profile.yaml").read_text(encoding="utf-8"))
        self.profile["px4_sys_autostart"] = 4998
        self.profile["controlled_joints"] = ["arm_joint_1", "arm_joint_2"]
        centres = [[.22990664, -.22990664, 0.], [-.22990664, .22990664, 0.],
                   [.22990664, .22990664, 0.], [-.22990664, -.22990664, 0.]]
        mappings = []
        for rotor, centre in zip(self.profile["rotors"], centres):
            mappings.append({"motor_number": rotor["motor_number"], "link": rotor["link"],
                             "turning_direction": rotor["turning_direction"],
                             "position_flu_m": centre,
                             "position_com_frd_m": ((np.array(centre)-self.com)*[1,-1,-1]).tolist()})
        # The arithmetic fixture has hashed source bytes but makes no claim
        # that this tiny XML stub is the full X650 geometry.
        (self.folder / "robot.urdf").write_bytes(b"<robot name='explicit_unit_fixture'/>\n")
        self.clearance = {"kind": "continuous_arm_limit_clearance_certificate", "passed": True,
                          "arm_self_collision": {"passed": True}}
        self.self_collision = {"kind": "arm_self_collision_certificate", "passed": True}
        self.report = {"schema_version": 1, "kind": "x650_original_two_link_composition",
                       "model_name": "x650_2link_v2", "rigid_geometry_and_inertia_validated": True,
                       "physical_geometry_validated": True, "arm_self_collision_resolved": True,
                       "px4_sys_autostart": 4998, "base_airframe_mass_kg": 3.02,
                       "two_link_mass_kg": .45, "adapter": {"mass_kg": .025},
                       "total_mass_kg": self.mass, "com_flu_q0_m": self.com.tolist(),
                       "motor_mapping_q0": mappings,
                       "native_ground_contact_validated": False, "flight_validated": False}
        self.save(refresh_hashes=True)

    def save(self, *, refresh_hashes=False):
        (self.folder / "flight_profile.yaml").write_text(yaml.safe_dump(self.profile), encoding="utf-8")
        (self.folder / "clearance_report.json").write_text(json.dumps(self.clearance), encoding="utf-8")
        (self.folder / "arm_self_collision_report.json").write_text(json.dumps(self.self_collision), encoding="utf-8")
        if refresh_hashes:
            for key, name in (("derived_urdf_sha256", "robot.urdf"),
                              ("clearance_certificate_sha256", "clearance_report.json"),
                              ("arm_self_collision_report_sha256", "arm_self_collision_report.json"),
                              ("flight_profile_sha256", "flight_profile.yaml")):
                self.report[key] = digest(self.folder / name)
        self.report_path.write_text(json.dumps(self.report), encoding="utf-8")

    def derive(self):
        text, metadata = generator.derive(self.report_path)
        return text, metadata, parse_parameters(text)

    def test_mass_hover_and_folded_com_are_recomputed_for_arm_vehicle(self):
        _, meta, params = self.derive()
        self.assertAlmostEqual(self.mass, 3.495, places=12)
        self.assertAlmostEqual(meta["total_mass_kg"], self.mass, places=12)
        np.testing.assert_allclose(meta["com_flu_q0_m"], self.com, atol=0., rtol=0.)
        self.assertGreater(self.com[1], .004)  # lateral elbow offset contributes to COM.
        force = sum(rotor["motor_constant"]*params[f"SIM_GZ_EC_MAX{i+1}"]**2
                    * params["MPC_THR_HOVER"] for i, rotor in enumerate(self.profile["rotors"]))
        self.assertAlmostEqual(force, self.mass*9.80665, places=9)
        self.assertGreater(params["MPC_THR_HOVER"], .3888125521)  # old no-arm initializer.

    def test_static_trim_balances_force_and_moment_without_saturation(self):
        _, meta, params = self.derive()
        axis = np.array([0., 0., -1.])
        columns = []
        for i in range(4):
            p = np.array([params[f"CA_ROTOR{i}_{key}"] for key in ("PX", "PY", "PZ")])
            ct, km = params[f"CA_ROTOR{i}_CT"], params[f"CA_ROTOR{i}_KM"]
            columns.append(np.r_[ct*np.cross(p, axis)-ct*km*axis, -ct])
        matrix = np.column_stack(columns)
        fractions = np.array(meta["static_trim_motor_thrust_fractions"])
        self.assertTrue(np.all(fractions > 0.))
        self.assertTrue(np.all(fractions < 1.))
        self.assertGreater(float(np.ptp(fractions)), .01)  # COM needs differential trim.
        np.testing.assert_allclose(matrix@fractions, [0.,0.,0.,-self.mass*9.80665], atol=2e-10, rtol=0.)
        self.assertEqual(meta["effectiveness_rank_after_threshold"], 4)
        self.assertGreater(np.max(np.abs(matrix[2])), .05)
        np.testing.assert_allclose(meta["static_trim_residual"], 0., atol=1e-12, rtol=0.)

    def test_motor_mapping_output_units_and_new_model_id_are_preserved(self):
        text, meta, params = self.derive()
        self.assertEqual(meta["airframe_id"], 4998)
        self.assertEqual(meta["model"], "amsim_urdf_x650_2link_v2_flight")
        self.assertIn("PX4_SIM_MODEL:="+meta["model"], text)
        self.assertEqual(params["THR_MDL_FAC"], 1.)
        self.assertEqual(params["CA_ROTOR_COUNT"], 4.)
        for i, (mapping, rotor) in enumerate(zip(self.report["motor_mapping_q0"], self.profile["rotors"])):
            self.assertEqual(params[f"SIM_GZ_EC_FUNC{i+1}"], 101+i)
            self.assertEqual(params[f"SIM_GZ_EC_MIN{i+1}"], 0.)
            self.assertEqual(params[f"SIM_GZ_EC_DIS{i+1}"], 0.)
            self.assertEqual(params[f"SIM_GZ_EC_MAX{i+1}"], 628.)
            self.assertLessEqual(params[f"SIM_GZ_EC_MAX{i+1}"], rotor["max_rot_velocity"])
            np.testing.assert_allclose([params[f"CA_ROTOR{i}_{key}"] for key in ("PX","PY","PZ")],
                                       (np.array(mapping["position_flu_m"])-self.com)*[1,-1,-1], atol=5e-13, rtol=0.)
            self.assertAlmostEqual(params[f"CA_ROTOR{i}_CT"], rotor["motor_constant"]*628**2, places=9)
            self.assertAlmostEqual(params[f"CA_ROTOR{i}_KM"], [.016,.016,-.016,-.016][i], places=12)
        for channel in range(5,9):
            self.assertEqual(params[f"SIM_GZ_EC_FUNC{channel}"], 0.)

    def test_derivation_keeps_input_bytes_and_does_not_claim_flight_validation(self):
        before = {p.name: p.read_bytes() for p in self.folder.iterdir()}
        template_before = generator.TEMPLATE.read_bytes()
        text, meta, _ = self.derive()
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.folder.iterdir()})
        self.assertEqual(template_before, generator.TEMPLATE.read_bytes())
        self.assertFalse(meta["flight_started"])
        self.assertFalse(meta["flight_validated"])
        self.assertFalse(meta["native_ground_contact_validated"])
        self.assertEqual(meta["airframe_sha256"], hashlib.sha256(text.encode()).hexdigest())
        self.assertEqual(meta["composition_report_sha256"], digest(self.report_path))
        self.assertEqual(meta["flight_profile_sha256"], digest(self.folder/"flight_profile.yaml"))

    def test_urdf_or_clearance_bytes_tampering_is_rejected(self):
        for filename in ("robot.urdf", "clearance_report.json"):
            with self.subTest(filename=filename):
                path = self.folder / filename
                original = path.read_bytes()
                path.write_bytes(original+b"\n")
                try:
                    with self.assertRaisesRegex(ValueError, "hash differs|certificate differs"):
                        self.derive()
                finally:
                    path.write_bytes(original)

    def test_changed_propulsion_profile_without_matching_report_hash_is_rejected(self):
        self.profile["rotors"][0]["motor_constant"] *= 1.1
        self.save(refresh_hashes=False)
        with self.assertRaisesRegex(ValueError, "profile|hash"):
            self.derive()

    def test_v1_airframe_only_clearance_is_not_a_complete_physical_certificate(self):
        # A former v1 could have a successful airframe clearance certificate
        # while the two rectangular arm links still intersected each other.
        # Keeping its old broad 'rigid' flag cannot grant v2 physical approval.
        self.report["model_name"] = "x650_2link_v1"
        self.report.pop("physical_geometry_validated")
        self.report["rigid_geometry_and_inertia_validated"] = True
        self.clearance.pop("arm_self_collision")
        self.save(refresh_hashes=True)
        with self.assertRaises(ValueError):
            self.derive()

    def test_negative_self_collision_result_cannot_be_hidden_by_outer_pass(self):
        self.clearance["passed"] = True
        self.clearance["arm_self_collision"]["passed"] = False
        self.save(refresh_hashes=True)
        with self.assertRaisesRegex(ValueError, "self|clearance|physical"):
            self.derive()

    def test_independent_self_collision_certificate_is_hash_bound_and_must_pass(self):
        path = self.folder / "arm_self_collision_report.json"
        path.write_text(json.dumps({"passed": False}), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "self-collision certificate differs"):
            self.derive()
        self.self_collision["passed"] = False
        self.save(refresh_hashes=True)
        with self.assertRaisesRegex(ValueError, "self-collision certificate did not pass"):
            self.derive()

    def test_unresolved_arm_self_collision_flag_is_rejected(self):
        self.report["arm_self_collision_resolved"] = False
        self.save(refresh_hashes=True)
        with self.assertRaisesRegex(ValueError, "self-collision checks"):
            self.derive()

    def test_report_mass_com_and_allocation_checks_refuse_invalid_inputs(self):
        original = deepcopy(self.report)
        changes = [dict(total_mass_kg=3.02), dict(total_mass_kg=3.6), dict(total_mass_kg=float("nan")),
                   dict(total_mass_kg=100.), dict(com_flu_q0_m=[0.,0.]),
                   dict(com_flu_q0_m=[float("nan"),0.,0.])]
        for values in changes:
            with self.subTest(values=values):
                self.report = deepcopy(original)
                self.report.update(values)
                self.save()
                with self.assertRaises(ValueError):
                    self.derive()
        self.report = deepcopy(original)
        self.report["motor_mapping_q0"][0]["position_com_frd_m"][0] += .01
        self.save()
        with self.assertRaisesRegex(ValueError, "COM"):
            self.derive()

    def test_old_no_arm_id_missing_joint_and_mapping_mismatch_are_rejected(self):
        original_profile, original_report = deepcopy(self.profile), deepcopy(self.report)
        for kind in ("old_id", "joint_list", "motor_order", "motor_link", "motor_direction", "unknown_direction", "negative_km"):
            with self.subTest(kind=kind):
                self.profile, self.report = deepcopy(original_profile), deepcopy(original_report)
                if kind == "old_id":
                    self.profile["px4_sys_autostart"] = self.report["px4_sys_autostart"] = 4999
                elif kind == "joint_list":
                    self.profile["controlled_joints"] = []
                elif kind == "motor_order":
                    self.report["motor_mapping_q0"][1]["motor_number"] = 0
                elif kind == "motor_link":
                    self.report["motor_mapping_q0"][1]["link"] = "link_005"
                elif kind == "motor_direction":
                    self.report["motor_mapping_q0"][1]["turning_direction"] = "cw"
                elif kind == "unknown_direction":
                    self.report["motor_mapping_q0"][1]["turning_direction"] = "unspecified"
                    self.profile["rotors"][1]["turning_direction"] = "unspecified"
                elif kind == "negative_km":
                    self.profile["rotors"][1]["moment_constant"] = -.016
                self.save(refresh_hashes=True)
                with self.assertRaises(ValueError):
                    self.derive()


if __name__ == "__main__":
    unittest.main()
