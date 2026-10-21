"""X650 composition geometry, zero conventions and articulated mass properties."""
import importlib.util
import itertools
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"scripts"))
SPEC = importlib.util.spec_from_file_location("x650_arm_prepare", ROOT/"scripts/prepare_x650_arm.py")
x650 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(x650)


def box_triangles(center, size):
    corners = np.asarray(list(itertools.product([-1, 1], repeat=3)))*np.asarray(size)/2+center
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5),
             (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6),
             (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return corners[np.asarray(faces)]


class X650ArmPreparationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.specs, cls.old_mount = x650.original_arm(ROOT/"assets/models/x500_underfold/model.sdf")

    def robot(self):
        robot = ET.Element("robot", name="fixture")
        base = ET.SubElement(robot, "link", name="base_link")
        x650.set_properties(base, 3.02, np.array([.00165, .00212, -.06334]), np.diag([.099, .097, .1207]))
        return robot

    def test_original_flight_arm_values_and_gazebo_zero_preserved(self):
        np.testing.assert_allclose(self.old_mount, [0, 0, -.10])
        np.testing.assert_allclose([s["mass"] for s in self.specs], [.25, .20])
        np.testing.assert_allclose([s["size"][0] for s in self.specs], [.30, .25])
        np.testing.assert_allclose([s["initial_angle"] for s in self.specs], np.radians([150, -150]))
        np.testing.assert_allclose([s["q_limits"] for s in self.specs], np.radians([[-60, 30], [-25, 60]]))
        robot = self.robot()
        x650.add_arm(robot, self.specs)
        for index, spec in enumerate(self.specs):
            node = robot.find(f"link[@name='{spec['link']}']")
            mass, com, inertia = x650.properties(node)
            self.assertEqual(mass, spec["mass"])
            np.testing.assert_allclose(com, [spec["size"][0]/2, 0, 0], atol=1e-15)
            np.testing.assert_allclose(inertia, spec["inertia"], atol=1e-15)
            joint = robot.find(f"joint[@name='{spec['joint']}']")
            np.testing.assert_allclose([float(joint.find("limit").get(k)) for k in ("lower", "upper")], spec["q_limits"])
            self.assertEqual(float(joint.find("limit").get("effort")), [10, 8][index])
            self.assertEqual(float(joint.find("limit").get("velocity")), 2.)

    def test_fk_matches_original_angles_with_mount_and_lateral_elbow_offset(self):
        robot = self.robot()
        x650.add_arm(robot, self.specs)
        rng = np.random.default_rng(420)
        for q in [np.zeros(2), *rng.uniform(np.radians([-60, -25]), np.radians([30, 60]), size=(50, 2))]:
            poses = x650.kinematic_poses(robot, dict(zip(x650.JOINT_NAMES, q)))
            a1, a2 = np.radians(150)+q[0], q.sum()
            elbow = x650.MOUNT+.30*np.array([np.cos(a1), 0, -np.sin(a1)])+x650.ELBOW_OFFSET
            tip = elbow+.25*np.array([np.cos(a2), 0, -np.sin(a2)])
            np.testing.assert_allclose(poses["arm_link_2"][:3, 3], elbow, atol=2e-15)
            actual_tip = (poses["arm_link_2"]@np.array([.25, 0, 0, 1]))[:3]
            np.testing.assert_allclose(actual_tip, tip, atol=2e-15)

    def test_mass_com_and_tensor_conserve_explicit_components(self):
        robot = self.robot()
        x650.add_arm(robot, self.specs)
        mass, com, inertia = x650.aggregate(robot)
        self.assertAlmostEqual(mass, 3.495)
        corners = x650.arm_corners(self.specs, [0, 0])
        expected_com = (3.02*np.array([.00165, .00212, -.06334])
                        +.025*np.array([0, 0, -.1646])
                        +.25*corners[0].mean(0)+.20*corners[1].mean(0))/mass
        np.testing.assert_allclose(com, expected_com, atol=2e-15)
        self.assertTrue(np.all(np.linalg.eigvalsh(inertia) > 0))
        for q in itertools.product(*(s["q_limits"] for s in self.specs)):
            moved_mass, moved_com, moved_i = x650.aggregate(robot, dict(zip(x650.JOINT_NAMES, q)))
            self.assertAlmostEqual(moved_mass, mass)
            self.assertGreater(np.linalg.norm(moved_com-com), .001)
            self.assertGreater(float(np.min(np.linalg.eigvalsh(moved_i))), 0.)

    def test_full_limit_certificate_covers_continuous_random_states_and_ground_policy(self):
        triangles = np.concatenate([box_triangles([0, 0, -.1482], [.128, .12, .002]),
                                    box_triangles([0, .20, -.36], [.30, .02, .0194119018554688]),
                                    box_triangles([0, -.20, -.36], [.30, .02, .0194119018554688])])
        report = x650.clearance_audit(triangles, self.specs, grid_step_deg=1.)
        self.assertTrue(report["passed"])
        self.assertGreater(report["certified_original_airframe_arm_clearance_m"], .006)
        self.assertFalse(report["physical_limits_changed"])
        self.assertFalse(report["landing"]["all_joint_positions_ground_safe"])
        self.assertAlmostEqual(report["landing"]["level_q0_clearance_m"], .0222059509277344)
        rng = np.random.default_rng(1021)
        for q in rng.uniform(np.radians([-60, -25]), np.radians([30, 60]), size=(1000, 2)):
            corners = x650.arm_corners(self.specs, q)
            self.assertTrue(np.all(corners[:, :, 2].max(1) <= report["certified_link_maximum_z_m"]))
            self.assertTrue(np.all(corners[:, :, 2].min(1) >= report["certified_link_minimum_z_m"]))

    def test_changed_mount_surface_is_rejected(self):
        triangles = box_triangles([0, 0, -.16], [.128, .12, .002])
        with self.assertRaisesRegex(ValueError, "mounting review"):
            x650.clearance_audit(triangles, self.specs, grid_step_deg=1.)

    def test_original_coplanar_overlap_is_exposed_and_v2_is_continuously_separated(self):
        report = x650.self_collision_audit(self.specs)
        self.assertTrue(report["passed"])
        self.assertAlmostEqual(report["certified_surface_clearance_m"], .0025)
        self.assertTrue(report["covers_continuous_entire_physical_joint_ranges"])
        original = report["original_coplanar_audit"]
        self.assertFalse(original["passed"])
        self.assertGreater(original["samples"][0]["maximum_overlap_radius_from_elbow_m"], .25)
        self.assertGreater(original["samples"][1]["maximum_overlap_radius_from_elbow_m"], .072)
        self.assertFalse(original["current_composed_model_self_collision_resolved"])

    def test_triangle_slab_clipping_uses_actual_cut_edges(self):
        triangle = np.array([[[0., 0., 0.], [1., 1., -10.], [-1., 1., -10.]]])
        floor, count = x650.triangle_y_slab_floor(triangle, .01, .02)
        self.assertEqual(count, 1)
        self.assertAlmostEqual(floor, -.2)

    def test_existing_output_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source = directory/"source.urdf"
            source.write_text("<robot/>")
            output = directory/"output"
            output.mkdir()
            marker = output/"marker"
            marker.write_text("preserve")
            with self.assertRaises(FileExistsError):
                x650.prepare(source, output, 4998)
            self.assertEqual(marker.read_text(), "preserve")


if __name__ == "__main__":
    unittest.main()
