"""Independent geometry derivatives, mass balance and report provenance checks."""
from __future__ import annotations
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from x650_wholebody.model import (ArmModel, tip_body, tip_world, joint_jacobian_body,
    joint_centripetal_body, inverse_tip_body, arm_com_kinematics, rotation_matrix)
from x650_wholebody.feedforward import arm_reaction_feedforward


def example_model(**overrides):
    fields = dict(masses=(.25, .20), lengths=(.30, .25), com_distances=(.15, .125),
                  angle_offsets=(np.deg2rad(150.), 0.), mount_body=(0., 0., -.18),
                  inertia_diagonal=((6.66666666667e-5, .00190833333333, .00190833333333),
                                    (4.08333333333e-5, .00106208333333, .00106208333333)),
                  total_mass=3.495, base_com_body=(.0016, .0021, -.064),
                  joint_lower=(np.deg2rad(-60.), np.deg2rad(-25.)),
                  joint_upper=(np.deg2rad(30.), np.deg2rad(60.)), profile_name="explicit_test_fixture")
    fields.update(overrides)
    return ArmModel(**fields)


class GeometryTests(unittest.TestCase):
    def test_jacobian_and_acceleration_from_independent_finite_difference(self):
        m = example_model(elbow_offset_body_m=(0., .04, 0.))
        q, dq, ddq = np.array([-.21, .30]), np.array([.16, -.11]), np.array([-.18, .22])
        eps = 1e-6
        numerical = np.column_stack([(tip_body(q+np.eye(2)[i]*eps, m)-tip_body(q-np.eye(2)[i]*eps, m))/(2*eps) for i in range(2)])
        np.testing.assert_allclose(joint_jacobian_body(q, m), numerical, atol=2e-11, rtol=1e-8)
        h = 2e-4
        motion = lambda t: q+dq*t+.5*ddq*t*t
        numeric_acc = (tip_body(motion(h), m)-2*tip_body(q, m)+tip_body(motion(-h), m))/h**2
        analytic = joint_jacobian_body(q, m)@ddq+joint_centripetal_body(q, dq, m)
        np.testing.assert_allclose(analytic, numeric_acc, atol=3e-9, rtol=1e-6)

    def test_com_derivatives_and_ik_round_trip(self):
        m = example_model()
        for q in ([0., 0.], [-.6, .7], [.3, -.2]):
            np.testing.assert_allclose(inverse_tip_body(tip_body(q, m), near=q, model=m), q, atol=1e-12)
        q, dq, ddq = np.array([-.2, .3]), np.array([.2, -.1]), np.array([.3, -.4])
        pos, vel, acc = arm_com_kinematics(q, dq, ddq, m)
        h = 1e-4
        get = lambda t: arm_com_kinematics(q+dq*t+.5*ddq*t*t, (0., 0.), (0., 0.), m)[0]
        np.testing.assert_allclose(vel, (get(h)-get(-h))/(2*h), atol=2e-10)
        np.testing.assert_allclose(acc, (get(h)-2*pos+get(-h))/h**2, atol=8e-9)

    def test_mount_and_nonzero_body_origin_are_respected(self):
        m = example_model()
        other = replace(m, mount_body=(.1, -.2, -.4))
        delta = np.array(other.mount_body)-m.mount_body
        np.testing.assert_allclose(tip_body([0., 0.], other)-tip_body([0., 0.], m), delta)
        R = rotation_matrix([np.sqrt(.5), 0., 0., np.sqrt(.5)])
        np.testing.assert_allclose(tip_world([1., 2., 3.], R, [.1, .2], m), [1., 2., 3.]+R@tip_body([.1, .2], m))
        with self.assertRaises(ValueError):
            rotation_matrix(np.diag([1., 1., -1.]))

    def test_lateral_elbow_offset_fk_com_and_ik(self):
        old = example_model()
        m = example_model(elbow_offset_body_m=(0., .04, 0.))
        for q in np.random.default_rng(12).uniform([-.6, -.3], [.4, .8], size=(20, 2)):
            np.testing.assert_allclose(tip_body(q, m)-tip_body(q, old), [0., .04, 0.], atol=1e-15)
            np.testing.assert_allclose(inverse_tip_body(tip_body(q, m), near=q, model=m), q, atol=1e-12)
            np.testing.assert_allclose(joint_jacobian_body(q, m), joint_jacobian_body(q, old), atol=1e-15)
        p = arm_com_kinematics([.1, .2], [.2, .3], [.4, .5], m)[0]
        np.testing.assert_allclose(p[:, 1], [0., .04], atol=1e-15)
        with self.assertRaises(ValueError):
            inverse_tip_body(tip_body([0., 0.], old), near=[0., 0.], model=m)

    def test_invalid_physical_data_and_unreachable_targets_fail(self):
        for fields in (dict(total_mass=.4), dict(masses=(0., .2)), dict(com_distances=(.4, .1)),
                       dict(inertia_diagonal=((1., 1., 3.), (1., 1., 1.)))):
            with self.assertRaises(ValueError):
                example_model(**fields)
        with self.assertRaises(ValueError):
            inverse_tip_body([2., 0., 0.], near=[0., 0.], model=example_model())


class FeedforwardTests(unittest.TestCase):
    def test_rest_has_no_extra_arm_gravity_acceleration(self):
        m = example_model()
        out = arm_reaction_feedforward([0., 0.], [0., 0.], [0., 0.], model=m)
        np.testing.assert_array_equal(out["acceleration_world"], np.zeros(3))
        self.assertAlmostEqual(out["predicted_reaction_force_world"][2], -.45*9.80665)
        self.assertNotEqual(float(out["required_arm_moment_body"][1]), 0.)

    def test_mass_scaling_and_norm_saturation(self):
        m = example_model(base_com_body=(0., 0., 0.))
        args = ([.1, .2], [.3, -.2], [1., -.8])
        a = arm_reaction_feedforward(*args, model=m, acceleration_limit=None)
        b = arm_reaction_feedforward(*args, model=replace(m, total_mass=2*m.total_mass), acceleration_limit=None)
        np.testing.assert_allclose(a["acceleration_world"], 2*b["acceleration_world"], atol=1e-14)
        c = arm_reaction_feedforward(*args, model=m, acceleration_limit=.001)
        self.assertTrue(c["saturated"])
        self.assertAlmostEqual(np.linalg.norm(c["acceleration_world"]), .001)

    def test_force_balance_is_invariant_to_body_origin_translation(self):
        m = example_model()
        shift = np.array([.03, -.02, .05])
        other = replace(m, mount_body=tuple(np.array(m.mount_body)-shift), base_com_body=tuple(np.array(m.base_com_body)-shift))
        omega, alpha = np.array([.1, -.2, .3]), np.array([-.4, .5, -.6])
        R = rotation_matrix([.9, .1, -.2, .3])
        origin_acc, g = np.array([.3, -.1, .2]), np.array([0., 0., -9.80665])
        shifted_acc = origin_acc+R@(np.cross(alpha, shift)+np.cross(omega, np.cross(omega, shift)))
        args = ([.1, .3], [.2, -.1], [-.4, .3])
        a = arm_reaction_feedforward(*args, model=m, R_world_body=R, omega_body=omega, alpha_body=alpha, acceleration_limit=None)
        b = arm_reaction_feedforward(*args, model=other, R_world_body=R, omega_body=omega, alpha_body=alpha, acceleration_limit=None)
        np.testing.assert_allclose(m.total_mass*(origin_acc-g)+a["relative_force_world"],
                                   m.total_mass*(shifted_acc-g)+b["relative_force_world"], atol=1e-14)


class ReportTests(unittest.TestCase):
    def make_report(self, folder):
        m = example_model()
        k = dict(body_frame="FLU", world_frame="ENU", mount_xyz_m=m.mount_body, mount_rpy_rad=[0., 0., 0.],
                 lengths_m=m.lengths, joint_limits_rad=np.column_stack((m.joint_lower, m.joint_upper)).tolist(),
                 initial_relative_angles_rad=[m.angle_offsets[0], m.angle_offsets[1]-m.angle_offsets[0]],
                 joint_axes_local=[[0., 1., 0.], [0., 1., 0.]], gazebo_zero_is_folded_pose=True,
                 elbow_offset_body_m=m.elbow_offset_body_m)
        (folder/"robot.urdf").write_text("<robot name='unit_fixture'/>")
        (folder/"kinematics.json").write_text(json.dumps(k))
        (folder/"clearance_report.json").write_text(json.dumps(dict(passed=True, physical_limits_changed=False,
                                                                 arm_self_collision=dict(passed=True))))
        (folder/"arm_self_collision_report.json").write_text(json.dumps(dict(passed=True)))
        fields = m.metadata()
        fields.pop("source_report_sha256")
        fields.update(body_frame="FLU", world_frame="ENU", base_mass=m.fixed_mass,
                      inertia_com_kg_m2=[np.diag(v).tolist() for v in m.inertia_diagonal])
        report = dict(kind="x650_original_two_link_composition", source_unchanged=True,
                      joint_state_offset_convention_preserved=True, rigid_geometry_and_inertia_validated=True,
                      physical_geometry_validated=True, arm_self_collision_resolved=True,
                      native_ground_contact_validated=False, flight_validated=False, total_mass_kg=m.total_mass,
                      controller_model=fields)
        for name, key in (("robot.urdf", "derived_urdf_sha256"), ("kinematics.json", "kinematics_sha256"),
                          ("clearance_report.json", "clearance_certificate_sha256"),
                          ("arm_self_collision_report.json", "arm_self_collision_report_sha256")):
            report[key] = hashlib.sha256((folder/name).read_bytes()).hexdigest()
        path = folder/"preparation_report.json"
        path.write_text(json.dumps(report))
        return path, report

    def test_audited_preflight_report_is_allowed_without_claiming_flight(self):
        with tempfile.TemporaryDirectory() as tmp:
            path, _ = self.make_report(Path(tmp))
            m = ArmModel.from_report(path)
            self.assertEqual(m.total_mass, 3.495)
            self.assertEqual(m.source_report_sha256, hashlib.sha256(path.read_bytes()).hexdigest())

    def test_changed_geometry_failed_certificate_or_report_mismatch_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            path, report = self.make_report(folder)
            (folder/"robot.urdf").write_text("changed")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                ArmModel.from_report(path)
            path, report = self.make_report(folder)
            report["controller_model"]["mount_body"] = [0., 0., -.1]
            path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, "geometry conflicts"):
                ArmModel.from_report(path)
            path, report = self.make_report(folder)
            report["rigid_geometry_and_inertia_validated"] = False
            path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, "Physical model validation"):
                ArmModel.from_report(path)
            path, report = self.make_report(folder)
            certificate = dict(passed=True, physical_limits_changed=False)
            (folder/"clearance_report.json").write_text(json.dumps(certificate))
            report["clearance_certificate_sha256"] = hashlib.sha256((folder/"clearance_report.json").read_bytes()).hexdigest()
            path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, "Physical clearance certificate"):
                ArmModel.from_report(path)


if __name__ == "__main__":
    unittest.main()
