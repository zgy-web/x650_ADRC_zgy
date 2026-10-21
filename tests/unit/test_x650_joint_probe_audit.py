"""Synthetic code fixtures only; these are not flight or joint-probe results."""
import ast
import importlib.util
from pathlib import Path
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("joint_probe_audit", ROOT/"scripts/analyze_x650_joint_probe.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class JointProbeAuditTests(unittest.TestCase):
    def data(self):
        t = np.arange(0., 30.0001, .004)
        q, dq = audit.reference(t, 30.)
        return dict(t=t, q=q, dq=dq, body_error=np.zeros((len(t), 3)), tilt=np.zeros(len(t)))

    def test_independent_reference_matches_deployed_formula_source(self):
        # Extract only the pure reference function, avoiding ROS/controller imports.
        tree = ast.parse((ROOT/"simulations/x650_wholebody_mpc_v1/run_case.py").read_text(encoding="utf-8"))
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "joint_probe_reference")
        namespace = {"np": np}
        exec(compile(ast.Module(body=[function], type_ignores=[]), "existing_reference", "exec"), namespace)
        t = np.unique(np.r_[np.linspace(-1, 31, 1003), 2., 2.+26/3, 2.+52/3, 28., 30.])
        desired, rate = audit.reference(t, 30.)
        actual = [namespace["joint_probe_reference"](v, 30.) for v in t]
        np.testing.assert_allclose(desired, [v[0] for v in actual], atol=1e-15)
        np.testing.assert_allclose(rate, [v[1] for v in actual], atol=1e-15)

    def test_correct_and_reversed_actual_directions_are_distinguished(self):
        data = self.data()
        _, _, report = audit.analyze_arrays(data, 30.)
        np.testing.assert_allclose(report["whole"]["sign_cosine"], [1., 1.])
        self.assertEqual(report["J1_only"]["inactive_joint_cross_talk"]["q2"]["position_peak_rad"], 0.)
        data["q"] = -data["q"]
        _, _, reversed_report = audit.analyze_arrays(data, 30.)
        np.testing.assert_allclose(reversed_report["whole"]["sign_cosine"], [-1., -1.])
        self.assertGreater(reversed_report["whole"]["q_tracking_peak_rad"][0], .159)

    def test_inactive_cross_talk_and_fixed_anchor_error_not_removed(self):
        data = self.data()
        mask = (data["t"] >= 2) & (data["t"] <= 2.+26/3)
        data["q"][mask, 1] = .01
        data["body_error"][:] = [.01, -.02, .02]
        _, _, report = audit.analyze_arrays(data, 30.)
        self.assertAlmostEqual(report["J1_only"]["inactive_joint_cross_talk"]["q2"]["position_rms_rad"], .01)
        self.assertAlmostEqual(report["whole"]["body_anchor_rmse_m"], .03)
        self.assertAlmostEqual(report["whole"]["body_anchor_peak_m"], .03)

    def test_native_gap_rejected_even_with_full_window_endpoints(self):
        rows = []
        for t in np.r_[np.arange(0, 10, .004), np.arange(10.02, 30.0001, .004)]:
            row = dict(sim_s=str(t), valid_base="1", valid_joint_state="1", valid_tip="1")
            row.update({"base_q"+a: str(int(a == "w")) for a in "wxyz"})
            row.update({"base_p"+a: "0" for a in "xyz"})
            row.update({f"arm_joint_{j}_{suffix}": "0" for j in (1, 2) for suffix in ("q", "qd")})
            rows.append(row)
        with self.assertRaisesRegex(ValueError, "sparse"):
            audit.native_arrays(rows, 0., 30., [0., 0., 0.])

    def test_actual_command_law_and_wrong_time_origin_are_checked(self):
        t = np.arange(0., 30.0001, .02)
        q, dq = audit.reference(t, 30.)
        rows = [dict(sim_s=str(33.508+v), t_s=str(v), phase="measure", q1=str(a[0]), q2=str(a[1]),
                     qd1_cmd=str(b[0]), qd2_cmd=str(b[1])) for v, a, b in zip(t, q, dq)]
        self.assertLess(audit.control_audit(rows, 33.508, 30.)["max_command_reconstruction_error_rad_s"], 1e-14)
        rows[200]["qd1_cmd"] = ".15"
        with self.assertRaisesRegex(ValueError, "differs"):
            audit.control_audit(rows, 33.508, 30.)
        rows[200]["qd1_cmd"] = str(dq[200, 0])
        rows[200]["t_s"] = str(t[200]+.02)
        with self.assertRaisesRegex(ValueError, "time origin"):
            audit.control_audit(rows, 33.508, 30.)

    def test_native_distal_pose_rejects_reversed_joint_axis_mapping(self):
        data = self.data()
        profile = dict(profile_name="x650_2link_v2", mount_body=[0., 0., -.18],
                       elbow_offset_body_m=[0., .04, 0.], angle_offsets=[np.deg2rad(150), 0.], lengths=[.3, .25])
        a1, a2 = np.deg2rad(150)+data["q"][:, 0], data["q"].sum(1)
        # Independent scalar geometry followed by a 90-degree world yaw.
        x = .3*np.cos(a1)+.25*np.cos(a2)
        z = -.18-.3*np.sin(a1)-.25*np.sin(a2)
        data["position"] = np.tile([1., 2., 3.], (len(x), 1))
        data["quaternion"] = np.tile([np.sqrt(.5), 0., 0., np.sqrt(.5)], (len(x), 1))
        data["tip"] = data["position"]+np.column_stack((np.full(len(x), -.04), x, z))
        self.assertLess(audit.native_tip_audit(data, profile)["maximum_error_m"], 1e-14)
        data["q"] *= -1
        with self.assertRaisesRegex(ValueError, "axes/zero"):
            audit.native_tip_audit(data, profile)


if __name__ == "__main__":
    unittest.main()
