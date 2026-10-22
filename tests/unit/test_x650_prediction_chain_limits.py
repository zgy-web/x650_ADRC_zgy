"""Independent actual-state and issued-command bounds; no flight fixtures."""
from copy import deepcopy
import unittest
import numpy as np
import test_x650_mixed_stage_audit as fixtures
from mixed_mpc_protocol import build_prediction_chain_protocol


class PredictionChainLimitTests(unittest.TestCase):
    def setUp(self):
        self.protocol = build_prediction_chain_protocol()
        self.control = []
        for i in range(4):
            self.control.append({"sim_s": 9.98+.02*i, "t_s": -.02+.02*i,
                "phase": "settle" if i == 0 else "measure", "control_dt_s": .02,
                **{"base_velocity_target_"+a: 0. for a in "xyz"}, "qd1_cmd": .005*i, "qd2_cmd": 0.})
        self.physical = {"q": np.zeros((11, 2)), "dq": np.zeros((11, 2))}

    def run_check(self):
        return fixtures.audit.prediction_chain_limit_audit(self.control, [10., 10.04], self.physical, self.protocol)

    def test_actual_limits_and_slew_are_not_inferred_from_solver_success(self):
        self.assertTrue(all(self.run_check()["criteria"].values()))
        self.physical["q"][5, 1] = -.351
        self.assertFalse(self.run_check()["criteria"]["native_joints_inside_common_soft_limits"])
        self.physical["q"][:] = 0
        self.physical["dq"][3, 1] = .351
        self.assertFalse(self.run_check()["criteria"]["native_joint_velocity_inside_common_caps"])
        self.control[2]["qd1_cmd"] = .1
        self.assertFalse(self.run_check()["criteria"]["sent_command_slew_inside_common_caps"])

    def test_missing_previous_input_and_invalid_dt_fail(self):
        original = deepcopy(self.control)
        self.control.pop(0)
        with self.assertRaisesRegex(ValueError, "preceding command"):
            self.run_check()
        self.control = original
        self.control[1]["control_dt_s"] = .1
        with self.assertRaisesRegex(ValueError, "control intervals"):
            self.run_check()

    def test_recorded_dt_cannot_hide_actual_slew_and_startup_uses_previous_send(self):
        self.control[2]["qd1_cmd"] = .025
        self.control[2]["control_dt_s"] = .05
        with self.assertRaisesRegex(ValueError, "differ from command timestamps"):
            self.run_check()
        self.control[2]["control_dt_s"] = .02
        self.assertFalse(self.run_check()["criteria"]["sent_command_slew_inside_common_caps"])

        # The first transition must preserve the actual prestream interval.
        self.control[1]["control_dt_s"] = .05
        self.control[1]["qd1_cmd"] = .02
        with self.assertRaisesRegex(ValueError, "differ from command timestamps"):
            self.run_check()
        self.control[1]["control_dt_s"] = .02
        result = self.run_check()
        self.assertFalse(result["criteria"]["sent_command_slew_inside_common_caps"])
        self.assertAlmostEqual(result["first_actual_command_interval_s"], .02)
        self.assertAlmostEqual(result["first_recorded_control_interval_s"], .02)
        self.assertAlmostEqual(result["maximum_sent_command_slew"][3], 1.)

    def test_first_transition_uses_actual_interval_and_rejects_reset_or_long_gap(self):
        self.control[0]["sim_s"] = 9.96
        for row in self.control[1:]:
            row["qd1_cmd"] = .02
        with self.assertRaisesRegex(ValueError, "differ from command timestamps"):
            self.run_check()
        self.control[1]["control_dt_s"] = .04
        result = self.run_check()
        self.assertTrue(all(result["criteria"].values()))
        self.assertAlmostEqual(result["first_actual_command_interval_s"], .04)
        self.assertAlmostEqual(result["first_recorded_control_interval_s"], .04)
        self.assertAlmostEqual(result["maximum_sent_command_slew"][3], .5)
        self.control[0]["sim_s"] = 9.94
        with self.assertRaisesRegex(ValueError, "actual command intervals"):
            self.run_check()

    def test_used_chain_must_bind_actual_joint_timestamps_and_payload(self):
        row = {"sim_s": 10., "joint_stamp_s": 9.988, "q1": .1, "q2": .2, "qd1": .01, "qd2": -.02}
        d = {"prediction_chain": self.protocol["mpc_controller_config"]["prediction_chain"],
             "prediction_control_time_s": 10., "prediction_joint_stamp_s": 9.988,
             "prediction_joint_measurement_age_s": .012,
             "prediction_measured_joint_position": [.1, .2], "prediction_measured_joint_velocity": [.01, -.02],
             "prediction_initial_joint_position": [.10012, .19976], "prediction_initial_joint_velocity": [.01, -.02],
             "predicted_rotations_flu_to_enu": np.repeat(np.eye(3)[None], 15, axis=0).tolist(),
             "predicted_states": np.zeros((15, 12)).tolist()}
        self.assertTrue(fixtures.audit.prediction_chain_use_audit(d, row, self.protocol))
        for key, value in (("prediction_joint_stamp_s", 10.),
                           ("prediction_measured_joint_velocity", [.02, -.02]),
                           ("predicted_rotations_flu_to_enu", np.zeros((15, 3, 3)).tolist())):
            altered = deepcopy(d)
            altered[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                fixtures.audit.prediction_chain_use_audit(altered, row, self.protocol)


if __name__ == "__main__":
    unittest.main()
