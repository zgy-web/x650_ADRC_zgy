"""Independent force/torque checks for the X650 PX4-to-Gazebo contract.

No PX4 process or simulator is launched. These checks do not establish flight
stability; they verify dimensions, coordinates, rotor authority and static trim.
"""
from pathlib import Path
import json
import math
import re
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "configs/vehicles/x650_no_arm"


def load_contract():
    parameters = {}
    text = (CONFIG / "4999_gz_x650_mn4014_v1").read_text(encoding="utf-8")
    for match in re.finditer(r"^param set (\w+) ([^\s#]+)", text, re.MULTILINE):
        name, value = match.groups()
        if name in parameters:
            raise AssertionError(f"Duplicate enforced airframe parameter: {name}")
        parameters[name] = float(value)
    profile = yaml.safe_load((CONFIG / "flight_profile.yaml").read_text(encoding="utf-8"))
    audit = json.loads((CONFIG / "requirements.json").read_text(encoding="utf-8"))
    return parameters, profile, audit


def effectiveness(parameters):
    axis = np.array([0., 0., -1.])  # Multirotor CA_AIRFRAME=0 uses FixedUpwards in FRD.
    columns = []
    for index in range(4):
        prefix = f"CA_ROTOR{index}_"
        position = np.array([parameters[prefix + key] for key in ("PX", "PY", "PZ")])
        ct, km = parameters[prefix+"CT"], parameters[prefix+"KM"]
        columns.append(np.r_[ct*np.cross(position, axis)-ct*km*axis, -ct])
    return np.array(columns).T


class X650AirframeContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params, cls.profile, cls.audit = load_contract()

    def test_airframe_mapping_and_frames_match_audited_geometry(self):
        self.assertEqual(self.profile["px4_sys_autostart"], 4999)
        self.assertEqual(self.params["CA_AIRFRAME"], 0)
        self.assertEqual(self.params["CA_ROTOR_COUNT"], 4)
        self.assertEqual(self.params["CA_R_REV"], 0)
        self.assertEqual(self.params["SIM_GZ_EC_REV"], 0)
        for channel in range(5, 9):
            self.assertEqual(self.params[f"SIM_GZ_EC_FUNC{channel}"], 0)
        self.assertEqual(self.profile["controlled_joints"], [])
        com = np.array(self.audit["known_model"]["nominal_zero_joint_com_in_root_flu_m"])
        geometry = {row["link"]: row for row in self.audit["known_model"]["rotor_links"]}
        expected_links = ["link_004", "link_002", "link_003", "link_005"]
        for index, rotor in enumerate(self.profile["rotors"]):
            self.assertEqual(rotor["motor_number"], index)
            self.assertEqual(rotor["link"], expected_links[index])
            self.assertEqual(self.params[f"SIM_GZ_EC_FUNC{index+1}"], 101+index)
            flu = np.array(geometry[rotor["link"]]["position_in_root_flu_m"])
            expected_frd = (flu-com)*[1,-1,-1]
            actual_frd = [self.params[f"CA_ROTOR{index}_{key}"] for key in ("PX","PY","PZ")]
            np.testing.assert_allclose(actual_frd, expected_frd, rtol=0, atol=1e-9)
            sign = 1 if rotor["turning_direction"] == "ccw" else -1
            self.assertAlmostEqual(self.params[f"CA_ROTOR{index}_KM"], sign*rotor["moment_constant"])

    def test_motor_thrust_linearization_and_integer_speed_cap(self):
        self.assertEqual(self.params["THR_MDL_FAC"], 1)
        for index, rotor in enumerate(self.profile["rotors"]):
            minimum = self.params[f"SIM_GZ_EC_MIN{index+1}"]
            maximum = self.params[f"SIM_GZ_EC_MAX{index+1}"]
            self.assertEqual(minimum, 0)
            self.assertEqual(self.params[f"SIM_GZ_EC_DIS{index+1}"], 0)
            self.assertEqual(maximum, int(maximum))
            self.assertLessEqual(maximum, rotor["max_rot_velocity"])
            kt = rotor["motor_constant"]
            max_thrust = kt*maximum**2
            self.assertAlmostEqual(self.params[f"CA_ROTOR{index}_CT"], max_thrust, places=8)
            for thrust_fraction in (.1,.25,.3888125521,.5,1.):
                # FunctionMotors' inverse quadratic with factor=1, then the
                # output limiter interpolates and the GZ bridge sends rad/s.
                velocity = minimum+(maximum-minimum)*math.sqrt(thrust_fraction)
                self.assertAlmostEqual(kt*velocity**2, max_thrust*thrust_fraction, places=10)
                # PX4's uint16 output produces at most the expected quantization.
                rounded_thrust = kt*round(velocity)**2
                self.assertLess(abs(rounded_thrust-max_thrust*thrust_fraction)/(max_thrust*thrust_fraction), .006)

    def test_hover_parameter_balances_weight_in_thrust_units(self):
        mg = self.audit["known_model"]["total_declared_mass_kg"]*9.80665
        commanded_force = 0.
        for index, rotor in enumerate(self.profile["rotors"]):
            omega = self.params[f"SIM_GZ_EC_MAX{index+1}"]*math.sqrt(self.params["MPC_THR_HOVER"])
            commanded_force += rotor["motor_constant"]*omega**2
        self.assertAlmostEqual(commanded_force, mg, places=7)
        # The old 0.623 speed fraction cannot be used as normalized thrust.
        self.assertLess(self.params["MPC_THR_HOVER"], .5)

    def test_four_axis_rank_survives_pinned_allocator_authority_threshold(self):
        matrix = effectiveness(self.params)
        self.assertEqual(np.linalg.matrix_rank(matrix), 4)
        retained = matrix.copy()
        for row in retained:
            if np.max(abs(row)) <= .05:
                row[:] = 0
        self.assertEqual(np.linalg.matrix_rank(retained), 4)
        self.assertGreater(np.max(abs(matrix[2])), .05)
        # Regression witness: unit CT with this KM would lose yaw authority.
        unit_ct = dict(self.params)
        for index in range(4):
            unit_ct[f"CA_ROTOR{index}_CT"] = 1.
        self.assertLessEqual(np.max(abs(effectiveness(unit_ct)[2])), .05)

    def test_com_static_trim_is_positive_and_within_available_thrust(self):
        matrix = effectiveness(self.params)
        mg = self.audit["known_model"]["total_declared_mass_kg"]*9.80665
        desired = np.array([0.,0.,0.,-mg])
        fractions = np.linalg.solve(matrix, desired)
        self.assertTrue(np.all(fractions > 0))
        self.assertTrue(np.all(fractions < 1))
        np.testing.assert_allclose(matrix@fractions, desired, atol=1e-12, rtol=0)
        self.assertAlmostEqual(float(fractions.mean()), self.params["MPC_THR_HOVER"], places=9)
        for index, fraction in enumerate(fractions):
            omega = self.params[f"SIM_GZ_EC_MAX{index+1}"]*math.sqrt(fraction)
            self.assertLess(omega, self.profile["rotors"][index]["max_rot_velocity"])


if __name__ == "__main__":
    unittest.main()
