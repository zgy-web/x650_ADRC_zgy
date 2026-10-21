import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("telemetry_config", ROOT/"scripts/configure_px4_estimator_telemetry.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

FIXTURE = """publications:
  - topic: /fmu/out/vehicle_local_position
    type: px4_msgs::msg::VehicleLocalPosition
    rate_limit: 50.

  - topic: /fmu/out/estimator_status_flags
    type: px4_msgs::msg::EstimatorStatusFlags
    rate_limit: 5.

  # - topic: /fmu/out/vehicle_angular_velocity
  #   type: px4_msgs::msg::VehicleAngularVelocity

  - topic: /fmu/out/vehicle_attitude
    type: px4_msgs::msg::VehicleAttitude

subscriptions:
  - topic: /fmu/in/trajectory_setpoint
    type: px4_msgs::msg::TrajectorySetpoint
"""


class TelemetryConfigTests(unittest.TestCase):
    def test_enable_only_reviewed_outputs_and_idempotent(self):
        text = module.reviewed_text(FIXTURE)
        self.assertIn("    rate_limit: 5.\n", text)
        self.assertIn("subscriptions:\n  - topic: /fmu/in/trajectory_setpoint", text)
        self.assertEqual(text.count("rate_limit: 50.0"), 2)
        self.assertEqual(module.reviewed_text(text), text)

    def test_missing_flags_rejected(self):
        with self.assertRaises(ValueError):
            module.reviewed_text(FIXTURE.replace("/fmu/out/estimator_status_flags", "/fmu/out/wrong"))

    def test_wrong_type_rejected(self):
        with self.assertRaises(ValueError):
            module.reviewed_text(FIXTURE.replace("msg::VehicleAttitude", "msg::VehicleStatus"))

    def test_duplicate_publication_rejected(self):
        with self.assertRaises(ValueError):
            module.reviewed_text(FIXTURE.replace("subscriptions:\n", "  - topic: /fmu/out/vehicle_attitude\n    type: px4_msgs::msg::VehicleAttitude\n\nsubscriptions:\n"))


if __name__ == "__main__":
    unittest.main()
