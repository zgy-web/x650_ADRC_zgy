from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("x650_hover_case", ROOT / "simulations/x650_hover_v1/run_case.py")
CASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CASE)


class HoverProtocolTests(unittest.TestCase):
    def setUp(self):
        self.config = yaml.safe_load((ROOT / "configs/scenarios/x650_hover_v1.yaml").read_text(encoding="utf-8"))

    def test_missing_flight_model_does_not_substitute_x500(self):
        with patch.object(CASE, "resolve", side_effect=KeyError("missing")):
            with self.assertRaisesRegex(RuntimeError, "No x500 propulsion or PID"):
                CASE.preflight(self.config, check_airframe=False)

    def test_rejects_arm_model(self):
        model = SimpleNamespace(source="urdf_flight", flight_ready=True, joints=["arm_joint_1"])
        with patch.object(CASE, "resolve", return_value=model):
            with self.assertRaisesRegex(ValueError, "zero arm joints"):
                CASE.preflight(self.config, check_airframe=False)

    def test_source_hash_cannot_be_relabelled(self):
        self.config["expected_urdf_sha256"] = "a" * 64
        model = SimpleNamespace(source="urdf_flight", flight_ready=True, joints=[], manifest_path=Path("manifest.json"), sdf_path=Path("model.sdf"))
        with patch.object(CASE, "resolve", return_value=model), patch.object(Path, "read_text", side_effect=["{}", json.dumps({"source": {"urdf_sha256": "different"}})]):
            with self.assertRaisesRegex(ValueError, "selected X650 URDF"):
                CASE.preflight(self.config, check_airframe=False)

    def test_unreviewed_derived_urdf_is_not_replaced_by_original_hash(self):
        self.config["expected_urdf_sha256"] = None
        model = SimpleNamespace(source="urdf_flight", flight_ready=True, joints=[])
        with patch.object(CASE, "resolve", return_value=model):
            with self.assertRaisesRegex(ValueError, "reviewed flight-preparation URDF hash"):
                CASE.preflight(self.config, check_airframe=False)

    def test_fixed_world_target_and_yaw_wrap(self):
        yaw = math.radians(-179)
        truth = SimpleNamespace(x=1.1, y=2.2, z=3.3, qx=0, qy=0, qz=math.sin(yaw/2), qw=math.cos(yaw/2))
        obs = SimpleNamespace(ground_truth=truth, vehicle=SimpleNamespace(x=20, y=30, z=-2), sim_time_s=1)
        row = CASE.sample(obs, (1, 2, 3), math.radians(179), "measure")
        self.assertAlmostEqual(row["position_error_m"], math.sqrt(.14))
        self.assertAlmostEqual(row["yaw_error_deg"], 2)
        self.assertAlmostEqual(row["estimate_ned_x_m"], 20)

    def test_only_measured_interval_counts_and_threshold_failure_is_visible(self):
        truth = SimpleNamespace(x=.1, y=0, z=2, qx=0, qy=0, qz=0, qw=1)
        obs = SimpleNamespace(ground_truth=truth, vehicle=SimpleNamespace(x=0, y=0, z=-2), sim_time_s=1)
        first = CASE.sample(obs, (0, 0, 2), 0, "measure")
        rows = [dict(first, phase="settle", position_error_m=100), first, dict(first, sim_time_s=2)]
        metrics = CASE.summarize(rows, self.config["acceptance"])
        self.assertTrue(metrics["accepted"])
        self.assertAlmostEqual(metrics["position_rmse_m"], .1)
        rows[-1]["position_error_m"] = 1
        self.assertFalse(CASE.summarize(rows, self.config["acceptance"])["accepted"])

    def test_timestamp_reversal_and_invalid_quaternion_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "quaternion"):
            CASE.euler(0, 0, 0, 0)
        with self.assertRaisesRegex(RuntimeError, "strictly increasing"):
            CASE.summarize([{"phase": "measure", "sim_time_s": 2}, {"phase": "measure", "sim_time_s": 1}], self.config["acceptance"])

    def test_scenario_discovery_selects_script(self):
        from amsim.scenario_registry import get
        scenario = get("x650_hover_v1")
        self.assertEqual(scenario.source, "builtin")
        self.assertTrue((ROOT / scenario.module).is_file())
        self.assertEqual(scenario.config["model"], "amsim_urdf_x650_mn4014_v1_flight")

    @staticmethod
    def capture_context():
        vehicle = SimpleNamespace(x=0, y=0, z=-2, armed=True)
        node = SimpleNamespace(
            data={
                "position": SimpleNamespace(timestamp=123),
                "status": SimpleNamespace(failsafe=False, arming_state=2, nav_state=4, timestamp=120),
                "land": SimpleNamespace(landed=False),
            },
            observation=lambda: SimpleNamespace(vehicle=vehicle),
        )
        return SimpleNamespace(node=node)

    @staticmethod
    def truth_message(stamp):
        sec = int(stamp)
        return SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=round((stamp-sec)*1e9)), frame_id="world"),
            child_frame_id="base_link",
            pose=SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=.1, y=.2, z=2),
                orientation=SimpleNamespace(x=0, y=0, z=math.sin(.2), w=math.cos(.2)),
            )),
        )

    def test_full_flight_capture_uses_real_timestamps_and_quaternions(self):
        with tempfile.TemporaryDirectory(prefix="x650-telemetry-") as directory:
            rows = []
            capture = CASE.FlightTelemetry(directory, 20, "a"*64, (0, 0, 2), 0, rows)
            capture.context = self.capture_context()
            for i in range(21):
                capture.phase = "takeoff" if i < 10 else "landing"
                capture.receive(self.truth_message(i*.01))
            capture.check()
            metadata = capture.close()
            self.assertEqual(len(rows), 5)
            for row, stamp in zip(rows, [0, .05, .1, .15, .2]):
                self.assertAlmostEqual(row["sim_time_s"], stamp)
            self.assertAlmostEqual(rows[0]["world_qz"], math.sin(.2))
            self.assertEqual(rows[-1]["phase"], "landing")
            self.assertEqual(rows[0]["frame_id"], "world")
            self.assertEqual(rows[0]["model_sha256"], "a"*64)
            self.assertEqual(rows[0]["px4_position_timestamp_us"], 123)
            self.assertEqual(metadata["phases"], {"takeoff": 2, "landing": 3})
            self.assertTrue((Path(directory) / "flight_samples_manifest.json").is_file())

    def test_capture_retains_gaps_and_reports_error_without_interrupting_landing_pump(self):
        with tempfile.TemporaryDirectory(prefix="x650-telemetry-") as directory:
            rows = []
            capture = CASE.FlightTelemetry(directory, 20, "a"*64, (0, 0, 2), 0, rows)
            capture.context = self.capture_context()
            capture.receive(self.truth_message(1))
            capture.receive(self.truth_message(1.7))
            capture.receive(self.truth_message(1.6))
            metadata = capture.close()
            self.assertEqual(len(rows), 2)
            for row, stamp in zip(rows, [1, 1.7]):
                self.assertAlmostEqual(row["sim_time_s"], stamp)
            with self.assertRaisesRegex(RuntimeError, "time moved backwards"):
                capture.check()
            self.assertAlmostEqual(metadata["observed_interval_s"]["max"], .7)
            self.assertIn("backwards", metadata["capture_error"])


if __name__ == "__main__":
    unittest.main()
