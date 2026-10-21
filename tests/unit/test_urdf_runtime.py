"""Model contracts and no-arm API behavior without ROS or a simulator."""
from __future__ import annotations

import importlib
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from amsim.recording import RosbagRecorder
from amsim.runtime.model_metadata import (
    REVIEWED_SHA, bridge_arguments, discover_model_topics, initial_joint_values,
    model_from_env, required_telemetry, resolve_active_model,
)


def custom_env(joints=(), **kwargs):
    return {
        "AMSIM_MODEL_NAME": "amsim_urdf_custom",
        "AMSIM_MODEL_SHA256": "a" * 64,
        "AMSIM_MODEL_JOINTS": json.dumps(list(joints)),
        "AMSIM_MODEL_SOURCE": "urdf_flight",
        "AMSIM_MODEL_FLIGHT_READY": "1",
        "AMSIM_PX4_SYS_AUTOSTART": "4999",
        **kwargs,
    }


class ModelMetadataTests(unittest.TestCase):
    def test_imported_runtime_never_infers_flight_profile_or_airframe(self):
        for field in ("AMSIM_MODEL_SOURCE", "AMSIM_MODEL_FLIGHT_READY", "AMSIM_PX4_SYS_AUTOSTART", "AMSIM_MODEL_JOINTS"):
            supplied = custom_env()
            del supplied[field]
            with self.subTest(field=field), self.assertRaisesRegex(RuntimeError, "explicit flight metadata"):
                model_from_env(supplied)
        for source in ("urdf_import", "generated", "reviewed"):
            with self.subTest(source=source), self.assertRaisesRegex(RuntimeError, "no flight profile"):
                model_from_env(custom_env(AMSIM_MODEL_SOURCE=source))
        legacy = model_from_env({"AMSIM_MODEL_NAME": "custom_generated_fixture", "AMSIM_MODEL_SHA256": "a" * 64})
        self.assertEqual(legacy["source"], "generated")
        self.assertEqual(legacy["px4_sys_autostart"], "4001")
        self.assertTrue(legacy["flight_ready"])

    def test_reviewed_defaults_and_strict_hash(self):
        model = model_from_env({})
        self.assertEqual(model["model_sha256"], REVIEWED_SHA)
        self.assertEqual(model["joint_names"], ["arm_joint_1", "arm_joint_2"])
        self.assertEqual(model["initial_joint_positions"], {"arm_joint_1": 0, "arm_joint_2": 0})
        with self.assertRaisesRegex(RuntimeError, "Reviewed"):
            model_from_env({"AMSIM_MODEL_SHA256": "a" * 64})

    def test_custom_zero_arm_and_nonzero_initial_pose(self):
        model = model_from_env(custom_env())
        self.assertEqual(model["joint_names"], [])
        self.assertEqual(model["initial_joint_positions"], {})
        model = model_from_env(custom_env(["shoulder", "elbow"], AMSIM_MODEL_BASE_LINK="body", AMSIM_PX4_SYS_AUTOSTART="4010", AMSIM_MODEL_INITIAL_JOINT_POSITIONS='{"shoulder": 0.25}'))
        self.assertEqual(model["base_link"], "body")
        self.assertEqual(model["px4_sys_autostart"], "4010")
        self.assertEqual(model["initial_joint_positions"], {"shoulder": 0.25, "elbow": 0.0})

    def test_malformed_model_contracts_rejected(self):
        cases = [
            {"AMSIM_MODEL_JOINTS": '"arm"'},
            {"AMSIM_MODEL_JOINTS": '["arm", "arm"]'},
            {"AMSIM_MODEL_JOINTS": '["../rotor"]'},
            {"AMSIM_MODEL_NAME": "../escape"},
            {"AMSIM_MODEL_INITIAL_JOINT_POSITIONS": '{"rotor": 1}'},
            {"AMSIM_PX4_SYS_AUTOSTART": "4001;exit"},
            {"AMSIM_MODEL_FLIGHT_READY": "maybe"},
            {"AMSIM_MODEL_SHA256": ""},
        ]
        for overrides in cases:
            with self.subTest(overrides=overrides), self.assertRaises(RuntimeError):
                model_from_env(custom_env(**overrides))

    def test_active_metadata_authority_and_conflicting_env(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "runtime.json"
            active = model_from_env(custom_env())
            path.write_text(json.dumps(active), encoding="utf-8")
            state = {"running": True, "runtime_meta": str(path), "model": active["model"]}
            self.assertEqual(resolve_active_model({}, state), active)
            self.assertEqual(resolve_active_model(custom_env(), state), active)
            with self.assertRaisesRegex(RuntimeError, "conflicts"):
                resolve_active_model({"AMSIM_MODEL_JOINTS": '["shoulder"]'}, state)
            with self.assertRaisesRegex(RuntimeError, "conflicts"):
                resolve_active_model({"AMSIM_PX4_SYS_AUTOSTART": "4010"}, state)
            with self.assertRaisesRegex(RuntimeError, "disagree"):
                resolve_active_model({}, {**state, "model": "other"})

    def test_active_missing_metadata_never_falls_back(self):
        with self.assertRaisesRegex(RuntimeError, "no model metadata"):
            resolve_active_model({}, {"running": True})
        with self.assertRaisesRegex(RuntimeError, "Cannot read"):
            resolve_active_model({}, {"running": True, "runtime_meta": "missing.json"})
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "runtime.json"
            path.write_text("{}", encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "missing model identity"):
                resolve_active_model({}, {"running": True, "runtime_meta": str(path)})

    def test_preview_profile_rejected_before_process_launch(self):
        from amsim.runtime import sitl_supervisor
        with patch.dict("os.environ", custom_env(AMSIM_MODEL_FLIGHT_READY="0"), clear=True), patch.object(sitl_supervisor.subprocess, "Popen") as launch:
            with self.assertRaisesRegex(RuntimeError, "no flight profile"):
                sitl_supervisor.main()
            launch.assert_not_called()


class RuntimeTopicTests(unittest.TestCase):
    def test_old_two_joint_discovery_and_bridge_contract(self):
        instance = "x500_underfold_0"
        joints = ["arm_joint_1", "arm_joint_2"]
        commands = [f"/model/{instance}/joint/{j}/cmd_vel" for j in joints]
        state = f"/world/default/model/{instance}/joint_state"
        found = discover_model_topics([*commands, state], "x500_underfold", joints)
        self.assertEqual(found["instance"], instance)
        self.assertEqual(found["joint_state_gz"], state)
        self.assertEqual(bridge_arguments(found, joints), [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            *[p + "@std_msgs/msg/Float64]gz.msgs.Double" for p in commands],
            state + "@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock", "--ros-args",
            "-r", commands[0] + ":=/arm/arm_joint_1/cmd_vel",
            "-r", commands[1] + ":=/arm/arm_joint_2/cmd_vel",
            "-r", state + ":=/arm/joint_states",
        ])

    def test_zero_arm_discovers_sensor_not_global_truth(self):
        imu = "/world/default/model/amsim_urdf_custom_0/link/body/sensor/imu_sensor/imu"
        self.assertIsNone(discover_model_topics(["/ground_truth/odometry"], "amsim_urdf_custom", [], "body"))
        found = discover_model_topics([imu], "amsim_urdf_custom", [], "body")
        self.assertEqual(found["instance"], "amsim_urdf_custom_0")
        self.assertIsNone(found["joint_state_gz"])
        self.assertEqual(found["joint_commands_gz"], {})
        self.assertFalse(any("joint" in arg or "rotor" in arg for arg in bridge_arguments(found, [])))
        self.assertIn("/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry", bridge_arguments(found, []))

    def test_multiple_instances_and_missing_control_topic_rejected(self):
        topics = [f"/world/default/model/test_{i}/link/base_link/sensor/imu_sensor/imu" for i in range(2)]
        self.assertIsNone(discover_model_topics(topics, "test", []))
        self.assertIsNone(discover_model_topics(topics[:1], "test", ["elbow"]))

    def test_telemetry_and_recording_skip_arm_only_for_zero_joints(self):
        self.assertEqual(required_telemetry([]), ["position", "status", "land", "truth"])
        self.assertIn("joints", required_telemetry(["elbow"]))
        no_arm = RosbagRecorder(Path("unused"), [])
        with_arm = RosbagRecorder(Path("unused"), ["elbow"])
        self.assertNotIn("/arm/joint_states", no_arm.required_topics)
        self.assertEqual(no_arm.command_topics, [])
        self.assertIn("/arm/joint_states", with_arm.required_topics)
        self.assertEqual(with_arm.command_topics, ["/arm/elbow/cmd_vel"])

    def test_initial_pose_checks_named_joints_and_ignores_rotors(self):
        msg = types.SimpleNamespace(name=["rotor_0", "elbow"], position=[999.0, 0.5], velocity=[900.0, 0.0])
        self.assertEqual(initial_joint_values([], {}, None), {})
        self.assertEqual(set(initial_joint_values(["elbow"], {"elbow": 0.5}, msg)), {"elbow"})
        with self.assertRaisesRegex(RuntimeError, "differs"):
            initial_joint_values(["elbow"], {"elbow": 0}, msg)
        msg.velocity[1] = 0.1
        with self.assertRaisesRegex(RuntimeError, "moving too fast"):
            initial_joint_values(["elbow"], {"elbow": 0.5}, msg)


def api_test_modules():
    """Only mock middleware objects; exercise the real ApiNode implementation."""
    class Node:
        def __init__(self, name):
            self.subscriptions = []
            self.created_publishers = []

        def create_subscription(self, typ, topic, callback, qos):
            self.subscriptions.append(topic)

        def create_publisher(self, typ, topic, qos):
            publisher = types.SimpleNamespace(topic=topic, messages=[])
            publisher.publish = publisher.messages.append
            self.created_publishers.append(publisher)
            return publisher

        def destroy_node(self):
            pass

    class Executor:
        def add_node(self, node):
            pass

        def spin_once(self, **kwargs):
            pass

        def shutdown(self):
            pass

    api_package = types.ModuleType("amsim.api")
    api_package.__path__ = [str(ROOT / "src/amsim/api")]
    modules = {"amsim.api": api_package}
    definitions = {
        "rclpy": {},
        "rclpy.node": {"Node": Node},
        "rclpy.executors": {"SingleThreadedExecutor": Executor},
        "rclpy.qos": {"qos_profile_sensor_data": object()},
        "nav_msgs.msg": {"Odometry": type("Odometry", (), {})},
        "px4_msgs.msg": {name: type(name, (), {"ARMING_STATE_ARMED": 2}) for name in ("VehicleLandDetected", "VehicleLocalPosition", "VehicleStatus")},
        "sensor_msgs.msg": {"JointState": type("JointState", (), {})},
        "std_msgs.msg": {"Float64": type("Float64", (), {})},
    }
    for name, attrs in definitions.items():
        module = types.ModuleType(name)
        module.__dict__.update(attrs)
        modules[name] = module
    return modules


class ApiZeroArmTests(unittest.TestCase):
    def test_readiness_no_arm_needs_only_vehicle_and_truth(self):
        modules = api_test_modules()
        pymavlink = types.ModuleType("pymavlink")
        pymavlink.mavutil = types.SimpleNamespace()
        modules["pymavlink"] = pymavlink
        with patch.dict(sys.modules, modules):
            sys.modules.pop("amsim.runtime.readiness_check", None)
            Telemetry = importlib.import_module("amsim.runtime.readiness_check").Telemetry
            node = Telemetry([])
            self.assertNotIn("/arm/joint_states", node.subscriptions)
            self.assertFalse(node.fresh())
            for key in required_telemetry([]):
                node.receive(key, object())
            self.assertTrue(node.fresh())
            node.zero()
            self.assertEqual(node.created_publishers, [])

    def test_api_observation_without_joint_state_and_no_motor_commands(self):
        with patch.dict(sys.modules, api_test_modules()):
            sys.modules.pop("amsim.api.node", None)
            ApiNode = importlib.import_module("amsim.api.node").ApiNode
            node = ApiNode([])
            self.assertNotIn("/arm/joint_states", node.subscriptions)
            node.data = {
                "position": types.SimpleNamespace(x=0, y=0, z=0, vx=0, vy=0, vz=0),
                "status": types.SimpleNamespace(arming_state=1, failsafe=False),
                "land": types.SimpleNamespace(landed=True),
                "truth": types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=2, nanosec=0))),
            }
            node.wait_for_data(timeout_s=0.01)
            self.assertEqual(node.observation().arm, {})
            node.zero_all()
            self.assertEqual(node.created_publishers, [])
            with self.assertRaises(KeyError):
                node.publish_velocity("rotor_0", 0)
            node.close()

    def test_api_named_arm_commands_do_not_include_rotors(self):
        with patch.dict(sys.modules, api_test_modules()):
            sys.modules.pop("amsim.api.node", None)
            ApiNode = importlib.import_module("amsim.api.node").ApiNode
            node = ApiNode(["elbow"])
            node.data["joints"] = types.SimpleNamespace(name=["rotor_0", "elbow"], position=[1, 0.2], velocity=[800, 0.0], effort=[])
            self.assertEqual(set(node.joint_values()), {"elbow"})
            node.zero_all()
            self.assertEqual([p.topic for p in node.created_publishers], ["/arm/elbow/cmd_vel"])
            self.assertEqual(node.created_publishers[0].messages[0].data, 0)


if __name__ == "__main__":
    unittest.main()
