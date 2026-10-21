"""Platform-only opt-in telemetry tests; no X650 or control-package import."""
import importlib.util
import itertools
from pathlib import Path
import re
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
NS = types.SimpleNamespace

probe_spec = importlib.util.spec_from_file_location("platform_estimator_probe_under_test", ROOT/"scripts/probe_px4_estimator_telemetry.py")
probe_module = importlib.util.module_from_spec(probe_spec)
probe_spec.loader.exec_module(probe_module)


class ApiNodeOptInTests(unittest.TestCase):
    def load_node(self):
        class FakeNode:
            def __init__(self,*args):self.subscriptions=[]
            def create_subscription(self,typ,topic,callback,qos):
                self.subscriptions.append((typ,topic,callback));return callback
            def create_publisher(self,*args):return NS(publish=lambda msg:None)
        class Executor:
            def add_node(self,node):pass
        modules={name:types.ModuleType(name) for name in (
            "rclpy","rclpy.executors","rclpy.node","rclpy.qos","nav_msgs","nav_msgs.msg",
            "px4_msgs","px4_msgs.msg","sensor_msgs","sensor_msgs.msg","std_msgs","std_msgs.msg",
            "rosgraph_msgs","rosgraph_msgs.msg")}
        modules["rclpy.executors"].SingleThreadedExecutor=Executor
        modules["rclpy.node"].Node=FakeNode
        modules["rclpy.qos"].qos_profile_sensor_data=object()
        for package,names in (("nav_msgs",("Odometry",)),("px4_msgs",("VehicleLandDetected","VehicleLocalPosition","VehicleStatus")),
                              ("sensor_msgs",("JointState",)),("std_msgs",("Float64",))):
            for name in names:setattr(modules[package+".msg"],name,type(name,(),{}))
        # Package shells bypass the public __init__ (which eagerly imports
        # unrelated pymavlink). Relative observation/model_metadata stay real.
        for name,folder in (("_amsim_node_test",ROOT/"src/amsim"),
                            ("_amsim_node_test.api",ROOT/"src/amsim/api"),
                            ("_amsim_node_test.runtime",ROOT/"src/amsim/runtime")):
            modules[name]=types.ModuleType(name);modules[name].__path__=[str(folder)]
        spec=importlib.util.spec_from_file_location("_amsim_node_test.api.node",ROOT/"src/amsim/api/node.py")
        module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules):spec.loader.exec_module(module)
        return module,modules

    def test_default_unchanged_opt_in_lazy_and_idempotent(self):
        module,modules=self.load_node()
        node=module.ApiNode(["arm_joint_1","arm_joint_2"])
        self.assertEqual(len(node.subscriptions),5)
        self.assertFalse(node.estimator_feedback_enabled)
        for name in ("VehicleAttitude","VehicleAngularVelocity","EstimatorStatusFlags"):
            setattr(modules["px4_msgs.msg"],name,type(name,(),{}))
        modules["rosgraph_msgs.msg"].Clock=type("Clock",(),{})
        with patch.dict(sys.modules,modules):
            node.enable_estimator_feedback();node.enable_estimator_feedback()
        self.assertEqual(len(node.subscriptions),9)
        self.assertEqual({topic for _,topic,_ in node.subscriptions[5:]},{"/clock","/fmu/out/vehicle_attitude",
            "/fmu/out/vehicle_angular_velocity","/fmu/out/estimator_status_flags"})
        self.assertTrue(node.estimator_feedback_enabled)

    def test_receipts_use_only_clock_and_backward_clock_is_sticky_error(self):
        module,_=self.load_node();node=module.ApiNode(["arm_joint_1","arm_joint_2"])
        with patch.object(module.time,"monotonic",return_value=100.):
            node._receive("truth",object());node._receive("angular_velocity",object())
            self.assertIsNone(node.received_sim["angular_velocity"])
            node._receive_estimator_clock(NS(clock=NS(sec=10,nanosec=20000000)))
            node._receive("angular_velocity",object())
        self.assertEqual(node.received_sim["angular_velocity"],10.02)
        self.assertEqual(node.callback_sequences["angular_velocity"],2)
        self.assertEqual(node.received["angular_velocity"],100.)
        node._receive_estimator_clock(NS(clock=NS(sec=9,nanosec=0)))
        self.assertIn("regressed",node.estimator_feedback_error)
        self.assertEqual(node.estimator_clock_s,10.02)
        node._receive_estimator_clock(NS(clock=NS(sec=11,nanosec=0)))
        self.assertIn("regressed",node.estimator_feedback_error)

    def test_invalid_clock_and_preclock_callback_cannot_acquire_truth_time(self):
        module,_=self.load_node();node=module.ApiNode([])
        node.data["truth"]=NS(header=NS(stamp=NS(sec=999,nanosec=0)))
        node._receive("position",object())
        self.assertIsNone(node.received_sim["position"])
        node._receive_estimator_clock(NS(clock=NS(sec=0,nanosec=1000000000)))
        self.assertIsNone(node.estimator_clock_s)
        self.assertIn("Invalid",node.estimator_feedback_error)


class TelemetryProbeTests(unittest.TestCase):
    def fixture(self):
        node=NS(data={},received={},received_sim={},callback_sequences={},
                estimator_clock_s=10.,estimator_feedback_error=None)
        for key,names in probe_module.REQUIRED_FIELDS.items():
            node.data[key]=NS(**{name:0 for name in names})
            node.received[key]=100.;node.received_sim[key]=10.;node.callback_sequences[key]=1
        node.data["status"].arming_state=1;node.data["land"].landed=True
        return node

    def test_pinned_vendor_messages_have_all_required_fields(self):
        for key,typ in probe_module.PX4_TYPES.items():
            source=(ROOT/"vendor/px4_msgs/msg"/(typ+".msg")).read_text(encoding="utf-8")
            fields={match.group(1) for line in source.splitlines()
                    if (match:=re.match(r"^\w+(?:\[\d*\])?\s+(\w+)\s*(?:#.*)?$",line.strip()))}
            with self.subTest(topic=key):
                self.assertFalse(set(probe_module.REQUIRED_FIELDS[key])-fields)
        # This source contract specifically includes flags timestamp_sample;
        # an older generated workspace must be rebuilt, not silently accepted.
        self.assertIn("timestamp_sample",probe_module.REQUIRED_FIELDS["estimator_flags"])

    def test_probe_reports_sequences_age_fields_and_remains_diagnostic(self):
        node=self.fixture();summary=probe_module.TelemetrySummary(["arm_joint_1"],100.)
        self.assertTrue(summary.observe(node,100.))
        node.estimator_clock_s=10.02
        for key in node.data:
            node.callback_sequences[key]+=1;node.received[key]=100.02;node.received_sim[key]=10.02
        self.assertTrue(summary.observe(node,100.04))
        report=summary.report(100.04)
        self.assertTrue(report["passed"]);self.assertTrue(report["diagnostic_only"])
        self.assertFalse(report["eligible_as_flight_gate"])
        self.assertEqual(report["topics"]["attitude"]["last_sequence"],2)
        self.assertAlmostEqual(report["topics"]["attitude"]["max_wall_age_s"],.02)
        self.assertFalse(report["commands_published"])

    def test_missing_field_clock_and_no_progress_are_reported_without_fallback(self):
        node=self.fixture();del node.data["estimator_flags"].timestamp_sample
        node.received_sim["angular_velocity"]=None
        summary=probe_module.TelemetrySummary([],100.);summary.observe(node,100.)
        report=summary.report(100.1)
        self.assertFalse(report["passed"])
        self.assertEqual(report["missing_fields"]["estimator_flags"],["timestamp_sample"])
        self.assertIn("angular_velocity",report["streams_without_clock_receipt"])
        self.assertIn("attitude",report["streams_without_sequence_progress"])

    def test_armed_or_nonlanded_causes_early_abort_without_commands(self):
        for key,name,value in (("status","arming_state",2),("land","landed",False)):
            node=self.fixture();setattr(node.data[key],name,value)
            summary=probe_module.TelemetrySummary([],100.)
            with self.subTest(key=key):
                self.assertFalse(summary.observe(node,100.))
                report=summary.report(100.01)
                self.assertFalse(report["passed"]);self.assertIsNotNone(report["failure"])
                self.assertFalse(report["commands_published"])

    def test_actual_probe_entry_never_closes_with_joint_commands(self):
        node=self.fixture();events=[]
        node._executor=NS(shutdown=lambda:events.append("executor_shutdown"))
        node.destroy_node=lambda:events.append("destroy_node")
        node.close=lambda:self.fail("Legacy close publishes zero commands")
        node.zero_all=lambda:self.fail("No joint commands allowed")
        node.enable_estimator_feedback=lambda:events.append("enable")
        def spin_once(timeout_sec):
            self.assertLessEqual(timeout_sec,.01)
            for key in node.callback_sequences:node.callback_sequences[key]+=1
        node.spin_once=spin_once
        names=("rclpy","amsim","amsim.api","amsim.api.node","amsim.runtime",
               "amsim.runtime.model_metadata","amsim.runtime.process")
        modules={name:types.ModuleType(name) for name in names}
        modules["rclpy"].ok=lambda:True
        modules["amsim.api.node"].ApiNode=lambda joints:node
        modules["amsim.runtime.model_metadata"].resolve_active_model=lambda runtime_state:{"joint_names":[]}
        modules["amsim.runtime.process"].status=lambda:{"running":True,"pid":123,"runtime_meta":"synthetic"}
        ticks=itertools.count(100.,.01)
        with patch.dict(sys.modules,modules),patch.object(probe_module.time,"monotonic",side_effect=lambda:next(ticks)):
            result=probe_module.probe(1.)
        self.assertTrue(result["passed"])
        self.assertEqual(events,["enable","executor_shutdown","destroy_node"])
        for duration in (0,61,float("inf")):
            with self.subTest(duration=duration),self.assertRaises(ValueError):probe_module.probe(duration)


if __name__ == "__main__":
    unittest.main()
