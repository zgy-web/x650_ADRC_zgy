"""Pure parsing and observer/fusion protocol checks; no Gazebo or flight."""
from pathlib import Path
import json
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from diagnostic_odometry import DiagnosticOdometry, model_entity_id, protobuf_odometry_samples, listener_samples, fusion_status


def odometry(seconds, x=0.):
    return 'header {\n stamp {\n sec: %d\n nsec: 20000000\n }\n}\npose {\n pose {\n position {\n x: %s\n y: 0\n z: 0.375\n }\n }\n}\n' % (seconds, x)


def visual(stamp, *, x="0.1", quality=0):
    return f'TOPIC: vehicle_visual_odometry\n timestamp: {stamp} (0.01 seconds ago)\n timestamp_sample: {stamp-100}\n pose_frame: 1\n position: [{x}, 0.0, -0.375]\n quality: {quality}\n'


def flags(value, stamp=2000000):
    return f'TOPIC: estimator_status_flags\n timestamp: {stamp} (0.01 seconds ago)\n cs_ev_pos: {value}\n'


class DiagnosticObserverTests(unittest.TestCase):
    def observer(self, folder):
        px4 = folder/"px4"
        listener = px4/"build/px4_sitl_default/bin/px4-listener"
        listener.parent.mkdir(parents=True)
        listener.write_text("mock executable", encoding="utf-8")
        (folder/".amsim").mkdir()
        metadata = folder/"runtime_meta.json"
        metadata.write_text(json.dumps({"instance": "amsim_urdf_x650_2link_v2_flight_0",
            "joint_state_gz": "/world/bandwidth_test/model/amsim_urdf_x650_2link_v2_flight_0/joint_state"}), encoding="utf-8")
        (folder/".amsim/runtime.json").write_text(json.dumps({"runtime_meta": str(metadata)}), encoding="utf-8")
        return DiagnosticOdometry(folder, "amsim_urdf_x650_2link_v2_flight_0", folder, px4_directory=px4)

    def test_model_entity_exact_match_and_ambiguous_rejection(self):
        source = 'model {\n name: "target_0"\n id: 12\n link {\n name: "base_link"\n id: 13\n }\n}\nmodel {\n name: "other"\n id: 21\n}'
        self.assertEqual(model_entity_id(source, "target_0"), 12)
        with self.assertRaises(RuntimeError):
            model_entity_id(source, "target")
        with self.assertRaises(RuntimeError):
            model_entity_id(source+source, "target_0")

    def test_live_gazebo_stream_is_finite_stamped_and_incomplete_tail_ignored(self):
        samples = protobuf_odometry_samples(odometry(10)+odometry(11, 0.2)+'header { stamp { sec:')
        self.assertEqual([s["stamp_ns"] for s in samples], [10020000000, 11020000000])
        self.assertEqual(samples[1]["position_enu_m"], [.2, 0., .375])
        with self.assertRaisesRegex(RuntimeError, "Nonfinite"):
            protobuf_odometry_samples(odometry(10, "nan"))

    def test_px4_visual_stream_requires_position_sample_frame_and_quality(self):
        samples = listener_samples(visual(1000000)+visual(1020000))
        self.assertEqual(len(samples), 2)
        self.assertEqual(samples[0]["quality"], 0)  # Native bridge may leave this unused; EKF flag is authoritative.
        self.assertEqual(listener_samples(visual(1000000).replace(" quality: 0\n", "")), [])
        with self.assertRaisesRegex(RuntimeError, "finite"):
            listener_samples(visual(1000000, x="nan"))
        with self.assertRaisesRegex(RuntimeError, "NED"):
            listener_samples(visual(1000000).replace("pose_frame: 1", "pose_frame: 2"))

    def test_fusion_gate_requires_explicit_fresh_horizontal_ev_flag(self):
        self.assertTrue(fusion_status(flags("True"))["cs_ev_pos"])
        self.assertFalse(fusion_status(flags("0"))["cs_ev_pos"])
        with self.assertRaises(RuntimeError):
            fusion_status("cs_ev_pos: true")
        with self.assertRaisesRegex(RuntimeError, "stale"):
            fusion_status(flags("true").replace("0.01 seconds", "3.1 seconds"))

    def test_precision_fusion_requires_explicit_flags_for_all_sources(self):
        expected = {"cs_ev_pos": True, "cs_ev_vel": True, "cs_ev_hgt": True,
                    "cs_gnss_pos": False, "cs_gnss_vel": False, "cs_gps_hgt": False, "cs_baro_hgt": False}
        text = flags("true")+"\n".join(name+": "+str(value) for name, value in expected.items() if name != "cs_ev_pos")
        report = fusion_status(text, expected)
        self.assertTrue(all(report[name] is value for name, value in expected.items()))
        with self.assertRaisesRegex(RuntimeError, "cs_ev_vel"):
            fusion_status(text.replace("cs_ev_vel: True", ""), expected)
        mixed = fusion_status(text.replace("cs_gnss_pos: False", "cs_gnss_pos: True"), expected)
        self.assertFalse(all(mixed[name] is value for name, value in expected.items()))

    def test_protocol_loads_model_observer_and_gate_is_separate(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            observer = self.observer(folder)
            calls = []
            visual_stamp = 1000000
            def command(args, pump, **kwargs):
                nonlocal visual_stamp
                calls.append(args)
                if "/world/bandwidth_test/scene/info" in args:
                    return 'model {\n name: "amsim_urdf_x650_2link_v2_flight_0"\n id: 20\n}'
                if "/world/bandwidth_test/entity/system/add" in args:
                    return "data: true\n"
                if args[:2] == ["gz", "topic"]:
                    return odometry(10)+odometry(11)
                if "estimator_status_flags" in args:
                    return flags("true")
                self.assertEqual(args[-2:], ["-n", "1"])
                visual_stamp += 100000
                return visual(visual_stamp)
            observer._run = command
            loaded = observer.load(lambda: None)
            self.assertTrue(loaded["observer_loaded"])
            self.assertTrue(loaded["input_stream_confirmed"])
            self.assertFalse(loaded["aiding_confirmed"])
            request = next(args[-1] for args in calls if "/world/bandwidth_test/entity/system/add" in args)
            self.assertIn("entity { id: 20 }", request)
            self.assertIn("/model/amsim_urdf_x650_2link_v2_flight_0/odometry_with_covariance", request)
            self.assertNotIn("/ground_truth/odometry", request)
            self.assertNotIn("mass", request)
            self.assertNotIn("inertial", request)
            self.assertTrue(observer.confirm_horizontal_fusion(lambda: None)["aiding_confirmed"])
            self.assertTrue((folder/"diagnostic_odometry.json").is_file())
            with self.assertRaisesRegex(RuntimeError, "already loaded"):
                observer.load(lambda: None)

    def test_failed_service_never_claims_loaded_or_aided(self):
        with tempfile.TemporaryDirectory() as tmp:
            observer = self.observer(Path(tmp))
            def command(args, pump, **kwargs):
                if "/world/bandwidth_test/scene/info" in args:
                    return 'model {\n name: "amsim_urdf_x650_2link_v2_flight_0"\n id: 20\n}'
                return "data: false"
            observer._run = command
            with self.assertRaisesRegex(RuntimeError, "did not confirm"):
                observer.load(lambda: None)
            self.assertFalse(observer.document["observer_loaded"])
            self.assertFalse(observer.document["aiding_confirmed"])
            with self.assertRaisesRegex(RuntimeError, "without"):
                observer.confirm_horizontal_fusion(lambda: None)

    def test_replayed_visual_samples_cannot_certify_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            observer = self.observer(Path(tmp))
            observer._run = lambda *args, **kwargs: visual(1000000)+visual(1000000)
            with self.assertRaisesRegex(RuntimeError, "advancing"):
                observer._visual_stream(lambda: None, timeout_s=.01)

    def test_snapshot_gate_waits_for_real_sample_time_progress(self):
        with tempfile.TemporaryDirectory() as tmp:
            observer = self.observer(Path(tmp))
            stamps = iter((1000000, 1000000, 1040000, 1100000))
            requests = []
            def command(args, pump, **kwargs):
                requests.append(args)
                return visual(next(stamps))
            observer._run = command
            samples = observer._visual_stream(lambda: None)
            self.assertEqual(samples[1]["timestamp_sample_us"]-samples[0]["timestamp_sample_us"], 100000)
            self.assertEqual(len(requests), 4)
            self.assertTrue(all(args[-2:] == ["-n", "1"] for args in requests))


if __name__ == "__main__":
    unittest.main()
