"""Source/frame guard unit tests only; not new GPS performance evidence."""
import copy
import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("gps_plot", ROOT/"scripts/plot_x650_gps_crosscheck.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class GPSPlotLoaderTests(unittest.TestCase):
    def fixture(self):
        params = {"CONTROL_"+str(i): float(i) for i in range(42)}
        effective = dict(EKF2_EV_CTRL=0, EKF2_GPS_CTRL=7, EKF2_BARO_CTRL=1, EKF2_HGT_REF=1)
        cfg = dict(verified=True, fusion_flags_entire_experiment=audit.GPS_FLAGS.copy(),
            origin_and_reset=dict(origin_and_reset_stable=True),
            effective_parameters=dict(parameters_at_measurement_start=effective, parameter_changes_in_window=[]),
            actual_control_42=params.copy())
        return dict(metadata=dict(ground_truth_aided=False, aiding_confirmed=False,
                task=dict(diagnostic_profile=None), actual_pid_parameters=params.copy()),
            reference=dict(frame_mode="global_wgs84"), precision=dict(native_only=True),
            guard=dict(verified=True, ground_truth_aided=False,
                all_42_boot_and_restored_parameters_match_original_gps=True, configuration=cfg))

    def test_global_gps_mode_is_separate_from_raw_ev_or_truth_aiding(self):
        audit.verify_gps_mode(self.fixture())
        for field, value in (("frame_mode", "raw_ev_world"),):
            data = self.fixture(); data["reference"][field] = value
            with self.assertRaisesRegex(ValueError, "never raw-EV"):
                audit.verify_gps_mode(data)
        data = self.fixture(); data["metadata"]["ground_truth_aided"] = True
        with self.assertRaisesRegex(ValueError, "unaided"):
            audit.verify_gps_mode(data)

    def test_changed_fusion_or_gains_do_not_pass_gps_contract(self):
        data = self.fixture(); data["guard"]["configuration"]["fusion_flags_entire_experiment"]["cs_ev_pos"] = 1
        with self.assertRaisesRegex(ValueError, "fusion/origin/parameter"):
            audit.verify_gps_mode(data)
        data = self.fixture(); data["guard"]["configuration"]["actual_control_42"]["CONTROL_1"] = 1.2
        with self.assertRaisesRegex(ValueError, "42 gains"):
            audit.verify_gps_mode(data)

    def test_current_run_hash_not_confused_with_old_baseline_same_basename(self):
        guard = dict(source_run="CURRENT", inputs_sha256={"/runs/CURRENT/controller_metadata.json": "current",
            "/runs/ORIGINAL/controller_metadata.json": "original"})
        self.assertEqual(audit.guard_source_hash(guard, "controller_metadata.json"), "current")
        guard["inputs_sha256"]["/other/CURRENT/controller_metadata.json"] = "duplicate"
        with self.assertRaisesRegex(ValueError, "exactly one current-run"):
            audit.guard_source_hash(guard, "controller_metadata.json")

    def test_full56_and_static20_windows_cannot_be_interchanged(self):
        audit.same_window([10., 66.], [10., 66.], "full")
        with self.assertRaisesRegex(ValueError, "window differs"):
            audit.same_window([10., 30.], [10., 66.], "full")


if __name__ == "__main__":
    unittest.main()
