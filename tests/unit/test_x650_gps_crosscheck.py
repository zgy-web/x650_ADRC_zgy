"""Read-only GPS audit gate tests with synthetic ULog records, never flight."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("x650_gps_guard_test", ROOT/"scripts/audit_x650_gps_crosscheck.py")
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


class GpsCrosscheckTests(unittest.TestCase):
    def setUp(self):
        self.times = np.array([10., 11., 12., 13.])
        names = guard.candidates.load_parameter_evidence().CORE_CONTROL_PARAMETERS
        self.originals = {name: {"value": 1.} for name in names}
        self.override = guard.candidates.PID_CANDIDATES["velocity_high"]
        params = {name: record["value"] for name, record in self.originals.items()}
        params.update(EKF2_EV_CTRL=0, EKF2_GPS_CTRL=7, EKF2_BARO_CTRL=1, EKF2_HGT_REF=1)
        self.flags = {"timestamp": np.arange(9., 15.)*1e6,
                      **{name: np.full(6, value) for name, value in guard.GPS_FLAGS.items()}}
        self.ulog = NS(initial_parameters=params,
            changed_parameters=[(9000000, name, value) for name, value in self.override.items()]
                +[(20000000, name, 1.) for name in self.override],
            data_list=[NS(name="estimator_status_flags", multi_id=0, data=self.flags)])
        self.origin = patch.object(guard.precision, "origin_and_fusion_audit", return_value={"verified": True})
        self.origin.start()
        self.addCleanup(self.origin.stop)

    def audit(self):
        return guard.effective_configuration(self.ulog, self.times, {"frame_mode": "global_wgs84"}, self.originals, self.override)

    def test_effective_window_pid_includes_preflight_change_not_later_restore(self):
        result = self.audit()
        self.assertTrue(result["verified"])
        self.assertEqual(result["candidate_readback"], self.override)
        self.assertEqual(len(result["actual_control_42"]), 42)
        self.assertEqual(result["fusion_flags_entire_experiment"], guard.GPS_FLAGS)

    def test_wrong_candidate_or_other_pid_gain_rejected(self):
        for name in ("MPC_XY_VEL_P_ACC", "MC_ROLL_P"):
            with self.subTest(name=name):
                self.ulog.changed_parameters.append((9500000, name, 99.))
                with self.assertRaisesRegex(ValueError, "Effective measurement PID"):
                    self.audit()
                self.ulog.changed_parameters.pop()

    def test_parameter_change_inside_response_window_rejected(self):
        self.ulog.changed_parameters.append((12500000, "MPC_XY_P", .8))
        with self.assertRaisesRegex(ValueError, "changed inside"):
            self.audit()

    def test_transient_gps_drop_or_ev_fusion_rejected(self):
        for name in ("cs_gnss_vel", "cs_baro_hgt", "cs_ev_pos"):
            with self.subTest(name=name):
                self.flags[name][2] = 1-guard.GPS_FLAGS[name]
                with self.assertRaisesRegex(ValueError, "fusion differs"):
                    self.audit()
                self.flags[name][2] = guard.GPS_FLAGS[name]

    def test_missing_flag_coverage_and_noncanonical_height_source_rejected(self):
        self.flags["timestamp"] = np.arange(5., 11.)*1e6
        with self.assertRaises(ValueError):
            self.audit()
        self.flags["timestamp"] = np.arange(9., 15.)*1e6
        self.ulog.initial_parameters["EKF2_HGT_REF"] = 3
        with self.assertRaisesRegex(ValueError, "Canonical GPS"):
            self.audit()


if __name__ == "__main__":
    unittest.main()
