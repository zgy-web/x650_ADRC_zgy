"""Synthetic audit fixtures only; never flight or controller-performance claims."""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("hover_precision_audit", ROOT/"scripts/analyze_x650_hover_precision.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class HoverPrecisionAnalysisTests(unittest.TestCase):
    def test_extended_native_capture_does_not_extend_primary_hover_metrics(self):
        self.assertEqual(audit.capture_window_covers({"task_window_sim_s": [10., 66.]}, [10., 30.]), [10., 66.])
        self.assertEqual(audit.capture_window_covers({"task_window_sim_s": [10., 30.]}, [10., 30.]), [10., 30.])
        for capture in ([10.01, 66.], [10., 29.99], [10., 10.], None, [10., float("nan")]):
            with self.assertRaisesRegex(ValueError, "cover"):
                audit.capture_window_covers({"task_window_sim_s": capture}, [10., 30.])

    def reference(self):
        bound = dict(ref_lat=47., ref_lon=8., ref_alt=10., xy_valid=True, z_valid=True,
                     xy_global=False, z_global=True, xy_reset_counter=0, z_reset_counter=0, heading_reset_counter=0)
        return dict(kind="x650_fixed_world_hover_reference", schema_version=1, profile="fixed_world_position_v1",
                    frame_mode="raw_ev_world", target_frozen=True, truth_feedback_in_setpoint=False,
                    fit_estimator_translation=False, coordinate_frame=1, type_mask=2552,
                    px4_source_contract_commit=audit.PINNED_PX4, target_world_enu_m=[1., 2., 3.],
                    target_local_ned_m=[2., 1., 7.], px4_reference_at_bind=bound,
                    source_sha256=audit.sha(audit.DEFAULT_REFERENCE_SOURCE))

    def fake_ulog(self):
        t = np.arange(.98, 2.0201, .004)
        ref = self.reference()
        lpos = {"timestamp_sample": t*1e6, "x": 2+2*t, "y": 1+t, "z": 7-3*t,
                "vx": np.full(len(t), 2.), "vy": np.full(len(t), 1.), "vz": np.full(len(t), -3.)}
        lpos.update({k: np.full(len(t), v) for k, v in ref["px4_reference_at_bind"].items()})
        sp = {"timestamp": t*1e6, "x": np.full(len(t), 2.), "y": np.full(len(t), 1.),
              "z": np.full(len(t), 7.), "vx": np.zeros(len(t)), "vy": np.zeros(len(t)), "vz": np.zeros(len(t))}
        gt = {k: v for k, v in lpos.items() if k in ("timestamp_sample", "vx", "vy", "vz")}
        flags = {"timestamp": t*1e6}
        flags.update({name: np.full(len(t), value) for name, value in zip(
            ("cs_ev_pos", "cs_ev_vel", "cs_ev_hgt", "cs_gnss_pos", "cs_gps_hgt", "cs_baro_hgt"), (1, 1, 1, 0, 0, 0))})
        params = dict(EKF2_EV_CTRL=7, EKF2_EV_NOISE_MD=1, EKF2_GPS_CTRL=0, EKF2_BARO_CTRL=0, EKF2_HGT_REF=3, EKF2_EV_DELAY=0)
        params.update({"EKF2_"+sensor+"_POS_"+axis: 0 for sensor in ("EV", "IMU") for axis in "XYZ"})
        rows = dict(vehicle_local_position=lpos, vehicle_local_position_setpoint=sp,
                    vehicle_local_position_groundtruth=gt, estimator_status_flags=flags)
        return NS(initial_parameters=params, changed_parameters=[],
                  data_list=[NS(name=name, multi_id=0, data=data) for name, data in rows.items()])

    def test_vector_metrics_distinguish_xy_z_and_preserve_bias(self):
        t = np.linspace(0, 20, 5001)
        result = audit.metrics(t, np.tile([3., 4., 12.], (len(t), 1)), "m")
        self.assertAlmostEqual(result["rmse_3d"], 13.)
        self.assertAlmostEqual(result["rmse_xy"], 5.)
        self.assertAlmostEqual(result["rmse_z"], 12.)
        self.assertAlmostEqual(result["mean_vector_norm"], 13.)
        self.assertAlmostEqual(result["centered_rms_3d"], 0.)

    def test_reference_source_type_mask_and_fixed_target_contract(self):
        ref = self.reference()
        np.testing.assert_array_equal(audit.verify_reference(ref, audit.DEFAULT_REFERENCE_SOURCE), [1., 2., 3.])
        ref["truth_feedback_in_setpoint"] = True
        with self.assertRaisesRegex(ValueError, "contract"):
            audit.verify_reference(ref, audit.DEFAULT_REFERENCE_SOURCE)

    def test_measurement_effective_parameters_not_initial_or_restored(self):
        ulog = NS(initial_parameters={"MPC_XY_P": .95}, changed_parameters=[(500000, "MPC_XY_P", 1.2), (2100000, "MPC_XY_P", .95)])
        report = audit.effective_parameters(ulog, (1., 2.))
        self.assertEqual(report["parameters_at_measurement_start"]["MPC_XY_P"], 1.2)
        ulog.changed_parameters.append((1500000, "MPC_XY_P", .75))
        with self.assertRaisesRegex(ValueError, "inside"):
            audit.effective_parameters(ulog, (1., 2.))

    def test_raw_ev_chain_uses_retained_altitude_origin_and_real_velocity(self):
        t = np.arange(1., 2.0001, .004)
        native = dict(t=t, position=np.column_stack((1+t, 2+2*t, 3+3*t)), velocity=np.tile([1., 2., 3.], (len(t), 1)))
        report = audit.ulog_chain(self.fake_ulog(), native, self.reference(), audit.DEFAULT_REFERENCE_SOURCE)
        self.assertLess(report["position_world_enu"]["native_minus_estimate"]["peak_3d"], 1e-13)
        self.assertLess(report["velocity_world_enu"]["native_minus_estimate"]["peak_3d"], 1e-13)
        self.assertTrue(report["fixed_world_target_verified_within_2mm"])
        self.assertAlmostEqual(report["velocity_world_enu"]["native_minus_internal_velocity_setpoint"]["rmse_3d"], np.sqrt(14.))

    def test_origin_reset_and_wrong_fusion_flags_rejected(self):
        t, ulog = np.arange(1., 2.0001, .004), self.fake_ulog()
        audit.origin_and_fusion_audit(ulog, self.reference(), t)
        lpos = audit.dataset(ulog, "vehicle_local_position")
        lpos["xy_reset_counter"][10] = 1
        with self.assertRaisesRegex(ValueError, "reset/origin"):
            audit.origin_and_fusion_audit(ulog, self.reference(), t)
        ulog = self.fake_ulog()
        audit.dataset(ulog, "estimator_status_flags")["cs_gnss_pos"][:] = 1
        with self.assertRaisesRegex(ValueError, "fusion flags"):
            audit.origin_and_fusion_audit(ulog, self.reference(), t)

    def test_full_experiment_guard_rejects_changes_after_primary_hover(self):
        short = dict(t=np.arange(1., 1.4001, .004))
        full = dict(t=np.arange(1., 2.0001, .004))
        ulog = self.fake_ulog()
        result = audit.full_experiment_ulog_guard(ulog, full, self.reference(), [1., 2.])
        self.assertTrue(result["verified"])
        self.assertFalse(result["position_setpoint_constancy_checked_here"])
        ulog.changed_parameters = [(1800000, "MPC_XY_VEL_P_ACC", 2.2)]
        audit.full_experiment_ulog_guard(ulog, short, self.reference(), [1., 1.4])
        with self.assertRaisesRegex(ValueError, "inside the measurement"):
            audit.full_experiment_ulog_guard(ulog, full, self.reference(), [1., 2.])
        ulog = self.fake_ulog()
        lpos = audit.dataset(ulog, "vehicle_local_position")
        lpos["xy_reset_counter"][lpos["timestamp_sample"] >= 1800000] = 1
        audit.full_experiment_ulog_guard(ulog, short, self.reference(), [1., 1.4])
        with self.assertRaisesRegex(ValueError, "reset/origin"):
            audit.full_experiment_ulog_guard(ulog, full, self.reference(), [1., 2.])
        ulog = self.fake_ulog()
        flags = audit.dataset(ulog, "estimator_status_flags")
        flags["cs_gnss_pos"][flags["timestamp"] >= 1800000] = 1
        audit.full_experiment_ulog_guard(ulog, short, self.reference(), [1., 1.4])
        with self.assertRaisesRegex(ValueError, "fusion flags"):
            audit.full_experiment_ulog_guard(ulog, full, self.reference(), [1., 2.])

    def test_interpolation_does_not_bridge_native_or_logged_gaps(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"SYNTHETIC_native.csv"
            names = ["sim_s", "valid_base", *["base_p"+a for a in "xyz"], *["base_v"+a for a in "xyz"], *["base_q"+a for a in "wxyz"]]
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=names); writer.writeheader()
                for t in np.r_[np.arange(0, 10, .004), np.arange(10.02, 20.0001, .004)]:
                    row = {k: 0 for k in names}; row.update(sim_s=t, valid_base=1, base_qw=1.)
                    writer.writerow(row)
            with self.assertRaisesRegex(ValueError, "coverage"):
                audit.native_states(path, [0., 20.])
        with self.assertRaisesRegex(ValueError, "gap"):
            audit.resample([0., 1.], np.zeros((2, 3)), [.1, .9], max_gap=.024)


if __name__ == "__main__":
    unittest.main()
