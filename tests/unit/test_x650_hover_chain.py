from pathlib import Path
from types import SimpleNamespace as NS
import csv
import importlib.util
import sys
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("x650_hover_chain", ROOT/"scripts/analyze_x650_hover_chain.py")
chain = importlib.util.module_from_spec(spec)
spec.loader.exec_module(chain)


class HoverChainTests(unittest.TestCase):
    def test_same_clock_interpolation_has_no_extrapolation_or_phase_fit(self):
        t = np.array([1., 1.1, 1.2])
        v = np.column_stack((t, t*2))
        np.testing.assert_allclose(chain.resample(t, v, [1.05], max_gap=.11), [[1.05, 2.1]])
        np.testing.assert_allclose(chain.resample(t, v, [1.05], max_gap=.11, hold=True), [[1., 2.]])
        with self.assertRaisesRegex(ValueError, "no extrapolation"):
            chain.resample(t, v, [.9], max_gap=.11)
        with self.assertRaisesRegex(ValueError, "missing-data gap"):
            chain.resample(t, v, [1.05], max_gap=.05)

    def test_known_geo_origin_does_not_infer_spawn_altitude_from_gt_reference(self):
        world = dict(latitude_deg=47., longitude_deg=8., elevation=488., heading_deg=0.)
        p = chain.geodetic_to_world(np.array([47.]), np.array([8.]), np.array([488.375]), world)
        np.testing.assert_allclose(p, [[0., 0., .375]], atol=2e-9)
        # EKF origin may be at the initial sensor's 488.375m altitude.
        local = chain.local_to_world(np.array([[0., 0., -2.]]), np.array([[47., 8., 488.375]]), world)
        np.testing.assert_allclose(local, [[0., 0., 2.375]], atol=2e-9)

    def test_partial_diagnostic_separates_real_target_from_post_takeoff_anchor(self):
        t = np.round(np.arange(0., 3.004, .004), 9)
        p = np.column_stack((.04*np.sin(t), .02*np.cos(t), np.full(len(t), 2.375)))
        estimated = np.column_stack((np.full(len(t), .01), np.full(len(t), .02), np.full(len(t), -2.)))
        def ds(name, times, positions):
            return NS(name=name, multi_id=0, data={"timestamp": times*1e6, "timestamp_sample": times*1e6,
                **{axis: positions[:, i] for i, axis in enumerate("xyz")}})
        datasets = [ds("vehicle_local_position", t, estimated),
                    ds("vehicle_local_position_groundtruth", t[::5], chain.transform_ned_enu(p[::5])),
                    ds("vehicle_local_position_setpoint", t[::25], np.tile([0., 0., -2.], (len(t[::25]), 1)))]
        ulog = NS(data_list=datasets, initial_parameters={"EKF2_IMU_POS_X": 0., "EKF2_IMU_POS_Y": 0., "EKF2_IMU_POS_Z": 0.})
        meta = {"task_start_sim_s": 1., "task_end_sim_s": 2., "reference_base_enu_m": [9., 9., 9.],
                "hover_metrics": {"fixed_target_enu_m": [0., 0., 2.375]}}
        with tempfile.TemporaryDirectory() as tmp:
            native = Path(tmp)/"native.csv"
            with native.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["sim_s", "valid_base", "base_px", "base_py", "base_pz"])
                writer.writerows((stamp, 1, *position) for stamp, position in zip(t, p))
            result, columns = chain.analyze(ulog, native, meta)
        self.assertFalse(result["world_chain_available"])
        self.assertEqual(result["time_alignment"]["shift_s"], 0.)
        self.assertTrue(result["time_alignment"]["bridge_error_enu_m"]["direct_no_shift_consistent_within_5mm"])
        self.assertAlmostEqual(result["metrics"]["estimate_minus_controller_setpoint_ned"]["rmse_3d_m"], np.hypot(.01, .02))
        self.assertLess(result["metrics"]["truth_minus_task_target_enu"]["rmse_3d_m"], .05)
        self.assertEqual(result["metrics"]["controller_setpoint_change_from_window_start_ned"]["peak_3d_m"], 0.)

    def test_epoch_timestamps_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "not DDS/ROS epoch"):
            chain.sample_time({"timestamp": np.array([1792554749151522, 1792554749155522])})


if __name__ == "__main__":
    unittest.main()
