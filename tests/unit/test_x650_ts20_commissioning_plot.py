"""SYNTHETIC plotting regressions only; no native run or actual score output.

Fixtures are temporary toy CSVs. The joint test mocks numerical propagation
to verify the recomputation path; it is not a predictor/oracle validation.
No figure, certificate, NPZ or actual experimental result is produced.
"""
from contextlib import contextmanager, redirect_stderr
import csv
import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "scripts/plot_x650_ts20_commissioning_results.py"
spec = importlib.util.spec_from_file_location("synthetic_ts20_plot", SOURCE)
plot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plot)


@contextmanager
def synthetic_space():
    folder = ROOT / (".tmp_synthetic_ts20_plot_" + uuid.uuid4().hex)
    folder.mkdir()
    try:
        yield folder
    finally:
        for file in folder.iterdir():
            file.unlink()
        folder.rmdir()


def write_csv(path, rows):
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def hold_fixture(folder, actual_duration):
    relative = np.arange(round(actual_duration / .004) + 1) * .004
    times = 17. + relative
    target = np.array([.2, -.1, .8])
    error = np.column_stack([.001 * relative, .0002 * np.sin(relative), np.zeros(len(relative))])
    nominal = relative <= 30. + 1e-9
    # The post-score tail is deliberately different. It must remain excluded
    # even when the actual loop completion is a few milliseconds after 30s.
    error[~nominal] = 100.
    tip = target + error
    native = [{"sim_s":float(t), "valid_tip":1, "valid_joint_state":1, "valid_base":1,
               "tip_x":float(position[0]), "tip_y":float(position[1]), "tip_z":float(position[2]),
               "arm_joint_1_q":0., "arm_joint_2_q":0.}
              for t, position in zip(times, tip)]
    write_csv(folder / "native_trace.csv", native)
    flight = [{"phase":"measure", "sim_time_s":17.+at,
               **{"tip_target_world_"+axis+"_m":float(value) for axis,value in zip("xyz",target)}}
              for at in (0.,15.,30.,actual_duration)]
    write_csv(folder / "flight_samples.csv", flight)
    nominal_error = tip[nominal]-target
    integrate = getattr(np, "trapezoid", None) or np.trapz
    metric = {"interval_task_s":[0.,30.],
              "position_rmse_m":float(np.sqrt(integrate(np.sum(nominal_error**2,axis=1),times[nominal]) / 30.)),
              "position_max_m":float(np.linalg.norm(nominal_error,axis=1).max())}
    return {"duration_s":30., "disturbance_enabled":False}, {"result":{"endpoint":metric}}, np.array([17.,17.+actual_duration])


class SyntheticHoldScoringWindow(unittest.TestCase):
    def test_nominal_certificate_window_keeps_actual_30_004_end_separate(self):
        with synthetic_space() as folder:
            cfg, cert, actual = hold_fixture(folder,30.004)
            binding = {"window_sim_s":actual.tolist()}
            data = plot.hold_data(folder,cfg,cert,actual,binding)
            self.assertEqual(binding["window_sim_s"],[17.,47.])
            self.assertEqual(binding["metric_interval_task_s"],[0.,30.])
            self.assertEqual(binding["actual_task_window_sim_s"],actual.tolist())
            self.assertEqual(binding["actual_task_end_sim_s"],float(actual[1]))
            self.assertEqual(data["time_sim_s"][-1],47.)
            self.assertEqual(len(data["time_sim_s"]),7501)
            self.assertAlmostEqual(binding["position_rmse_m"],cert["result"]["endpoint"]["position_rmse_m"],places=12)

    def test_30_02_runtime_tail_is_not_added_to_nominal_scores_or_plot(self):
        with synthetic_space() as folder:
            cfg, cert, actual = hold_fixture(folder,30.02)
            binding = {}
            data = plot.hold_data(folder,cfg,cert,actual,binding)
            self.assertEqual(binding["actual_task_end_sim_s"],float(actual[1]))
            self.assertLess(float(np.linalg.norm(data["tip_error_world_enu_m"],axis=1).max()),.04)
            self.assertEqual(binding["window_sim_s"],[17.,47.])

    def test_missing_or_partial_certificate_scoring_interval_is_rejected(self):
        with synthetic_space() as folder:
            cfg, cert, actual = hold_fixture(folder,30.004)
            cert["result"]["endpoint"].pop("interval_task_s")
            with self.assertRaisesRegex(ValueError,"exact endpoint scoring interval"):
                plot.hold_data(folder,cfg,cert,actual,{})
            cert["result"]["endpoint"]["interval_task_s"]=[0.,29.996]
            with self.assertRaisesRegex(ValueError,"complete declared task-relative interval"):
                plot.hold_data(folder,cfg,cert,actual,{})


class SyntheticJointForecastRecomputation(unittest.TestCase):
    def fixture(self,folder):
        t=np.arange(7501)*.004
        q=np.column_stack([.05*np.sin(t),.03*np.cos(t)-.03])
        endpoint=np.column_stack([.05*np.cos(t),-.03*np.sin(t)])
        interval=endpoint-.0001
        state=[{"sim_s":float(at),"q1_physical":float(qi[0]),"q2_physical":float(qi[1]),
                "dq1_internal":float(vi[0]),"dq2_internal":float(vi[1]),
                "dq1_physical":float(vm[0]),"dq2_physical":float(vm[1])}
               for at,qi,vi,vm in zip(t,q,endpoint,interval)]
        write_csv(folder/"ts20_state.csv",state)
        packets=[{"send_s":"0", "qref1":"0", "qref2":"0", "dqref1":"0", "dqref2":"0"}]
        write_csv(folder/"ts20_packets.csv",packets)
        qe=np.column_stack([.001*np.sin(t),.0005*np.sin(t)])
        ve=np.column_stack([.005*np.sin(t),.002*np.sin(t)])
        cert={"actuator":{"prediction":{"q_peak_abs_rad":float(abs(qe).max()),"dq_peak_abs_rad_s":float(abs(ve).max())}}}
        return t,q,endpoint,interval,packets,q+qe,endpoint+ve,cert

    def test_external_array_attribute_cannot_bypass_fresh_packet_propagation(self):
        with synthetic_space() as folder:
            t,q,v,mean_v,packets,pq,pv,cert=self.fixture(folder)
            cfg={"synthetic_fixture":True}
            args=SimpleNamespace(forecast_npz="untrusted_ignored_attribute.npz")
            binding={}
            with patch.object(plot,"free_forecast",return_value=(pq,pv)) as propagate, \
                 patch.object(plot.np,"load",side_effect=AssertionError("External NPZ must not be read")):
                data=plot.joint_data(folder,cfg,cert,np.array([0.,30.]),binding,args)
            call=propagate.call_args.args
            self.assertIs(call[0],cfg);self.assertIs(call[1],cert)
            np.testing.assert_array_equal(call[2],t)
            np.testing.assert_array_equal(call[3],q[0]);np.testing.assert_array_equal(call[4],v[0])
            self.assertEqual(call[5],packets)
            np.testing.assert_array_equal(data["actual_physical_interval_dq_rad_s"],mean_v)
            np.testing.assert_array_equal(data["actual_endpoint_dq_rad_s"],v)
            self.assertTrue(binding["first_state_only"])
            self.assertFalse(binding["later_measured_state_resets"])

    def test_recomputed_forecast_still_must_match_certificate_scores(self):
        with synthetic_space() as folder:
            t,q,v,mean_v,packets,pq,pv,cert=self.fixture(folder)
            with patch.object(plot,"free_forecast",return_value=(q,v)):
                with self.assertRaisesRegex(ValueError,"differs from accepted whole-window certificate"):
                    plot.joint_data(folder,{},cert,np.array([0.,30.]),{},SimpleNamespace())

    def test_removed_forecast_npz_cli_is_rejected_before_run_io(self):
        argv=[str(SOURCE),"--run-dir","synthetic_run","--certificate","synthetic_certificate.json",
              "--output","synthetic_output","--forecast-npz","untrusted.npz"]
        with patch.object(plot.sys,"argv",argv),patch.object(plot,"bound_run") as run_io,redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                plot.main()
            self.assertEqual(error.exception.code,2)
            run_io.assert_not_called()


if __name__=="__main__":
    unittest.main()
