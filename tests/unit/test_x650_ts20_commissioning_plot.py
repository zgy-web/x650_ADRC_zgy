"""SYNTHETIC plotting regressions only; no native run or actual score output.

Fixtures are temporary toy CSVs. The joint test mocks numerical propagation
to verify the recomputation path; it is not a predictor/oracle validation.
Temporary certificate-shaped JSONs are synthetic identity fixtures only.
No figure, NPZ or actual experimental result is produced.
"""
from contextlib import contextmanager, redirect_stderr
import csv
import importlib.util
import io
import json
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


def write_json(path, value):
    path.write_text(json.dumps(value), encoding="utf-8")


def oracle_fixture(folder):
    source = folder/"synthetic_prediction.py"
    source.write_text("# SYNTHETIC source identity only; never executed.\n", encoding="utf-8")
    oracle = folder/"synthetic_oracle.json"
    write_json(oracle, {"complete": True, "pass": True, "candidate_sha256": plot.sha(source)})
    cfg = {"ts20_inputs": {"oracle_validation": str(oracle)}}
    cert = {"ts20_operating_condition": {"oracle_validation_sha256": plot.sha(oracle)}}
    return source, oracle, cfg, cert


def recorded_prior():
    return {"path": "/synthetic/old_model_prior.json", "sha256": "a"*64,
            "source_model_sha256": "b"*64, "new_plant_identified": False}


class SyntheticPlotSourceProvenance(unittest.TestCase):
    def test_actual_imported_module_source_is_bound_to_the_hash_bound_oracle(self):
        with synthetic_space() as folder:
            source, oracle, cfg, cert = oracle_fixture(folder)
            with patch.object(plot.importlib, "import_module", return_value=SimpleNamespace(__file__=str(source))) as imported:
                binding = plot.predictor_source_binding(cfg, cert)
            imported.assert_called_once_with("x650_wholebody.ts20_prediction")
            self.assertEqual(binding["path"], str(source.resolve()))
            self.assertEqual(binding["sha256"], plot.sha(source))
            self.assertEqual(binding["oracle_candidate_sha256"], plot.sha(source))
            self.assertEqual(binding["oracle_validation_sha256"], plot.sha(oracle))
            self.assertTrue(binding["matches_accepted_oracle"])

    def test_changed_imported_source_cannot_reuse_an_accepted_candidate_identity(self):
        with synthetic_space() as folder:
            source, oracle, cfg, cert = oracle_fixture(folder)
            source.write_text("# SYNTHETIC different bytes.\n", encoding="utf-8")
            with patch.object(plot.importlib, "import_module", return_value=SimpleNamespace(__file__=str(source))):
                with self.assertRaisesRegex(ValueError, "prediction source differs"):
                    plot.predictor_source_binding(cfg, cert)

    def test_replaced_oracle_is_rejected_before_importing_predictor(self):
        with synthetic_space() as folder:
            source, oracle, cfg, cert = oracle_fixture(folder)
            write_json(oracle, {"complete": True, "pass": True, "candidate_sha256": "d"*64})
            with patch.object(plot.importlib, "import_module") as imported:
                with self.assertRaisesRegex(ValueError, "Oracle validation differs"):
                    plot.predictor_source_binding(cfg, cert)
                imported.assert_not_called()

    def test_incomplete_or_failed_oracle_is_rejected_even_with_matching_file_hash(self):
        with synthetic_space() as folder:
            source, oracle, cfg, cert = oracle_fixture(folder)
            for field in ("complete", "pass"):
                with self.subTest(field=field):
                    doc = {"complete": True, "pass": True, "candidate_sha256": plot.sha(source)}
                    doc[field] = False
                    write_json(oracle, doc)
                    cert["ts20_operating_condition"]["oracle_validation_sha256"] = plot.sha(oracle)
                    with self.assertRaisesRegex(ValueError, "Complete accepted predictor oracle"):
                        plot.predictor_source_binding(cfg, cert)

    def test_recorded_prior_preserves_identity_and_does_not_reopen_old_absolute_path(self):
        prior = recorded_prior()
        meta = {"base_response_is_prior": True,
                "provenance": {"base_response_is_prior": True, "base_response_prior": prior}}
        binding = plot.base_prior_binding(meta, "tip_hold")
        for key, value in prior.items():
            self.assertEqual(binding[key], value)
        self.assertEqual(binding["status"], "recorded")
        self.assertFalse(binding["new_plant_identified"])
        caption = plot.base_prior_caption({"base_response_prior": binding})
        self.assertIn(prior["source_model_sha256"][:12], caption)
        self.assertIn(prior["sha256"][:12], caption)
        self.assertIn("new-plant identification: no", caption)

    def test_joint_probe_missing_prior_is_explicitly_unknown_and_hold_refuses(self):
        meta = {"provenance": {"base_response_is_prior": True}}
        binding = plot.base_prior_binding(meta, "joint_probe")
        self.assertEqual(binding["status"], "unknown")
        self.assertIsNone(binding["sha256"])
        self.assertIsNone(binding["source_model_sha256"])
        self.assertIsNone(binding["new_plant_identified"])
        self.assertIn("unknown", plot.base_prior_caption({"base_response_prior": binding}))
        with self.assertRaisesRegex(ValueError, "hold lacks recorded"):
            plot.base_prior_binding(meta, "tip_hold")

    def test_incomplete_or_conflicting_prior_metadata_is_not_invented(self):
        prior = recorded_prior()
        meta = {"base_response_is_prior": True,
                "provenance": {"base_response_prior": prior}}
        prior.pop("source_model_sha256")
        with self.assertRaisesRegex(ValueError, "identity is incomplete"):
            plot.base_prior_binding(meta, "tip_hold")
        meta["provenance"]["base_response_prior"] = recorded_prior()
        meta["provenance"]["base_response_is_prior"] = False
        with self.assertRaisesRegex(ValueError, "flags are invalid or inconsistent"):
            plot.base_prior_binding(meta, "tip_hold")
        meta["provenance"]["base_response_is_prior"] = True
        meta["provenance"]["base_response_prior"]["new_plant_identified"] = True
        with self.assertRaisesRegex(ValueError, "not new-plant identification"):
            plot.base_prior_binding(meta, "tip_hold")

    def test_bound_run_exports_both_source_and_prior_identity_into_manifest_metadata(self):
        with synthetic_space() as folder:
            source, oracle, cfg, cert = oracle_fixture(folder)
            cfg.update(stage="tip_hold", scenario="x650_ts20_damped_hold_v1", duration_s=30.)
            meta = {"task": cfg, "complete": True, "landed_and_disarmed": True,
                    "diagnostic_parameters_restored": True, "diagnostic_restore_pending": False,
                    "ts20_operating_condition": cert["ts20_operating_condition"],
                    "task_start_sim_s": 17., "task_end_sim_s": 47.,
                    "base_response_is_prior": True,
                    "provenance": {"base_response_prior": recorded_prior()}}
            write_json(folder/"controller_task.json", cfg)
            write_json(folder/"controller_metadata.json", meta)
            files = ("stage_result.json", "native_trace.csv", "flight_samples.csv",
                     "ts20_state.csv", "ts20_packets.csv")
            for name in files:
                (folder/name).write_text("SYNTHETIC identity fixture only\n", encoding="utf-8")
            names = ("controller_task.json", "controller_metadata.json", *files)
            source_dir = "/synthetic/"+folder.name
            cert.update(kind="x650_ts20_conditional_stage_certificate_v1", complete=True,
                        passed=True, stage="tip_hold", actuator_log_verified=True,
                        full_window_ulog_verified=True, new_joint_prediction_verified=True,
                        window_sim_s=[17.,47.], source_run_directory=source_dir,
                        qualification={"supply_24v_qualified": False, "actual_aerial_arm_load_qualified": False},
                        artifacts={source_dir+"/"+name: plot.sha(folder/name) for name in names})
            cert_path = folder/"synthetic_certificate.json"
            write_json(cert_path, cert)
            with patch.object(plot.importlib, "import_module", return_value=SimpleNamespace(__file__=str(source))):
                _, _, _, _, binding = plot.bound_run(folder, cert_path)
            self.assertEqual(binding["prediction_source"]["sha256"], plot.sha(source))
            self.assertEqual(binding["base_response_prior"]["source_model_sha256"], "b"*64)
            self.assertFalse(binding["hardware_24v_loaded_qualified"])
            self.assertEqual(binding["actual_task_window_sim_s"], [17.,47.])


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
