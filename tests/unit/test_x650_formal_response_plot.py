"""Synthetic evidence-viewer tests; fixtures never count as native experiments."""
from copy import deepcopy
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import zipfile

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"scripts"))
sys.path.insert(0,str(Path(__file__).parent))
import plot_x650_formal_response as plot
from test_x650_formal_response_analysis import fixture, analytic_response
from diagnostic_parameters import PRECISION_PROFILE
from hover_reference import PINNED_PX4
import research_condition as research


def save(path,document):
    path=Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(document,indent=2)+"\n",encoding="utf-8")


def write_csv(path,fields,records):
    with Path(path).open("w",newline="",encoding="utf-8") as stream:
        writer=csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(records)


def write_run(root,role,identity,params,pid,candidate=None):
    run,parameters=fixture(role)
    # Match the real runner's t_s = absolute sim_s - task_start operation,
    # including binary floating-point values exactly at pulse boundaries.
    run["truth_times"]=(run["truth_times"]+10.)-10.
    run["command_times"]=(run["command_times"]+10.)-10.
    run["commands"]=np.asarray([plot.identifier.protocol_velocity(float(t),run["protocol"])
                                for t in run["command_times"]])
    run["velocities"]=np.column_stack([analytic_response(run["truth_times"],run["command_times"],
        run["commands"][:,axis],axis,parameters[name]) for axis,name in enumerate("xyz")])
    # Independent tiny measured residuals exercise nonzero plot/metric replay.
    run["velocities"] += .00015*np.sin(run["truth_times"][:,None]*np.array([9.,13.,17.])+(.2 if role=="holdout" else 0.))
    folder=root/role
    folder.mkdir()
    protocol=run["protocol"]
    condition=identity["operating_condition"]
    common={key:identity[key] for key in ("model_sha256","source_urdf_sha256")}
    common.update(operating_condition=condition,operating_condition_sha256=plot.canonical_hash(condition),
                  pid_parameters_sha256=identity["pid_parameters_sha256"],response_probe_protocol=protocol,
                  response_probe_protocol_sha256=plot.identifier.response_protocol_hash(protocol))
    task={"stage":"response_probe","controller":"pid_ff","response_probe_protocol":protocol}
    meta={**common,"task":task,"complete":True,"landed_and_disarmed":True,"eligible_as_stage_gate":False,
          "acceptance_pending":"full_ulog_audit","actual_pid_parameters":pid,"task_start_sim_s":10.,"task_end_sim_s":70.}
    save(folder/"controller_metadata.json",meta)
    save(folder/"controller_task.json",task)
    save(folder/"stage_result.json",{**common,"complete":True,"passed":True})
    t,ct,v,u=run["truth_times"],run["command_times"],run["velocities"],run["commands"]
    write_csv(folder/"native_trace.csv",["sim_s","valid_base","valid_joint_state","base_vx","base_vy","base_vz",
        "arm_joint_1_q","arm_joint_2_q","arm_joint_1_qd","arm_joint_2_qd"],
        ([10.+time,1,1,*velocity,0.,0.,0.,0.] for time,velocity in zip(t,v)))
    write_csv(folder/"control_trace.csv",["sim_s","t_s","phase","base_velocity_target_x","base_velocity_target_y",
        "base_velocity_target_z","command_acceleration_x","command_acceleration_y","command_acceleration_z"],
        ([10.+time,time,"measure",*command,0.,0.,0.] for time,command in zip(ct,u)))
    save(folder/"native_trace_manifest.json",{**common,"complete":True,"sha256":plot.sha(folder/"native_trace.csv")})
    (folder/"px4.ulg").write_bytes(b"synthetic audit binding, not a real ULog")
    launch_folder=root/(role+"_launch")
    launch_folder.mkdir()
    launch={"complete":True,"runtime_closed":True,"task":task,"run_directories":[str(folder)],
            "started_utc":datetime.fromtimestamp(60. if role=="holdout" else 20.,timezone.utc).isoformat(),
            "finished_utc":datetime.fromtimestamp(80. if role=="holdout" else 40.,timezone.utc).isoformat()}
    extras={}
    if candidate is not None:
        copied=launch_folder/"frozen_response_candidate.json"
        copied.write_bytes(candidate.read_bytes())
        launch["frozen_response_candidate_archive"]={"path":str(copied),"sha256":plot.sha(copied),"source_path":str(candidate)}
        extras[str(copied)]=plot.sha(copied)
    save(launch_folder/"launch.json",launch)
    raw={str(path):plot.sha(path) for path in folder.iterdir() if path.is_file()}
    raw.update({str(launch_folder/"launch.json"):plot.sha(launch_folder/"launch.json"),**extras})
    accepted={**common,"stage":"response_probe","source_run":folder.name,"source_run_directory":str(folder),
        "complete":True,"passed":True,"research_acceptance_verified":True,"full_window_ulog_verified":True,
        "parameter_restore_verified":True,"upper_command_verified":True,"feedback_contract_verified":True}
    audit_folder=root/(role+"_audit")
    audit_folder.mkdir()
    audit_path=audit_folder/"upper_stage_audit.json"
    save(audit_path,{**accepted,"kind":"x650_precision_response_protocol_audit","window_sim_s":[10.,70.],"inputs_sha256":raw})
    raw[str(audit_path)]=plot.sha(audit_path)
    cert_path=audit_folder/"research_stage.json"
    save(cert_path,{**accepted,"kind":"x650_native_stage_evidence","eligible_as_stage_gate":True,
                    "diagnostic_only":False,"artifacts":raw})
    raw[str(cert_path)]=plot.sha(cert_path)
    fingerprint=plot.canonical_hash({"command_times":np.round(ct,9).tolist(),"commands":np.round(u,9).tolist(),
                                    "truth_times":np.round(t,9).tolist(),"velocities":np.round(v,9).tolist()})
    summary={"folder":str(folder),"command_csv_sha256":plot.sha(folder/"control_trace.csv"),"normalized_measurement_sha256":fingerprint}
    data=plot.identifier.prepare_run(run)
    axes=[]
    for index,axis in enumerate("xyz"):
        pred=plot.identifier.predict(data,index,params[axis])
        metrics=plot.identifier.segment_metrics(data,index,pred,protocol["acceptance"]["score_windows_s"][index])
        axes.append({"axis":axis,"parameters":params[axis],"passed":True,
                     "training" if role=="train" else "holdout":{"whole_axis_window":metrics,"passed":True,
                         "reasons":[],"pulse_and_zero_rest_segments":[]}})
        if role=="train":
            axes[-1]["fit"]={"fit_success":True,"identifiable":True,
                "log_bound_distance_fraction":[.3,.4],"relative_mse_increase_at_20percent_parameter_change":{"wn_rad_s":.1,"zeta":.1}}
    return raw,summary,axes


def evidence_bundle(root,*,training_passed=True,holdout=None):
    sources=plot.identifier.analysis_sources()
    archive=root/"preregistration.sources.zip"
    members={}
    with zipfile.ZipFile(archive,"w",zipfile.ZIP_DEFLATED) as zipped:
        for index,(source,digest) in enumerate(sorted(sources.items())):
            member=str(index)+"/"+Path(source).name
            zipped.write(source,member)
            members[member]={"source_path":source,"sha256":digest}
    protocols={role:plot.identifier.build_response_protocol(role) for role in ("train","holdout")}
    plan_path=root/"preregistration.json"
    plan={"kind":"x650_response_preregistration","created_unix":10.,"protocols":protocols,
          "analysis_sources_sha256":sources,"source_sha256":sources,
          "source_archive":{"path":str(archive),"sha256":plot.sha(archive),"members":members}}
    save(plan_path,plan)
    pid={name:1. for name in plot.CONTROL_NAMES}
    condition={"kind":"x650_research_operating_condition",**research._protocol_fields(research.UPPER_RESEARCH_PROFILE_NAME),
        "model_sha256":"a"*64,"estimator_parameters":dict(PRECISION_PROFILE),
        "observer":{"rate_hz":50.,"gaussian_noise":0.,"measurement_point":"base_link"},
        "coordinate_contract":"raw_ev_world","px4_source_contract_commit":PINNED_PX4,
        "pid_parameters_sha256":plot.canonical_hash(pid),"world_sdf_sha256":"c"*64,
        "ground_truth_aided":True,"gps_performance_claim":False,"hardware_performance_claim":False}
    identity={"model_sha256":"a"*64,"source_urdf_sha256":"b"*64,"operating_condition":condition,
              "operating_condition_sha256":plot.canonical_hash(condition),"pid_parameters_sha256":condition["pid_parameters_sha256"]}
    _,params=fixture()
    raw,summary,axes=write_run(root,"train",identity,params,pid)
    if not training_passed:
        axes[0]["passed"]=False  # Example: failed parameter-identifiability gate despite small residuals.
        axes[0]["fit"]["identifiable"]=False
        axes[0]["fit"]["relative_mse_increase_at_20percent_parameter_change"]["zeta"]=.01
    report={"kind":"x650_response_training_report","schema_version":1,"complete":True,"passed":training_passed,
        "training_passed":training_passed,"identified":False,"eligible_as_stage_gate":False,"parameters":params,
        "training":summary,"axes":axes,"identity":identity,"artifacts":raw,"preregistration_path":str(plan_path),
        "preregistration_sha256":plot.sha(plan_path),"analysis_sources_sha256":sources}
    training=root/"training_report.json"
    save(training,report)
    candidate_path=None
    validation=None
    if training_passed:
        candidate_path=root/"frozen_response_candidate.json"
        save(candidate_path,{"kind":"x650_frozen_response_candidate","schema_version":1,"training_passed":True,"identified":False,
            "eligible_as_stage_gate":False,"created_unix":50.,"training_report_path":str(training),
            "training_report_sha256":plot.sha(training),"parameters":params,"identity":identity,"training":summary,
            "protocols":protocols,"preregistration_sha256":plot.sha(plan_path),"analysis_sources_sha256":sources})
    if holdout is not None:
        assert candidate_path is not None
        other,hs,ha=write_run(root,"holdout",identity,params,pid,candidate_path)
        if not holdout:
            ha[0]["passed"]=False
            ha[0]["holdout"].update(passed=False,reasons=["synthetic zero-velocity-rest bias rejection"],
                pulse_and_zero_rest_segments=[{"kind":"zero_velocity_rest","index":1,"passed":False,"criteria":{"bias":False}}])
        validation=root/"response_identification.json"
        save(validation,{"kind":"x650_native_stage_evidence","schema_version":3,"stage":"response_identification",
            "complete":True,"passed":holdout,"identified":holdout,"eligible_as_stage_gate":holdout,
            **identity,"parameters":params,"training":summary,"holdout":hs,"axes":ha,
            "preregistration_path":str(plan_path),"preregistration_sha256":plot.sha(plan_path),
            "analysis_sources_sha256":sources,"frozen_candidate_path":str(candidate_path),"frozen_candidate_sha256":plot.sha(candidate_path),
            "artifacts":{**raw,**other,str(candidate_path):plot.sha(candidate_path),str(training):plot.sha(training)}})
    return training,candidate_path,validation


class FormalResponsePlotTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root=Path(temp.name)

    def test_completed_rejected_training_replays_without_any_optimizer(self):
        training,_,_=evidence_bundle(self.root,training_passed=False)
        with patch.object(plot.identifier,"fit_axis",side_effect=AssertionError("must never fit")), \
             patch.object(plot.identifier,"least_squares",side_effect=AssertionError("must never optimize")), \
             patch.object(plot.identifier,"minimize_scalar",side_effect=AssertionError("must never optimize")), \
             patch.object(plot.identifier,"segment_metrics",side_effect=AssertionError("must not recalculate acceptance")), \
             patch.object(plot.identifier,"evaluate_axis",side_effect=AssertionError("must not recalculate acceptance")):
            bundle=plot.load_bundle(training)
        self.assertEqual(bundle["runs"][0]["status"],"REJECTED / TRAIN ONLY")
        self.assertEqual(len(bundle["runs"][0]["data"]["times"]),13501)
        self.assertEqual(bundle["runs"][0]["data"]["times"][-1],54.)

    def test_holdout_outcome_and_candidate_chain_are_explicit(self):
        training,candidate,validation=evidence_bundle(self.root,holdout=False)
        bundle=plot.load_bundle(training,frozen_candidate=candidate,validation=validation)
        self.assertEqual([item["status"] for item in bundle["runs"]],
                         ["TRAINING ACCEPTED / TRAIN ONLY","HOLDOUT REJECTED"])
        report=plot.read_json(validation)
        report.update(passed=True,identified=True,eligible_as_stage_gate=True)
        report["axes"][0]["passed"]=True
        report["axes"][0]["holdout"].update(passed=True,reasons=[],pulse_and_zero_rest_segments=[])
        save(validation,report)
        self.assertEqual(plot.load_bundle(training,validation=validation)["runs"][1]["status"],"HOLDOUT PASS")
        candidate.write_text(candidate.read_text()+"\n")
        with self.assertRaisesRegex(ValueError,"missing or changed"):
            plot.load_bundle(training,validation=validation)

    def test_changed_raw_bytes_are_rejected_without_fitting(self):
        training,_,_=evidence_bundle(self.root)
        path=self.root/"train/control_trace.csv"
        path.write_text(path.read_text()+"\n")
        with self.assertRaisesRegex(ValueError,"missing or changed"):
            plot.load_bundle(training)

    def test_changed_parameters_cannot_render_original_metrics(self):
        training,_,_=evidence_bundle(self.root)
        report=plot.read_json(training)
        report["parameters"]["x"]["wn_rad_s"]=2.
        report["axes"][0]["parameters"]=report["parameters"]["x"]
        save(training,report)
        with self.assertRaisesRegex(ValueError,"does not reproduce"):
            plot.load_bundle(training)

    def test_incomplete_failure_is_not_invented_as_a_curve(self):
        path=self.root/"training_report.json"
        save(path,{"kind":"x650_response_training_report","complete":False,"passed":False})
        with self.assertRaisesRegex(ValueError,"Completed numerical"):
            plot.load_bundle(path)

    def test_relocated_selected_bytes_keep_original_hashes_and_paths(self):
        training,_,_=evidence_bundle(self.root)
        original=self.root/"train/native_trace.csv"
        relocated=self.root/"compact_native.csv"
        original.rename(relocated)
        mapping=self.root/"input_map.json"
        save(mapping,{str(original):str(relocated)})
        bundle=plot.load_bundle(training,input_map=mapping)
        self.assertEqual(bundle["original_bound_artifacts"][str(original)],plot.sha(relocated))
        self.assertEqual(bundle["inputs_sha256"][str(relocated)],plot.sha(relocated))

    def test_render_manifest_binds_png_pdf_and_never_grants_a_stage(self):
        training,_,_=evidence_bundle(self.root,training_passed=False)
        bundle=plot.load_bundle(training)
        out=self.root/"render"
        manifest=plot.render(bundle,out)
        self.assertFalse(manifest["eligible_as_stage_gate"])
        self.assertFalse(manifest["fitting_performed"])
        self.assertFalse(manifest["acceptance_recomputed"])
        self.assertEqual(manifest["axis_reported_diagnostics"][0]["fit"],bundle["training"]["axes"][0]["fit"])
        self.assertEqual(manifest["axis_reported_diagnostics"][0]["evaluation"],bundle["training"]["axes"][0]["training"])
        self.assertEqual({Path(item["path"]).suffix for item in manifest["files"]},{".png",".pdf"})
        for item in manifest["files"]:
            self.assertEqual(plot.sha(item["path"]),item["sha256"])
        self.assertTrue((out/"formal_response_train.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n"))
        with self.assertRaisesRegex(ValueError,"new plot directory"):
            plot.render(bundle,out)

    def test_nested_fit_or_evaluation_decisions_cannot_contradict_axis_result(self):
        training,_,validation=evidence_bundle(self.root,holdout=True)
        for original,role,change in (
            (plot.read_json(training),"training",lambda doc:doc["axes"][0]["fit"].update(identifiable=False)),
            (plot.read_json(training),"training",lambda doc:doc["axes"][0]["training"].update(passed=False)),
            (plot.read_json(validation),"holdout",lambda doc:doc["axes"][0]["holdout"].update(passed=False))):
            value=deepcopy(original)
            change(value)
            with self.subTest(role=role),self.assertRaisesRegex(ValueError,"Nested evaluation/fit"):
                plot.report_axes(value,role)

    def test_holdout_rejection_reasons_and_segments_are_copied_without_new_acceptance(self):
        training,_,validation=evidence_bundle(self.root,holdout=False)
        bundle=plot.load_bundle(training,validation=validation)
        diagnostics=bundle["runs"][1]["panels"][0]["reported_diagnostics"]
        original=plot.read_json(validation)["axes"][0]["holdout"]
        self.assertEqual(diagnostics["failure_reasons"],original["reasons"])
        self.assertEqual(diagnostics["evaluation"],original)


if __name__=="__main__":
    unittest.main()
