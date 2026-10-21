"""Engineering baseline evidence checks using synthetic native-shaped records."""
from __future__ import annotations

import csv
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/"scripts"),str(ROOT/"control_packages/x650_wholebody_v1"),
               str(ROOT/"control_packages/x650_wholebody_v1/tests")]
SPEC = importlib.util.spec_from_file_location("x650_pid_freeze_under_test",ROOT/"scripts/freeze_x650_pid_baseline.py")
audit = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = audit
SPEC.loader.exec_module(audit)
from test_model import example_model
from x650_wholebody.model import tip_world
import test_x650_upper_response_identification as response_fixture
from test_x650_upper_response_identification import write_json,analytic_response


class PidBaselineFreezeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="x650_pid_freeze_")
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.paths = {}
        self.fixture = response_fixture.UpperResponseIdentificationTests()
        self.fixture.folder = self.folder
        for index,(role,stage) in enumerate(audit.ROLES.items()):
            self.paths[role] = self.make_run(role,stage,seed=30+index)

    def make_run(self,role,stage,*,seed):
        folder = self.fixture.make_run(role,seed=seed)
        meta = audit.json_read(folder/"controller_metadata.json")
        evidence = audit.json_read(folder/"stage_result.json")
        for name in audit.REQUIRED_PID_GAINS:
            meta["actual_pid_parameters"].setdefault(name,.1)
        for phase in ("before","after"):
            parameter_path = folder/f"px4_parameters_{phase}.json"
            snapshot = audit.json_read(parameter_path)
            snapshot["parameters"] = {name:{"value":value,"param_type":9} for name,value in meta["actual_pid_parameters"].items()}
            write_json(parameter_path,snapshot)
            evidence["artifacts"][parameter_path.name] = audit.sha(parameter_path)
        model = example_model(elbow_offset_body_m=(0.,.04,0.),source_report_sha256=meta["provenance"]["preparation_report_sha256"])
        duration = 20. if role == "hover" else 40. if role == "small" else 30.
        origin = 100.+seed
        anchor = np.array([0.,0.,2.])
        center = tip_world(anchor,np.eye(3),[0.,0.],model)
        parameters = dict(amplitude_x=.075,amplitude_z=.03,period_s=16.,cycles=2,start_s=2.,ramp_s=2.)
        command_t = np.arange(0.,duration+.0001,.02)
        observation_t = command_t-.006
        commands = np.zeros((len(command_t),3))
        if stage == "response_probe":
            for i,t in enumerate(command_t):
                if 2. <= t < 14.:
                    axis,part = int((t-2)//4),(t-2)%4
                    if part < 1.: commands[i,axis] = .08 if axis < 2 else .05
                    elif 2. <= part < 3.: commands[i,axis] = -.08 if axis < 2 else -.05
        velocities = np.column_stack([analytic_response(observation_t,command_t,commands[:,i],tau)
                                      for i,tau in enumerate((.16,.20,.25))])
        velocities += .00002*np.sin(observation_t[:,None]*[11.,13.,17.]+seed)
        fields = ["sim_s","t_s","phase","truth_stamp_s","q1","q2","qd1","qd2","qd1_cmd","qd2_cmd"]
        fields += ["base_v"+a for a in "xyz"]+["base_velocity_target_"+a for a in "xyz"]
        with (folder/"control_trace.csv").open("w",newline="",encoding="utf-8") as stream:
            writer = csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
            for t,obs,u,v in zip(command_t,observation_t,commands,velocities):
                row = dict(sim_s=origin+t,t_s=t,phase="measure",truth_stamp_s=origin+obs,q1=0.,q2=0.,qd1=0.,qd2=0.,qd1_cmd=0.,qd2_cmd=0.)
                row.update({"base_v"+a:float(x) for a,x in zip("xyz",v)})
                row.update({"base_velocity_target_"+a:float(x) for a,x in zip("xyz",u)})
                writer.writerow(row)
        stamps = np.arange(0.,duration+.0001,.008)
        q = audit.joint_reference(stamps,duration) if role == "joint" else np.zeros((len(stamps),2))
        qd = np.gradient(q,.008,axis=0)
        desired = audit.small_reference(stamps,center,parameters) if role == "small" else np.tile(center,(len(stamps),1))
        positions = np.tile(anchor,(len(stamps),1))
        if role == "small": positions += desired-center
        tip = desired if role != "joint" else np.array([tip_world(p,np.eye(3),v,model) for p,v in zip(positions,q)])
        native_fields = ["sim_s",*["base_p"+a for a in "xyz"],*["base_q"+a for a in "wxyz"],
                         "arm_joint_1_q","arm_joint_2_q","arm_joint_1_qd","arm_joint_2_qd",
                         *["tip_"+a for a in "xyz"],"valid_base","valid_joint_state","valid_tip"]
        with (folder/"native_trace.csv").open("w",newline="",encoding="utf-8") as stream:
            writer = csv.DictWriter(stream,fieldnames=native_fields);writer.writeheader()
            for t,p,j,dj,ee in zip(stamps,positions,q,qd,tip):
                row = dict(zip(native_fields,[origin+t,*p,1.,0.,0.,0.,*j,*dj,*ee,1,1,1]))
                writer.writerow(row)
        flight_fields = ["sim_time_s","phase","failsafe","armed","landed",*["target_world_"+a+"_m" for a in "xyz"]]
        with (folder/"flight_samples.csv").open("w",newline="",encoding="utf-8") as stream:
            writer = csv.DictWriter(stream,fieldnames=flight_fields);writer.writeheader()
            for t in np.arange(0.,duration+.0001,.05):
                writer.writerow(dict(zip(flight_fields,[origin+t,"measure",0,1,0,*anchor])))
        for filename,manifest in (("native_trace.csv","native_trace_manifest.json"),("flight_samples.csv","flight_samples_manifest.json")):
            write_json(folder/manifest,{"complete":True,"file":filename,"sha256":audit.sha(folder/filename),
                "model_sha256":meta["model_sha256"],"source_urdf_sha256":meta["source_urdf_sha256"]})
        meta.update(stage=stage,folded_before_landing=True,task_start_sim_s=origin,task_end_sim_s=origin+duration,
                    requested_duration_s=duration,reference_base_enu_m=anchor.tolist(),reference_rotation=np.eye(3).tolist(),
                    physical_profile=model.metadata(),task={"trajectory_parameters":parameters})
        meta["provenance"]["source_files_sha256"] = {name:audit.sha(ROOT/name) for name in audit.CRITICAL_SOURCES}
        write_json(folder/"controller_metadata.json",meta)
        evidence["stage"] = stage
        evidence["artifacts"].update({name:audit.sha(folder/name) for name in (
            "control_trace.csv","native_trace.csv","native_trace_manifest.json","flight_samples.csv","flight_samples_manifest.json")})
        write_json(folder/"stage_result.json",evidence)
        return folder

    def replace_native_tip(self,role,change):
        folder = self.paths[role]
        with (folder/"native_trace.csv").open(newline="",encoding="utf-8") as stream:
            reader = csv.DictReader(stream);fields=reader.fieldnames;rows=list(reader)
        for row in rows: change(row)
        with (folder/"native_trace.csv").open("w",newline="",encoding="utf-8") as stream:
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
        manifest = audit.json_read(folder/"native_trace_manifest.json")
        manifest["sha256"] = audit.sha(folder/"native_trace.csv")
        write_json(folder/"native_trace_manifest.json",manifest)
        evidence = audit.json_read(folder/"stage_result.json")
        for name in ("native_trace.csv","native_trace_manifest.json"): evidence["artifacts"][name]=audit.sha(folder/name)
        write_json(folder/"stage_result.json",evidence)

    def test_stable_actual_profile_passes_engineering_gate_without_optimization_claim(self):
        profile,report = audit.audit_baseline(self.paths)
        self.assertTrue(report["passed"])
        self.assertTrue(report["engineering_stability_passed"])
        self.assertEqual(report["baseline_scope"],"engineering_stability_only")
        self.assertFalse(report["tuning_claim"])
        self.assertFalse(report["formal_fully_tuned_comparison_ready"])
        self.assertFalse(report["tuning_evidence_audit"]["provided"])
        self.assertEqual(profile["parameters"]["MPC_XY_P"],1.7)
        self.assertFalse(profile["parameter_writes"])
        self.assertTrue(all(v["passed"] for v in report["stage_metrics"].values()))

    def test_strict_mode_keeps_stability_separate_from_missing_tuning_evidence(self):
        _,report = audit.audit_baseline(self.paths,strict_tuning=True)
        self.assertTrue(report["engineering_stability_passed"])
        self.assertFalse(report["passed"])
        self.assertFalse(report["formal_fully_tuned_comparison_ready"])

    def test_finished_but_stationary_small_trajectory_is_independently_rejected(self):
        model = example_model(elbow_offset_body_m=(0.,.04,0.))
        center = tip_world([0.,0.,2.],np.eye(3),[0.,0.],model)
        self.replace_native_tip("small",lambda row:row.update({"tip_"+a:str(v) for a,v in zip("xyz",center)}))
        _,report = audit.audit_baseline(self.paths)
        self.assertFalse(report["passed"])
        metrics = report["stage_metrics"]["small"]["metrics"]
        self.assertGreater(metrics["position_rmse_m"],.05)
        self.assertEqual(metrics["position_rmse_limit_m"],.04)

    def test_tip_metrics_recomputed_from_native_rows_not_cached_passed_boolean(self):
        self.replace_native_tip("tip",lambda row:row.update(tip_x=str(float(row["tip_x"])+.15)))
        _,report = audit.audit_baseline(self.paths)
        self.assertFalse(report["passed"])
        self.assertAlmostEqual(report["stage_metrics"]["tip"]["metrics"]["position_rmse_m"],.15,places=10)

    def test_raw_hash_parameter_and_source_changes_are_not_silently_frozen(self):
        path = self.paths["tip"]/"native_trace.csv"
        path.write_bytes(path.read_bytes()+b"\n")
        with self.assertRaisesRegex(ValueError,"hash mismatch"):
            audit.audit_baseline(self.paths)

    def test_changed_pid_during_one_run_and_changed_controller_source_are_rejected(self):
        path = self.paths["tip"]/"controller_metadata.json"
        original = audit.json_read(path)
        changed = audit.json_read(path)
        changed["provenance"]["source_files_sha256"][audit.CRITICAL_SOURCES[0]] = "f"*64
        write_json(path,changed)
        with self.assertRaisesRegex(ValueError,"source differs"):
            audit.audit_baseline(self.paths)
        write_json(path,original)
        path = self.paths["tip"]/"px4_parameters_after.json"
        snapshot = audit.json_read(path)
        snapshot["parameters"]["MPC_XY_P"]["value"] = 2.
        write_json(path,snapshot)
        stage_path = self.paths["tip"]/"stage_result.json"
        stage = audit.json_read(stage_path)
        stage["artifacts"][path.name] = audit.sha(path)
        write_json(stage_path,stage)
        with self.assertRaisesRegex(ValueError,"changed during"):
            audit.audit_baseline(self.paths)

    def test_outputs_bound_profile_with_no_overwrite_and_no_pass_on_missing_inputs(self):
        output = self.folder/"baseline"
        report = audit.write_baseline(self.paths,output)
        self.assertTrue(report["passed"])
        self.assertEqual(report["artifacts"]["pid_profile.json"],audit.sha(output/"pid_profile.json"))
        with self.assertRaisesRegex(ValueError,"never overwritten"):
            audit.write_baseline(self.paths,output)
        invalid = dict(self.paths);invalid["tip"] = self.folder/"missing"
        failed = audit.write_baseline(invalid,self.folder/"failed")
        self.assertFalse(failed["complete"])
        self.assertFalse(failed["passed"])


if __name__ == "__main__": unittest.main()
