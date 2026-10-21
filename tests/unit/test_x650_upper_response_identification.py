"""Synthetic independent-response and provenance rejection checks.

Synthetic velocities use analytic step-response superposition, independently
of the identifier's event integration. They are not native flight evidence.
"""
from __future__ import annotations

import csv
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "x650_response_identification_under_test", ROOT/"scripts/identify_x650_upper_response.py")
identifier = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = identifier
SPEC.loader.exec_module(identifier)


def write_json(path,value):
    Path(path).write_text(json.dumps(value,indent=2)+"\n",encoding="utf-8")


def analytic_response(t,command_t,u,tau,delay=0.):
    changes = np.r_[u[0],np.diff(u)]
    active = np.flatnonzero(changes)
    output = np.zeros(len(t))
    for index in active:
        elapsed = t-command_t[index]-delay
        positive = elapsed >= 0.
        output[positive] += changes[index]*(1-np.exp(-elapsed[positive]/tau))
    return output


class UpperResponseIdentificationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="x650_upper_response_")
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.train = self.make_run("train",seed=10)
        self.holdout = self.make_run("holdout",seed=13)

    def make_run(self,name,*,seed,taus=(.16,.20,.25),delay=0.,gain=1.,amplitude_scale=1.,flat=False):
        folder = self.folder/name
        folder.mkdir()
        rng = np.random.default_rng(seed)
        command_t = np.r_[0.,np.cumsum(rng.uniform(.018,.022,930))]
        observation_t = command_t-.006
        u = np.zeros((len(command_t),3))
        for index,t in enumerate(command_t):
            if 2. <= t < 14.:
                axis,part = int((t-2)//4),(t-2)%4
                if part < 1.: u[index,axis] = .08 if axis < 2 else .05
                elif 2. <= part < 3.: u[index,axis] = -.08 if axis < 2 else -.05
        u *= amplitude_scale
        v = np.column_stack([analytic_response(observation_t,command_t,u[:,i],tau,delay)
                             for i,tau in enumerate(taus)])*gain
        v += .00002*np.sin(observation_t[:,None]*[11.,13.,17.]+seed)
        if flat: v[:] = 0.
        origin = 100.+seed
        fields = ["sim_s","t_s","phase","truth_stamp_s","q1","q2","qd1","qd2"]
        fields += ["base_v"+a for a in "xyz"]+["base_velocity_target_"+a for a in "xyz"]
        with (folder/"control_trace.csv").open("w",newline="",encoding="utf-8") as stream:
            writer = csv.DictWriter(stream,fieldnames=fields)
            writer.writeheader()
            for t,obs,command,velocity in zip(command_t,observation_t,u,v):
                row = dict(sim_s=origin+t,t_s=t,phase="measure",truth_stamp_s=origin+obs,q1=0.,q2=0.,qd1=0.,qd2=0.)
                row.update({"base_v"+axis:float(value) for axis,value in zip("xyz",velocity)})
                row.update({"base_velocity_target_"+axis:float(value) for axis,value in zip("xyz",command)})
                writer.writerow(row)
        identities = {key:letter*64 for key,letter in zip(("model_sha256","source_urdf_sha256","airframe_sha256",
                    "flight_profile_sha256","preparation_report_sha256"),"abcde")}
        pid = {"MPC_XY_P":1.7,"MPC_Z_P":.9,"MPC_XY_VEL_P_ACC":1.8,"MPC_XY_VEL_I_ACC":.4,
               "MPC_XY_VEL_D_ACC":.1,"MPC_Z_VEL_P_ACC":4.,"MC_ROLLRATE_P":.15,"MC_PITCHRATE_P":.15}
        for phase in ("before","after"):
            write_json(folder/f"px4_parameters_{phase}.json",{
                "contract_matches":True,"model_sha256":identities["model_sha256"],
                "airframe_sha256":identities["airframe_sha256"],
                "parameters":{key:{"value":value,"param_type":9} for key,value in pid.items()}})
        write_json(folder/"controller_metadata.json",{"complete":True,"landed_and_disarmed":True,
            "stage":"response_probe","controller":"pid_ff","model_sha256":identities["model_sha256"],
            "source_urdf_sha256":identities["source_urdf_sha256"],"provenance":identities,"actual_pid_parameters":pid})
        write_json(folder/"stage_result.json",{"kind":"x650_native_stage_evidence","stage":"response_probe",
            "simulation_backend":"gazebo_px4","complete":True,"passed":True,"success":True,"landed_and_disarmed":True,
            "model_sha256":identities["model_sha256"],"source_urdf_sha256":identities["source_urdf_sha256"],
            "metadata":"controller_metadata.json","artifacts":{}})
        self.refresh_artifacts(folder)
        return folder

    def refresh_artifacts(self,folder):
        path = folder/"stage_result.json"
        stage = identifier.json_read(path)
        stage["artifacts"] = {name:identifier.sha(folder/name) for name in (
            "control_trace.csv","px4_parameters_before.json","px4_parameters_after.json")}
        write_json(path,stage)

    def test_known_first_order_taus_recovered_with_variable_dt_and_unshifted_truth_times(self):
        model,report = identifier.analyze_runs(self.train,self.holdout)
        self.assertTrue(model["identified"])
        self.assertTrue(report["passed"])
        np.testing.assert_allclose(model["base_time_constant_s"],[.16,.20,.25],atol=4e-4,rtol=0.)
        self.assertNotEqual(model["training_control_trace_sha256"],model["holdout_control_trace_sha256"])
        self.assertEqual(model["gain"],[1.,1.,1.])
        self.assertEqual(model["command_delay_s"],0.)
        for axis in report["axes"]:
            self.assertEqual(axis["fit_source"],"training only")
            self.assertLess(axis["holdout"]["rmse_m_s"],.0001)
            self.assertFalse(axis["holdout"]["delay_diagnostic"]["applied_to_output_model"])

    def test_command_event_between_observation_times_is_not_rounded_or_shifted(self):
        times = np.array([0.,.02,.055,.10])
        cmd_t,u,tau = np.array([0.,.033]),np.array([0.,.1]),.2
        predicted = identifier.predict_velocity(tau,cmd_t,u,times,0.)
        expected = analytic_response(times,cmd_t,u,tau)
        np.testing.assert_allclose(predicted,expected,atol=1e-15,rtol=0.)
        self.assertEqual(predicted[1],0.)

    def test_same_run_same_raw_hash_and_copied_normalized_observations_are_rejected(self):
        with self.assertRaisesRegex(ValueError,"independent"):
            identifier.analyze_runs(self.train,self.train)
        shutil.copyfile(self.train/"control_trace.csv",self.holdout/"control_trace.csv")
        self.refresh_artifacts(self.holdout)
        with self.assertRaisesRegex(ValueError,"independent"):
            identifier.analyze_runs(self.train,self.holdout)
        # Whitespace can change file hashes without creating an independent run.
        csv_path = self.holdout/"control_trace.csv"
        csv_path.write_bytes(csv_path.read_bytes().replace(b"\r\n",b"\n"))
        self.refresh_artifacts(self.holdout)
        with self.assertRaisesRegex(ValueError,"copied"):
            identifier.analyze_runs(self.train,self.holdout)

    def test_independent_holdout_mismatch_rejects_instead_of_refitting_tau(self):
        other = self.make_run("different_dynamics",seed=17,taus=(.5,.6,.7))
        model,report = identifier.analyze_runs(self.train,other)
        self.assertFalse(model["identified"])
        self.assertFalse(report["passed"])
        np.testing.assert_allclose(model["base_time_constant_s"],[.16,.20,.25],atol=4e-4,rtol=0.)
        self.assertTrue(any(not axis["holdout"]["passed"] for axis in report["axes"]))

    def test_delay_is_rejected_not_used_to_realign_curves(self):
        other = self.make_run("delayed_holdout",seed=19,delay=.09)
        model,report = identifier.analyze_runs(self.train,other)
        self.assertFalse(report["passed"])
        self.assertEqual(model["command_delay_s"],0.)
        self.assertTrue(any(axis["holdout"]["delay_diagnostic"]["material_unmodeled_delay"] for axis in report["axes"]))

    def test_insufficient_command_or_actual_excitation_cannot_pass(self):
        for name,kwargs in (("small",{"amplitude_scale":.2}),("flat",{"flat":True})):
            other = self.make_run(name,seed=23,**kwargs)
            model,report = identifier.analyze_runs(self.train,other)
            self.assertFalse(model["passed"])
            self.assertTrue(any(not axis["holdout_excitation"]["passed"] for axis in report["axes"]))

    def test_input_tampering_and_actual_pid_changes_are_refused(self):
        path = self.holdout/"control_trace.csv"
        path.write_bytes(path.read_bytes()+b"\n")
        with self.assertRaisesRegex(ValueError,"hash mismatch"):
            identifier.analyze_runs(self.train,self.holdout)
        self.refresh_artifacts(self.holdout)
        path = self.holdout/"px4_parameters_after.json"
        values = identifier.json_read(path)
        values["parameters"]["MPC_XY_P"]["value"] = 2.
        write_json(path,values)
        self.refresh_artifacts(self.holdout)
        with self.assertRaisesRegex(ValueError,"changed during"):
            identifier.analyze_runs(self.train,self.holdout)

    def test_different_actual_pid_and_incomplete_native_stage_cannot_pass(self):
        pid = identifier.json_read(self.holdout/"controller_metadata.json")["actual_pid_parameters"]
        pid["MPC_XY_P"] = 2.
        meta = identifier.json_read(self.holdout/"controller_metadata.json")
        meta["actual_pid_parameters"] = pid
        write_json(self.holdout/"controller_metadata.json",meta)
        for phase in ("before","after"):
            path = self.holdout/f"px4_parameters_{phase}.json"
            snapshot = identifier.json_read(path)
            snapshot["parameters"]["MPC_XY_P"]["value"] = 2.
            write_json(path,snapshot)
        self.refresh_artifacts(self.holdout)
        with self.assertRaisesRegex(ValueError,"hashes differ"):
            identifier.analyze_runs(self.train,self.holdout)
        stage = identifier.json_read(self.train/"stage_result.json")
        stage["landed_and_disarmed"] = False
        write_json(self.train/"stage_result.json",stage)
        with self.assertRaisesRegex(ValueError,"completed and landed"):
            identifier.analyze_runs(self.train,self.holdout)

    def test_writes_hash_bound_success_and_explicit_failed_stage_without_overwrite(self):
        output = self.folder/"accepted"
        report = identifier.write_identification(self.train,self.holdout,output)
        self.assertTrue(report["passed"])
        self.assertEqual(report["stage"],"response_identification")
        self.assertEqual(report["artifacts"]["response_model.json"],identifier.sha(output/"response_model.json"))
        for name,digest in report["artifacts"].items():
            path = Path(name)
            path = path if path.is_absolute() else output/path
            self.assertEqual(identifier.sha(path),digest)
        with self.assertRaisesRegex(ValueError,"never overwritten"):
            identifier.write_identification(self.train,self.holdout,output)
        rejected = identifier.write_identification(self.train,self.train,self.folder/"rejected")
        self.assertFalse(rejected["complete"])
        self.assertFalse(rejected["passed"])
        self.assertFalse(identifier.json_read(self.folder/"rejected/response_model.json")["identified"])


if __name__ == "__main__":
    unittest.main()
