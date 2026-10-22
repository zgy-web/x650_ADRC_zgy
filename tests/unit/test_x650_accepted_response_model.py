"""Synthetic byte-bound evidence graphs; never simulation performance evidence."""
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import zipfile

import test_x650_mixed_mpc_protocol as protocol_fixtures
import accepted_response_model as accepted
import mixed_mpc_protocol as protocol
import research_condition as research
from response_protocol import build_response_protocol


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2)+"\n", encoding="utf-8")


def artifact_map(paths):
    return {str(p.resolve()): accepted.sha(p) for p in paths}


def evidence_bundle(folder, cfg):
    """Build a deliberately tiny but structurally complete hash/chronology graph."""
    condition, _ = research.expected_gate_condition(research.condition_for_task(cfg), "response_identification", cfg["stage"])
    identity = {key: ("a" if key == "model_sha256" else "b")*64 for key in
                ("model_sha256", "source_urdf_sha256", "airframe_sha256", "flight_profile_sha256", "preparation_report_sha256")}
    identity.update(operating_condition=condition, operating_condition_sha256=research.canonical_hash(condition),
                    pid_parameters_sha256=condition["pid_parameters_sha256"],
                    feedback_contract=condition["feedback_contract"], control_interface=condition["control_interface"])
    parameters = {"x": {"wn_rad_s": 4., "zeta": .7}, "y": {"wn_rad_s": 5., "zeta": 1.2}, "z": {"tau_s": .2}}
    analysis_source, controller_source = folder/"synthetic_analysis.py", folder/"synthetic_controller.py"
    analysis_source.write_text("# frozen synthetic analysis\n")
    controller_source.write_text("# historical synthetic controller\n")
    sources = artifact_map((analysis_source, controller_source))
    archive_path = folder/"preregistration.sources.zip"
    members = {}
    with zipfile.ZipFile(archive_path, "w") as archive:
        for index, source in enumerate((analysis_source, controller_source)):
            member = str(index)+"/"+source.name
            archive.write(source, member)
            members[member] = {"source_path": str(source), "sha256": accepted.sha(source)}
    binary = folder/"px4"
    binary.write_bytes(b"synthetic PX4 binary identity")
    protocols = {role: build_response_protocol(role) for role in ("train", "holdout")}
    templates = {role: {"stage": "response_probe", "controller": "pid_ff", "response_probe_protocol": doc}
                 for role, doc in protocols.items()}
    plan_path = folder/"preregistration.json"
    plan = {"kind": "x650_response_preregistration", "schema_version": 1, "created_unix": 10.,
            "protocols": protocols, "task_templates": templates, "source_sha256": sources,
            "analysis_sources_sha256": artifact_map((analysis_source,)),
            "source_archive": {"path": str(archive_path), "sha256": accepted.sha(archive_path), "members": members},
            "px4_binary_path": str(binary), "px4_binary_sha256": accepted.sha(binary)}
    save(plan_path, plan)
    candidate_path = folder/"frozen_response_candidate.json"

    def run(role):
        directory, launch_folder, audit_folder = (folder/(role+suffix) for suffix in ("_run", "_launch", "_audit"))
        for item in (directory, launch_folder, audit_folder):
            item.mkdir()
        task = deepcopy(templates[role])
        extra = {}
        if role == "holdout":
            task.update(frozen_response_candidate=str(candidate_path), frozen_response_candidate_sha256=accepted.sha(candidate_path))
            copy = launch_folder/"frozen_response_candidate.json"
            copy.write_bytes(candidate_path.read_bytes())
            extra = {"frozen_response_candidate_archive": {"path": str(copy), "source_path": str(candidate_path),
                                                           "sha256": accepted.sha(copy)}}
        source_hashes = {controller_source.name: accepted.sha(controller_source)}
        with zipfile.ZipFile(launch_folder/"controller_sources.zip", "w") as archive:
            archive.write(controller_source, controller_source.name)
        launch = {"complete": True, "runtime_closed": True, "exit_code": 0, "task": task,
                  "provenance": {"source_files_sha256": source_hashes, **{key: identity[key] for key in accepted.IDENTITY_KEYS[:5]}}, "run_directories": [str(directory)],
                  "started_utc": datetime.fromtimestamp(60. if role == "holdout" else 20., timezone.utc).isoformat(),
                  "finished_utc": datetime.fromtimestamp(80. if role == "holdout" else 40., timezone.utc).isoformat(), **extra}
        save(launch_folder/"launch.json", launch)
        save(launch_folder/"task.json", task)
        save(directory/"controller_metadata.json", {"task": task, "provenance": launch["provenance"],
            "stage": "response_probe", "controller": "pid_ff", "complete": True, "landed_and_disarmed": True,
            "diagnostic_only": False, "ground_truth_aided": True, "eligible_as_stage_gate": False,
            "acceptance_pending": "full_ulog_audit", "actual_pid_parameters": accepted.read(cfg["pid_profile"])["parameters"]})
        save(directory/"controller_task.json", task)
        save(directory/"stage_result.json", {"complete": True, "passed": True})
        for name in ("native_trace.csv", "control_trace.csv", "px4.ulg"):
            (directory/name).write_text("synthetic "+role+" "+name)
        save(directory/"native_trace_manifest.json", {"complete": True, "sha256": accepted.sha(directory/"native_trace.csv")})
        for name in accepted.RAW_RUN_FILES:
            if not (directory/name).exists():
                save(directory/name, {"synthetic": True})
        raw = artifact_map([*directory.iterdir(), *launch_folder.iterdir(), controller_source, analysis_source])
        common = {"source_run_directory": str(directory), "source_run": directory.name,
                  "stage": "response_probe", "simulation_backend": "gazebo_px4",
                  "response_probe_protocol": protocols[role], "response_probe_protocol_sha256": research.canonical_hash(protocols[role]),
                  "operating_condition": condition, **{key: True for key in accepted.VERIFIED}}
        audit_path = audit_folder/"upper_stage_audit.json"
        save(audit_path, {**common, "kind": "x650_precision_response_protocol_audit", "inputs_sha256": dict(raw)})
        raw.update(artifact_map((audit_path,)))
        cert = audit_folder/"research_stage.json"
        save(cert, {**common, "kind": "x650_native_stage_evidence", "eligible_as_stage_gate": True, "artifacts": dict(raw)})
        raw.update(artifact_map((cert,)))
        summary = {"folder": str(directory), "command_csv_sha256": accepted.sha(directory/"control_trace.csv"),
                   "normalized_measurement_sha256": ("1" if role == "train" else "2")*64}
        return raw, summary

    raw, summary = run("train")
    axes = [{"axis": axis, "parameters": parameters[axis], "passed": True, "fit": {"identifiable": True},
             "training": {"passed": True, "reasons": []}} for axis in "xyz"]
    training_path = folder/"training_report.json"
    shared = {"parameters": parameters, "training": summary, "preregistration_path": str(plan_path),
              "preregistration_sha256": accepted.sha(plan_path), "analysis_sources_sha256": plan["analysis_sources_sha256"]}
    training = {"kind": "x650_response_training_report", "schema_version": 1, "complete": True,
                "passed": True, "training_passed": True, "identified": False, "eligible_as_stage_gate": False,
                "identity": identity, "axes": axes, "artifacts": raw, **shared}
    save(training_path, training)
    candidate = {"kind": "x650_frozen_response_candidate", "schema_version": 1, "training_passed": True,
                 "identified": False, "eligible_as_stage_gate": False, "created_unix": 50., "identity": identity,
                 "training_report_path": str(training_path), "training_report_sha256": accepted.sha(training_path),
                 "protocols": protocols, **shared}
    save(candidate_path, candidate)
    other, holdout = run("holdout")
    model_path = Path(cfg["response_model"])
    model = {"kind": accepted.MODEL_KIND, "schema_version": 1,
             **{key: True for key in ("identified", "passed", "response_identified", "eligible_as_stage_gate", "research_acceptance_verified", "ground_truth_aided")},
             "diagnostic_only": False, **identity, "parameters": parameters, "model_structure": accepted.STRUCTURE,
             "state_order": ["vx", "ax", "vy", "ay", "vz"], "gain": [1., 1., 1.], "command_delay_s": 0.,
             "bias_m_s": [0., 0., 0.], "acceleration_feedforward_contract": accepted.FF_CONTRACT,
             "joint_response_identified": False, "full_body_dynamics_identified": False,
             "evidence": "response_identification.json", "frozen_candidate_sha256": accepted.sha(candidate_path),
             "preregistration_sha256": accepted.sha(plan_path), "analysis_sources_sha256": plan["analysis_sources_sha256"]}
    save(model_path, model)
    evidence_path = model_path.parent/"response_identification.json"
    evidence = {"kind": "x650_native_stage_evidence", "schema_version": 3, "stage": "response_identification",
                "simulation_backend": "gazebo_px4", **{key: True for key in accepted.VERIFIED},
                "success": True, "identified": True, "eligible_as_stage_gate": True, "ground_truth_aided": True,
                "diagnostic_only": False, **identity, **shared, "holdout": holdout,
                "axes": [{"axis": axis["axis"], "parameters": axis["parameters"], "passed": True,
                          "training": axis, "holdout": {"passed": True, "reasons": []}} for axis in axes],
                "reasons": [], "protocols": protocols, "frozen_candidate_path": str(candidate_path),
                "frozen_candidate_sha256": accepted.sha(candidate_path),
                **{key: model[key] for key in ("model_structure", "gain", "bias_m_s", "command_delay_s", "acceleration_feedforward_contract")},
                "artifacts": {**raw, **other, **artifact_map((plan_path, candidate_path, training_path, archive_path)),
                              "response_model.json": accepted.sha(model_path)}}
    save(evidence_path, evidence)
    cfg["stage_evidence"]["response_identification"] = str(evidence_path)
    return {"binary": binary, "source": controller_source, "analysis": analysis_source, "model": model_path,
            "evidence": evidence_path, "plan": plan_path, "archive": archive_path, "candidate": candidate_path}


class AcceptedResponseModelTests(unittest.TestCase):
    setUp = protocol_fixtures.MixedMpcProtocolTests.setUp
    write_profile = protocol_fixtures.MixedMpcProtocolTests.write_profile
    cfg = protocol_fixtures.MixedMpcProtocolTests.cfg

    def bundle(self):
        cfg = self.cfg()
        bundle = evidence_bundle(self.folder, cfg)
        return cfg, bundle

    def load(self, cfg, bundle):
        with patch.object(accepted, "runtime_px4_binary", return_value=bundle["binary"]):
            return accepted.load_accepted_response_model(cfg)

    def test_complete_chain_loads_nominal_only_and_core_gets_mixed_parameters(self):
        cfg, bundle = self.bundle()
        result = self.load(cfg, bundle)
        binding = result["binding"]
        self.assertEqual(binding["role"], "nominal_response_only")
        self.assertEqual(binding["model"]["sha256"], accepted.sha(bundle["model"]))
        config = protocol.controller_config(cfg["mixed_mpc_protocol"], result["model"])
        self.assertEqual(config["response_model"], {"kind": "xy_second_order_z_first_order_v1", "parameters": result["model"]["parameters"]})
        self.assertTrue(config["response_identified"])
        self.assertNotIn("base_time_constant_s", config)
        self.assertEqual({key: config[key] for key in protocol.baseline_config(cfg["mixed_mpc_protocol"])},
                         protocol.baseline_config(cfg["mixed_mpc_protocol"]))

    def test_controller_source_evolution_needs_exact_preregistered_archive(self):
        cfg, bundle = self.bundle()
        bundle["source"].write_text("# newer reviewed controller\n")
        self.assertEqual(self.load(cfg, bundle)["binding"]["role"], "nominal_response_only")
        bundle["archive"].write_bytes(bundle["archive"].read_bytes()+b"changed")
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            self.load(cfg, bundle)

    def test_analysis_sources_and_raw_records_cannot_use_archive_substitution(self):
        cfg, bundle = self.bundle()
        for path in (bundle["analysis"], self.folder/"train_run/native_trace.csv"):
            original = path.read_bytes()
            path.write_bytes(original+b"changed")
            with self.subTest(path=path), self.assertRaises(ValueError):
                self.load(cfg, bundle)
            path.write_bytes(original)

    def test_training_candidate_and_unbound_or_changed_model_are_rejected(self):
        cfg, bundle = self.bundle()
        with self.assertRaisesRegex(ValueError, "formally accepted"):
            self.load({**cfg, "response_model": str(bundle["candidate"])}, bundle)
        original = bundle["model"].read_bytes()
        bundle["model"].write_bytes(original+b"\n")
        with self.assertRaisesRegex(ValueError, "exact model bytes"):
            self.load(cfg, bundle)
        bundle["model"].write_bytes(original)
        cfg["stage_evidence"].clear()
        with self.assertRaises(ValueError):
            self.load(cfg, bundle)

    def test_legacy_or_unstable_or_boolean_parameters_cannot_enter_core(self):
        cfg, bundle = self.bundle()
        original = accepted.read(bundle["model"])
        for edit in (lambda m: m.update(base_time_constant_s=[.2, .2, .2]),
                     lambda m: m["parameters"]["x"].update(zeta=0.),
                     lambda m: m["parameters"]["x"].update(wn_rad_s=31.),
                     lambda m: m["parameters"]["z"].update(tau_s=True),
                     lambda m: m.update(command_delay_s=.01),
                     lambda m: m.update(identified=1)):
            model = deepcopy(original)
            edit(model)
            with self.assertRaises(ValueError):
                protocol.controller_config(cfg["mixed_mpc_protocol"], model)

    def test_runtime_binary_must_match_preregistered_bytes(self):
        cfg, bundle = self.bundle()
        changed = self.folder/"other_px4"
        changed.write_bytes(b"different binary")
        with patch.object(accepted, "runtime_px4_binary", return_value=changed), self.assertRaisesRegex(ValueError, "Current PX4 binary"):
            accepted.load_accepted_response_model(cfg)

    def test_nested_holdout_rejection_cannot_hide_under_passed_final_flag(self):
        cfg, bundle = self.bundle()
        evidence = accepted.read(bundle["evidence"])
        evidence["axes"][0]["holdout"].update(passed=False, reasons=["synthetic rejection"])
        save(bundle["evidence"], evidence)
        with self.assertRaisesRegex(ValueError, "Holdout axis"):
            self.load(cfg, bundle)


if __name__ == "__main__":
    unittest.main()
