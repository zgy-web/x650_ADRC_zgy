"""Explicit v1 preparation -> v2 upper-interface contracts, using synthetic data."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import test_x650_research_condition as fixtures

task, research = fixtures.task, fixtures.research


class UpperResearchConditionTests(unittest.TestCase):
    setUp = fixtures.ResearchConditionTests.setUp
    write_profile = fixtures.ResearchConditionTests.write_profile

    def cfg(self, stage="response_probe"):
        cfg = fixtures.ResearchConditionTests.cfg(self, stage)
        cfg["research_profile"]["name"] = research.UPPER_RESEARCH_PROFILE_NAME
        if stage == "tip_hold":
            cfg["response_model"] = str(self.folder/"response_model.json")
        return cfg

    def bundle(self, cfg):
        provenance = fixtures.ResearchConditionTests.evidence(self, cfg)
        condition = research.condition_for_task(cfg)
        for gate, path in cfg["stage_evidence"].items():
            expected, _ = research.expected_gate_condition(condition, gate, cfg["stage"])
            document = json.loads(Path(path).read_text())
            document.update(operating_condition=expected,
                            operating_condition_sha256=research.canonical_hash(expected))
            if expected["schema_version"] == 2:
                audit_path = self.folder/"upper_stage_audit.json"
                audit_path.write_text("synthetic upper audit fixture")
                document["artifacts"].pop("precision_stage_audit.json")
                document["artifacts"][audit_path.name] = task.sha(audit_path)
                document.update(upper_command_verified=True, feedback_contract_verified=True)
            Path(path).write_text(json.dumps(document))
        if cfg["stage"] == "tip_hold":
            response = {"identified": True, "passed": True, "model_sha256": condition["model_sha256"],
                        "pid_parameters_sha256": condition["pid_parameters_sha256"],
                        "operating_condition": condition,
                        "operating_condition_sha256": research.canonical_hash(condition)}
            Path(cfg["response_model"]).write_text(json.dumps(response))
            gate_path = Path(cfg["stage_evidence"]["response_identification"])
            document = json.loads(gate_path.read_text())
            document["artifacts"]["response_model.json"] = task.sha(cfg["response_model"])
            gate_path.write_text(json.dumps(document))
        return provenance

    def edit_gate(self, cfg, gate, change):
        path = Path(cfg["stage_evidence"][gate])
        value = json.loads(path.read_text())
        change(value)
        path.write_text(json.dumps(value))

    def test_only_v2_response_and_tip_pid_ff_are_enabled(self):
        for stage in research.UPPER_RESEARCH_STAGES:
            cfg = self.cfg(stage)
            self.assertEqual(task.validate_task_config(cfg)["research_profile"], cfg["research_profile"])
        for stage in ("hover_folded", "joint_probe", "small", "normal", "fast"):
            with self.subTest(stage=stage), self.assertRaises(ValueError):
                task.validate_task_config(self.cfg(stage))
        cfg = self.cfg("tip_hold")
        cfg["controller"] = "mpc"
        with self.assertRaises(ValueError):
            task.validate_task_config(cfg)
        for stage in ("response_probe", "tip_hold"):
            cfg = self.cfg(stage)
            cfg["research_profile"]["name"] = research.RESEARCH_PROFILE_NAME
            with self.assertRaises(ValueError):
                task.validate_task_config(cfg)

    def test_v2_frame_reference_is_required_but_contract_identifies_pva_output(self):
        cfg = self.cfg()
        condition = research.condition_for_task(cfg)
        self.assertEqual(condition["schema_version"], 2)
        self.assertEqual(condition["scope"], "simulation_truth_aided_ev_upper_control")
        self.assertEqual(condition["control_interface"], "raw_ev_world_pva_v2")
        self.assertEqual(condition["feedback_contract"], "px4_estimator_receipt_enu_v1")
        self.assertEqual(cfg["hover_reference_profile"]["name"], "fixed_world_position_v1")
        cfg["hover_reference_profile"] = None
        with self.assertRaises(ValueError):
            task.validate_task_config(cfg)

    def test_v1_condition_canonical_bytes_and_shared_fields_are_unchanged(self):
        v1 = research.condition_for_task(fixtures.ResearchConditionTests.cfg(self))
        expected = {"kind": "x650_research_operating_condition", "schema_version": 1,
            "research_profile": "precision_ev_selected_pid_v1",
            "scope": "simulation_truth_aided_ev_commissioning", "model_sha256": self.profile["model_sha256"],
            "estimator_parameters": dict(research.PRECISION_PROFILE),
            "observer": {"rate_hz": 50., "gaussian_noise": 0., "measurement_point": "base_link"},
            "coordinate_contract": "raw_ev_world", "px4_source_contract_commit": research.PINNED_PX4,
            "control_interface": "fixed_world_position_v1", "pid_parameters_sha256": self.profile["pid_parameters_sha256"],
            "world_sdf_sha256": task.sha(self.world), "ground_truth_aided": True,
            "gps_performance_claim": False, "hardware_performance_claim": False}
        self.assertEqual(v1, expected)
        self.assertEqual(research.canonical_hash(v1), research.canonical_hash(expected))
        v2 = research.condition_for_task(self.cfg())
        changed = {key for key in set(v1)|set(v2) if v1.get(key) != v2.get(key)}
        self.assertEqual(changed, {"schema_version", "research_profile", "scope", "control_interface", "feedback_contract"})

    def test_both_versions_strictly_reject_mixed_fields_and_boolean_aliases(self):
        v1 = research.condition_for_task(fixtures.ResearchConditionTests.cfg(self))
        v2 = research.condition_for_task(self.cfg())
        self.assertEqual(research.validate_operating_condition(v1), v1)
        self.assertEqual(research.validate_operating_condition(v2), v2)
        for value in ({**v1, "feedback_contract": v2["feedback_contract"]},
                      {key: val for key, val in v2.items() if key != "feedback_contract"},
                      {**v2, "schema_version": 1}, {**v2, "control_interface": v1["control_interface"]},
                      {**v2, "ground_truth_aided": 1}, {**v2, "feedback_contract": "truth_feedback"}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                research.validate_operating_condition(value)

    def test_required_gates_add_joint_then_response_without_pid_baseline(self):
        self.assertEqual(task.required_gates(self.cfg()), ["hover_folded", "joint_probe", "native_ground"])
        self.assertEqual(task.required_gates(self.cfg("tip_hold")),
                         ["hover_folded", "joint_probe", "native_ground", "response_identification"])
        legacy = task.build_task_config("response_probe")
        self.assertEqual(task.required_gates(legacy), ["hover_folded", "native_ground"])
        self.assertNotIn("pid_baseline", task.required_gates(self.cfg("tip_hold")))

    def test_v1_preparation_is_accepted_only_as_explicit_preparation_role(self):
        for stage in research.UPPER_RESEARCH_STAGES:
            cfg = self.cfg(stage)
            provenance = self.bundle(cfg)
            gates = task.validate_stage_gates(cfg, provenance)
            condition = research.condition_for_task(cfg)
            for name in research.PREPARATION_GATES:
                self.assertEqual(gates[name]["role"], "preparation_only")
                self.assertEqual(gates[name]["evidence"]["operating_condition"]["schema_version"], 1)
                self.assertEqual(gates[name]["target_operating_condition_sha256"], research.canonical_hash(condition))
                self.assertNotEqual(gates[name]["source_operating_condition_sha256"],
                                    gates[name]["target_operating_condition_sha256"])
            if stage == "tip_hold":
                self.assertEqual(gates["response_identification"]["role"], "same_condition")

    def test_v2_or_gps_or_diagnostic_preparation_cannot_replace_v1_preparation(self):
        for mode in ("v2", "gps", "diagnostic", "unaudited"):
            cfg = self.cfg()
            provenance = self.bundle(cfg)
            v2 = research.condition_for_task(cfg)
            def change(value):
                if mode == "v2":
                    value.update(operating_condition=v2, operating_condition_sha256=research.canonical_hash(v2))
                elif mode == "gps":
                    for name in ("operating_condition", "operating_condition_sha256", "ground_truth_aided"):
                        value.pop(name)
                elif mode == "diagnostic":
                    value["diagnostic_only"] = True
                else:
                    value["full_window_ulog_verified"] = False
            self.edit_gate(cfg, "joint_probe", change)
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                task.validate_stage_gates(cfg, provenance)

    def test_preparation_shared_condition_changes_are_rejected_even_with_updated_hash(self):
        for name, val in (("model_sha256", "c"*64), ("pid_parameters_sha256", "c"*64),
                          ("world_sdf_sha256", "c"*64), ("coordinate_contract", "global_wgs84"),
                          ("estimator_parameters", {**research.PRECISION_PROFILE, "EKF2_EV_CTRL": 1}),
                          ("observer", {"rate_hz": 100., "gaussian_noise": 0., "measurement_point": "base_link"}),
                          ("px4_source_contract_commit", "c"*40)):
            cfg = self.cfg()
            provenance = self.bundle(cfg)
            def change(value):
                value["operating_condition"][name] = val
                value["operating_condition_sha256"] = research.canonical_hash(value["operating_condition"])
                if name == "pid_parameters_sha256":
                    value[name] = val
            self.edit_gate(cfg, "hover_folded", change)
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "operating condition"):
                task.validate_stage_gates(cfg, provenance)

    def test_v1_response_cannot_unlock_v2_tip_or_reverse_the_preparation_mapping(self):
        cfg = self.cfg("tip_hold")
        provenance = self.bundle(cfg)
        v1 = research.condition_for_task(fixtures.ResearchConditionTests.cfg(self))
        self.edit_gate(cfg, "response_identification", lambda value: value.update(
            operating_condition=v1, operating_condition_sha256=research.canonical_hash(v1)))
        with self.assertRaisesRegex(ValueError, "operating condition"):
            task.validate_stage_gates(cfg, provenance)
        v2 = research.condition_for_task(self.cfg())
        for stage, gate in (("small", "hover_folded"), ("response_probe", "response_identification"),
                            ("tip_hold", "pid_baseline")):
            with self.assertRaises(ValueError):
                research.expected_gate_condition(v2, gate, stage)
        legacy = task.build_task_config("response_probe")
        legacy["stage_evidence"] = cfg["stage_evidence"]
        with self.assertRaisesRegex(ValueError, "legacy GPS"):
            task.validate_stage_gates(legacy, provenance)

    def test_response_model_requires_exact_same_v2_certified_file_and_content(self):
        cfg = self.cfg("tip_hold")
        provenance = self.bundle(cfg)
        provenance["stage_gates"] = task.validate_stage_gates(cfg, provenance)
        binding = task.validate_upper_response_model(cfg, provenance)
        self.assertEqual(binding["sha256"], task.sha(cfg["response_model"]))
        other = self.folder/"other_response.json"
        other.write_bytes(Path(cfg["response_model"]).read_bytes())
        with self.assertRaisesRegex(ValueError, "not the response_model.json"):
            task.validate_upper_response_model({**cfg, "response_model": str(other)}, provenance)
        response = json.loads(Path(cfg["response_model"]).read_text())
        for field, val in (("identified", False), ("passed", False), ("model_sha256", "c"*64),
                           ("pid_parameters_sha256", "c"*64), ("operating_condition_sha256", "c"*64)):
            Path(cfg["response_model"]).write_text(json.dumps({**response, field: val}))
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, "v2 operating condition"):
                task.validate_upper_response_model(cfg, provenance)
        with self.assertRaises(ValueError):
            task.validate_task_config({**cfg, "response_model": None})

    def test_v2_identification_requires_upper_audit_and_feedback_verification(self):
        for field in ("upper_command_verified", "feedback_contract_verified"):
            cfg = self.cfg("tip_hold")
            provenance = self.bundle(cfg)
            self.edit_gate(cfg, "response_identification", lambda value: value.pop(field))
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, "command/feedback"):
                task.validate_stage_gates(cfg, provenance)
        cfg = self.cfg("tip_hold")
        provenance = self.bundle(cfg)
        def wrong_audit(value):
            value["artifacts"].pop("upper_stage_audit.json")
            value["artifacts"]["precision_stage_audit.json"] = task.sha(self.folder/"precision_stage_audit.json")
        self.edit_gate(cfg, "response_identification", wrong_audit)
        with self.assertRaisesRegex(ValueError, "hashed full ULog"):
            task.validate_stage_gates(cfg, provenance)

    def test_response_model_condition_hash_checks_actual_json_types(self):
        cfg = self.cfg("tip_hold")
        provenance = self.bundle(cfg)
        provenance["stage_gates"] = task.validate_stage_gates(cfg, provenance)
        path = Path(cfg["response_model"])
        response = json.loads(path.read_text())
        response["operating_condition"]["ground_truth_aided"] = 1
        path.write_text(json.dumps(response))
        with self.assertRaisesRegex(ValueError, "v2 operating condition"):
            task.validate_upper_response_model(cfg, provenance)

    def test_source_archive_requires_actual_estimator_chain_only_for_v2(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(task, "ROOT", Path(tmp)):
            fake_root = Path(tmp)
            # Legacy source enumeration does not acquire new v2 prerequisites.
            self.assertTrue(task.source_files_for_task(task.build_task_config()))
            with self.assertRaisesRegex(ValueError, "feedback source evidence is missing"):
                task.source_files_for_task(self.cfg())
            for relative in task.UPPER_RESEARCH_SOURCES:
                path = fake_root/relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("synthetic source only")
            files = task.source_files_for_task(self.cfg())
            self.assertTrue(set(fake_root/p for p in task.UPPER_RESEARCH_SOURCES) <= set(files))
            self.assertEqual(len(files), len(set(files)))
            (fake_root/"vendor/px4_msgs/msg/VehicleAttitude.msg").unlink()
            with self.assertRaisesRegex(ValueError, "VehicleAttitude.msg"):
                task.source_files_for_task(self.cfg())


if __name__ == "__main__":
    unittest.main()
