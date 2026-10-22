"""Admission and stage-order tests only; no fixture is a flight result."""
from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import test_x650_research_condition as fixtures
import research_condition as research
import mixed_mpc_protocol as protocol

task = fixtures.task


class MixedMpcProtocolTests(unittest.TestCase):
    setUp = fixtures.ResearchConditionTests.setUp
    write_profile = fixtures.ResearchConditionTests.write_profile

    def cfg(self, stage="tip_hold", controller="pid_ff"):
        cfg = task.build_task_config(stage, controller, mixed_mpc_protocol=protocol.build_mixed_mpc_protocol())
        cfg.update(research_profile={"name": research.MIXED_MPC_PROFILE_NAME, "world_sdf": str(self.world)},
                   hover_reference_profile={"name": "fixed_world_position_v1", "world_sdf": str(self.world)},
                   pid_profile=str(self.pid_path), response_model=str(self.folder/"response_model.json"))
        return cfg

    def test_legacy_defaults_and_v1_v2_condition_bytes_do_not_change(self):
        original = task.build_task_config()
        self.assertNotIn("mixed_mpc_protocol", original)
        for name in (research.RESEARCH_PROFILE_NAME, research.UPPER_RESEARCH_PROFILE_NAME):
            cfg = fixtures.ResearchConditionTests.cfg(self, "hover_folded" if name == research.RESEARCH_PROFILE_NAME else "response_probe")
            cfg["research_profile"]["name"] = name
            condition = research.condition_for_task(cfg)
            self.assertEqual(condition["schema_version"], 1 if name == research.RESEARCH_PROFILE_NAME else 2)
            self.assertNotIn("mixed_mpc_protocol", condition)
            self.assertEqual(task.validate_task_config(cfg), cfg)
        self.assertEqual(task.validate_task_config({**original, "mixed_mpc_protocol": None})["mixed_mpc_protocol"], None)

    def test_v3_condition_changes_only_protocol_fields_and_maps_one_way(self):
        cfg = self.cfg()
        condition = research.condition_for_task(cfg)
        self.assertTrue(research.is_upper_research(cfg))
        self.assertEqual(condition["schema_version"], 3)
        self.assertEqual(condition["feedback_contract"], "px4_estimator_acceleration_receipt_enu_v2")
        expected, role = research.expected_gate_condition(condition, "response_identification", "tip_hold")
        self.assertEqual(role, "nominal_response_only")
        self.assertEqual(expected["research_profile"], research.UPPER_RESEARCH_PROFILE_NAME)
        self.assertEqual({key for key in condition if condition[key] != expected.get(key)},
                         {"schema_version", "research_profile", "scope", "feedback_contract"})
        for gate in research.PREPARATION_GATES:
            prep, role = research.expected_gate_condition(condition, gate, "tip_hold")
            self.assertEqual(role, "preparation_only")
            self.assertEqual(prep["schema_version"], 1)
        with self.assertRaises(ValueError):
            research.expected_gate_condition(expected, "pid_baseline", "tip_hold")

    def test_only_explicit_v3_comparison_presets_and_zero_feedforward(self):
        for stage in research.MIXED_MPC_STAGES:
            for controller in ("pid_ff", "mpc"):
                cfg = self.cfg(stage, controller)
                self.assertEqual(task.validate_task_config(cfg), cfg)
                self.assertFalse(cfg["feedforward_enabled"])
        for change in (lambda c: c.pop("mixed_mpc_protocol"), lambda c: c.update(feedforward_enabled=True),
                       lambda c: c.update(response_model=None), lambda c: c.update(mpc_config={"nodes": 2}),
                       lambda c: c["research_profile"].update(name=research.UPPER_RESEARCH_PROFILE_NAME)):
            cfg = self.cfg()
            change(cfg)
            with self.assertRaises(ValueError):
                task.validate_task_config(cfg)
        with self.assertRaises(ValueError):
            task.build_task_config("response_probe", mixed_mpc_protocol=protocol.build_mixed_mpc_protocol())

    def test_no_open_tuning_and_configuration_copies_are_independent(self):
        original = protocol.build_mixed_mpc_protocol()
        for change in (lambda p: p["mpc_controller_config"].update(nodes=16),
                       lambda p: p["common_controller_config"].update(arm_share=.8),
                       lambda p: p["acceptance"].update(full_window_required=1),
                       lambda p: p.update(extra=True)):
            changed = deepcopy(original)
            change(changed)
            with self.assertRaises(ValueError):
                protocol.validate_mixed_mpc_protocol(changed)
        baseline = protocol.baseline_config(original)
        baseline["velocity_limit"][0] = 99.
        self.assertEqual(protocol.baseline_config(original)["velocity_limit"][0], .55)

    def test_stage_order_requires_matched_pid_and_prior_hold_or_trajectory(self):
        common = {"native_ground", "hover_folded", "joint_probe", "response_identification"}
        for stage, extra in (("tip_hold", set()), ("small", {"tip_hold"}),
                             ("normal", {"tip_hold", "small"}), ("fast", {"tip_hold", "small", "normal"})):
            for controller in ("pid_ff", "mpc"):
                expected = common|extra|({"pid_baseline"} if controller == "mpc" else set())
                self.assertEqual(set(task.required_gates(self.cfg(stage, controller))), expected)

    def gate_bundle(self, cfg):
        provenance = fixtures.ResearchConditionTests.evidence(self, cfg)
        condition = research.condition_for_task(cfg)
        for gate, filename in cfg["stage_evidence"].items():
            path = Path(filename)
            data = json.loads(path.read_text())
            source, role = research.expected_gate_condition(condition, gate, cfg["stage"])
            data.update(operating_condition=source, operating_condition_sha256=research.canonical_hash(source))
            if source["schema_version"] > 1:
                name = "mixed_mpc_stage_audit.json" if source["schema_version"] == 3 else "upper_stage_audit.json"
                audit = self.folder/name
                audit.write_text("Synthetic admission test only")
                data["artifacts"].pop("precision_stage_audit.json")
                data["artifacts"][name] = task.sha(audit)
                data.update(upper_command_verified=True, feedback_contract_verified=True)
            if source["schema_version"] == 3:
                data.update(mixed_mpc_protocol=cfg["mixed_mpc_protocol"],
                    mixed_mpc_protocol_sha256=protocol.protocol_hash(cfg["mixed_mpc_protocol"]),
                    controller="pid_ff" if gate == "pid_baseline" else cfg["controller"],
                    source_stage=cfg["stage"] if gate == "pid_baseline" else gate,
                    acceleration_feedforward="zero")
            path.write_text(json.dumps(data))
        response_path = Path(cfg["stage_evidence"]["response_identification"])
        provenance["accepted_response_binding"] = {"evidence": {"path": str(response_path), "sha256": task.sha(response_path)}}
        return provenance

    def test_wrong_trajectory_baseline_and_controller_progression_cannot_unlock(self):
        for gate, field, value in (("pid_baseline", "source_stage", "small"),
                                  ("pid_baseline", "controller", "mpc"),
                                  ("small", "controller", "pid_ff"),
                                  ("tip_hold", "acceleration_feedforward", "legacy"),
                                  ("small", "mixed_mpc_protocol_sha256", "f"*64)):
            cfg = self.cfg("normal", "mpc")
            provenance = self.gate_bundle(cfg)
            task.validate_stage_gates(cfg, provenance)
            path = Path(cfg["stage_evidence"][gate])
            data = json.loads(path.read_text())
            data[field] = value
            path.write_text(json.dumps(data))
            with self.subTest(gate=gate, field=field), self.assertRaises(ValueError):
                task.validate_stage_gates(cfg, provenance)


if __name__ == "__main__":
    unittest.main()
