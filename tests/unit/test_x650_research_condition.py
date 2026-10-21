"""Synthetic contracts only: no test result is a native flight certificate."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("x650_research_task_under_test",
    ROOT/"simulations/x650_wholebody_mpc_v1/task.py")
task = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task)
import research_condition as research
from diagnostic_parameters import PRECISION_PROFILE, PID_CANDIDATES, load_parameter_evidence


class ResearchConditionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.world = self.folder/"world.sdf"
        self.world.write_text("<sdf><world name='synthetic_fixture'/></sdf>")
        parameters = {name: 1. for name in load_parameter_evidence().CORE_CONTROL_PARAMETERS}
        parameters.update({name: struct.unpack("<f", struct.pack("<f", value))[0]
                           for name, value in PID_CANDIDATES["velocity_high"].items()})
        self.profile = {"model_sha256": "a"*64, "parameters": parameters,
                        "pid_parameters_sha256": research.canonical_hash(parameters)}
        self.pid_path = self.folder/"pid_profile.json"
        self.write_profile()

    def write_profile(self):
        self.pid_path.write_text(json.dumps(self.profile))

    def cfg(self, stage="hover_folded"):
        cfg = task.build_task_config(stage)
        cfg.update(research_profile={"name": research.RESEARCH_PROFILE_NAME, "world_sdf": str(self.world)},
                   hover_reference_profile={"name": "fixed_world_position_v1", "world_sdf": str(self.world)},
                   pid_profile=str(self.pid_path))
        return cfg

    def evidence(self, cfg, *, research_mode=True):
        condition = research.condition_for_task(cfg)
        provenance = {"model_sha256": "a"*64, "source_urdf_sha256": "b"*64}
        artifact_names = ("stage_result.json", "controller_metadata.json", "precision_stage_audit.json", "px4.ulg")
        artifacts = {}
        for name in artifact_names:
            path = self.folder/name
            path.write_text("Explicitly synthetic unit-test artifact: "+name)
            artifacts[name] = task.sha(path)
        for gate in task.required_gates(cfg):
            document = {"kind": "x650_native_stage_evidence", "stage": gate,
                        "complete": True, "passed": True, "simulation_backend": "gazebo_px4",
                        **provenance, "artifacts": dict(artifacts)}
            if research_mode:
                document.update(diagnostic_only=False, ground_truth_aided=True, eligible_as_stage_gate=True,
                    research_acceptance_verified=True, full_window_ulog_verified=True, parameter_restore_verified=True,
                    operating_condition=condition, operating_condition_sha256=research.canonical_hash(condition),
                    pid_parameters_sha256=condition["pid_parameters_sha256"])
            path = self.folder/("gate_"+gate+".json")
            path.write_text(json.dumps(document))
            cfg["stage_evidence"][gate] = str(path)
        return provenance

    def edit_first_gate(self, cfg, edit):
        path = Path(cfg["stage_evidence"][task.required_gates(cfg)[0]])
        value = json.loads(path.read_text())
        edit(value)
        path.write_text(json.dumps(value))

    def test_only_reviewed_fixed_hover_and_joint_are_enabled(self):
        for stage in ("hover_folded", "joint_probe"):
            self.assertEqual(task.validate_task_config(self.cfg(stage))["research_profile"]["name"],
                             research.RESEARCH_PROFILE_NAME)
        for stage in ("response_probe", "tip_hold", "small", "normal", "fast"):
            with self.subTest(stage=stage), self.assertRaises(ValueError):
                task.validate_task_config(self.cfg(stage))

    def test_archived_tasks_default_to_no_research(self):
        cfg = task.build_task_config()
        del cfg["research_profile"]
        self.assertIsNone(task.validate_task_config(cfg)["research_profile"])
        self.assertIsNone(research.condition_for_task(cfg))
        self.assertEqual(research.research_runtime_parameters(cfg), (None, None))

    def test_diagnostic_fields_cannot_be_promoted_by_research_label(self):
        for field, value in (("diagnostic_profile", PRECISION_PROFILE),
                             ("pid_tuning_candidate", "velocity_high"),
                             ("precision_test_profile", "hover20_steps36_v1")):
            cfg = self.cfg()
            cfg[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                task.validate_task_config(cfg)

    def test_research_needs_exact_profile_and_consistent_explicit_reference(self):
        for change in (lambda c: c["research_profile"].update(name="unreviewed"),
                       lambda c: c["research_profile"].update(extra=True),
                       lambda c: c["research_profile"].update(world_sdf="relative.sdf"),
                       lambda c: c.update(hover_reference_profile=None),
                       lambda c: c["hover_reference_profile"].update(world_sdf=str(self.folder/"other.sdf")),
                       lambda c: c.update(pid_profile=None)):
            cfg = self.cfg()
            change(cfg)
            with self.assertRaises(ValueError):
                task.validate_task_config(cfg)

    def test_frozen_pid_requires_full_actual_numeric_profile_and_matching_digest(self):
        original = deepcopy(self.profile)
        changes = (lambda p: p["parameters"].pop("MC_AIRMODE"),
                   lambda p: p["parameters"].update(extra=1.),
                   lambda p: p["parameters"].update(MC_AIRMODE=True),
                   lambda p: p["parameters"].update(MC_ROLL_P=float("nan")),
                   lambda p: p["parameters"].update(MPC_XY_VEL_P_ACC=3.),
                   lambda p: p.update(model_sha256="missing"),
                   lambda p: p.update(pid_parameters_sha256="0"*64))
        for change in changes:
            self.profile = deepcopy(original)
            change(self.profile)
            self.write_profile()
            with self.assertRaises(ValueError):
                task.validate_task_config(self.cfg())

    def test_condition_binds_content_not_host_path_and_parameters_are_independent_copies(self):
        cfg = self.cfg()
        first = research.condition_for_task(cfg)
        moved_world = self.folder/"same_world_elsewhere.sdf"
        moved_world.write_bytes(self.world.read_bytes())
        cfg["research_profile"]["world_sdf"] = str(moved_world)
        cfg["hover_reference_profile"]["world_sdf"] = str(moved_world)
        self.assertEqual(first, research.condition_for_task(cfg))
        self.assertEqual(first["pid_parameters_sha256"], self.profile["pid_parameters_sha256"])
        estimator, pid = research.research_runtime_parameters(cfg)
        self.assertEqual(estimator, PRECISION_PROFILE)
        self.assertEqual(pid, PID_CANDIDATES["velocity_high"])
        estimator["EKF2_EV_CTRL"] = 0
        pid["MPC_Z_P"] = 99.
        self.assertEqual(research.research_runtime_parameters(cfg),
                         (PRECISION_PROFILE, PID_CANDIDATES["velocity_high"]))

    def test_runtime_condition_validator_rejects_changed_protocol_or_boolean_alias(self):
        condition = research.condition_for_task(self.cfg())
        for change in (lambda c: c.update(extra=True),
                       lambda c: c.update(ground_truth_aided=1),
                       lambda c: c.update(hardware_performance_claim=0),
                       lambda c: c.update(pid_parameters_sha256="not-a-hash"),
                       lambda c: c["observer"].update(rate_hz=100.),
                       lambda c: c.update(coordinate_contract="global_wgs84"),
                       lambda c: c.update(control_interface="velocity")):
            value = deepcopy(condition)
            change(value)
            with self.assertRaises(ValueError):
                research.validate_operating_condition(value)

    def test_full_audited_same_condition_gates_pass_and_tampered_artifact_fails(self):
        cfg = self.cfg("joint_probe")
        provenance = self.evidence(cfg)
        self.assertEqual(len(task.validate_stage_gates(cfg, provenance)), 2)
        (self.folder/"px4.ulg").write_text("tampered")
        with self.assertRaisesRegex(ValueError, "artifact mismatch"):
            task.validate_stage_gates(cfg, provenance)

    def test_legacy_gps_cannot_unlock_research_and_research_cannot_unlock_legacy(self):
        cfg = self.cfg("joint_probe")
        provenance = self.evidence(cfg, research_mode=False)
        with self.assertRaisesRegex(ValueError, "operating condition"):
            task.validate_stage_gates(cfg, provenance)
        provenance = self.evidence(cfg)
        legacy = task.build_task_config("joint_probe")
        legacy["stage_evidence"] = cfg["stage_evidence"]
        with self.assertRaisesRegex(ValueError, "legacy GPS"):
            task.validate_stage_gates(legacy, provenance)

    def test_provisional_or_diagnostic_or_incomplete_audit_cannot_unlock_research(self):
        for field, value in (("diagnostic_only", True), ("ground_truth_aided", False),
                             ("eligible_as_stage_gate", False), ("research_acceptance_verified", False),
                             ("full_window_ulog_verified", False), ("parameter_restore_verified", False)):
            cfg = self.cfg("joint_probe")
            provenance = self.evidence(cfg)
            self.edit_first_gate(cfg, lambda data: data.update({field: value}))
            with self.subTest(field=field), self.assertRaises(ValueError):
                task.validate_stage_gates(cfg, provenance)

    def test_condition_pid_world_and_model_mismatch_fail(self):
        cfg = self.cfg("joint_probe")
        provenance = self.evidence(cfg)
        self.edit_first_gate(cfg, lambda data: data.update(pid_parameters_sha256="c"*64))
        with self.assertRaisesRegex(ValueError, "operating condition"):
            task.validate_stage_gates(cfg, provenance)
        self.evidence(cfg)
        self.world.write_text("changed physical world bytes")
        with self.assertRaisesRegex(ValueError, "operating condition"):
            task.validate_stage_gates(cfg, provenance)
        with self.assertRaisesRegex(ValueError, "physical model"):
            task.validate_stage_gates(self.cfg(), {**provenance, "model_sha256": "c"*64})

    def test_acceptance_flags_alone_do_not_replace_required_raw_and_audit_files(self):
        for name in ("stage_result.json", "controller_metadata.json", "precision_stage_audit.json", "px4.ulg"):
            cfg = self.cfg("joint_probe")
            provenance = self.evidence(cfg)
            self.edit_first_gate(cfg, lambda data: data["artifacts"].pop(name))
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "hashed full ULog"):
                task.validate_stage_gates(cfg, provenance)


if __name__ == "__main__":
    unittest.main()
