"""Frozen command design contracts; synthetic tests never grant flight evidence."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"simulations/x650_wholebody_mpc_v1"))
sys.path.insert(0, str(Path(__file__).parent))
import response_protocol as protocol
import test_x650_upper_research_condition as fixtures
from run_case import research_annotations, response_probe_velocity

task, research = fixtures.task, fixtures.research


class FrozenProtocolTests(unittest.TestCase):
    def test_schedules_are_distinct_and_balanced_with_safe_zero_gaps(self):
        hashes = set()
        widths = []
        for role in protocol.ROLES:
            spec = protocol.build_response_protocol(role)
            hashes.add(protocol.response_protocol_hash(spec))
            pulses = spec["local_pulses"]
            self.assertEqual(spec["duration_s"], 60.)
            self.assertEqual(spec["return_anchor_start_s"], 54.)
            self.assertAlmostEqual(sum((p["end_s"]-p["start_s"])*p["sign"] for p in pulses), 0.)
            self.assertTrue(all(b["start_s"]-a["end_s"] >= 2.3-1e-12 for a,b in zip(pulses,pulses[1:])))
            widths.append({round(p["end_s"]-p["start_s"],6) for p in pulses})
            for axis, (offset, end) in enumerate(spec["axis_windows_s"]):
                self.assertEqual(protocol.protocol_velocity(offset+1.,spec), [0.,0.,0.])
                for pulse in pulses:
                    expected = np.zeros(3)
                    expected[axis] = pulse["sign"]*spec["amplitude_m_s"][axis]
                    np.testing.assert_array_equal(protocol.protocol_velocity(offset+(pulse["start_s"]+pulse["end_s"])/2,spec), expected)
                self.assertEqual(protocol.protocol_velocity(end-.1,spec), [0.,0.,0.])
        self.assertEqual(len(hashes),2)
        self.assertFalse(widths[0] & widths[1])

    def test_boundaries_are_half_open_and_final_return_is_not_hidden(self):
        spec = protocol.build_response_protocol("train")
        self.assertEqual(protocol.protocol_velocity(2.,spec), [.08,0.,0.])
        self.assertEqual(protocol.protocol_velocity(2.8,spec), [0.,0.,0.])
        self.assertEqual(protocol.protocol_velocity(5.3,spec), [-.08,0.,0.])
        for t in (-1.,0.,14.,18.,36.,53.99,54.,60.):
            self.assertEqual(protocol.protocol_velocity(t,spec), [0.,0.,0.])
        with self.assertRaises(ValueError):
            protocol.protocol_velocity(float("nan"),spec)

    def test_tampered_or_boolean_aliased_protocol_is_rejected(self):
        original = protocol.build_response_protocol("holdout")
        for change in (lambda p:p.update(extra=True), lambda p:p.update(duration_s=61.),
                       lambda p:p.update(independent_holdout=1),
                       lambda p:p["local_pulses"][0].update(start_s=1.9),
                       lambda p:p.update(role="development")):
            value = deepcopy(original)
            change(value)
            with self.assertRaises(ValueError):
                protocol.validate_response_protocol(value)
        copy = protocol.validate_response_protocol(original)
        copy["local_pulses"][0]["start_s"] = 0.
        self.assertEqual(protocol.build_response_protocol("holdout"),original)


class FrozenTaskTests(unittest.TestCase):
    setUp = fixtures.UpperResearchConditionTests.setUp
    write_profile = fixtures.UpperResearchConditionTests.write_profile
    cfg = fixtures.UpperResearchConditionTests.cfg

    def formal(self, role="train"):
        cfg = self.cfg()
        spec = protocol.build_response_protocol(role)
        cfg.update(response_probe_protocol=spec,duration_s=spec["duration_s"])
        if role == "holdout":
            cfg.update(frozen_response_candidate=None, frozen_response_candidate_sha256=None)
        return cfg

    def test_archived_task_keys_condition_and_legacy_pulses_unchanged(self):
        old = self.cfg()
        self.assertNotIn("response_probe_protocol",old)
        self.assertEqual(task.validate_task_config(old),old)
        self.assertEqual(protocol.protocol_annotation(old),{})
        explicit = dict(old,response_probe_protocol=None)
        self.assertIsNone(task.validate_task_config(explicit)["response_probe_protocol"])
        self.assertEqual(research.condition_for_task(self.formal()),research.condition_for_task(old))
        np.testing.assert_array_equal(response_probe_velocity(2.,30.),[.08,0.,0.])
        np.testing.assert_array_equal(response_probe_velocity(4.,30.),[-.08,0.,0.])
        np.testing.assert_array_equal(response_probe_velocity(14.,30.),[0.,0.,0.])

    def test_only_exact_formal_task_with_existing_upper_feedback_is_valid(self):
        cfg = self.formal()
        self.assertEqual(task.validate_task_config(cfg),cfg)
        generated = task.build_task_config("response_probe",response_probe_protocol=cfg["response_probe_protocol"])
        self.assertEqual(generated["duration_s"],60.)
        for change in (lambda c:c.update(duration_s=30.),
                       lambda c:c.update(research_profile=None),
                       lambda c:c.update(stage="tip_hold"),
                       lambda c:c.update(controller="mpc")):
            value = deepcopy(cfg)
            change(value)
            with self.assertRaises(ValueError):
                task.validate_task_config(value)

    def test_role_identity_is_separate_from_plant_condition_and_preparation_gates(self):
        train,holdout = self.formal(),self.formal("holdout")
        self.assertEqual(research.condition_for_task(train),research.condition_for_task(holdout))
        annotations = [research_annotations(c) for c in (train,holdout)]
        self.assertNotEqual(annotations[0]["response_probe_protocol_sha256"],annotations[1]["response_probe_protocol_sha256"])
        self.assertEqual(annotations[0]["operating_condition_sha256"],annotations[1]["operating_condition_sha256"])
        self.assertEqual(task.required_gates(train),["hover_folded","joint_probe","native_ground"])
        self.assertFalse(annotations[0]["eligible_as_stage_gate"])
        sources = task.source_files_for_task(train)
        self.assertIn(ROOT/"simulations/x650_wholebody_mpc_v1/response_protocol.py",sources)

    def test_holdout_draft_can_be_declared_but_cannot_pass_candidate_preflight(self):
        cfg = self.formal("holdout")
        self.assertEqual(task.validate_task_config(cfg),cfg)
        with self.assertRaisesRegex(ValueError,"existing frozen"):
            task.validate_frozen_response_candidate(cfg)
        changed = dict(cfg,frozen_response_candidate_sha256="a"*64)
        with self.assertRaisesRegex(ValueError,"together"):
            task.validate_task_config(changed)
        train = dict(self.formal(),frozen_response_candidate=None,frozen_response_candidate_sha256=None)
        with self.assertRaises(ValueError):
            task.validate_task_config(train)

    def test_holdout_frozen_candidate_is_byte_bound_and_never_stage_acceptance(self):
        cfg = self.formal("holdout")
        condition = research.condition_for_task(cfg)
        candidate = {"kind":"x650_frozen_response_candidate","schema_version":1,
            "training_passed":True,"identified":False,"eligible_as_stage_gate":False,
            "created_unix":100.,"protocols":{r:protocol.build_response_protocol(r) for r in protocol.ROLES},
            "parameters":{"x":{"wn_rad_s":2.8,"zeta":.7},"y":{"wn_rad_s":2.8,"zeta":.7},"z":{"tau_s":.2}},
            "identity":{"operating_condition":condition,"operating_condition_sha256":research.canonical_hash(condition),
                        "pid_parameters_sha256":condition["pid_parameters_sha256"]}}
        path = self.folder/"frozen_response_candidate.json"
        def save(value):
            path.write_text(json.dumps(value))
            cfg.update(frozen_response_candidate=str(path),frozen_response_candidate_sha256=task.sha(path))
        save(candidate)
        self.assertEqual(task.validate_task_config(cfg),cfg)
        binding = task.validate_frozen_response_candidate(cfg)
        self.assertFalse(binding["eligible_as_stage_gate"])
        path.write_text(json.dumps(candidate)+"\n")
        with self.assertRaisesRegex(ValueError,"SHA-256 changed"):
            task.validate_frozen_response_candidate(cfg)
        for change in (lambda c:c.update(identified=True),
                       lambda c:c.update(training_passed=False),
                       lambda c:c.update(created_unix=-1.),
                       lambda c:c.update(created_unix=1e20),
                       lambda c:c["parameters"]["x"].update(wn_rad_s=40.),
                       lambda c:c["identity"].update(operating_condition_sha256="f"*64)):
            value = deepcopy(candidate)
            change(value)
            save(value)
            with self.assertRaises(ValueError):
                task.validate_frozen_response_candidate(cfg)


if __name__ == "__main__":
    unittest.main()
