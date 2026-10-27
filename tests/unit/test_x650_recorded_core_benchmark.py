"""Frozen evidence reconstruction tests; no ROS node, SSH or actuator is used."""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"scripts"))
import prepare_x650_recorded_core_benchmark as prepare


class CdrAndCommitTests(unittest.TestCase):
    def test_cdr_payload_alignment_and_finite_values(self):
        # Fixed golden bytes: CDR encapsulation; uint16=7; padding; uint64=9;
        # bool=true; padding; float64=-1.25. Padding belongs to the payload.
        raw = bytes.fromhex("00010000 0700 000000000000 0900000000000000 01 00000000000000 000000000000f4bf")
        layout = prepare.message_layout(b"uint16 schema_version 1\nuint64 seq\nbool ready true\nfloat64 x\n")
        self.assertEqual(prepare.decode_cdr(raw, layout),
                         {"schema_version": 7, "seq": 9, "ready": True, "x": -1.25})
        for data in (raw[:-1], raw+b"\x00", b"\x00\x00\x00\x00"+raw[4:]):
            with self.assertRaises(ValueError):
                prepare.decode_cdr(data, layout)
        nonfinite = raw[:-8]+struct.pack("<d", float("nan"))
        with self.assertRaises(ValueError):
            prepare.decode_cdr(nonfinite, layout)
        bad_bool = raw[:20]+b"\x02"+raw[21:]
        with self.assertRaises(ValueError):
            prepare.decode_cdr(bad_bool, layout)

    def test_layout_rejects_variable_length_and_duplicate_fields(self):
        for source in (b"float64[] x", b"string x", b"float64 x\nfloat64 x"):
            with self.assertRaises(ValueError):
                prepare.message_layout(source)

    def test_memory_successful_commit_preserves_input_and_prunes_exact_boundary(self):
        memory = {"initialized": True, "anchor": {"q": [0., 0.]}, "previous_command": [0.]*5,
            "step_count": 4, "joint_history": [
                {"time": .249999, "velocity": [1., 2.]},
                {"time": .25, "velocity": [3., 4.]}]}
        before = deepcopy(memory)
        value = {"t": 1., "state": {"stamp": 3.1, "control_time": 3.2}}
        result = {"command": [1., 2., 3., 4., 5.], "joint_velocity": [4., 5.],
                  "diagnostics": {"plan": [8., 9.], "horizon": [.02, .04]}}
        updated = prepare.commit_recorded(memory, value, result)
        self.assertEqual(memory, before)
        self.assertEqual(updated["step_count"], 5)
        self.assertEqual(updated["joint_history"], [
            {"time": .25, "velocity": [3., 4.]}, {"time": 1., "velocity": [4., 5.]}])
        self.assertEqual(updated["origin"], 3.2-1.)
        self.assertEqual(updated["previous_stamp"], 3.1)
        updated["plan"][0] = 999
        self.assertEqual(result["diagnostics"]["plan"][0], 8.)

    def test_trajectory_steady_phase_and_post_stop_are_not_constant_acceleration_fallback(self):
        center = [1., 2., 3.]
        self.assertEqual(prepare.frozen_target(0., center, "normal"),
                         {"position": center, "velocity": [0., 0., 0.], "acceleration": [0., 0., 0.]})
        # Normal midpoint of first steady half-loop: phi = pi / 2 at t = 6.
        value = prepare.frozen_target(6., center, "normal")
        self.assertAlmostEqual(value["position"][0], 1.25, places=15)
        self.assertAlmostEqual(value["position"][2], 3., places=15)
        stopped = prepare.frozen_target(32., center, "normal")
        self.assertEqual(stopped["velocity"], [0., 0., 0.])
        self.assertEqual(stopped["acceleration"], [0., 0., 0.])

    def test_chunking_keeps_every_original_case_and_bounds_native_schema(self):
        cases = [{"id": str(i)} for i in range(1001)]
        provenance = {"cases": [{"sequence": i} for i in range(1001)]}
        chunks = list(prepare.bundles({}, cases, provenance, chunk_size=400, measured_rounds=10))
        self.assertEqual([len(v["cases"]) for v in chunks], [400, 400, 201])
        self.assertEqual([c for v in chunks for c in v["cases"]], cases)
        self.assertEqual(chunks[-1]["provenance"]["chunk"]["last_original_sequence"], 1001)
        for values in ({"chunk_size": 513}, {"measured_rounds": 10000}, {"cpu": -2}):
            with self.assertRaises(ValueError):
                list(prepare.bundles({}, cases, provenance, **values))


class FrozenArchiveRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.recovered = {}
        for stage in ("normal", "fast"):
            folder = ROOT/f"docs/validation/cpp_{stage}_20261024"
            if not (folder/f"{stage}_full_raw_v1.zip").exists():
                raise unittest.SkipTest("Accepted original archive is not available")
            cls.recovered[stage] = prepare.recover(folder/f"{stage}_full_raw_v1.zip",
                folder/f"{stage}_full_raw_v1_receipt.json", folder/"report.json", stage)

    def test_all_real_calls_and_complete_history_recovered(self):
        for stage, count in (("normal", 1598), ("fast", 1198)):
            config, cases, provenance = self.recovered[stage]
            self.assertEqual(len(cases), count)
            self.assertEqual(provenance["historical_attempt_distribution"], {"1": count})
            self.assertFalse(provenance["synthetic_inputs"])
            self.assertFalse(provenance["memory_was_directly_recorded"])
            self.assertFalse(provenance["native_equivalence_already_verified"])
            self.assertEqual(cases[0]["memory"]["step_count"], 0)
            self.assertEqual(cases[-1]["memory"]["step_count"], count-1)
            for index in (1, 399, 400, count-1):
                case, prior = cases[index], cases[index-1]
                self.assertEqual(case["memory"]["previous_command"], prior["oracle"]["command"])
                self.assertEqual(case["memory"]["last_t"], prior["input"]["t"])
                self.assertEqual(case["memory"]["previous_stamp"], prior["input"]["state"]["stamp"])
                self.assertEqual(case["memory"]["joint_history"][-1]["velocity"], prior["oracle"]["joint_velocity"])
            self.assertEqual(len(cases[-1]["memory"]["plan"]), 75)
            self.assertEqual(len(cases[-1]["input"]["preview"]), 15)
            chunks = list(prepare.bundles(config, cases, provenance))
            self.assertEqual(sum(len(v["cases"]) for v in chunks), count)
            self.assertTrue(all(len(prepare.json_bytes(v)) < 32*1024*1024 for v in chunks))

    def test_recorded_command_oracles_are_unchanged_original_results(self):
        for stage in ("normal", "fast"):
            _, cases, provenance = self.recovered[stage]
            path = ROOT/f"docs/validation/cpp_{stage}_20261024/{stage}_full_raw_v1.zip"
            with zipfile.ZipFile(path) as archive:
                for index in (0, 399, len(cases)-1):
                    case, source = cases[index], provenance["cases"][index]
                    original = json.loads(archive.read(source["record_archive_entry"]))
                    for key in prepare.OUTPUT_FIELDS:
                        self.assertEqual(case["oracle"][key], original["result"][key])
                    self.assertEqual(case["memory"]["step_count"], original["sequence"]-1)

    def test_derived_policy_preserves_all_history_and_primary_oracles(self):
        config, cases, provenance = self.recovered["fast"]
        original = next(prepare.bundles(config, cases, provenance))
        before = deepcopy(original)
        derived = prepare.apply_field_policy(original, "recorded-derived-v2", "a"*64)
        self.assertEqual(original, before)
        self.assertEqual(derived["run"], original["run"])
        for case, old, row in zip(derived["cases"], original["cases"], derived["provenance"]["cases"]):
            self.assertEqual(case["memory"], old["memory"])
            self.assertEqual(case["input"], old["input"])
            self.assertEqual(set(case["oracle"]), {"command", "position_reference", "velocity_feedforward"})
            self.assertEqual(row["historical_output_oracle"], old["oracle"])
            for key in case["oracle"]:
                self.assertEqual(case["oracle"][key], old["oracle"][key])
        with self.assertRaises(ValueError):
            prepare.apply_field_policy(original, "silent-loosen")
        with self.assertRaises(ValueError):
            prepare.apply_field_policy(derived, "recorded-derived-v2")

    def test_wrong_receipt_rejected_before_recovery(self):
        folder = ROOT/"docs/validation/cpp_fast_20261024"
        receipt = SimpleNamespace(read_bytes=lambda: prepare.json_bytes(
            {"bytes": 58852185, "sha256": "0"*64}))
        with self.assertRaisesRegex(ValueError, "receipt"):
            prepare.recover(folder/"fast_full_raw_v1.zip", receipt, folder/"report.json", "fast")


if __name__ == "__main__":
    unittest.main()
