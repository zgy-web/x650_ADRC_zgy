"""Offline evidence-contract tests for the x86/Orin matched-input comparison."""
from __future__ import annotations

import copy
import hashlib
import io
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import compare_x650_paired_core_benchmark as paired


def save(path, value):
    path.write_text(json.dumps(value, allow_nan=False), encoding="utf-8")


class PairedCoreTests(unittest.TestCase):
    def fixture(self, directory):
        root = Path(directory)
        data = b"frozen original source\n"
        package = root / "code.tar.gz"
        with tarfile.open(package, "w:gz") as tar:
            row = tarfile.TarInfo("src/original.cpp")
            row.size = len(data)
            tar.addfile(row, io.BytesIO(data))
        source = paired.identity(package)
        manifest = {"kind": "x650_orin_code_only_source_package_v1", "archive": source,
                    "sources": [{"path": "src/original.cpp", "bytes": len(data),
                                 "sha256": hashlib.sha256(data).hexdigest()}]}
        save(root / "preparation.json", {"source_package": manifest})
        cases = [{"id": str(i), "memory": {"step_count": 4},
                  "input": {"t": float(i), "dt": .02 if i == 0 else .016},
                  "oracle": {"command": [0.] * 5, "position_reference": [0.] * 3,
                             "velocity_feedforward": [0.] * 3}} for i in range(2)]
        bundle = {"kind": "x650_mpc_memory_benchmark_input_v1", "schema_version": 1,
                  "config": {"max_dt": .05}, "cases": cases,
                  "provenance": {"synthetic_inputs": True},
                  "run": {"cpu": 2, "warmup_rounds": 1, "measured_rounds": 2,
                          "oracle_absolute_tolerance": 1e-7}}
        save(root / "input.json", bundle)
        input_id = paired.identity(root / "input.json")
        for name, architecture, letter, timing in (("server", "x86_64", "a", 4_000_000),
                                                   ("nx", "aarch64", "b", 5_000_000)):
            folder = root / name
            folder.mkdir()
            expected = {key: letter * 64 for key in paired.BUILD_KEYS}
            files = {buildkey: {"sha256": expected[key], "bytes": 100}
                     for key, buildkey in paired.BUILD_KEYS.items()}
            build = {"complete": True, "architecture": architecture,
                     "package_sha256": source["sha256"], "files": files,
                     "commands": [["cmake", "-S", "/ram/src", "-B", "/ram/build",
                                   "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF"]]}
            save(root / (name + "_build.json"), build)
            def sample(index, round_number):
                output = {key: [0.] * size for key, size in paired.OUTPUT_DIMENSIONS.items()}
                return {"case_index": index, "round": round_number,
                    "core_step_wall_ns": timing, "core_step_thread_cpu_ns": timing - 1000,
                    "returned": True, "core_error": False, "unexpected_exception": False,
                    "error": None, "valid": True, "exceeded_20ms_target": False,
                    "exceeded_50ms_wall": False, "production_deadline_error": False,
                    "memory_unchanged": False, "memory_commit_valid": True,
                    "step_count_after": 5, "oracle_checked": True, "oracle_match": True,
                    "oracle_max_absolute_error": 0., "output": output,
                    "diagnostics": {"accepted": True, "nonlinear_passed": True,
                        "actual_bounds": True, "compute_ms": timing / 1e6,
                        "subproblems": 1, "active_constraints": index,
                        "qp_rows": 230, "qp_columns": 75, "candidate_substeps": 20,
                        "previous_plan_fallback": False,
                        "attempts": [{"passed": True, "actual_bounds": True,
                                      "q_error": 0., "dq_error": 0., "elapsed_ms": 1.}]}}
            def summary(rows):
                n = len(rows)
                def times(v):
                    return {"sample_count": n, "min_ns": v, "median_ns": v,
                            "p95_ns": v, "p99_ns": v, "max_ns": v, "mean_ns": float(v)}
                return {"sample_count": n, "returned_count": n, "valid_count": n,
                    "oracle_checked_count": n, "oracle_match_count": n,
                    "core_error_count": 0, "unexpected_exception_count": 0,
                    "exceeded_20ms_target_count": 0, "exceeded_50ms_wall_count": 0,
                    "production_deadline_error_count": 0, "failed_calls_with_unchanged_memory": 0,
                    "all_calls_valid": True, "statistics_include_all_failed_and_slow_calls": True,
                    "percentile_method": "nearest_rank", "wall": times(timing),
                    "thread_cpu": times(timing - 1000)}
            warmup = [sample(i, 0) for i in range(2)]
            measured = [sample(i % 2, i // 2) for i in range(4)]
            scope = copy.deepcopy(paired.EXPECTED_SCOPE)
            scope["oracle_field_absolute_tolerances"] = {k: 1e-9 if k == "position_reference" else 1e-7
                                                        for k in paired.OUTPUT_DIMENSIONS}
            result = {"kind": "x650_mpc_memory_benchmark_result_v1", "schema_version": 1,
                "complete": True, "backend": "cpu", "mode": "fixed_snapshot_paired",
                "input_sha256": input_id["sha256"], "run": bundle["run"],
                "provenance": bundle["provenance"], "case_count": 2,
                "cases": [{"id": c["id"], "input_t_s": c["input"]["t"], "dt_s": c["input"]["dt"],
                           "step_count_before": 4} for c in cases],
                "loaded_code": {k: {"sha256": v} for k, v in expected.items()},
                "allowed_cpus": [2], "scope": scope, "clock_calibration": {"subtracted_from_samples": False},
                "all_calls_valid": True, "cold_first_call": sample(0, 0),
                "warmup_samples": warmup, "samples": measured,
                "summary": {"warmup": summary(warmup), "measured": summary(measured)}}
            save(folder / "native_stdout.json", result)
            (folder / "ssh_stderr.txt").write_bytes(b"")
            transport = {"complete": True, "pass": True, "exit_code": 0, "timed_out": False,
                "all_calls_valid": True, "transport_wall_is_not_algorithm_time": True,
                "remote_input_files_created": False, "remote_result_files_created": False,
                "input": input_id, "expected_loaded_code_sha256": expected,
                "stdout": paired.identity(folder / "native_stdout.json"),
                "stderr": paired.identity(folder / "ssh_stderr.txt")}
            save(folder / "transport_report.json", transport)
        return root

    def compare(self, root, field_policy="strict-v1"):
        return paired.compare(root / "input.json", root / "server", root / "nx",
                              root / "preparation.json", root / "server_build.json",
                              root / "nx_build.json", field_policy=field_policy)

    def recorded_fixture(self, directory):
        root = self.fixture(directory)
        input_path = root / "input.json"
        bundle = paired.load_json(input_path)
        historical = []
        for case in bundle["cases"]:
            case["memory"]["previous_command"] = [0.] * 5
            historical.append({"id": case["id"], "historical_output_oracle":
                {field: [0.] * size for field, size in paired.OUTPUT_DIMENSIONS.items()}})
        bundle["provenance"]["cases"] = historical
        save(input_path, bundle)
        input_id = paired.identity(input_path)
        for side in ("server", "nx"):
            path = root / side / "native_stdout.json"
            result = paired.load_json(path)
            result["provenance"] = bundle["provenance"]
            result["input_sha256"] = input_id["sha256"]
            save(path, result)
            report_path = root / side / "transport_report.json"
            report = paired.load_json(report_path)
            report["input"] = input_id
            report["stdout"] = paired.identity(path)
            save(report_path, report)
        return root

    def change_result(self, root, mutate):
        path = root / "nx/native_stdout.json"
        value = paired.load_json(path)
        mutate(value)
        save(path, value)
        transport_path = root / "nx/transport_report.json"
        report = paired.load_json(transport_path)
        report["stdout"] = paired.identity(path)
        save(transport_path, report)

    def test_same_source_different_architecture_elf_and_exact_input_pass(self):
        with tempfile.TemporaryDirectory() as temp:
            root = self.fixture(temp)
            report = self.compare(root)
            self.assertTrue(report["pass"])
            self.assertEqual(report["independent_output_pairs_checked"], 7)
            self.assertEqual(report["measured"]["wall"]["nx_minus_server"]["p50"], 1.)
            self.assertEqual(len(report["groups"]["case"]), 2)
            self.assertEqual(len(report["groups"]["dt_s"]), 2)
            self.assertEqual(report["measured"]["wall"]["server_over_nx_ratio"]["p50"], .8)

    def test_changed_input_or_run_cannot_be_compared(self):
        with tempfile.TemporaryDirectory() as temp:
            root = self.fixture(temp)
            self.change_result(root, lambda x: x["run"].update(measured_rounds=3))
            with self.assertRaisesRegex(ValueError, "run settings"):
                self.compare(root)

    def test_source_build_or_loaded_elf_mismatch_rejected(self):
        for alteration, message in (("source", "source package"), ("elf", "ELF hashes")):
            with self.subTest(alteration=alteration), tempfile.TemporaryDirectory() as temp:
                root = self.fixture(temp)
                path = root / "nx_build.json"
                build = paired.load_json(path)
                if alteration == "source":
                    build["package_sha256"] = "f" * 64
                else:
                    build["files"]["core_library"]["sha256"] = "f" * 64
                save(path, build)
                with self.assertRaisesRegex(ValueError, message):
                    self.compare(root)

    def test_timing_scope_and_affinity_cannot_change(self):
        for change in (lambda x: x["scope"].update(memory_restore_inside_core_timer=True),
                       lambda x: x.update(allowed_cpus=[2, 3])):
            with tempfile.TemporaryDirectory() as temp:
                root = self.fixture(temp)
                self.change_result(root, change)
                with self.assertRaises(ValueError):
                    self.compare(root)

    def test_nonoracle_output_difference_is_reported_as_failed_comparison(self):
        with tempfile.TemporaryDirectory() as temp:
            root = self.fixture(temp)
            self.change_result(root, lambda x: x["samples"][0]["output"]["joint_reference"].__setitem__(0, .1))
            report = self.compare(root)
            self.assertFalse(report["pass"])
            self.assertEqual(report["cross_target_max_absolute_errors"]["joint_reference"], .1)
            self.assertEqual(report["cross_target_mismatches"][0]["fields"], ["joint_reference"])

    def test_missing_call_or_changed_memory_contract_rejected(self):
        for change in (lambda x: x["samples"].pop(),
                       lambda x: x["samples"][0].update(memory_unchanged=True)):
            with tempfile.TemporaryDirectory() as temp:
                root = self.fixture(temp)
                self.change_result(root, change)
                with self.assertRaises(ValueError):
                    self.compare(root)

    def test_changed_raw_file_rejected_before_output_is_trusted(self):
        with tempfile.TemporaryDirectory() as temp:
            root = self.fixture(temp)
            (root / "nx/native_stdout.json").write_bytes(b"{}")
            with self.assertRaisesRegex(ValueError, "stdout identity"):
                self.compare(root)

    def test_changed_source_member_rejected_even_if_tar_digest_updated(self):
        with tempfile.TemporaryDirectory() as temp:
            root = self.fixture(temp)
            manifest_path = root / "preparation.json"
            manifest = paired.load_json(manifest_path)
            manifest["source_package"]["sources"][0]["sha256"] = "a" * 64
            save(manifest_path, manifest)
            with self.assertRaisesRegex(ValueError, "tar member differs"):
                self.compare(root)

    def test_comparison_retains_failed_and_slow_timing_rows(self):
        good = {"valid": True, "returned": True, "core_error": False, "unexpected_exception": False,
                "exceeded_20ms_target": False, "exceeded_50ms_wall": False,
                "core_step_wall_ns": 4_000_000, "core_step_thread_cpu_ns": 3_900_000}
        failed = dict(good, valid=False, returned=False, core_error=True,
                      core_step_wall_ns=60_000_000, core_step_thread_cpu_ns=59_000_000,
                      exceeded_20ms_target=True, exceeded_50ms_wall=True)
        report = paired.comparison([(good, good), (good, failed)])
        self.assertEqual(report["wall"]["nx"]["max"], 60.)
        self.assertEqual(report["nx_invalid_calls"], 1)
        self.assertEqual(report["status_mismatches"], 1)

    def test_duplicate_json_keys_and_nan_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "bad.json"
            for raw in ('{"a":1,"a":2}', '{"a":NaN}'):
                path.write_text(raw)
                with self.assertRaises(ValueError):
                    paired.load_json(path)

    def test_derived_policy_preserves_strict_failure_and_accepts_consistent_derivative(self):
        with tempfile.TemporaryDirectory() as temp:
            root = self.recorded_fixture(temp)
            def change(result):
                row = result["samples"][0]
                row["output"]["command"][3] = 5e-8
                row["output"]["joint_velocity"][0] = 5e-8
                row["output"]["joint_acceleration_reference"][0] = 5e-8 / .02
                row["oracle_max_absolute_error"] = 5e-8
            self.change_result(root, change)
            strict = self.compare(root)
            self.assertFalse(strict["pass"])
            derived = self.compare(root, "recorded-derived-v2")
            self.assertTrue(derived["pass"])
            self.assertFalse(derived["strict_full_field_passed"])
            self.assertEqual(derived["strict_failure_count"], 1)
            self.assertEqual(derived["strict_measured_failure_count"], 1)
            self.assertEqual(derived["strict_failure_check_count"], 2)
            self.assertEqual(derived["historical_output_pairs_checked"], {"server": 7, "nx": 7})
            self.assertEqual(derived["derived_acceleration_relationship_failures"], [])
            self.assertEqual(derived["field_tolerances_by_case"][0]["absolute_tolerances"]
                             ["joint_acceleration_reference"], 1e-7 / .02)

    def test_derived_policy_rejects_inconsistent_acceleration_within_propagated_budget(self):
        with tempfile.TemporaryDirectory() as temp:
            root = self.recorded_fixture(temp)
            self.change_result(root, lambda x: x["samples"][0]["output"]
                               ["joint_acceleration_reference"].__setitem__(0, 2e-6))
            report = self.compare(root, "recorded-derived-v2")
            self.assertFalse(report["pass"])
            self.assertEqual(report["cross_target_mismatches"], [])
            self.assertEqual(report["historical_output_mismatches"], [])
            self.assertEqual(len(report["derived_acceleration_relationship_failures"]), 1)

    def test_derived_policy_cannot_accept_true_command_error_above_original_tolerance(self):
        with tempfile.TemporaryDirectory() as temp:
            root = self.recorded_fixture(temp)
            # Both native primary oracles legitimately agree with these outputs;
            # the independent historical oracle remains zero. This isolates the
            # comparator's command threshold from runner validity bookkeeping.
            bundle_path = root / "input.json"
            bundle = paired.load_json(bundle_path)
            for case in bundle["cases"]:
                case["oracle"]["command"][3] = 2e-7
            save(bundle_path, bundle)
            input_id = paired.identity(bundle_path)
            for side in ("server", "nx"):
                path = root / side / "native_stdout.json"
                result = paired.load_json(path)
                result["input_sha256"] = input_id["sha256"]
                rows = [result["cold_first_call"], *result["warmup_samples"], *result["samples"]]
                for row in rows:
                    dt = bundle["cases"][row["case_index"]]["input"]["dt"]
                    row["output"]["command"][3] = 2e-7
                    row["output"]["joint_acceleration_reference"][0] = 2e-7 / dt
                save(path, result)
                transport_path = root / side / "transport_report.json"
                transport = paired.load_json(transport_path)
                transport["input"] = input_id
                transport["stdout"] = paired.identity(path)
                save(transport_path, transport)
            report = self.compare(root, "recorded-derived-v2")
            self.assertFalse(report["pass"])
            self.assertTrue(any("command" in row["fields"] for row in report["historical_output_mismatches"]))
            self.assertFalse(report["strict_full_field_passed"])

    def test_derived_policy_requires_ten_field_historical_oracle(self):
        with tempfile.TemporaryDirectory() as temp:
            root = self.fixture(temp)
            with self.assertRaisesRegex(ValueError, "historical provenance"):
                self.compare(root, "recorded-derived-v2")


if __name__ == "__main__":
    unittest.main()
