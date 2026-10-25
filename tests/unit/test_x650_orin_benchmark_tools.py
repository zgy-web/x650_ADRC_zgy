"""Transport/provenance tests; no NX, SSH session or hardware is required."""
from __future__ import annotations

import hashlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import shlex
import sys
import tarfile
import tempfile
from types import SimpleNamespace
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import prepare_x650_orin_benchmark as prepare
import run_x650_orin_benchmark as run


class OrinToolsTests(unittest.TestCase):
    def fixture(self, folder, *, changed_input=False, mixed_config=False):
        paths = {k: folder / k for k in ("raw.zip", "receipt.json", "report.json")}
        certificate = {key: True for key in ("passed", "complete", "certificate", "physical_acceptance")}
        certificate_raw = prepare.json_bytes(certificate)
        paths["report.json"].write_bytes(certificate_raw)
        cases, rows = [], []
        for i in range(48):
            value = {"config": {"max_dt": .05, "nodes": 15},
                     "memory": {"plan": [0., 1.], "previous_stamp": i * .02},
                     "input": {"dt": .02, "t": i * .02, "preview": [],
                               "context": {"commands": [[-.1, [1., 2.]]]}},
                     "oracle": {"command": [0.] * 5, "problems": ["keep original oracle locally"]},
                     "label": {"synthetic_state": True, "index": i}}
            if mixed_config and i == 47:
                value["config"]["max_dt"] = .06
            data = prepare.json_bytes(value)
            row = {"path": f"/original/case_{i:03d}.json", "bytes": len(data),
                   "sha256": prepare.sha256(data)}
            old = {"pass": True, "input_sha256": row["sha256"], "command": [0.] * 5,
                   "position_reference": [1., 2., 3.], "velocity_feedforward": [0.] * 3}
            if changed_input and i == 0:
                old["input_sha256"] = "0" * 64
            cases.append((data, prepare.json_bytes(old)))
            rows.append(row)
        manifest = {"complete": True, "source_files_unchanged": True, "synthetic_inputs": True,
                    "no_flight": True, "physical_acceptance": False, "rows": rows}
        manifest_raw = prepare.json_bytes(manifest)
        old_report = {"complete": True, "pass": True, "libraries_unchanged": True,
                      "cases": {"sha256": prepare.sha256(manifest_raw)}, "total": 48, "passed": 48,
                      "predictor": {}, "core_library": {}, "optimization_library": {}}
        with zipfile.ZipFile(paths["raw.zip"], "w") as archive:
            archive.writestr("software_and_audit/fast_audit_v1/report.json", certificate_raw)
            archive.writestr(prepare.RAW_PREFIX + "manifest.json", manifest_raw)
            archive.writestr(prepare.CHECK_PREFIX + "report.json", prepare.json_bytes(old_report))
            for i, (raw, result) in enumerate(cases):
                archive.writestr(prepare.RAW_PREFIX + f"case_{i:03d}.json", raw)
                archive.writestr(prepare.CHECK_PREFIX + f"case_{i:03d}.json", result)
        archive_raw = paths["raw.zip"].read_bytes()
        receipt = {"bytes": len(archive_raw), "sha256": prepare.sha256(archive_raw),
                   "physical_certificate_sha256": prepare.sha256(certificate_raw)}
        paths["receipt.json"].write_bytes(prepare.json_bytes(receipt))
        return paths, [prepare.strict_json(c[0]) for c in cases]

    def load(self, paths, **kwargs):
        return prepare.prepare_input(paths["raw.zip"], paths["receipt.json"], paths["report.json"],
                                     warmup_rounds=2, measured_rounds=20, cpu=-1, **kwargs)

    def test_original_values_and_oracle_provenance_are_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            paths, originals = self.fixture(Path(directory))
            result = self.load(paths)
            self.assertEqual(len(result["cases"]), 48)
            self.assertEqual(result["config"], originals[0]["config"])
            for old, new in zip(originals, result["cases"]):
                self.assertEqual(new["memory"], old["memory"])
                self.assertEqual(new["input"], old["input"])
                self.assertEqual(new["oracle"]["command"], old["oracle"]["command"])
            self.assertFalse(result["provenance"]["recorded_physical_state_corpus"])
            self.assertFalse(result["provenance"]["continuous_closed_loop_replay"])
            self.assertEqual(result["config"]["max_dt"], .05)

    def test_raw_archive_tampering_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            paths, _ = self.fixture(Path(directory))
            with paths["raw.zip"].open("ab") as stream:
                stream.write(b"changed")
            with self.assertRaisesRegex(ValueError, "frozen receipt"):
                self.load(paths)

    def test_historical_output_must_bind_exact_raw_case(self):
        with tempfile.TemporaryDirectory() as directory:
            paths, _ = self.fixture(Path(directory), changed_input=True)
            with self.assertRaisesRegex(ValueError, "not bound"):
                self.load(paths)

    def test_mixed_configurations_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            paths, _ = self.fixture(Path(directory), mixed_config=True)
            with self.assertRaisesRegex(ValueError, "Mixed configurations"):
                self.load(paths)

    def test_only_explicit_sources_are_packaged(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            for relative in prepare.SOURCE_PATHS:
                path = folder / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("source " + relative)
            (folder / "input.json").write_text("sensitive input")
            (folder / "result.log").write_text("diagnostics")
            output = folder / "code.tar.gz"
            record = prepare.package_sources(folder, output)
            with tarfile.open(output) as archive:
                self.assertEqual(set(archive.getnames()), set(prepare.SOURCE_PATHS))
                self.assertTrue(all(i.isfile() and not i.issym() for i in archive.getmembers()))
            self.assertFalse(record["includes_input_corpus"])
            self.assertFalse(record["includes_results_logs_meshes"])

    def test_ssh_command_uses_one_quoted_process_and_disables_core_dumps(self):
        options = SimpleNamespace(host="192.168.30.145", user="jetson", port=22, ssh="ssh",
            identity_file=None, binary="/home/jetson/code/bin/mpc;touch /tmp/bad",
            predictor="/home/jetson/code/lib/libts20.so", library_dir="/home/jetson/code/lib",
            predictor_sha256="a" * 64)
        command = run.make_command(options, "b" * 64)
        remote = command[-1]
        self.assertTrue(remote.startswith("ulimit -c 0 && exec env "))
        self.assertIn(options.binary, shlex.split(remote))
        self.assertNotIn(" > ", remote)
        self.assertNotIn("tee", remote)
        self.assertIn("jetson@192.168.30.145", command)
        options.host = "-oProxyCommand=bad"
        with self.assertRaises(ValueError):
            run.make_command(options, "b" * 64)

    def test_remote_paths_reject_control_characters_and_parent_segments(self):
        for value in ("relative/path", "/home/jetson/../input", "/tmp/x\nmalicious"):
            with self.assertRaises(ValueError):
                run.remote_path(value)

    def test_full_result_keeps_failures_and_binds_inputs_and_loaded_binaries(self):
        bundle, result, expected = self.result_fixture(failed=True)
        record = run.validate_result(result, bundle, "e" * 64, expected)
        self.assertTrue(record["complete"])
        self.assertFalse(record["all_calls_valid"])
        result["samples"].pop()
        with self.assertRaisesRegex(ValueError, "disappeared"):
            run.validate_result(result, bundle, "e" * 64, expected)

    def result_fixture(self, *, failed=False):
        cases = [{"id": name, "memory": {"step_count": 4},
                  "oracle": {"command": [0.] * 5, "position_reference": [1., 2., 3.],
                             "velocity_feedforward": [0.] * 3}} for name in ("a", "b")]
        bundle = {"cases": cases, "run": {"warmup_rounds": 1, "measured_rounds": 2,
                  "oracle_absolute_tolerance": 1e-7}, "provenance": {"synthetic_inputs": True}}
        expected = {name: c * 64 for name, c in
                    (("executable", "a"), ("core", "b"), ("optimization", "c"), ("predictor", "d"))}
        def sample(index, round_index):
            output = {"command": [0.] * 5, "position_reference": [1., 2., 3.],
                      "velocity_feedforward": [0.] * 3, "acceleration_feedforward": [0.] * 3,
                      "joint_velocity": [0.] * 2, "joint_reference": [0.] * 2,
                      "joint_velocity_reference": [0.] * 2, "joint_acceleration_reference": [0.] * 2,
                      "nominal_acceleration": [0.] * 3, "tip_actual": [0.] * 3}
            return {"case_index": index, "round": round_index, "core_step_wall_ns": 4_000_000,
                    "core_step_thread_cpu_ns": 3_000_000, "returned": not failed,
                    "core_error": failed, "unexpected_exception": False,
                    "memory_unchanged": failed, "memory_commit_valid": not failed,
                    "oracle_checked": not failed, "valid": not failed,
                    "production_deadline_error": False, "exceeded_20ms_target": False,
                    "exceeded_50ms_wall": False, "error": "intentional invalid input" if failed else None,
                    "oracle_match": None if failed else True,
                    "oracle_max_absolute_error": None if failed else 0.,
                    "step_count_after": 4 if failed else 5,
                    "output": None if failed else output,
                    "diagnostics": {"accepted": not failed, "nonlinear_passed": not failed,
                                    "actual_bounds": not failed, "compute_ms": 3.9}}
        def summary(samples):
            n = len(samples)
            def stats(value):
                return {"sample_count": n, "min_ns": value, "median_ns": value,
                        "p95_ns": value, "p99_ns": value, "max_ns": value, "mean_ns": float(value)}
            return {"sample_count": n, "returned_count": 0 if failed else n,
                    "valid_count": 0 if failed else n, "oracle_checked_count": 0 if failed else n,
                    "oracle_match_count": 0 if failed else n, "core_error_count": n if failed else 0,
                    "unexpected_exception_count": 0, "exceeded_20ms_target_count": 0,
                    "exceeded_50ms_wall_count": 0, "production_deadline_error_count": 0,
                    "failed_calls_with_unchanged_memory": n if failed else 0,
                    "all_calls_valid": not failed, "wall": stats(4_000_000), "thread_cpu": stats(3_000_000),
                    "statistics_include_all_failed_and_slow_calls": True, "percentile_method": "nearest_rank"}
        warmup = [sample(i, 0) for i in range(2)]
        measured = [sample(i % 2, i // 2) for i in range(4)]
        result = {"kind": "x650_mpc_memory_benchmark_result_v1", "schema_version": 1,
                  "complete": True, "backend": "cpu", "mode": "fixed_snapshot_paired",
                  "input_sha256": "e" * 64, "case_count": 2,
                  "cases": [{"id": c["id"]} for c in cases],
                  "run": bundle["run"], "provenance": bundle["provenance"],
                  "samples": measured, "warmup_samples": warmup, "cold_first_call": sample(0, 0),
                  "summary": {"warmup": summary(warmup), "measured": summary(measured)},
                  "all_calls_valid": not failed,
                  "scope": {"cuda_implemented": False, "hardware_closed_loop_claimed": False,
                            "local_control_stack_latency_measured": False},
                  "loaded_code": {key: {"sha256": digest} for key, digest in expected.items()}}
        return bundle, result, expected

    def test_raw_results_are_independently_recomputed(self):
        bundle, result, expected = self.result_fixture()
        self.assertTrue(run.validate_result(result, bundle, "e" * 64, expected)["all_calls_valid"])
        for field in ("all_calls_valid",):
            changed = copy.deepcopy(result)
            changed[field] = False
            with self.assertRaisesRegex(ValueError, "validity"):
                run.validate_result(changed, bundle, "e" * 64, expected)
        changed = copy.deepcopy(result)
        changed["samples"][0]["output"]["command"][0] = .1
        with self.assertRaisesRegex(ValueError, "oracle match"):
            run.validate_result(changed, bundle, "e" * 64, expected)
        changed = copy.deepcopy(result)
        changed["samples"][1]["case_index"] = 0
        with self.assertRaisesRegex(ValueError, "ordering"):
            run.validate_result(changed, bundle, "e" * 64, expected)
        changed = copy.deepcopy(result)
        changed["summary"]["measured"]["wall"]["p99_ns"] -= 1
        with self.assertRaisesRegex(ValueError, "percentile"):
            run.validate_result(changed, bundle, "e" * 64, expected)

    def test_cold_and_warmup_failures_cannot_be_hidden(self):
        bundle, result, expected = self.result_fixture()
        result["warmup_samples"].pop()
        with self.assertRaisesRegex(ValueError, "Warmup failures"):
            run.validate_result(result, bundle, "e" * 64, expected)
        bundle, result, expected = self.result_fixture()
        result["cold_first_call"]["valid"] = False
        with self.assertRaisesRegex(ValueError, "validity"):
            run.validate_result(result, bundle, "e" * 64, expected)

    def test_cpu_benchmark_cannot_claim_cuda_or_closed_loop(self):
        for field in ("cuda_implemented", "hardware_closed_loop_claimed", "local_control_stack_latency_measured"):
            bundle, result, expected = self.result_fixture()
            result["scope"][field] = True
            with self.assertRaisesRegex(ValueError, "may not claim"):
                run.validate_result(result, bundle, "e" * 64, expected)

    def test_atomic_local_outputs_cannot_overwrite_an_earlier_report(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "result.json"
            prepare.atomic_write(path, b"first")
            with self.assertRaises(FileExistsError):
                prepare.atomic_write(path, b"second")
            self.assertEqual(path.read_bytes(), b"first")


if __name__ == "__main__":
    unittest.main()
