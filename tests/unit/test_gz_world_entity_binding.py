"""Synthetic guard tests only; never connects to Gazebo or compiles C++."""
from contextlib import contextmanager
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import uuid

ROOT = Path(__file__).resolve().parents[2]
HERE = ROOT / "benchmark/whole_body_closed_loop_v1"


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


binding = module("test_gz_binding_module", HERE / "gz_world_entity_binding.py")
loader = module("test_gz_disturbance_loader", HERE / "disturbance_loader.py")


@contextmanager
def space():
    # Plain mkdir avoids Windows TemporaryDirectory ACL differences. Only these
    # flat synthetic files under a uniquely created workspace directory exist.
    path = ROOT / (".tmp_entity_binding_test_" + uuid.uuid4().hex)
    path.mkdir()
    try:
        yield path
    finally:
        for child in path.iterdir():
            child.unlink()
        path.rmdir()


def good_probe():
    return {"schema": binding.SCHEMA, "verified": True, "read_only": True,
            "api": "gz_sim8_SerializedStepMap_local_ECM", "world": "lab_world",
            "world_id": 37, "model": "uav_0", "model_id": 52, "model_parent_id": 37,
            "link": "base_link", "link_id": 53, "link_parent_id": 52,
            "service": "/world/lab_world/state", "sim_sec": 100, "sim_nsec": 4000000,
            "component_types": {"World": 101, "Name": 102, "Model": 103, "Link": 104, "ParentEntity": 105}}


class ProbeValidation(unittest.TestCase):
    def test_nondefault_world_id_is_accepted(self):
        self.assertEqual(binding.validate_probe(good_probe(), "lab_world", "uav_0")["world_id"], 37)

    def test_scene_only_wrong_parent_missing_marker_and_bad_ids_rejected(self):
        variants = []
        for key, value in [("api", "scene/info"), ("world", "default"),
                           ("model_parent_id", 1), ("link_parent_id", 37),
                           ("world_id", True), ("link_id", 52), ("sim_nsec", 10**9)]:
            item = good_probe(); item[key] = value; variants.append(item)
        item = good_probe(); item["component_types"].pop("World"); variants.append(item)
        for item in variants:
            with self.subTest(item=item), self.assertRaises(ValueError):
                binding.validate_probe(item, "lab_world", "uav_0")

    def fixture(self, out):
        identity = {"project_root": str(ROOT), "world": "lab_world", "model": "uav_0",
                    "runtime_meta_sha256": "b" * 64, "process": {"pid": 400, "start_ticks": 200, "boot_id": "test"}}
        executable = out / "probe_not_executable"
        executable.write_bytes(b"synthetic binary hash fixture, not a real probe")
        files = {"ecs_state.pb": b"synthetic: this is not actual ECS acceptance evidence",
                 "probe_stdout.json": json.dumps(good_probe()).encode(), "probe_stderr.log": b""}
        for name, content in files.items():
            (out / name).write_bytes(content)
        document = {"schema": binding.SCHEMA, "complete": True, "runtime": identity,
                    "probe": good_probe(), "captured_unix": 100.,
                    "probe_executable": str(executable), "probe_executable_sha256": binding.sha(executable),
                    "wrapper_source_sha256": binding.sha(binding.__file__),
                    "evidence": {name: binding.sha(out / name) for name in files}}
        path = out / "entity_binding.json"
        path.write_text(json.dumps(document))
        return path, identity, document

    def test_hash_bound_fresh_runtime_then_tamper_and_reused_pid_rejected(self):
        with space() as out:
            path, identity, document = self.fixture(out)
            with patch.object(binding, "runtime_identity", return_value=identity), patch.object(binding.time, "time", return_value=101.):
                self.assertEqual(binding.validate_binding_for_load(path, "uav_0")["world_id"], 37)
                (out / "ecs_state.pb").write_bytes(b"changed")
                with self.assertRaisesRegex(ValueError, "hash mismatch"):
                    binding.validate_binding_for_load(path, "uav_0")
            changed = deepcopy(identity); changed["process"]["start_ticks"] += 1
            with patch.object(binding, "runtime_identity", return_value=changed), patch.object(binding.time, "time", return_value=101.):
                with self.assertRaisesRegex(ValueError, "changed/different runtime"):
                    binding.validate_binding_for_load(path, "uav_0")

    def test_stale_and_future_capture_rejected(self):
        with space() as out:
            path, identity, document = self.fixture(out)
            for now in (99., 116.):
                with patch.object(binding, "runtime_identity", return_value=identity), patch.object(binding.time, "time", return_value=now):
                    with self.subTest(now=now), self.assertRaisesRegex(ValueError, "stale or from a future"):
                        binding.validate_binding_for_load(path, "uav_0")


class LoaderRequest(unittest.TestCase):
    class Completed:
        returncode = 0
        def poll(self): return 0

    def fake_call(self, calls):
        def invoke(command, **kwargs):
            calls.append(command)
            kwargs["stdout"].write("data: true\n")
            return self.Completed()
        return invoke

    def test_legacy_request_and_schedule_remain_compatible(self):
        with space() as out:
            calls = []
            with patch.object(loader.subprocess, "Popen", side_effect=self.fake_call(calls)):
                self.assertIsNone(loader.load_disturbance(out, "uav_0", [{"start_s": 1., "end_s": 1.3,
                    "force_world_n": [1., 0., 0.], "torque_world_nm": [0., 0., 0.]}], 3., lambda: None))
            self.assertIn("/world/default/entity/system/add", calls[0])
            self.assertIn("entity { id: 1 }", calls[0][-1])
            self.assertIn("event=1,1.3,1,0,0,0,0,0", (out / "disturbance_config.txt").read_text())

    def test_verified_request_uses_actual_id_and_does_not_claim_force_success(self):
        with space() as out:
            (out / "libdisturbance.so").write_bytes(b"synthetic library hash fixture")
            calls = []
            with patch.object(loader, "HERE", out), patch.object(loader, "_read_binding", return_value=good_probe()), \
                 patch.object(loader.subprocess, "Popen", side_effect=self.fake_call(calls)):
                result = loader.load_disturbance(out, "uav_0", [], 3., lambda: None, binding="synthetic.json")
            self.assertIn("/world/lab_world/entity/system/add", calls[0])
            self.assertIn("entity { id: 37 }", calls[0][-1])
            self.assertTrue(result["service_acknowledged"])
            self.assertFalse(result["force_experiment_complete"])

    def test_invalid_binding_rejected_before_service(self):
        with space() as out, patch.object(loader, "_read_binding", side_effect=ValueError("invalid ECS binding")), \
             patch.object(loader.subprocess, "Popen") as start:
            with self.assertRaisesRegex(ValueError, "invalid ECS"):
                loader.load_disturbance(out, "uav_0", [], 3., lambda: None, binding="bad.json")
            start.assert_not_called()


if __name__ == "__main__":
    unittest.main()
