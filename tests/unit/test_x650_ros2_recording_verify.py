"""Independent synthetic CDR-decode fixtures; no ROS or recorder implementation import."""
import base64
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
from types import SimpleNamespace
import unittest
from uuid import uuid4
import zipfile

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("recording_verify", ROOT/"scripts/verify_x650_ros2_recording.py")
VERIFY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VERIFY)
CONFIG = bytes(range(32))


def encoded(value):
    return (json.dumps(value, separators=(",", ":"))+"\n").encode()


def sha(data):
    return hashlib.sha256(data).hexdigest()


class VerifierTests(unittest.TestCase):
    def setUp(self):
        parent = ROOT/"tmp"
        parent.mkdir(exist_ok=True)
        self.folder = parent/("ros_verify_test_"+uuid4().hex)
        self.folder.mkdir()
        def cleanup():
            self.folder.resolve().relative_to(parent.resolve())
            shutil.rmtree(self.folder)
        self.addCleanup(cleanup)
        self.common = dict(schema_version=1, session_id=77, epoch=1, replay_only=True)
        self.expected, self.rows = {}, []
        previous = "0"*64
        for sequence in (1, 2):
            for stream in (0, 1, 2):
                message = {**self.common, "sequence": sequence, "values": [sequence+.125, -.5]}
                if stream < 2:
                    message["configuration_sha256"] = list(CONFIG)
                payload = encoded(message)
                self.expected[stream, sequence] = SimpleNamespace(**message)
                row = {**self.common, "stream": stream, "sequence": sequence,
                       "message_type": VERIFY.TYPES[stream], "record_sequence": len(self.rows)+1,
                       "received_monotonic_ns": 100+len(self.rows), "previous_row_sha256": previous,
                       "cdr_base64": base64.b64encode(payload).decode(), "cdr_sha256": sha(payload)}
                self.rows.append(row)
                previous = sha(encoded(row))
        source = b"# archived actual source\n"
        with zipfile.ZipFile(self.folder/"sources.zip", "x") as archive:
            archive.writestr("package/source.py", source)
        sources = {"archive_sha256": sha((self.folder/"sources.zip").read_bytes()),
                   "files": {"package/source.py": {"path": "/no/live/path.py", "bytes": len(source), "sha256": sha(source)}}}
        session = {**self.common, "capacity": 8192, "max_cdr_bytes": 262144, "streams": list(VERIFY.TYPES)}
        self.manifest = {**self.common, "kind": "flyarm_offline_cdr_recording_v1", "complete": True,
                         "failed": False, "failure_reason": None, "counts": [2, 2, 2], "final_sequences": [2, 2, 2],
                         "record_count": 6, "records_match_persisted_receipts": True, "sources_unchanged": True,
                         "configuration_sha256": CONFIG.hex()}
        for key, name, data in (("source_manifest", "sources.json", sources), ("session_manifest", "session.json", session)):
            raw = encoded(data)
            (self.folder/name).write_bytes(raw)
            self.manifest[key] = dict(file=name, bytes=len(raw), sha256=sha(raw))
        self.write_rows()

    def write_rows(self):
        previous, raw = "0"*64, b""
        for row in self.rows:
            row["previous_row_sha256"] = previous
            line = encoded(row)
            raw += line
            previous = sha(line)
        (self.folder/"records.jsonl").write_bytes(raw)
        self.manifest.update(records=dict(file="records.jsonl", bytes=len(raw), sha256=sha(raw)), last_row_sha256=previous)
        self.write_manifest()

    def write_manifest(self):
        (self.folder/"recording_manifest.json").write_bytes(encoded(self.manifest))

    @staticmethod
    def decode(payload, stream):
        return SimpleNamespace(**json.loads(payload))

    def test_positive_byte_hash_and_every_decoded_field_and_receipt_identity(self):
        result = VERIFY.verify(self.folder, self.decode, self.expected)
        self.assertTrue(result["passed"] and result["decoded_verified"] and result["expected_messages_verified"])
        self.assertEqual(result["counts"], [2, 2, 2])
        self.assertEqual(result["final_receipt"], dict(stream=255, sequence=6,
            sha256=sha((self.folder/"recording_manifest.json").read_bytes())))
        lines = (self.folder/"records.jsonl").read_bytes().splitlines(keepends=True)
        self.assertEqual([row["sha256"] for row in result["row_receipts"]], [sha(line) for line in lines])
        self.assertFalse(VERIFY.verify(self.folder)["decoded_verified"])
        self.assertTrue(VERIFY._equal(SimpleNamespace(values=(1., 2.)), SimpleNamespace(values=[1., 2.])))

    def test_numpy_sequences_compare_every_numeric_element_without_ambiguous_truth(self):
        import numpy as np
        expected = copy.deepcopy(self.expected)
        expected[0, 1].values = np.array(expected[0, 1].values, dtype=np.float64)
        self.assertTrue(VERIFY.verify(self.folder, self.decode, expected)["passed"])
        expected[0, 1].values[1] += .0000001
        with self.assertRaisesRegex(ValueError, "fields differ"):
            VERIFY.verify(self.folder, self.decode, expected)

    def test_file_hash_mismatch_and_missing_or_unterminated_line_reject(self):
        original = (self.folder/"records.jsonl").read_bytes()
        (self.folder/"records.jsonl").write_bytes(original[:-1])
        with self.assertRaisesRegex(ValueError, "hash/size"):
            VERIFY.verify(self.folder)
        self.manifest["records"].update(bytes=len(original)-1, sha256=sha(original[:-1]))
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "truncated"):
            VERIFY.verify(self.folder)
        self.rows.pop()
        self.write_rows()
        with self.assertRaisesRegex(ValueError, "Count"):
            VERIFY.verify(self.folder)

    def test_payload_hash_decoder_failure_and_decoded_identity_reject(self):
        self.rows[0]["cdr_sha256"] = "0"*64
        self.write_rows()
        with self.assertRaisesRegex(ValueError, "CDR hash"):
            VERIFY.verify(self.folder)
        self.rows[0]["cdr_sha256"] = sha(base64.b64decode(self.rows[0]["cdr_base64"]))
        self.write_rows()
        def truncated(payload, stream):
            return self.decode(payload[:-4], stream)
        with self.assertRaisesRegex(ValueError, "decode failed"):
            VERIFY.verify(self.folder, truncated)
        def wrong_session(payload, stream):
            return SimpleNamespace(**{**json.loads(payload), "session_id": 78})
        with self.assertRaisesRegex(ValueError, "identity differs"):
            VERIFY.verify(self.folder, wrong_session)

    def test_expected_field_change_extra_missing_and_config_reject(self):
        expected = copy.deepcopy(self.expected)
        expected[1, 1].values[0] += .000001
        with self.assertRaisesRegex(ValueError, "fields differ"):
            VERIFY.verify(self.folder, self.decode, expected)
        for key in ((2, 2), (2, 3)):
            expected = dict(self.expected)
            if key in expected:
                expected.pop(key)
            else:
                expected[key] = expected[2, 2]
            with self.assertRaises(ValueError):
                VERIFY.verify(self.folder, self.decode, expected)
        self.manifest["configuration_sha256"] = "0"*64
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "configuration"):
            VERIFY.verify(self.folder, self.decode)

    def test_rehashed_sequence_metadata_type_and_final_count_reject(self):
        original = copy.deepcopy(self.rows)
        for change in ({"sequence": 1}, {"session_id": 78}, {"stream": 1}, {"replay_only": 1}):
            self.rows = copy.deepcopy(original)
            self.rows[-1].update(change)
            self.write_rows()
            with self.subTest(change=change), self.assertRaises(ValueError):
                VERIFY.verify(self.folder)
        self.rows = original
        self.write_rows()
        self.manifest["final_sequences"] = [2, 2, 3]
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "Count"):
            VERIFY.verify(self.folder)

    def test_row_chain_sources_session_binding_and_incomplete_reject(self):
        path = self.folder/"records.jsonl"
        raw = path.read_bytes().replace(b'"previous_row_sha256":"0', b'"previous_row_sha256":"1', 1)
        path.write_bytes(raw)
        self.manifest["records"].update(bytes=len(raw), sha256=sha(raw))
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "chain"):
            VERIFY.verify(self.folder)
        self.write_rows()
        (self.folder/"sources.zip").write_bytes(b"changed archive")
        with self.assertRaisesRegex(ValueError, "archive hash"):
            VERIFY.verify(self.folder)
        self.manifest["complete"] = False
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "not complete"):
            VERIFY.verify(self.folder)

    def test_rehashed_session_and_source_archive_are_still_cross_bound(self):
        session_path = self.folder/"session.json"
        original_session = session_path.read_bytes()
        session = json.loads(original_session)
        session["session_id"] += 1
        changed = encoded(session)
        session_path.write_bytes(changed)
        self.manifest["session_manifest"].update(bytes=len(changed), sha256=sha(changed))
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "Session identity"):
            VERIFY.verify(self.folder)
        session_path.write_bytes(original_session)
        self.manifest["session_manifest"].update(bytes=len(original_session), sha256=sha(original_session))
        with zipfile.ZipFile(self.folder/"sources.zip", "w") as archive:
            archive.writestr("package/source.py", b"# altered source\n")
        sources_path = self.folder/"sources.json"
        sources = json.loads(sources_path.read_bytes())
        sources["archive_sha256"] = sha((self.folder/"sources.zip").read_bytes())
        changed = encoded(sources)
        sources_path.write_bytes(changed)
        self.manifest["source_manifest"].update(bytes=len(changed), sha256=sha(changed))
        self.write_manifest()
        with self.assertRaisesRegex(ValueError, "Archived source"):
            VERIFY.verify(self.folder)


if __name__ == "__main__":
    unittest.main()
