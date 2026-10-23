"""Byte/metadata unit contracts only; typed ROS roundtrips are separate tests."""
import base64
import hashlib
import json
from pathlib import Path
import shutil
import sys
import unittest
from unittest.mock import patch
from uuid import uuid4
import zipfile

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from flyarm_control_recorder.core import RecorderCore, RecorderFailure

META = dict(schema_version=1, session_id=7, epoch=1, replay_only=True)
CONFIG = bytes(range(32))


class RecorderTests(unittest.TestCase):
    def setUp(self):
        parent = PACKAGE.parents[3]/"tmp"
        parent.mkdir(exist_ok=True)
        self.temp = parent/("ros_recorder_test_"+uuid4().hex)
        self.temp.mkdir()
        self.source = self.temp/"source.py"
        self.source.write_bytes(b"# exact synthetic source fixture\n")
        def cleanup():
            self.temp.resolve().relative_to(parent.resolve())
            shutil.rmtree(self.temp)
        self.addCleanup(cleanup)

    def recorder(self, capacity=8192):
        core = RecorderCore(self.temp/uuid4().hex, session_id=7, epoch=1,
            source_files={"source.py": self.source}, capacity=capacity)
        self.addCleanup(core.close)
        return core

    def record(self, core, stream=0, sequence=1, **changes):
        meta = {**META, "sequence": sequence, **changes}
        return core.record(stream, b"\x00\x01synthetic-cdr", configuration_sha256=CONFIG if stream < 2 else None, **meta)

    def test_durable_rows_closed_file_and_final_manifest_receipt(self):
        core = self.recorder()
        calls = []
        import os
        original = os.fsync
        def sync(fd):
            calls.append(core.total)
            return original(fd)
        with patch("flyarm_control_recorder.core.os.fsync", side_effect=sync):
            receipt = self.record(core)[0]
        self.assertEqual(calls, [0])  # No successful row count/receipt before fsync.
        for stream in (1, 2):
            self.record(core, stream)
        final = core.drain(**META, final_sequences=[1, 1, 1])[0]
        self.assertTrue(final.persisted and final.complete and not final.failed)
        self.assertEqual(final.stream, 255)
        self.assertEqual(final.sequence, 3)
        lines = core.path.read_bytes().splitlines(keepends=True)
        self.assertEqual(receipt.sha256, hashlib.sha256(lines[0]).digest())
        previous = "0"*64
        for line in lines:
            row = json.loads(line)
            self.assertEqual(row["previous_row_sha256"], previous)
            payload = base64.b64decode(row["cdr_base64"])
            self.assertEqual(payload, b"\x00\x01synthetic-cdr")
            self.assertEqual(row["cdr_sha256"], hashlib.sha256(payload).hexdigest())
            previous = hashlib.sha256(line).hexdigest()
        manifest_bytes = (core.output/"recording_manifest.json").read_bytes()
        self.assertEqual(final.sha256, hashlib.sha256(manifest_bytes).digest())
        manifest = json.loads(manifest_bytes)
        self.assertEqual(manifest["counts"], [1, 1, 1])
        self.assertEqual(manifest["records"]["sha256"], hashlib.sha256(core.path.read_bytes()).hexdigest())
        self.assertEqual(manifest["configuration_sha256"], CONFIG.hex())
        with zipfile.ZipFile(core.output/"sources.zip") as archive:
            self.assertEqual(archive.read("source.py"), self.source.read_bytes())

    def test_cross_topic_arrival_and_pending_drain_do_not_block_or_finish_early(self):
        core = self.recorder()
        self.assertEqual(core.drain(**META, final_sequences=[1, 1, 1]), [])
        self.assertFalse(core.closed)
        self.assertEqual(len(self.record(core, 1)), 1)  # proposal can arrive before snapshot
        self.assertEqual(len(self.record(core, 2)), 1)
        self.assertFalse((core.output/"recording_manifest.json").exists())
        receipts = self.record(core, 0)
        self.assertEqual(len(receipts), 2)
        self.assertTrue(receipts[-1].complete)

    def test_missing_duplicate_and_reordered_sequence_fail_closed(self):
        for previous, incoming in (([], 2), ([1], 1), ([1, 2], 1)):
            core = self.recorder()
            for sequence in previous:
                self.record(core, sequence=sequence)
            with self.subTest(previous=previous), self.assertRaises(RecorderFailure) as raised:
                self.record(core, sequence=incoming)
            self.assertTrue(raised.exception.receipt.failed)
            self.assertFalse(raised.exception.receipt.persisted)
            self.assertTrue(core.closed)
            self.assertFalse(core.manifest["complete"])

    def test_metadata_configuration_and_capacity_rejections(self):
        for changes in ({"schema_version": 2}, {"session_id": 8}, {"epoch": 2}, {"replay_only": False}):
            core = self.recorder()
            with self.subTest(changes=changes), self.assertRaises(RecorderFailure):
                self.record(core, **changes)
        core = self.recorder(capacity=2)
        self.record(core, 0)
        self.record(core, 1)
        with self.assertRaisesRegex(RecorderFailure, "capacity"):
            self.record(core, 2)
        core = self.recorder()
        self.record(core, 1)
        with self.assertRaisesRegex(RecorderFailure, "configuration"):
            core.record(0, b"cdr", **META, sequence=1, configuration_sha256=bytes(32))

    def test_missing_records_drain_timeout_and_wrong_final_counts_fail(self):
        core = self.recorder()
        self.record(core)
        self.assertEqual(core.drain(**META, final_sequences=[1, 1, 1]), [])
        with self.assertRaisesRegex(RecorderFailure, "missing"):
            core.expire_drain()
        self.assertFalse(core.manifest["complete"])
        core = self.recorder()
        self.record(core)
        with self.assertRaisesRegex(RecorderFailure, "below observed"):
            core.drain(**META, final_sequences=[0, 0, 0])

    def test_write_fsync_failure_never_returns_a_success_receipt(self):
        core = self.recorder()
        with patch("flyarm_control_recorder.core.os.fsync", side_effect=OSError("disk sync failure")):
            with self.assertRaisesRegex(RecorderFailure, "disk sync failure") as raised:
                self.record(core)
        self.assertEqual(core.total, 0)
        self.assertFalse(raised.exception.receipt.persisted)
        self.assertTrue(core.failed and core.closed)
        self.assertIsNotNone(core.failure_manifest_error)

    def test_source_or_data_tamper_cannot_receive_complete_final_receipt(self):
        for what in ("source", "data"):
            core = self.recorder()
            self.record(core)
            if what == "source":
                self.source.write_bytes(b"# changed after initialization\n")
            else:
                data = core.path.read_bytes()
                core.path.write_bytes(b"X"+data[1:])
            receipt = core.drain(**META, final_sequences=[1, 0, 0])[0]
            self.assertTrue(receipt.failed)
            self.assertFalse(receipt.complete)

    def test_no_overwrite_and_fixed_capacity_bound(self):
        core = self.recorder()
        sentinel = core.path.read_bytes()
        with self.assertRaises(FileExistsError):
            RecorderCore(core.output, session_id=7, epoch=1, source_files={"source.py": self.source})
        self.assertEqual(core.path.read_bytes(), sentinel)
        for capacity in (0, 8193, True):
            with self.subTest(capacity=capacity), self.assertRaises(ValueError):
                self.recorder(capacity)

    def test_zero_identity_and_empty_snapshot_drain_cannot_complete(self):
        for session_id, epoch in ((0, 1), (7, 0)):
            output = self.temp/uuid4().hex
            with self.subTest(session_id=session_id, epoch=epoch), self.assertRaisesRegex(ValueError, "positive"):
                RecorderCore(output, session_id=session_id, epoch=epoch, source_files={"source.py": self.source})
            self.assertFalse(output.exists())
        core = self.recorder()
        with self.assertRaisesRegex(RecorderFailure, "at least one snapshot") as raised:
            core.drain(**META, final_sequences=[0, 0, 0])
        self.assertFalse(raised.exception.receipt.complete)
        self.assertTrue(core.manifest["failed"])
        self.assertFalse(core.manifest["complete"])


if __name__ == "__main__":
    unittest.main()
