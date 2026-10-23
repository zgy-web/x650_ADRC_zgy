"""ROS-independent bounded recorder. Receipts mean fsync, never execution ACKs."""
from dataclasses import dataclass
import base64
import hashlib
import json
import os
from pathlib import Path
import time
import zipfile

SCHEMA_VERSION = 1
MAX_CAPACITY = 8192
MAX_CDR_BYTES = 262144
STREAM_TYPES = ("flyarm_control_interfaces/msg/ControlSnapshot",
                "flyarm_control_interfaces/msg/ControlProposal",
                "flyarm_control_interfaces/msg/GateEvent")


def _uint(value, name, maximum=(1 << 64)-1):
    if type(value) is not int or not 0 <= value <= maximum:
        raise ValueError(name+" must be an unsigned integer")
    return value


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _json_bytes(value):
    return (json.dumps(value, allow_nan=False, separators=(",", ":"))+"\n").encode("utf-8")


def _write_durable(path, value):
    data = _json_bytes(value)
    with Path(path).open("xb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    return {"file": Path(path).name, "sha256": _sha(data), "bytes": len(data)}


@dataclass(frozen=True)
class Receipt:
    stream: int
    sequence: int
    persisted: bool
    complete: bool
    failed: bool
    sha256: bytes


class RecorderFailure(RuntimeError):
    def __init__(self, reason, receipt):
        super().__init__(reason)
        self.receipt = receipt


class RecorderCore:
    def __init__(self, output_dir, *, session_id, epoch, source_files, capacity=8192, session_metadata=None):
        self.session_id, self.epoch = _uint(session_id, "session_id"), _uint(epoch, "epoch")
        if not self.session_id or not self.epoch:
            raise ValueError("session_id and epoch must be positive")
        if type(capacity) is not int or not 1 <= capacity <= MAX_CAPACITY:
            raise ValueError("capacity must be 1..8192 total rows")
        if not isinstance(source_files, dict) or not source_files:
            raise ValueError("Actual package and typed-interface source files are required")
        sources = {}
        for name, path in source_files.items():
            member = Path(name)
            if not isinstance(name, str) or member.is_absolute() or ".." in member.parts:
                raise ValueError("Source archive members must be relative")
            path = Path(path).resolve(strict=True)
            data = path.read_bytes()
            sources[name] = {"path": str(path), "sha256": _sha(data), "bytes": len(data)}
        self.output = Path(output_dir).absolute()
        self.output.mkdir(parents=True, exist_ok=False)
        self.capacity, self.sources = capacity, sources
        with zipfile.ZipFile(self.output/"sources.zip", "x", zipfile.ZIP_DEFLATED) as archive:
            for name, source in sources.items():
                data = Path(source["path"]).read_bytes()
                if _sha(data) != source["sha256"]:
                    raise RuntimeError("Source changed during recorder initialization")
                archive.writestr(name, data)
        self.source_manifest = _write_durable(self.output/"sources.json", {"files": sources,
            "archive_sha256": _sha((self.output/"sources.zip").read_bytes())})
        self.session_manifest = _write_durable(self.output/"session.json", {"schema_version": 1,
            "session_id": session_id, "epoch": epoch, "replay_only": True, "capacity": capacity,
            "max_cdr_bytes": MAX_CDR_BYTES, "streams": list(STREAM_TYPES),
            "metadata": session_metadata or {}, "receipt_scope": "persisted bytes only; no control execution acknowledgement"})
        self.path = self.output/"records.jsonl"
        self.stream = self.path.open("xb")
        self.counts, self.total = [0, 0, 0], 0
        self.pending_final = None
        self.configuration_sha256 = self.proposal_configuration = None
        self.failed, self.closed, self.failure_reason = False, False, None
        self.manifest, self.manifest_identity, self.failure_manifest_error = None, None, None
        self.running_hash = hashlib.sha256()
        self.previous_row_hash = "0"*64

    def _metadata(self, schema_version, session_id, epoch, replay_only):
        if (type(schema_version) is not int or schema_version != SCHEMA_VERSION
                or _uint(session_id, "session_id") != self.session_id
                or _uint(epoch, "epoch") != self.epoch or replay_only is not True):
            raise ValueError("Schema/session/epoch/replay-only metadata differs")

    def _seal(self, complete):
        if self.closed:
            raise RuntimeError("Recorder already sealed")
        self.stream.flush()
        os.fsync(self.stream.fileno())
        self.stream.close()
        self.closed = True
        data = self.path.read_bytes()
        integrity = _sha(data) == self.running_hash.hexdigest()
        sources_unchanged = all(_sha(Path(item["path"]).read_bytes()) == item["sha256"]
                                for item in self.sources.values())
        if complete and (not integrity or not sources_unchanged):
            self.failed, self.failure_reason = True, "Persisted bytes or source files changed"
            complete = False
        self.manifest = {"kind": "flyarm_offline_cdr_recording_v1", "schema_version": 1,
            "session_id": self.session_id, "epoch": self.epoch, "replay_only": True,
            "complete": bool(complete and not self.failed), "failed": self.failed,
            "failure_reason": self.failure_reason, "counts": list(self.counts), "record_count": self.total,
            "final_sequences": None if self.pending_final is None else list(self.pending_final),
            "records": {"file": self.path.name, "sha256": _sha(data), "bytes": len(data)},
            "records_match_persisted_receipts": integrity, "last_row_sha256": self.previous_row_hash,
            "source_manifest": self.source_manifest, "session_manifest": self.session_manifest,
            "sources_unchanged": sources_unchanged,
            "configuration_sha256": None if self.configuration_sha256 is None else self.configuration_sha256.hex(),
            "configuration_source": "first persisted snapshot, cross-checked with every proposal",
            "final_receipt_scope": "closed/fsynced file and fsynced manifest; never an actuator ACK"}
        self.manifest_identity = _write_durable(self.output/"recording_manifest.json", self.manifest)
        return Receipt(255, self.total, True, self.manifest["complete"], self.failed,
                       bytes.fromhex(self.manifest_identity["sha256"]))

    def _fail(self, reason, stream=255, sequence=0):
        self.failed, self.failure_reason = True, str(reason)
        if not self.closed:
            try:
                self._seal(False)
            except Exception as error:
                self.failure_manifest_error = repr(error)
                try:
                    self.stream.close()
                finally:
                    self.closed = True
        safe_sequence = sequence if type(sequence) is int and 0 <= sequence < 1 << 64 else 0
        raise RecorderFailure(str(reason), Receipt(stream, safe_sequence, False, False, True, bytes(32)))

    def record(self, stream, payload, *, schema_version, session_id, epoch, sequence, replay_only,
               configuration_sha256=None):
        try:
            if self.closed or self.failed:
                raise ValueError("Recorder no longer accepts records")
            self._metadata(schema_version, session_id, epoch, replay_only)
            if type(stream) is not int or stream not in (0, 1, 2):
                raise ValueError("Unknown recorder stream")
            _uint(sequence, "sequence")
            if sequence != self.counts[stream]+1:
                raise ValueError("Missing, reordered or duplicate stream sequence")
            if self.pending_final is not None and sequence > self.pending_final[stream]:
                raise ValueError("Record exceeds declared drain sequence")
            if self.total >= self.capacity:
                raise ValueError("Recorder capacity exceeded")
            if not isinstance(payload, bytes) or not 0 < len(payload) <= MAX_CDR_BYTES:
                raise ValueError("CDR payload is empty or exceeds fixed bound")
            if stream in (0, 1):
                if not isinstance(configuration_sha256, bytes) or len(configuration_sha256) != 32:
                    raise ValueError("Snapshot/proposal configuration digest must contain 32 bytes")
                expected = self.configuration_sha256 or self.proposal_configuration
                if expected is not None and expected != configuration_sha256:
                    raise ValueError("Snapshot/proposal configuration digest changed")
                if stream == 0:
                    self.configuration_sha256 = configuration_sha256
                else:
                    self.proposal_configuration = configuration_sha256
            row = {"schema_version": 1, "session_id": self.session_id, "epoch": self.epoch,
                "stream": stream, "message_type": STREAM_TYPES[stream], "sequence": sequence,
                "record_sequence": self.total+1, "replay_only": True,
                "received_monotonic_ns": time.monotonic_ns(), "previous_row_sha256": self.previous_row_hash,
                "cdr_sha256": _sha(payload), "cdr_base64": base64.b64encode(payload).decode("ascii")}
            data = _json_bytes(row)
            self.stream.write(data)
            self.stream.flush()
            os.fsync(self.stream.fileno())
            self.running_hash.update(data)
            self.previous_row_hash = _sha(data)
            self.counts[stream], self.total = sequence, self.total+1
            receipts = [Receipt(stream, sequence, True, False, False, bytes.fromhex(self.previous_row_hash))]
            if self.pending_final is not None and tuple(self.counts) == self.pending_final:
                receipts.append(self._complete())
            return receipts
        except RecorderFailure:
            raise
        except Exception as error:
            self._fail(error, stream if type(stream) is int and stream in (0, 1, 2) else 255, sequence)

    def _complete(self):
        if self.counts[0] < 1 or self.configuration_sha256 is None:
            self._fail("Drain completed without a snapshot configuration source")
        return self._seal(True)

    def drain(self, *, schema_version, session_id, epoch, replay_only, final_sequences):
        try:
            if self.closed or self.failed or self.pending_final is not None:
                raise ValueError("Duplicate drain or closed recorder")
            self._metadata(schema_version, session_id, epoch, replay_only)
            if len(final_sequences) != 3:
                raise ValueError("Drain needs three final stream sequences")
            final = tuple(_uint(value, "final sequence") for value in final_sequences)
            if sum(final) > self.capacity or any(want < got for want, got in zip(final, self.counts)):
                raise ValueError("Drain count is below observed records or exceeds capacity")
            if final[0] < 1:
                raise ValueError("Drain requires at least one snapshot")
            self.pending_final = final
            return [self._complete()] if tuple(self.counts) == final else []
        except RecorderFailure:
            raise
        except Exception as error:
            self._fail(error)

    def expire_drain(self):
        if self.pending_final is not None and not self.closed:
            self._fail("Drain expired with missing stream records")

    def close(self):
        if not self.closed:
            try:
                self._fail("Recorder stopped without a complete drain")
            except RecorderFailure:
                pass
