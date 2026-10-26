#!/usr/bin/env python3
"""Bind original CDR records for C++ prefix reconstruction; never invent memory.

The original run is read only. This adapter deserializes ROS wire data and
records source identities. Controller state is reconstructed separately by the
original C++ Core from LiveStart and the complete recorded snapshot prefix.
"""
from __future__ import annotations

import argparse
import array
import hashlib
import json
from pathlib import Path


def plain(value):
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, array.array, bytes, bytearray)):
        return [plain(v) for v in value]
    if hasattr(value, "tolist"):
        return plain(value.tolist())
    if hasattr(value, "item"):
        return plain(value.item())
    return value


def identity(path):
    path = Path(path).resolve(strict=True)
    data = path.read_bytes()
    return {"path": str(path), "bytes": len(data),
            "sha256": hashlib.sha256(data).hexdigest()}


def write_new(path, value):
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(plain(value), stream, allow_nan=False, separators=(",", ":"))
        stream.write("\n")


def export(run, output, last_sequence):
    from flyarm_control_interfaces.msg import LiveStart, ControlSnapshot, ControlProposal
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict

    run = Path(run).resolve(strict=True)
    source = run / "isolated_live"
    output = Path(output).resolve()
    if output.exists():
        raise ValueError("Evidence output must be a new directory")
    originals = []

    def bound(path, expected=None):
        receipt = identity(path)
        if expected is not None:
            for field in ("bytes", "sha256"):
                if receipt[field] != expected[field]:
                    raise ValueError(f"Original {field} differs: {path}")
        originals.append(receipt)
        return receipt

    startup_id = bound(source / "worker_startup.json")
    startup = json.loads(Path(startup_id["path"]).read_text())
    if startup["core_abi"] != 1:
        raise ValueError("Original ABI 1 recording required")
    manifest = startup["startup_manifest"]
    start_id = bound(source / "worker_live_start.cdr")
    start = plain(message_to_ordereddict(deserialize_message(
        Path(start_id["path"]).read_bytes(), LiveStart)))
    json_start_id = bound(source / "live_start.json")
    json_start = json.loads(Path(json_start_id["path"]).read_text())
    for field, value in start.items():
        if field not in json_start:
            continue
        saved = json_start[field]
        if field == "configuration_sha256" and isinstance(saved, str):
            saved = list(bytes.fromhex(saved))
        if saved != value:
            raise ValueError(f"CDR/JSON LiveStart differs: {field}")
    if start["session_id"] != startup["session_id"] or start["epoch"] != startup["epoch"]:
        raise ValueError("Original LiveStart/startup binding differs")
    if bytes(start["configuration_sha256"]).hex() != startup["configuration_sha256"]:
        raise ValueError("Original configuration binding differs")
    paths = sorted((source / "native_records").glob("[0-9]*.json"))
    if last_sequence:
        paths = paths[:last_sequence]
    if not paths or (last_sequence and len(paths) != last_sequence):
        raise ValueError("Incomplete original prefix")
    output.mkdir(parents=True)
    rows = []
    for sequence, path in enumerate(paths, 1):
        record_id = bound(path)
        record = json.loads(path.read_text())
        if record["sequence"] != sequence or path.stem != f"{sequence:06d}":
            raise ValueError("Original sequence missing or out of order")
        decoded = {}
        for kind, cls in (("snapshot", ControlSnapshot), ("proposal", ControlProposal)):
            saved = record[kind + "_cdr"]
            receipt = bound(saved["path"], saved)
            decoded[kind + "_identity"] = receipt
            decoded[kind] = plain(message_to_ordereddict(deserialize_message(
                Path(receipt["path"]).read_bytes(), cls)))
            value = decoded[kind]
            for field in ("session_id", "epoch", "configuration_sha256"):
                if value[field] != start[field]:
                    raise ValueError(f"Original {kind}/{field} differs")
            if value["sequence"] != sequence:
                raise ValueError("Original CDR sequence differs")
        row_path = output / f"{sequence:06d}.json"
        write_new(row_path, dict(sequence=sequence, record_identity=record_id, **decoded))
        rows.append(dict(sequence=sequence, **identity(row_path)))
    for receipt in originals:
        if identity(receipt["path"]) != receipt:
            raise ValueError("Original source changed while exporting")
    result = dict(kind="x650_original_cpp_recording_export_v1", schema_version=1,
                  config=manifest["config"], trajectory=manifest["trajectory"],
                  live_start=start, rows=rows, startup_identity=startup_id,
                  live_start_cdr_identity=start_id, live_start_json_identity=json_start_id,
                  exporter_identity=identity(__file__), originals_verified_unchanged=True,
                  controller_memory_reconstructed=False,
                  physical_validation_claimed=False)
    write_new(output / "recording.json", result)
    return dict(row_count=len(rows), recording=identity(output / "recording.json"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--last-sequence", type=int, default=379)
    args = parser.parse_args()
    print(json.dumps(export(args.run, args.output, args.last_sequence), indent=2))
