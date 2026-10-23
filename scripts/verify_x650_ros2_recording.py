"""Independent read-only CDR recording verifier; DDS receipts must be matched by caller."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import zipfile

TYPES = tuple("flyarm_control_interfaces/msg/"+name for name in
              ("ControlSnapshot", "ControlProposal", "GateEvent"))


def _require(ok, reason):
    if not ok:
        raise ValueError(reason)


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _json(data):
    def pairs(items):
        result = {}
        for key, value in items:
            _require(key not in result, "Duplicate JSON key")
            result[key] = value
        return result
    return json.loads(data, object_pairs_hook=pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError("Nonfinite JSON")))


def _fields(value):
    if hasattr(value, "get_fields_and_field_types"):
        return {key: getattr(value, key) for key in value.get_fields_and_field_types()}
    return vars(value) if hasattr(value, "__dict__") else None


def _equal(actual, expected):
    left, right = _fields(actual), _fields(expected)
    if left is not None or right is not None:
        return left is not None and right is not None and left.keys() == right.keys() and all(
            _equal(left[key], right[key]) for key in left)
    if isinstance(actual, dict) or isinstance(expected, dict):
        return isinstance(actual, dict) and isinstance(expected, dict) and actual.keys() == expected.keys() and all(
            _equal(actual[key], expected[key]) for key in actual)
    if not isinstance(actual, (str, bytes)) and hasattr(actual, "__iter__"):
        if isinstance(expected, (str, bytes)) or not hasattr(expected, "__iter__"):
            return False
        left, right = list(actual), list(expected)
        return len(left) == len(right) and all(_equal(a, b) for a, b in zip(left, right))
    try:
        return bool(actual == expected)
    except (TypeError, ValueError):
        return False


def verify(folder, deserialize=None, expected_messages=None):
    """Raise ValueError on rejection. Decoder(payload, stream) must bind the three types."""
    folder = Path(folder)
    raw_manifest = (folder/"recording_manifest.json").read_bytes()
    manifest = _json(raw_manifest)
    _require(manifest["kind"] == "flyarm_offline_cdr_recording_v1", "Wrong recording kind")
    _require(manifest["complete"] is True and manifest["failed"] is False
             and manifest["failure_reason"] is None, "Recording is not complete")
    common = {key: manifest[key] for key in ("schema_version", "session_id", "epoch", "replay_only")}
    _require(type(common["schema_version"]) is int and common["schema_version"] == 1 and common["replay_only"] is True
             and all(type(common[key]) is int and 0 < common[key] < 1 << 64
                     for key in ("session_id", "epoch")), "Invalid recording metadata")
    def bound(identity, name):
        _require(identity["file"] == name, "Unexpected bound filename")
        data = (folder/name).read_bytes()
        _require(identity["sha256"] == _sha(data) and identity["bytes"] == len(data), name+" hash/size differs")
        return data
    sources_raw = bound(manifest["source_manifest"], "sources.json")
    session_raw = bound(manifest["session_manifest"], "session.json")
    sources, session = _json(sources_raw), _json(session_raw)
    def identity(value):
        return all(type(value.get(key)) is type(item) and value.get(key) == item for key, item in common.items())
    _require(identity(session), "Session identity differs")
    _require(session["streams"] == list(TYPES) and type(session["capacity"]) is int
             and 1 <= session["capacity"] <= 8192 and type(session["max_cdr_bytes"]) is int
             and 0 < session["max_cdr_bytes"] <= 262144,
             "Session bounds/types differ")
    archive_raw = (folder/"sources.zip").read_bytes()
    _require(_sha(archive_raw) == sources["archive_sha256"], "Source archive hash differs")
    with zipfile.ZipFile(folder/"sources.zip") as archive:
        names = archive.namelist()
        _require(bool(names) and len(names) == len(set(names)) and set(names) == set(sources["files"]),
                 "Source archive members differ/overlap")
        for name in names:
            _require(not Path(name).is_absolute() and ".." not in Path(name).parts, "Unsafe source member")
            data, item = archive.read(name), sources["files"][name]
            _require(_sha(data) == item["sha256"] and len(data) == item["bytes"], "Archived source differs")
    raw = bound(manifest["records"], "records.jsonl")
    lines = raw.splitlines(keepends=True)
    _require(bool(lines) and all(line.endswith(b"\n") for line in lines), "Missing/truncated JSONL line")
    counts, previous, receipts, keys, cdr_bytes = [0, 0, 0], "0"*64, [], set(), 0
    _require(expected_messages is None or deserialize is not None, "Expected messages require decoder")
    for number, line in enumerate(lines, 1):
        row = _json(line)
        stream, sequence = row["stream"], row["sequence"]
        _require(type(stream) is int and stream in (0, 1, 2), "Invalid stream")
        _require(type(sequence) is int and sequence == counts[stream]+1, "Missing/duplicate/reordered sequence")
        _require(identity(row) and type(row["record_sequence"]) is int
                 and row["record_sequence"] == number and row["message_type"] == TYPES[stream], "Row metadata differs")
        _require(row["previous_row_sha256"] == previous, "Row hash chain differs")
        payload = base64.b64decode(row["cdr_base64"], validate=True)
        _require(0 < len(payload) <= session["max_cdr_bytes"] and _sha(payload) == row["cdr_sha256"], "CDR hash/size differs")
        if deserialize is not None:
            try:
                message = deserialize(payload, stream)
            except Exception as error:
                raise ValueError("CDR decode failed") from error
            fields = _fields(message)
            _require(fields is not None and identity(fields)
                     and type(fields.get("sequence")) is int and fields.get("sequence") == sequence,
                     "Decoded message identity differs")
            if stream < 2:
                _require(bytes(fields["configuration_sha256"]).hex() == manifest["configuration_sha256"],
                         "Decoded configuration differs")
            if expected_messages is not None:
                _require((stream, sequence) in expected_messages and _equal(message, expected_messages[stream, sequence]),
                         "Decoded fields differ from actual expected message")
        counts[stream] += 1
        keys.add((stream, sequence))
        cdr_bytes += len(payload)
        previous = _sha(line)
        receipts.append({"stream": stream, "sequence": sequence, "sha256": previous})
    _require(expected_messages is None or keys == set(expected_messages), "Expected message coverage differs")
    _require(all(isinstance(manifest[key], list) and len(manifest[key]) == 3
                 and all(type(value) is int for value in manifest[key]) for key in ("counts", "final_sequences"))
             and counts[0] > 0 and counts == manifest["counts"] == manifest["final_sequences"]
             and type(manifest["record_count"]) is int and len(lines) == manifest["record_count"] <= session["capacity"],
             "Count/final sequence differs")
    _require(previous == manifest["last_row_sha256"] and manifest["records_match_persisted_receipts"] is True
             and manifest["sources_unchanged"] is True, "Final integrity declarations differ")
    digest = manifest["configuration_sha256"]
    _require(isinstance(digest, str) and len(digest) == 64 and len(bytes.fromhex(digest)) == 32, "Invalid configuration digest")
    return {"passed": True, **common, "counts": counts, "record_count": len(lines), "cdr_bytes": cdr_bytes,
            "decoded_verified": deserialize is not None, "expected_messages_verified": expected_messages is not None,
            "records_sha256": _sha(raw), "sources_archive_sha256": _sha(archive_raw), "row_receipts": receipts,
            "final_receipt": {"stream": 255, "sequence": len(lines), "sha256": _sha(raw_manifest)},
            "scope": "archived bytes/decoded values; caller must match actual DDS receipts"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--folder", required=True)
    parser.add_argument("--output")
    parser.add_argument("--decode", action="store_true", help="Require installed ROS typed CDR deserializer")
    args = parser.parse_args()
    decoder = None
    if args.decode:
        from rclpy.serialization import deserialize_message
        from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal, GateEvent
        types = (ControlSnapshot, ControlProposal, GateEvent)
        decoder = lambda payload, stream: deserialize_message(payload, types[stream])
    result = verify(args.folder, decoder)
    output = json.dumps(result, allow_nan=False, indent=2)+"\n"
    if args.output:
        with Path(args.output).open("x", encoding="utf-8") as stream:
            stream.write(output)
    else:
        print(output, end="")


if __name__ == "__main__":
    main()
