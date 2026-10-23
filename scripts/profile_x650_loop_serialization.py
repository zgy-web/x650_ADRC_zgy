"""Offline JSON conversion/encoding/write/flush evidence; never flight timing.

Decoded JSONL trees have already lost their historical NumPy/container types.
``profile_tree`` can separately receive a newly reconstructed live NumPy tree.
No observer result establishes the cause or duration of a historical delay.
"""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
from time import perf_counter_ns, thread_time_ns

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / "simulations/x650_wholebody_mpc_v1/recording.py"
_PLAIN = (str, int, float, bool, type(None))


def primitive_fast_value(value):
    """Same branches as json_value, with exact builtin leaves inlined.

    In particular, ndarray.tolist and np.generic.item remain non-recursive;
    object-array acceptance is not expanded. Keys are still str(key).
    """
    if type(value) in _PLAIN:
        return value
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): item if type(item) in _PLAIN else primitive_fast_value(item)
                for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [item if type(item) in _PLAIN else primitive_fast_value(item) for item in value]
    return value


def numpy_default(value):
    """Experimental direct encoder; explicitly NOT universally equivalent."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return json.JSONEncoder().default(value)


def identity(path):
    path = Path(path).resolve(strict=True)
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "bytes": path.stat().st_size}


def reference_converter(expected_sha256=None):
    ref = identity(REFERENCE)
    if expected_sha256 is not None and ref["sha256"] != expected_sha256:
        raise ValueError("Actual recording.py differs from caller-pinned identity")
    spec = importlib.util.spec_from_file_location("_serialization_recording_reference", REFERENCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.json_value, ref


def clocks():
    return perf_counter_ns(), thread_time_ns()


def timed(operation):
    start = clocks()
    result = operation()
    end = clocks()
    return result, {"wall_ms": (end[0]-start[0])/1e6, "thread_cpu_ms": (end[1]-start[1])/1e6}


def encoded(value, convert=None, direct=False):
    try:
        return {"returned": True, "text": json.dumps(value if convert is None else convert(value),
                allow_nan=False, default=numpy_default if direct else None)}
    except Exception as error:
        return {"returned": False, "error_type": type(error).__name__, "error": str(error)}


def semantic_probe(convert):
    cases = {
        "plain": {"x": [1., 2, True, None, "unicode \u5173\u8282"]},
        "numeric_arrays": {"q": np.array([.1, -.2]), "n": np.array([[1, 2]], dtype=np.int64)},
        "scalar_types": [np.float32(.1), np.float64(.2), np.int64(2), np.uint64(3), np.bool_(True), np.str_("x")],
        "tuple": (1, (np.float32(.1), 2)),
        "bool_none_keys": {True: "yes", None: "missing", False: "no"},
        "tuple_key": {(1, 2): "tuple"},
        "numpy_key": {np.int64(2): "numpy"},
        "key_collision": {True: "first", "True": "second", None: "first", "None": "second"},
        "nan_key": {float("nan"): "key"},
        "nan_value": [float("nan")],
        "numpy_nan": [np.float32("nan")],
        "array_nan": np.array([float("nan")]),
        "object_array_numpy": np.array([np.int64(3)], dtype=object),
        "object_array_plain": np.array([{"x": [1, 2]}], dtype=object),
        "complex_scalar": np.complex128(1+2j),
        "complex_array": np.array([1+2j]),
        "longdouble": np.longdouble(.1),
        "bytes": b"unsupported",
        "set": {1, 2},
        "datetime_scalar": np.datetime64("2026-10-23"),
        "zero_dim_array": np.array(3),
    }
    rows = []
    for name, value in cases.items():
        old, fast, direct = encoded(value, convert), encoded(value, primitive_fast_value), encoded(value, direct=True)
        rows.append({"case": name, "primitive_fast_exact": old == fast,
                     "direct_default_exact": old == direct,
                     "reference": old, "primitive_fast": fast, "direct_default": direct})
    return {"primitive_fast_all_exact": all(r["primitive_fast_exact"] for r in rows),
            "direct_default_all_exact": all(r["direct_default_exact"] for r in rows), "cases": rows}


def profile_tree(value, label, out, convert):
    """Measure one tree once per variant, retaining all results and output bytes."""
    reports, reference_text = [], None
    for name, transform, direct in (("reference", convert, False),
                                    ("primitive_fast", primitive_fast_value, False),
                                    ("direct_default_not_universal", None, True)):
        converted, conversion = timed(lambda: value if transform is None else transform(value))
        text, encoding = timed(lambda: json.dumps(converted, allow_nan=False,
                                                 default=numpy_default if direct else None)+"\n")
        if reference_text is None:
            reference_text = text
        path = out / (label+"_"+name+".json")
        with path.open("x", encoding="utf-8", newline="") as stream:
            _, writing = timed(lambda: stream.write(text))
            _, flushing = timed(stream.flush)
        reports.append({"variant": name, "conversion": conversion, "encoding": encoding,
                        "write": writing, "flush": flushing, "exact_reference_bytes": text == reference_text,
                        "output": identity(path)})
    return {"label": label, "variants": reports}


def last_records(path, count):
    """Read only complete original JSONL records; retain byte/line provenance."""
    records, offset, lines = deque(maxlen=count), 0, 0
    with path.open("rb") as stream:
        for number, line in enumerate(stream):
            if not line.endswith(b"\n"):
                raise ValueError("Input is not a frozen complete JSONL file")
            current = offset
            offset += len(line)
            lines += 1
            row = json.loads(line)
            if row.get("phase") == "measure":
                records.append((number, current, line, row))
    if not records:
        raise ValueError("No original measure record found")
    return list(records), lines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--input-jsonl", type=Path)
    parser.add_argument("--input-sha256")
    parser.add_argument("--reference-sha256")
    parser.add_argument("--last-records", type=int, default=3)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    out = args.output.resolve(strict=True)
    convert, ref = reference_converter(args.reference_sha256)
    shutil.copyfile(__file__, out/"executed_profile_x650_loop_serialization.py")
    probe = semantic_probe(convert)
    if not probe["primitive_fast_all_exact"]:
        raise AssertionError("Primitive candidate failed exact semantic probes")
    results, source, total_lines = [], None, None
    if not args.self_test:
        if args.input_jsonl is None or args.input_sha256 is None or args.reference_sha256 is None or args.last_records < 1:
            raise ValueError("Explicit frozen input/reference identities and positive count required")
        source = identity(args.input_jsonl)
        if source["sha256"] != args.input_sha256:
            raise ValueError("Original diagnostic JSONL identity mismatch")
        selected, total_lines = last_records(args.input_jsonl, args.last_records)
        for line, offset, original, value in selected:
            label = "raw_jsonl_line_"+str(line)
            (out/(label+"_original.json")).write_bytes(original)
            row = profile_tree(value, label, out, convert)
            row.update(original_line_zero_based=line, original_byte_offset=offset,
                       sim_s=value.get("sim_s"), t_s=value.get("t_s"), phase=value.get("phase"),
                       source_tree="decoded historical JSON, NOT original in-memory NumPy tree")
            results.append(row)
    if identity(REFERENCE) != ref:
        raise RuntimeError("Reference source changed during observation")
    summary = {"kind": "x650_loop_serialization_offline_profile_v1", "harness": identity(__file__),
        "reference": ref, "input": source, "total_lines": total_lines, "semantic_probe": probe,
        "results": results, "no_flight": True, "control_or_logging_source_changed": False,
        "historical_duration_or_cause_established": False,
        "scope": "One offline conversion/encoding/write/flush sample per variant and selected record; wall/thread CPU only. No fsync or GC policy changes. Raw JSON loses historical NumPy/container types; live reconstructed NumPy requires a separate labeled profile_tree call."}
    (out/"profile_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"summary": identity(out/"profile_summary.json"),
                      "primitive_fast_all_exact": probe["primitive_fast_all_exact"],
                      "direct_default_all_exact": probe["direct_default_all_exact"],
                      "profiled_records": len(results)}, indent=2))


if __name__ == "__main__":
    main()
