"""Pinned offline serialization evidence; never a flight/deadline certificate.

Decoded recorded JSON cannot recover historical NumPy types, aliases or heap.
The separate fresh NumPy fixture is one inner allocation, not an adapter heap.
Natural GC is observed without collection or policy changes. Optional
tracemalloc measurements are separate from ordinary timing samples.
"""
from __future__ import annotations
import argparse
from collections import Counter, deque
import gc
import hashlib
import json
from pathlib import Path
import struct
import sys
import threading
import time
import traceback
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/"scripts"), str(ROOT/"control_packages/x650_wholebody_v1")]
VERSION = "ts20_diagnostic_serialization_software_observation_v1"
ACTIVE_OUTPUT = None


def identity(path, expected=None):
    path = Path(path).resolve(strict=True)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if expected is not None and (len(expected) != 64 or set(expected)-set("0123456789abcdef") or digest != expected):
        raise ValueError("Caller-pinned source/input/library SHA differs: "+str(path))
    return {"path": str(path), "sha256": digest, "bytes": path.stat().st_size}


def save(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False)+"\n")


def _fingerprint_item(item, digest, active):
    kind = type(item)
    digest.update((kind.__module__+"."+kind.__qualname__).encode()+b"\0")
    if isinstance(item, np.ndarray):
        if item.dtype.hasobject:
            raise ValueError("Object-array input fingerprints are outside this numeric diagnostic harness")
        digest.update(repr((item.dtype.str, item.shape, item.strides, item.flags.writeable)).encode())
        digest.update(item.tobytes(order="C"))
    elif isinstance(item, np.generic):
        if item.dtype.hasobject:
            raise ValueError("Object scalar input fingerprints are outside this numeric diagnostic harness")
        digest.update(item.dtype.str.encode()+item.tobytes())
    elif kind in (dict, list, tuple):
        if id(item) in active:
            raise ValueError("Cyclic diagnostic inputs are outside this harness")
        active.add(id(item))
        digest.update(str(len(item)).encode()+b"\0")
        if kind is dict:
            for key, child in item.items():
                _fingerprint_item(key, digest, active)
                _fingerprint_item(child, digest, active)
        else:
            for child in item:
                _fingerprint_item(child, digest, active)
        active.remove(id(item))
    elif kind is float:
        digest.update(struct.pack("!d", item))
    elif kind in (str, int, bool, type(None)):
        digest.update(repr(item).encode())
    else:
        raise ValueError("Unsupported fingerprint type: "+repr(kind))


def fingerprint(value):
    """No recursive closure cycles; exact values/types/order/array layout."""
    digest = hashlib.sha256()
    _fingerprint_item(value, digest, set())
    return digest.hexdigest()


def _count_item(item, counts, arrays):
    kind = type(item)
    counts[kind.__module__+"."+kind.__qualname__] += 1
    if isinstance(item, np.ndarray):
        arrays[item.dtype.str] += 1
    elif kind is dict:
        for key, child in item.items():
            _count_item(key, counts, arrays)
            _count_item(child, counts, arrays)
    elif kind in (list, tuple):
        for child in item:
            _count_item(child, counts, arrays)


def type_counts(value):
    counts, arrays = Counter(), Counter()
    _count_item(value, counts, arrays)
    return {"path_occurrences_by_type": dict(counts), "ndarray_occurrences_by_dtype": dict(arrays),
            "alias_identity_reconstructed": False}


def collection_counts():
    return [entry["collections"] for entry in gc.get_stats()]


class NaturalGcObservation:
    def __init__(self):
        self.events, self.active = deque(maxlen=4096), {}
        self.context, self.completed, self.errors, self.unpaired = None, 0, 0, 0
        self.callback = self.observe
        self.callbacks_before, self.thresholds, self.enabled = tuple(gc.callbacks), gc.get_threshold(), gc.isenabled()
        gc.callbacks.append(self.callback)

    def observe(self, phase, info):
        try:
            generation = info["generation"]
            stamp = (time.monotonic(), time.perf_counter(), time.thread_time(), threading.get_ident())
            if phase == "start":
                self.active[generation] = (stamp, self.context)
            elif phase == "stop":
                record = self.active.pop(generation, None)
                if record is None:
                    self.unpaired += 1
                    return
                start, context = record
                self.events.append({"generation": generation, "context_at_start": context,
                    "start_wall_monotonic_s": start[0], "stop_wall_monotonic_s": stamp[0],
                    "start_perf_counter_s": start[1], "stop_perf_counter_s": stamp[1],
                    "wall_ms": 1000*(stamp[1]-start[1]), "thread_cpu_ms": 1000*(stamp[2]-start[2]),
                    "start_thread_ident": start[3], "stop_thread_ident": stamp[3],
                    "collected": info.get("collected"), "uncollectable": info.get("uncollectable")})
                self.completed += 1
        except BaseException:
            self.errors += 1

    def close(self):
        for index, callback in enumerate(gc.callbacks):
            if callback is self.callback:
                del gc.callbacks[index]
                break
        unchanged = (tuple(gc.callbacks) == self.callbacks_before and gc.get_threshold() == self.thresholds
                     and gc.isenabled() == self.enabled)
        return {"natural_events": list(self.events), "completed_count": self.completed,
            "event_capacity": 4096, "events_dropped": max(0, self.completed-4096),
            "callback_errors": self.errors, "unpaired_stops": self.unpaired,
            "active_generations_at_close": sorted(self.active), "gc_policy_and_other_callbacks_unchanged": unchanged,
            "observations_complete": unchanged and self.completed <= 4096 and self.errors == self.unpaired == 0 and not self.active,
            "active_gc_requested": False,
            "duration_scope": "natural collection plus intervening callbacks; not exclusive collector cost"}


def main(argv=None):
    global ACTIVE_OUTPUT
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("run-dir", "native-library", "output"):
        parser.add_argument("--"+name, type=Path, required=True)
    for name in ("diagnostics-sha256", "reference-source-sha256", "serializer-source-sha256",
                 "fixture-source-sha256", "native-sha256"):
        parser.add_argument("--"+name, required=True)
    parser.add_argument("--rows", type=int, default=20)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--tracemalloc", action="store_true")
    args = parser.parse_args(argv)
    if not 1 <= args.rows <= 20 or not 1 <= args.repeats <= 50:
        raise ValueError("Use 1..20 recorded rows and 1..50 interleaved repeats")
    output = args.output.absolute()
    output.mkdir(parents=True, exist_ok=False)
    ACTIVE_OUTPUT = output
    from simulations.x650_wholebody_mpc_v1.recording import json_value
    from simulations.x650_ts20_hold_v1 import diagnostic_serialization as new
    import profile_x650_ts20_matrix_fixture as fixture_module
    reference_source = identity(ROOT/"simulations/x650_wholebody_mpc_v1/recording.py", args.reference_source_sha256)
    serializer_source = identity(new.__file__, args.serializer_source_sha256)
    fixture_source = identity(fixture_module.__file__, args.fixture_source_sha256)
    native = identity(args.native_library, args.native_sha256)
    run = args.run_dir.resolve(strict=True)
    run.relative_to(ROOT/"runs")
    input_identity = identity(run/"controller_diagnostics.jsonl", args.diagnostics_sha256)
    save(output/"invocation.json", {"kind": VERSION, "argv": sys.argv if argv is None else argv,
        "run_directory": str(run), "sources": [reference_source, serializer_source, fixture_source],
        "native_library": native, "input": input_identity, "harness": identity(__file__),
        "not_a_flight_or_deadline_gate": True})
    cases, selected_raw = [], deque(maxlen=args.rows)
    for index, raw in enumerate((run/"controller_diagnostics.jsonl").read_bytes().splitlines(keepends=True), 1):
        row = json.loads(raw)
        if row.get("phase") == "measure":
            selected_raw.append((index, raw, row))
    if len(selected_raw) != args.rows:
        raise ValueError("Insufficient real measurement diagnostic rows")
    with (output/"recorded_selected_rows.jsonl").open("xb") as stream:
        for index, raw, row in selected_raw:
            stream.write(raw)
            cases.append({"name": "recorded_line_"+str(index), "kind": "decoded_recorded_json", "value": row,
                "input_line_number": index, "input_raw_sha256": hashlib.sha256(raw).hexdigest(),
                "original_numpy_types_and_aliases_recoverable": False})
    c, predictor, state, target = fixture_module.fixture(args.native_library, args.native_sha256)
    fixture_begin = time.perf_counter()
    allocation = c.step(0., .02, state, target)
    fixture_ms = 1000*(time.perf_counter()-fixture_begin)
    if allocation["diagnostics"].get("accepted_optimum") is not True:
        raise ValueError("Fresh NumPy inner fixture was not accepted")
    cases.append({"name": "fresh_numpy_inner_allocation", "kind": "fresh_numpy_inner_diagnostic",
        "value": {"sim_s": float(state.control_time_s), "t_s": 0., "phase": "measure",
                  "diagnostics": allocation["diagnostics"]},
        "historical_heap_restored": False, "complete_adapter_heap_reconstructed": False})
    actual_native = predictor.metadata()["native_accelerator"]
    if (actual_native is None or actual_native["sha256"] != args.native_sha256
            or Path(predictor._native._name).resolve(strict=True) != args.native_library.resolve(strict=True)):
        raise ValueError("Fresh fixture used another native library")
    def old_encode(value):
        return json.dumps(json_value(value), allow_nan=False)+"\n"
    def new_encode(value):
        return new.encode_diagnostic_row(value, reference_converter=json_value)
    functions = {"old": old_encode, "new": new_encode}
    case_reports = []
    for case in cases:
        value, before = case["value"], fingerprint(case["value"])
        old, candidate = old_encode(value), new_encode(value)
        if old != candidate or fingerprint(value) != before:
            raise ValueError("Serialization bytes/input differ: "+case["name"])
        case["expected_sha256"] = hashlib.sha256(old.encode()).hexdigest()
        case["fingerprint"] = before
        report = {key: item for key, item in case.items() if key != "value"}
        report.update(input_type_counts=type_counts(value), supports_fast_tree=new.supports_fast_tree(value),
                      byte_identical=True, input_unchanged=True, encoded_bytes=len(old.encode()))
        if case["kind"] == "fresh_numpy_inner_diagnostic":
            if not report["input_type_counts"]["ndarray_occurrences_by_dtype"]:
                raise ValueError("Fresh fixture no longer contains actual NumPy arrays")
            (output/"fresh_inner_diagnostic.json").write_text(old, encoding="utf-8")
        case_reports.append(report)
    modules = {name: identity(module.__file__) for name, module in sorted(sys.modules.items())
        if getattr(module, "__file__", None) and (name == "x650_wholebody" or name.startswith("x650_wholebody.")
            or name.startswith("simulations.x650_ts20_hold_v1.")
            or name in {"test_ts20_mpc", "test_ts20_prediction", "test_mixed_mpc", "test_model",
                        "profile_x650_ts20_matrix_fixture", "simulations.x650_wholebody_mpc_v1.recording"})}
    source_identities = {"harness": identity(__file__),
        "native_cpp": identity(ROOT/"control_packages/x650_wholebody_v1/x650_wholebody/ts20_prediction_native.cpp"), **modules}
    with zipfile.ZipFile(output/"executed_sources.zip", "x", zipfile.ZIP_DEFLATED) as archive:
        for name, item in source_identities.items():
            content = Path(item["path"]).read_bytes()
            if hashlib.sha256(content).hexdigest() != item["sha256"]:
                raise ValueError("Source changed before archive")
            member = name.replace(".", "/")+Path(item["path"]).suffix
            archive.writestr(member, content)
            item["archive_member"] = member
    timings, allocation_observations = [], []
    observer = NaturalGcObservation()
    try:
        for repeat in range(args.repeats):
            for index, case in enumerate(cases):
                for variant in (("old", "new") if (repeat+index) % 2 == 0 else ("new", "old")):
                    observer.context = {"section": "ordinary_timing", "case": case["name"], "variant": variant, "repeat": repeat}
                    counts_before = collection_counts()
                    wall, cpu, monotonic = time.perf_counter(), time.thread_time(), time.monotonic()
                    encoded = functions[variant](case["value"])
                    wall_end, cpu_end, mono_end = time.perf_counter(), time.thread_time(), time.monotonic()
                    counts_after = collection_counts()
                    observer.context = {"section": "post_sample_validation"}
                    if hashlib.sha256(encoded.encode()).hexdigest() != case["expected_sha256"] or fingerprint(case["value"]) != case["fingerprint"]:
                        raise ValueError("Timed encoder changed bytes or input")
                    timings.append({"case": case["name"], "kind": case["kind"], "repeat": repeat,
                        "variant": variant, "wall_ms": 1000*(wall_end-wall), "thread_cpu_ms": 1000*(cpu_end-cpu),
                        "start_wall_monotonic_s": monotonic, "stop_wall_monotonic_s": mono_end,
                        "gc_count_before": counts_before, "gc_count_after": counts_after,
                        "gc_count_delta": [a-b for a, b in zip(counts_after, counts_before)]})
                    del encoded
        if args.tracemalloc:
            import tracemalloc
            if tracemalloc.is_tracing():
                raise RuntimeError("Refuse to replace an existing tracemalloc session")
            for index, case in enumerate(cases):
                for variant in (("old", "new") if index % 2 == 0 else ("new", "old")):
                    observer.context = {"section": "separate_tracemalloc", "case": case["name"], "variant": variant}
                    tracemalloc.start()
                    try:
                        encoded = functions[variant](case["value"])
                        current, peak = tracemalloc.get_traced_memory()
                    finally:
                        tracemalloc.stop()
                    if hashlib.sha256(encoded.encode()).hexdigest() != case["expected_sha256"] or fingerprint(case["value"]) != case["fingerprint"]:
                        raise ValueError("Traced encoder changed bytes or input")
                    allocation_observations.append({"case": case["name"], "variant": variant,
                        "current_traced_bytes_with_output_live": current, "peak_traced_bytes": peak,
                        "not_an_ordinary_timing_sample": True})
                    del encoded
    finally:
        natural_gc = observer.close()
    for item in [*source_identities.values(), native, input_identity]:
        identity(item["path"], item["sha256"])
    summary = {}
    for kind in {case["kind"] for case in cases}:
        summary[kind] = {variant: {"samples": len(values), "median_wall_ms": float(np.median(values)),
                                  "min_wall_ms": min(values), "max_wall_ms": max(values)}
            for variant in functions for values in [[sample["wall_ms"] for sample in timings
                if sample["kind"] == kind and sample["variant"] == variant]]}
    report = {"kind": VERSION, "complete": natural_gc["observations_complete"],
        "all_bytes_equal_and_inputs_unchanged": True, "flight_started": False,
        "historical_numpy_types_or_heap_reconstructed": False, "full_hold_success_guaranteed": False,
        "timing_scope": "encoder call only; input fingerprints/type counting/hashes outside ordinary timing",
        "natural_gc_scope": "interleaved encoder samples, their validation bookkeeping and optional traced section; setup/fresh solve/source archiving excluded",
        "natural_gc_observation_overhead_present": True, "policy_changed": False,
        "input": input_identity, "native_library": native, "loaded_native": actual_native,
        "fresh_fixture_wall_ms": fixture_ms, "fresh_fixture_is_a_deadline_gate": False,
        "cases": case_reports, "raw_timings": timings, "summary": summary,
        "natural_gc": natural_gc, "separate_tracemalloc": allocation_observations,
        "source_identities": source_identities, "source_archive": identity(output/"executed_sources.zip"),
        "python": {"executable": sys.executable, "version": sys.version,
                   "numpy_version": np.__version__, "numpy_path": np.__file__,
                   "scipy_version": sys.modules["scipy"].__version__, "scipy_path": sys.modules["scipy"].__file__}}
    save(output/"serialization_observation.json", report)
    print(json.dumps({"complete": report["complete"], "summary": summary,
        "report": identity(output/"serialization_observation.json")}, indent=2))
    if not report["complete"]:
        raise RuntimeError("Natural GC observation incomplete; ordinary raw samples are retained")


if __name__ == "__main__":
    try:
        main()
    except BaseException as error:
        if ACTIVE_OUTPUT is not None:
            save(ACTIVE_OUTPUT/"failure_report.json", {"complete": False, "kind": VERSION,
                "exception": repr(error), "traceback": traceback.format_exc(), "not_a_flight_or_deadline_gate": True})
        raise
