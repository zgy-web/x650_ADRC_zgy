"""Capture a reconstructed v3 MPC NumPy tree for serialization-only evidence.

This does not recreate v4's missing failed snapshot or historical GC heap.
It uses the existing two-branch late-warm reconstruction without changing its
control model, then observes the returned live Python/NumPy diagnostics.
"""
from contextlib import redirect_stdout
import argparse
import json
from pathlib import Path
import shutil
import sys
from unittest.mock import patch

import numpy as np

from profile_x650_loop_serialization import (ROOT, identity, profile_tree,
                                             reference_converter, semantic_probe)


def types(value, counts):
    counts[type(value).__module__+"."+type(value).__qualname__] = counts.get(
        type(value).__module__+"."+type(value).__qualname__, 0)+1
    if isinstance(value, dict):
        for item in value.values():
            types(item, counts)
    elif isinstance(value, (list, tuple)):
        for item in value:
            types(item, counts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--fixture-sha256", required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--reference-sha256", required=True)
    parser.add_argument("--reconstruction-harness-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    out = args.output.resolve(strict=True)
    sys.path[:0] = [str(ROOT/"control_packages/x650_wholebody_v1"), str(ROOT/"scripts")]
    import profile_x650_ts20_late_warm as reconstruction
    from x650_wholebody.mpc import MpcController
    if identity(reconstruction.__file__)["sha256"] != args.reconstruction_harness_sha256:
        raise ValueError("Caller-pinned reconstruction source differs")
    convert, ref = reference_converter(args.reference_sha256)
    captured = []
    original = MpcController.step
    def observe(controller, t, dt, state, target):
        result = original(controller, t, dt, state, target)
        captured.append({"sim_s": state.control_time_s, "t_s": t, "phase": "measure",
                         "diagnostics": result["diagnostics"]})
        return result
    argv = [str(reconstruction.__file__), "--fixture", str(args.fixture),
            "--fixture-sha256", args.fixture_sha256, "--run-dir", str(args.run_dir),
            "--output", str(out/"reconstruction")]
    with (out/"reconstruction_stdout.log").open("x") as log, \
            patch.object(sys, "argv", argv), patch.object(MpcController, "step", observe), redirect_stdout(log):
        reconstruction.main()
    if len(captured) != 2:
        raise ValueError("Expected one returned inner candidate for each pending-packet branch")
    rows = []
    for branch, tree in zip(("pending_packet_omitted", "pending_packet_included"), captured):
        counts = {}
        types(tree, counts)
        if counts.get("numpy.ndarray", 0) == 0:
            raise ValueError("Live NumPy tree was lost before serialization observation")
        report = profile_tree(tree, "live_v3_"+branch, out, convert)
        report.update(type_counts=counts, branch=branch,
            source_tree="Fresh reconstructed v3 MPC inner diagnostics with live NumPy arrays; not original v4/historical in-memory objects")
        rows.append(report)
    probe = semantic_probe(convert)
    shutil.copyfile(__file__, out/"executed_profile_x650_loop_serialization_live.py")
    summary = {"kind": "x650_loop_serialization_live_numpy_profile_v1", "harness": identity(__file__),
        "serializer_harness": identity(ROOT/"scripts/profile_x650_loop_serialization.py"),
        "reference": ref, "fixture": identity(args.fixture),
        "reconstruction_harness": identity(reconstruction.__file__),
        "reconstruction_identity_and_limits": identity(out/"reconstruction/profile_summary.json"),
        "results": rows, "semantic_probe": probe, "no_flight": True,
        "historical_duration_or_cause_established": False,
        "v4_failed_snapshot_reconstructed": False,
        "scope": "Serialization-only offline measurements of fresh v3 NumPy output; no model/parameter/guard changes. Raw v4 JSON and live v3 NumPy are distinct evidence scopes."}
    (out/"profile_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"summary": identity(out/"profile_summary.json"), "captured_trees": len(rows)}, indent=2))


if __name__ == "__main__":
    main()
