#!/usr/bin/env python3
"""Build the explicit TS20 numerical accelerator with an existing compiler.

No installation, implicit loading, model identification or flight is performed.
Use the resulting library hash in the independent oracle CLI before admission.
"""
from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/"control_packages/x650_wholebody_v1/x650_wholebody/ts20_prediction_native.cpp"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cxx", default="c++")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists():
        raise FileExistsError("Refusing to replace an existing build directory: "+str(output))
    compiler = shutil.which(args.cxx)
    if compiler is None:
        raise RuntimeError("An existing C++17 compiler is required; no download is performed")
    output.mkdir(parents=True, exist_ok=False)
    source_copy = output/SOURCE.name
    source_copy.write_bytes(SOURCE.read_bytes())
    library = output/"libts20_prediction.so"
    command = [compiler, "-std=c++17", "-O3", "-fPIC", "-shared", str(source_copy), "-o", str(library)]
    report = {"complete": False, "passed": False, "source_path": str(SOURCE),
              "source_sha256": sha(SOURCE), "archived_source": str(source_copy),
              "compiler": compiler, "command": command,
              "hardware_accessed": False, "numerical_oracle_passed": False}
    try:
        report["compiler_version"] = subprocess.run([compiler, "--version"], check=True,
            capture_output=True, text=True).stdout.splitlines()[0]
        with (output/"compile.log").open("w", encoding="utf-8") as log:
            subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
        loaded = ctypes.CDLL(str(library))
        loaded.ts20_prediction_abi.restype = ctypes.c_int
        abi = int(loaded.ts20_prediction_abi())
        if abi != 1:
            raise RuntimeError("Unexpected numerical accelerator ABI")
        report.update(complete=True, passed=True, abi=abi, library=str(library), library_sha256=sha(library))
    except BaseException as exc:
        report["error"] = repr(exc)
        raise
    finally:
        (output/"build_manifest.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
