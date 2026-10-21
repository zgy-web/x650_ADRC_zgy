#!/usr/bin/env python3
"""Build the native Gazebo recorder without installing over a runtime library."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile

PACKAGES = ("gz-sim8", "gz-plugin2-register")

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=Path(__file__).resolve().parent / "build")
    parser.add_argument("--cxx", default=os.environ.get("CXX", "c++"), help="Compiler executable, without additional arguments")
    args = parser.parse_args()
    source = Path(__file__).resolve().with_name("wholebody_recorder.cpp")
    build = args.build_dir.expanduser().resolve()
    if build == source.parent:
        parser.error("Use a separate build directory; the legacy runtime library directory is not a build destination")
    compiler = shutil.which(args.cxx)
    pkg_config = shutil.which("pkg-config")
    if compiler is None or pkg_config is None:
        parser.error("A C++17 compiler and pkg-config must already be installed")
    flags = subprocess.run([pkg_config, "--cflags", "--libs", *PACKAGES], text=True, capture_output=True)
    if flags.returncode:
        parser.error("Missing Gazebo development packages: " + ", ".join(PACKAGES))
    versions = {}
    for package in PACKAGES:
        result = subprocess.run([pkg_config, "--modversion", package], text=True, capture_output=True, check=True)
        versions[package] = result.stdout.strip()
    compiler_version = subprocess.run([compiler, "--version"], text=True, capture_output=True, check=True).stdout.splitlines()[0]
    build.mkdir(parents=True, exist_ok=True)
    library = build / "libwholebody_recorder.so"
    log = build / "compile.log"
    with tempfile.TemporaryDirectory(prefix=".compile-", dir=build) as temporary:
        staged = Path(temporary) / library.name
        command = [compiler, "-std=c++17", "-O2", "-fPIC", "-shared", "-Wl,--no-undefined",
                   str(source), "-o", str(staged), *shlex.split(flags.stdout)]
        with log.open("w", encoding="utf-8") as stream:
            result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
        if result.returncode:
            raise SystemExit("Compilation failed; inspect " + str(log))
        os.replace(staged, library)
    manifest = {
        "schema_version": 1,
        "source": str(source), "source_sha256": digest(source),
        "library": str(library), "library_sha256": digest(library),
        "compiler": compiler_version, "packages": versions,
        "command": command, "compile_log": str(log),
        "installed": False,
    }
    (build / "build_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))

if __name__ == "__main__":
    main()
