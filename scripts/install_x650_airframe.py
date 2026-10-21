"""Install the reviewed X650 SITL airframe without replacing unrelated files.

Does not launch PX4, change live parameters, arm, or rebuild/download anything.
Re-run after rebuilding PX4 because the build may recreate its ROMFS directory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PINNED_PX4 = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
NAME = "4999_gz_x650_mn4014_v1"
SOURCE = ROOT / "configs/vehicles/x650_no_arm" / NAME


def install(px4: Path, source: Path | None = None) -> dict:
    source = SOURCE if source is None else Path(source).resolve(strict=True)
    name = source.name
    match = re.fullmatch(r"([1-9][0-9]*)_[A-Za-z0-9_]+", name)
    if match is None:
        raise ValueError("Airframe source filename must start with a positive numeric ID")
    airframe_id = int(match.group(1))
    px4 = px4.resolve(strict=True)
    head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=px4,
                          check=True, text=True, capture_output=True).stdout.strip()
    if head != PINNED_PX4:
        raise ValueError(f"PX4 commit changed: {head}; review output/allocator semantics before installation")
    content = source.read_text(encoding="utf-8").replace("\r\n", "\n").encode()
    directories = [px4 / "ROMFS/px4fmu_common/init.d-posix/airframes",
                   px4 / "build/px4_sitl_default/etc/init.d-posix/airframes"]
    targets = []
    for directory in directories:
        if not directory.is_dir() or not directory.resolve().is_relative_to(px4):
            raise ValueError(f"Missing or external airframe directory: {directory}")
        target = directory / name
        conflicts = [p.name for p in directory.glob(f"{airframe_id}_*") if p.name != name]
        if conflicts:
            raise ValueError(f"Airframe ID {airframe_id} already used in {directory}: {conflicts}")
        if target.is_symlink() or (target.exists() and target.read_bytes() != content):
            raise ValueError(f"Refusing to overwrite different existing airframe: {target}")
        targets.append(target)
    # All destinations were checked before the first write.
    for target in targets:
        if not target.exists():
            with target.open("xb") as stream:
                stream.write(content)
    return {"px4_commit": head, "airframe_id": airframe_id,
            "airframe_sha256": hashlib.sha256(content).hexdigest(),
            "installed_paths": [str(p) for p in targets], "flight_started": False,
            "note": "No CMake source list changed; reinstall after any PX4 rebuild."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4", type=Path, required=True)
    parser.add_argument("--source", type=Path, help="Explicit reviewed airframe; defaults to the no-arm X650")
    args = parser.parse_args()
    print(json.dumps(install(args.px4, args.source), indent=2))


if __name__ == "__main__":
    main()
