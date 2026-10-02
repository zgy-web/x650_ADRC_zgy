#!/usr/bin/env python3

import json
import os
from pathlib import Path
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
BASE = Path("/root/autodl-tmp")

RESULTS = Path(
    os.environ.get(
        "RESULTS_DIR",
        str(BASE / "px4_baseline_bags"),
    )
)

ULOG_ROOT = (
    BASE
    / "PX4-Autopilot/build/px4_sitl_default/rootfs/log"
)

PENDING = HERE / "pending_ulog_finalize.json"

BASELINE_PY = HERE / "baseline_underfold.py"

CLOUD_PY = (
    BASE
    / "cloud-arm-venv/bin/python3"
)


def active_ulog():
    files = list(
        ULOG_ROOT.rglob("*.ulg")
    )

    if not files:
        raise RuntimeError(
            "No active PX4 ULog found."
        )

    src = max(
        files,
        key=lambda p: p.stat().st_mtime,
    )

    age = time.time() - src.stat().st_mtime

    if age > 10:
        raise RuntimeError(
            "Newest ULog is not being updated "
            f"(age={age:.1f}s): {src}"
        )

    return src.resolve()


def main():
    if PENDING.exists():
        raise RuntimeError(
            "A previous baseline still needs ULog finalization.\n"
            "Stop the simulation session first."
        )

    src = active_ulog()
    st = src.stat()

    print(
        "Active PX4 ULog:",
        src,
        flush=True,
    )

    before = {
        p.resolve()
        for p in RESULTS.glob(
            "baseline_underfold_*"
        )
        if p.is_dir()
    }

    rc = subprocess.run(
        [
            str(CLOUD_PY),
            str(BASELINE_PY),
        ]
    ).returncode

    after = {
        p.resolve()
        for p in RESULTS.glob(
            "baseline_underfold_*"
        )
        if p.is_dir()
    }

    created = sorted(
        after - before,
        key=lambda p: p.stat().st_mtime,
    )

    if len(created) != 1:
        raise RuntimeError(
            "Could not uniquely identify new baseline folder: "
            + repr([str(p) for p in created])
        )

    folder = created[0]

    marker = {
        "result_dir": str(folder),
        "ulog_source": str(src),
        "source_dev": int(st.st_dev),
        "source_inode": int(st.st_ino),
        "baseline_returncode": int(rc),
        "created_wall_time": time.time(),
    }

    tmp = PENDING.with_suffix(".tmp")

    tmp.write_text(
        json.dumps(
            marker,
            indent=2,
        )
    )

    tmp.replace(PENDING)

    print()
    print(
        "ULog finalization queued:",
        PENDING,
        flush=True,
    )

    print(
        "Result directory:",
        folder,
        flush=True,
    )

    raise SystemExit(rc)


if __name__ == "__main__":
    main()
