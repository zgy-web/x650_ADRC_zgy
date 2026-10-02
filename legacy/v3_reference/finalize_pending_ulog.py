#!/usr/bin/env python3

import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time


HERE = Path(__file__).resolve().parent
PENDING = HERE / "pending_ulog_finalize.json"


def sha256(path):
    h = hashlib.sha256()

    with open(path, "rb") as f:
        while True:
            block = f.read(4 * 1024 * 1024)

            if not block:
                break

            h.update(block)

    return h.hexdigest()


def main():
    if not PENDING.exists():
        print(
            "[INFO] No pending ULog finalization."
        )
        return

    marker = json.loads(
        PENDING.read_text()
    )

    folder = Path(
        marker["result_dir"]
    )

    src = Path(
        marker["ulog_source"]
    )

    if not src.is_file():
        raise RuntimeError(
            f"ULog source missing: {src}"
        )

    st = src.stat()

    if (
        int(st.st_dev)
        != int(marker["source_dev"])
        or
        int(st.st_ino)
        != int(marker["source_inode"])
    ):
        raise RuntimeError(
            "ULog source identity changed; "
            "refusing automatic archive."
        )

    result_path = folder / "result.json"

    if not result_path.is_file():
        raise RuntimeError(
            f"Missing result.json: {result_path}"
        )

    dst = folder / "px4.ulg"
    tmp = folder / ".px4.ulg.tmp"

    shutil.copy2(
        src,
        tmp,
    )

    os.replace(
        tmp,
        dst,
    )

    digest = sha256(dst)

    result = json.loads(
        result_path.read_text()
    )

    result["ulog_source"] = str(src)
    result["ulog_copied"] = True
    result["ulog_pending_finalize"] = False
    result["ulog_archive_method"] = (
        "copy_after_px4_shutdown"
    )
    result["ulog_sha256"] = digest
    result["ulog_size_bytes"] = (
        dst.stat().st_size
    )
    result["ulog_finalized_wall_time"] = (
        time.time()
    )

    result_path.write_text(
        json.dumps(
            result,
            indent=2,
        )
    )

    PENDING.unlink()

    print(
        "[ OK ] ULog finalized"
    )
    print(
        "Result:",
        folder,
    )
    print(
        "ULog:",
        dst,
    )
    print(
        "Size:",
        dst.stat().st_size,
    )
    print(
        "SHA256:",
        digest,
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(
            "[FAIL] ULog finalization:",
            exc,
            file=sys.stderr,
        )
        raise
