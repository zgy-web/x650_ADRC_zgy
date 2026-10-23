#!/usr/bin/env python3
"""Private local diagnostic sink; JSON/file work is isolated from control."""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import pickle
import signal
import socket
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--socket", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root / "ros2_ws/src/flyarm_mpc_reference"))
    from flyarm_mpc_reference.diagnostic_channel import HEADER, MAGIC, MAX_PACKET
    # numpy is needed to decode the owned worker's detached arrays. No ROS or
    # controller context is imported into this process.
    import numpy as np
    def default(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, bytes):
            return {"base64": base64.b64encode(value).decode("ascii")}
        raise TypeError(type(value).__name__)
    args.output.mkdir(exist_ok=False)
    stopping = [False]
    def stop(*unused):
        stopping[0] = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * MAX_PACKET)
    sock.bind(str(args.socket))
    os.chmod(args.socket, 0o600)
    sock.settimeout(.1)
    rows, last, gaps, error = 0, 0, [], None
    digest = hashlib.sha256()
    begin = time.monotonic_ns()
    try:
        with (args.output / "diagnostics.jsonl").open("xb") as stream:
            while True:
                try:
                    payload, _, flags, _ = sock.recvmsg(MAX_PACKET)
                except socket.timeout:
                    if stopping[0]:
                        break
                    continue
                if flags & socket.MSG_TRUNC or len(payload) < HEADER.size:
                    raise ValueError("Truncated diagnostic datagram")
                magic, seq = HEADER.unpack(payload[:HEADER.size])
                if magic != MAGIC or seq <= last:
                    raise ValueError("Malformed or reordered diagnostic sequence")
                if seq != last + 1:
                    gaps.append([last + 1, seq - 1])
                value = pickle.loads(payload[HEADER.size:])  # private owned socket ONLY
                data = (json.dumps({"sequence": seq, "received_ns": time.monotonic_ns(), **value},
                    default=default, allow_nan=False, separators=(",", ":")) + "\n").encode()
                stream.write(data)
                digest.update(data)
                rows += 1
                last = seq
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException as exc:
        error = repr(exc)
    finally:
        sock.close()
        args.socket.unlink(missing_ok=True)
        report = {"kind": "isolated_diagnostics_v1", "rows": rows, "last_sequence": last,
            "gaps": gaps, "error": error, "records_sha256": digest.hexdigest(),
            "bytes": (args.output / "diagnostics.jsonl").stat().st_size,
            "started_ns": begin, "ended_ns": time.monotonic_ns(),
            "drained_and_fsynced": error is None and stopping[0],
            "requires_sender_count_reconciliation": True}
        (args.output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    return 1 if error else 0


if __name__ == "__main__":
    raise SystemExit(main())
