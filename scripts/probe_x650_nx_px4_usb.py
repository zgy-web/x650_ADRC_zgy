"""Launch an observational USB probe on NX; write its report only on this PC.

Requires pymavlink on the launching computer. No pip install, source file, bytecode
cache or experiment report is created on NX. This does not launch MPC or ROS.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import inspect
import json
from pathlib import Path
import shlex
import subprocess
import sys
import zlib


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", default="jetson")
    parser.add_argument("--ssh", default="ssh")
    parser.add_argument("--identity", type=Path, required=True)
    parser.add_argument("--known-hosts", type=Path, required=True)
    parser.add_argument("--device", default="/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00")
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--dialect-source", type=Path, help="Optional locally generated, firmware-matched Python dialect")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 8 <= args.duration <= 120:
        parser.error("duration must be 8..120 seconds")
    if args.output.exists():
        parser.error("output already exists; choose a fresh report path")
    if args.dialect_source:
        source = args.dialect_source.read_bytes()
    else:
        from pymavlink.dialects.v20 import common
        source = inspect.getsource(common).encode("utf-8")
    remote = Path(__file__).with_name("px4_usb_readonly_remote.py").read_text(encoding="utf-8")
    payload = dict(device=args.device, duration_s=args.duration,
                   dialect_sha256=hashlib.sha256(source).hexdigest(),
                   dialect_zlib_base64=base64.b64encode(zlib.compress(source)).decode("ascii"))
    command = [args.ssh, "-T", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
               "-o", "IdentitiesOnly=yes", "-o", "ConnectTimeout=8",
               "-i", str(args.identity), "-o", "UserKnownHostsFile="+str(args.known_hosts.resolve()),
               args.user+"@"+args.host,
               shlex.join(["env", "PYTHONDONTWRITEBYTECODE=1", "python3", "-c", remote])]
    result = subprocess.run(command, input=json.dumps(payload).encode("ascii"),
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=args.duration+35)
    try:
        report = json.loads(result.stdout.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        report = dict(basic_link_pass=False, stdout=result.stdout.decode("utf-8", errors="replace"))
    report["launcher"] = dict(host=args.host, ssh_exit_code=result.returncode,
                              stderr=result.stderr.decode("utf-8", errors="replace"),
                              remote_probe_sha256=hashlib.sha256(remote.encode("utf-8")).hexdigest())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(json.dumps(dict(report=str(args.output.resolve()), basic_link_pass=report.get("basic_link_pass"),
                         checks=report.get("checks"), firmware=report.get("firmware", {}).get("semver"),
                         heartbeat=report.get("heartbeat"), error=report.get("error"),
                         stderr=report["launcher"]["stderr"]), indent=2))
    return 0 if result.returncode == 0 and report.get("basic_link_pass") else 2


if __name__ == "__main__":
    sys.exit(main())
