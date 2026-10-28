"""PC-side deployment of explicitly approved, user-scoped NX ROS dependencies.

Supply public pinned source archives and their manifest; no software download or
system package install runs on NX. All build output is retained on this PC only.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys

PINNED_PATH = Path(__file__).resolve().parents[1] / "configs/hardware/nx_ros2_state_dependencies_v1.json"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", required=True)
    ap.add_argument("--user", default="jetson")
    ap.add_argument("--ssh", default="ssh")
    ap.add_argument("--identity", required=True)
    ap.add_argument("--known-hosts", type=Path, required=True)
    ap.add_argument("--sources", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--apply-approved-user-directory", action="store_true", required=True)
    args = ap.parse_args()
    manifest = json.loads((args.sources / "dependencies.json").read_text())
    pinned = json.loads(PINNED_PATH.read_text())
    for name, expected in pinned.items():
        item = manifest[name]
        data = (args.sources / (name + ".zip")).read_bytes()
        if item != expected or hashlib.sha256(data).hexdigest() != expected["sha256"]:
            raise ValueError("Unmatched source archive: " + name)
        item["zip_base64"] = base64.b64encode(data).decode("ascii")
    remote = Path(__file__).with_name("nx_ros2_dependencies_remote.py").read_text(encoding="utf-8")
    command = [args.ssh, "-T", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
               "-o", "IdentitiesOnly=yes", "-o", "ConnectTimeout=8", "-i", args.identity,
               "-o", "UserKnownHostsFile=" + str(args.known_hosts.resolve()), args.user + "@" + args.host,
               shlex.join(["env", "PYTHONDONTWRITEBYTECODE=1", "python3", "-u", "-c", remote])]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as log:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        process.stdin.write(json.dumps(manifest).encode("ascii"))
        process.stdin.close()
        for line in process.stdout:
            log.write(line)
            log.flush()
            print(line.decode("utf-8", errors="replace"), end="", flush=True)
        return process.wait()


if __name__ == "__main__":
    sys.exit(main())
