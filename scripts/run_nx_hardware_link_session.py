"""PC recorder for an explicitly approved, disarmed PX4 diagnostic roundtrip."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import secrets
import shlex
import subprocess
import sys
import zlib


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", required=True)
    p.add_argument("--user", default="jetson")
    p.add_argument("--ssh", default="ssh")
    p.add_argument("--identity", required=True)
    p.add_argument("--known-hosts", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--source-sha256", required=True)
    p.add_argument("--usb-mode", choices=("fresh-mavlink", "existing-dds"), required=True)
    p.add_argument("--dialect-source", type=Path)
    p.add_argument("--approved-diagnostic-command", action="store_true", required=True)
    p.add_argument("--bench-power-disconnected", action="store_true", required=True)
    a = p.parse_args()
    helper = Path(__file__).with_name("px4_usb_dds_session_remote.py").read_bytes()
    request = dict(approved_diagnostic_command=True, bench_power_disconnected=True, usb_mode=a.usb_mode,
                   source_sha256=a.source_sha256, source_component=191+secrets.randbelow(30),
                   usb_helper_base64=base64.b64encode(helper).decode(), usb_helper_sha256=hashlib.sha256(helper).hexdigest(),
                   expected_firmware_commit="d6f12ad1c4f70ad3230afd7d86e971421e02fef4")
    if a.usb_mode == "fresh-mavlink":
        if not a.dialect_source:
            p.error("fresh-mavlink needs --dialect-source")
        data = a.dialect_source.read_bytes()
        request.update(dialect_zlib_base64=base64.b64encode(zlib.compress(data)).decode(), dialect_sha256=hashlib.sha256(data).hexdigest())
    remote = Path(__file__).with_name("nx_hardware_link_session_remote.py").read_text(encoding="utf-8")
    command = [a.ssh, "-T", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", "-o", "IdentitiesOnly=yes",
               "-o", "ConnectTimeout=8", "-i", a.identity, "-o", "UserKnownHostsFile="+str(a.known_hosts.resolve()),
               a.user+"@"+a.host, shlex.join(["env", "PYTHONDONTWRITEBYTECODE=1", "python3", "-u", "-c", remote])]
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open("xb") as log:
        child = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        child.stdin.write(json.dumps(request).encode())
        child.stdin.close()
        for line in child.stdout:
            log.write(line)
            log.flush()
            try:
                row = json.loads(line)
                if row.get("event") == "agent":
                    continue
                if row.get("schema") == "flyarm.hardware_link.v1" and not row.get("final") and int(row.get("elapsed_s", 0)) % 10:
                    continue
            except ValueError:
                pass
            print(line.decode("utf-8", errors="replace"), end="", flush=True)
        return child.wait()


if __name__ == "__main__":
    sys.exit(main())
