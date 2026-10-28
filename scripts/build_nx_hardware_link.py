"""Compile/test hardware link in tmpfs; optional separately approved user install."""
import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import shlex
import subprocess
import sys
import zipfile


def payload():
    source = Path(__file__).resolve().parents[1] / "ros2_ws/src/flyarm_hardware_state"
    files, buffer = {}, io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(source.rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts:
                name = path.relative_to(source.parent).as_posix()
                data = path.read_bytes()
                files[name] = hashlib.sha256(data).hexdigest()
                info = zipfile.ZipInfo(name, date_time=(2026, 1, 1, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                archive.writestr(info, data)
    raw = buffer.getvalue()
    return dict(zip_base64=base64.b64encode(raw).decode(), sha256=hashlib.sha256(raw).hexdigest(), files=files)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", required=True)
    p.add_argument("--user", default="jetson")
    p.add_argument("--ssh", default="ssh")
    p.add_argument("--identity", required=True)
    p.add_argument("--known-hosts", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--action", choices=("check", "install"), default="check")
    p.add_argument("--approved-user-deployment", action="store_true")
    a = p.parse_args()
    if a.action == "install" and not a.approved_user_deployment:
        p.error("Persistent user directory installation requires explicit authorization")
    request = payload()
    request.update(action=a.action, approved_user_deployment=a.approved_user_deployment)
    source = Path(__file__).with_name("nx_hardware_link_build_remote.py").read_text(encoding="utf-8")
    args = [a.ssh, "-T", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", "-o", "IdentitiesOnly=yes",
            "-o", "ConnectTimeout=8", "-i", a.identity, "-o", "UserKnownHostsFile=" + str(a.known_hosts.resolve()),
            a.user + "@" + a.host, shlex.join(["env", "PYTHONDONTWRITEBYTECODE=1", "python3", "-u", "-c", source])]
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open("xb") as log:
        child = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        child.stdin.write(json.dumps(request).encode())
        child.stdin.close()
        for line in child.stdout:
            log.write(line)
            log.flush()
            print(line.decode("utf-8", errors="replace"), end="", flush=True)
        return child.wait()


if __name__ == "__main__":
    sys.exit(main())
