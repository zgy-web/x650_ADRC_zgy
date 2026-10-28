"""PC launcher: build the explicitly approved observation-only C++ ROS package."""
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


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host",required=True)
    p.add_argument("--user",default="jetson")
    p.add_argument("--ssh",default="ssh")
    p.add_argument("--identity",required=True)
    p.add_argument("--known-hosts",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--apply-approved-user-directory",action="store_true",required=True)
    a=p.parse_args()
    source=Path(__file__).resolve().parents[1]/"ros2_ws/src/flyarm_hardware_state"
    files={}; buf=io.BytesIO()
    with zipfile.ZipFile(buf,"w",zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(source.rglob("*")):
            if path.is_file():
                name=path.relative_to(source.parent).as_posix()
                data=path.read_bytes(); files[name]=hashlib.sha256(data).hexdigest()
                archive.writestr(name,data)
    payload=dict(zip_base64=base64.b64encode(buf.getvalue()).decode("ascii"),
                 sha256=hashlib.sha256(buf.getvalue()).hexdigest(),files=files)
    remote=Path(__file__).with_name("nx_ros2_probe_build_remote.py").read_text(encoding="utf-8")
    args=[a.ssh,"-T","-o","BatchMode=yes","-o","StrictHostKeyChecking=yes","-o","IdentitiesOnly=yes",
          "-o","ConnectTimeout=8","-i",a.identity,"-o","UserKnownHostsFile="+str(a.known_hosts.resolve()),
          a.user+"@"+a.host,shlex.join(["env","PYTHONDONTWRITEBYTECODE=1","python3","-u","-c",remote])]
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open("xb") as log:
        process=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        process.stdin.write(json.dumps(payload).encode());process.stdin.close()
        for line in process.stdout:
            log.write(line);log.flush();print(line.decode("utf-8",errors="replace"),end="",flush=True)
        return process.wait()


if __name__=="__main__":sys.exit(main())
