"""Build/test the power-disconnected ownership probe in NX RAM; export to PC."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
from build_nx_hardware_link import payload


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ("host","identity","known-hosts","output"):p.add_argument("--"+k,required=True)
    p.add_argument("--ssh",default="ssh");a=p.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    request=payload();request.update(action="check",export_offboard_bench=True)
    (out/"source_manifest.json").write_text(json.dumps({k:v for k,v in request.items() if k!="zip_base64"},indent=2))
    source=Path(__file__).with_name("nx_hardware_link_build_remote.py").read_text()
    cmd=[a.ssh,"-T","-o","BatchMode=yes","-o","StrictHostKeyChecking=yes","-o","IdentitiesOnly=yes","-o","ConnectTimeout=8",
         "-i",a.identity,"-o","UserKnownHostsFile="+str(Path(a.known_hosts).resolve()),"jetson@"+a.host,
         shlex.join(["env","PYTHONDONTWRITEBYTECODE=1","python3","-u","-c",source])]
    child=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    child.stdin.write(json.dumps(request).encode());child.stdin.close()
    exported=ready=cleaned=False
    with (out/"build.log").open("wb") as log:
        for line in child.stdout:
            if line.startswith(b'{"event": "offboard_binary"'):
                row=json.loads(line);data=base64.b64decode(row.pop("base64"),validate=True)
                if hashlib.sha256(data).hexdigest()!=row["sha256"] or row["source_sha256"]!=request["sha256"]:raise ValueError("Export identity mismatch")
                (out/"px4_offboard_bench").write_bytes(data);(out/"binary_manifest.json").write_text(json.dumps(row,indent=2));exported=True
                line=(json.dumps(row)+"\n").encode()
            if line.startswith(b"HARDWARE_LINK_BUILD_READY "):
                marker=json.loads(line.split(b" ",1)[1]);ready=marker.get("offboard_mock_tests_pass") is True and marker["source_sha256"]==request["sha256"]
                (out/"build_report.json").write_text(json.dumps(marker,indent=2))
            if b'"ram_work_removed": true' in line:cleaned=True
            log.write(line);log.flush();print(line.decode(errors="replace"),end="",flush=True)
    if child.wait() or not(exported and ready and cleaned):raise RuntimeError("Build/test/export incomplete")


if __name__=="__main__":main()
