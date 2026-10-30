"""Prepare source identity, run temporary offline C++ replay, save only on PC."""
import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import shlex
import subprocess
import tarfile
import zipfile


BOUND_HEADER="3ce12c395eb047167b5eda8fefb502fe56622e610cdfd3e82a4302f4a53c0093"


def payload(root,inputs,code_archive,library_report,mavlink_generated=None):
    manifest=json.loads((inputs/"manifest.json").read_text())
    report=json.loads(library_report.read_text())
    libraries=report["libraries"]
    if code_archive:
        with tarfile.open(code_archive,"r:gz") as a:
            header=a.extractfile("control_packages/x650_wholebody_v1/native/mpc_core.hpp").read()
    else:
        header=(root/"control_packages/x650_mpc_controller_v1/compat/abi4_20261027/mpc_core.hpp").read_bytes()
    if hashlib.sha256(header).hexdigest()!=BOUND_HEADER:raise ValueError("Bound ABI4 header differs")
    files={"bound_native/mpc_core.hpp":header}
    # The current benchmark JSON parser uses only the bound public structures.
    paths=[root/"control_packages/x650_mpc_controller_v1",root/"ros2_ws/src/flyarm_hardware_state/include"]
    for path in paths:
        for p in sorted(path.rglob("*")):
            if p.is_file() and (p.suffix in (".hpp",".cpp") or p.name=="CMakeLists.txt"):
                files[p.relative_to(root).as_posix()]=p.read_bytes()
    io_header="control_packages/x650_wholebody_v1/benchmark/include/benchmark_io.hpp"
    files[io_header]=(root/io_header).read_bytes()
    if mavlink_generated:
        generated=Path(mavlink_generated)
        dialect_manifest=json.loads((generated/'manifest.json').read_text())
        lock=json.loads((root/'configs/hardware/nx_mavlink_dialect_lock.json').read_text())
        if dialect_manifest['lock']!=lock:raise ValueError('MAVLink firmware/dialect identity differs')
        for name,sha in dialect_manifest['files'].items():
            if name.startswith('include/'):
                data=(generated/name).read_bytes()
                if hashlib.sha256(data).hexdigest()!=sha:raise ValueError('Generated MAVLink header changed')
                files['mavlink_generated/'+name]=data
        for p in (root/'hardware/nx/mavlink_uart').rglob('*'):
            if p.is_file() and p.suffix in ('.hpp','.cpp'):
                files[p.relative_to(root).as_posix()]=p.read_bytes()
    for item in manifest["inputs"]:
        data=(inputs/item["file"]).read_bytes()
        if hashlib.sha256(data).hexdigest()!=item["sha256"]:raise ValueError("Prepared data changed")
        files["inputs/"+item["file"]]=data
    buffer=io.BytesIO()
    with zipfile.ZipFile(buffer,"w",zipfile.ZIP_DEFLATED) as a:
        for name,data in sorted(files.items()):
            info=zipfile.ZipInfo(name,date_time=(2026,1,1,0,0,0));info.compress_type=zipfile.ZIP_DEFLATED;a.writestr(info,data)
    raw=buffer.getvalue()
    return dict(archive=base64.b64encode(raw).decode(),archive_sha256=hashlib.sha256(raw).hexdigest(),
                files={k:hashlib.sha256(v).hexdigest() for k,v in files.items()},inputs=manifest["inputs"],libraries=libraries,
                mavlink_wire_replay=bool(mavlink_generated))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--inputs",type=Path,required=True);p.add_argument("--output",type=Path,required=True)
    p.add_argument("--code-archive",type=Path,help="Optional original archive; default uses committed hash-bound compatibility header")
    p.add_argument("--library-report",type=Path,default=Path("artifacts/orin_pil_20261027/nx_no_fma_pil_build_report.json"))
    p.add_argument("--mavlink-generated",type=Path,help="Optional hash-bound generated dialect for additional offline wire verification")
    p.add_argument("--host",default="192.168.30.145");p.add_argument("--ssh",default="ssh")
    p.add_argument("--identity",required=True);p.add_argument("--known-hosts",required=True)
    a=p.parse_args();root=Path(__file__).resolve().parents[1]
    request=payload(root,a.inputs,a.code_archive,a.library_report,a.mavlink_generated)
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/"request_manifest.json").write_text(json.dumps({k:v for k,v in request.items() if k!="archive"},indent=2))
    source=Path(__file__).with_name("nx_full_interface_replay_remote.py").read_text()
    command=[a.ssh,"-T","-o","BatchMode=yes","-o","StrictHostKeyChecking=yes","-o","IdentitiesOnly=yes",
             "-o","ConnectTimeout=8","-i",a.identity,"-o","UserKnownHostsFile="+str(Path(a.known_hosts).resolve()),
             "jetson@"+a.host,shlex.join(["env","PYTHONDONTWRITEBYTECODE=1","python3","-u","-c",source])]
    child=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    output,stderr=child.communicate(json.dumps(request).encode(),timeout=1200)
    (a.output/"transport.jsonl").write_bytes(output);(a.output/"ssh_stderr.txt").write_bytes(stderr)
    events=[]
    for line in output.splitlines():
        event=json.loads(line);events.append(event)
        if event["event"]=="artifact":
            name=event["name"]
            if Path(name).name!=name:raise ValueError("Unsafe output name")
            data=base64.b64decode(event["bytes_base64"],validate=True)
            if hashlib.sha256(data).hexdigest()!=event["sha256"]:raise ValueError("Output hash mismatch")
            (a.output/name).write_bytes(data)
            print(json.dumps({k:v for k,v in event.items() if k!="bytes_base64"}))
        else:print(json.dumps(event))
    if child.returncode or not events or events[-1].get("event")!="complete" or events[-1].get("passed") is not True:
        raise RuntimeError("Remote replay incomplete; see retained PC logs")


if __name__=="__main__":main()
