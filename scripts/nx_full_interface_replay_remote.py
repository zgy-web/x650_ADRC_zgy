"""RAM-only offline replay. No install, ROS node, agent, USB or publishers."""
import base64
import hashlib
import io
import json
import os
from pathlib import Path
import resource
import shlex
import subprocess
import sys
import tempfile
import zipfile


def emit(**value):
    print(json.dumps(value,allow_nan=False),flush=True)


def main():
    request=json.load(sys.stdin)
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    raw=base64.b64decode(request["archive"],validate=True)
    if hashlib.sha256(raw).hexdigest()!=request["archive_sha256"]:
        raise ValueError("Source archive identity differs")
    libs=request["libraries"]
    def identities():
        return {key:hashlib.sha256(Path(v["path"]).read_bytes()).hexdigest() for key,v in libs.items()}
    expected={k:v["sha256"] for k,v in libs.items()}
    if identities()!=expected:raise ValueError("Previously verified libraries changed")
    runtime=Path.home()/"flyarm/ros2_hw_runtime/v1"
    results=[]
    with tempfile.TemporaryDirectory(prefix="flyarm_full_replay_",dir="/dev/shm") as tmp:
        root=Path(tmp)
        with zipfile.ZipFile(io.BytesIO(raw)) as archive:
            if set(archive.namelist())!=set(request["files"]):raise ValueError("Archive inventory differs")
            for name in archive.namelist():
                if not (root/name).resolve().is_relative_to(root):raise ValueError("Archive path escapes RAM")
                data=archive.read(name)
                if hashlib.sha256(data).hexdigest()!=request["files"][name]:raise ValueError("Archive member differs")
                p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
        env=os.environ.copy()
        env.update(PYTHONDONTWRITEBYTECODE="1",TMPDIR=tmp,ROS_HOME=tmp+"/ros_home",ROS_LOG_DIR=tmp+"/ros_logs",
                   ROS_DOMAIN_ID="210",ROS_LOCALHOST_ONLY="1",OPENBLAS_NUM_THREADS="1",OMP_NUM_THREADS="1")
        prefix="source /opt/ros/humble/setup.bash && source "+shlex.quote(str(runtime/"ros/local_setup.bash"))+" && "
        def run(args,name,timeout=240):
            emit(event="phase",name=name)
            p=subprocess.run(["bash","-c",prefix+shlex.join(map(str,args))],env=env,cwd=tmp,
                             stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=timeout)
            emit(event="artifact",name=name+".log",bytes_base64=base64.b64encode(p.stdout).decode(),
                 sha256=hashlib.sha256(p.stdout).hexdigest(),exit_code=p.returncode)
            if p.returncode:raise RuntimeError(name+" failed with exit "+str(p.returncode))
            return p.stdout
        source=root/"control_packages/x650_mpc_controller_v1"
        build=root/"build"
        run(["cmake","-S",source,"-B",build,"-DCMAKE_BUILD_TYPE=Release","-DBUILD_TESTING=ON",
             "-DX650_MPC_BUILD_HARDWARE_REPLAY=ON","-DX650_MPC_BUILD_REPLAY=OFF",
             "-DX650_NATIVE_INCLUDE_DIR="+str(root/"bound_native"),
             "-DX650_BACKEND_LIBRARY_DIR="+str(Path(libs["core"]["path"]).parent),
             "-DX650_TS20_LIBRARY="+libs["predictor"]["path"],"-DCMAKE_CXX_FLAGS=-ffp-contract=off"],"configure")
        run(["cmake","--build",build,"-j2"],"build")
        run(["ctest","--test-dir",build,"--output-on-failure"],"ctest")
        executable=build/"x650_hardware_replay"
        emit(event="build_identity",executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
             source_archive_sha256=request["archive_sha256"],bound_header_sha256=request["files"]["bound_native/mpc_core.hpp"],
             math_libraries=expected,ram_only=True)
        for item in request["inputs"]:
            # Failed rows are retained in the output; the overall command fails.
            output=run([executable,root/"inputs"/item["file"],item["sha256"]],item["name"],timeout=300)
            r=json.loads(output)
            if not r.get("passed") or not r.get("complete") or r.get("row_count")!=item["count"]:
                raise RuntimeError("Incomplete replay result")
            results.append(dict(name=item["name"],count=r["row_count"],passed=True))
        if identities()!=expected:raise ValueError("Algorithm libraries changed during replay")
    emit(event="cleanup",ram_removed=not root.exists(),persistent_files_created=[],libraries_unchanged=True)
    emit(event="complete",passed=True,results=results,publishers_created=0,physical_validation=False)


if __name__=="__main__":
    try:main()
    except Exception as e:
        emit(event="failure",error=repr(e));sys.exit(1)
