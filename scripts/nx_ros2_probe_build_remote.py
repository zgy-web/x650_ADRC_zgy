"""Build only the approved observation package in tmpfs; install user-scoped code."""
import base64
import hashlib
import io
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import zipfile


def main():
    request = json.load(sys.stdin)
    data = base64.b64decode(request["zip_base64"])
    if hashlib.sha256(data).hexdigest() != request["sha256"]:
        raise RuntimeError("Source archive hash mismatch")
    root = Path.home() / "flyarm/ros2_hw_runtime/v1"
    if not (root / "ros/setup.bash").is_file():
        raise RuntimeError("Approved PX4 message dependency is missing")
    install = root / "probe"
    if install.exists() and not (install / "flyarm_probe_source.json").exists():
        raise RuntimeError("Refusing to overwrite an unrecognized directory")
    work = Path(tempfile.mkdtemp(prefix="flyarm_state_build_", dir="/dev/shm"))
    try:
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            for m in archive.infolist():
                if not (work / m.filename).resolve().is_relative_to(work.resolve()):
                    raise RuntimeError("Unsafe archive path")
                if (m.external_attr >> 16) & 0o170000 == 0o120000:
                    raise RuntimeError("Symlinks not allowed")
            archive.extractall(work)
        install.mkdir(parents=True, exist_ok=True)
        manifest = dict(source_sha256=request["sha256"], files=request["files"], build_complete=False)
        marker = install / "flyarm_probe_source.json"
        marker.write_text(json.dumps(manifest, indent=2))
        env = os.environ.copy()
        env.update(ROS_LOG_DIR=str(work / "ros_log"), ROS_HOME=str(work / "ros_home"),
                   PYTHONDONTWRITEBYTECODE="1", CMAKE_BUILD_PARALLEL_LEVEL="3")
        def run(args):
            command = ("source /opt/ros/humble/setup.bash && source " +
                       shlex.quote(str(root / "ros/local_setup.bash")) + " && " + shlex.join(map(str,args)))
            subprocess.run(["bash","-c",command],cwd=work,env=env,check=True)
        run(["colcon","--log-base",work/"logs","build","--base-paths",work/"flyarm_hardware_state",
             "--build-base",work/"build","--install-base",install,"--merge-install","--executor","sequential",
             "--cmake-args","-DCMAKE_BUILD_TYPE=Release","-DBUILD_TESTING=ON"])
        run(["ctest","--test-dir",work/"build/flyarm_hardware_state","--output-on-failure"])
        manifest["build_complete"] = True
        marker.write_text(json.dumps(manifest,indent=2))
        print("STATE_PROBE_BUILD_READY",json.dumps(manifest),flush=True)
    finally:
        if work.resolve().parent==Path("/dev/shm") and work.name.startswith("flyarm_state_build_"):
            shutil.rmtree(work)


if __name__=="__main__": main()
