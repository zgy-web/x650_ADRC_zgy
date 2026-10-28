"""Build/test in NX RAM. Persistent installation is a separate approved action."""
import base64
import hashlib
import io
import json
import os
from pathlib import Path
import resource
import shlex
import shutil
import subprocess
import sys
import tempfile
import zipfile


def main():
    request = json.load(sys.stdin)
    action = request.get("action", "check")
    if action not in ("check", "install"):
        raise ValueError("Unknown build action")
    if action == "install" and request.get("approved_user_deployment") is not True:
        raise ValueError("User directory deployment requires approval")
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    data = base64.b64decode(request["zip_base64"], validate=True)
    digest = hashlib.sha256(data).hexdigest()
    if digest != request["sha256"]:
        raise ValueError("Source hash mismatch")
    runtime = Path.home() / "flyarm/ros2_hw_runtime/v1"
    if not (runtime / "ros/local_setup.bash").is_file():
        raise RuntimeError("Previously approved message runtime missing")
    work = Path(tempfile.mkdtemp(prefix="flyarm_hwlink_build_", dir="/dev/shm"))
    install = work / "install" if action == "check" else runtime / "link_check" / digest[:12]
    if action == "install" and install.exists():
        work.rmdir()
        raise RuntimeError("Refusing to overwrite an existing link-check installation")
    try:
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            for item in archive.infolist():
                if not (work / item.filename).resolve().is_relative_to(work.resolve()):
                    raise ValueError("Unsafe archive path")
                if (item.external_attr >> 16) & 0o170000 == 0o120000:
                    raise ValueError("Symlinks not allowed")
            archive.extractall(work)
        for name, expected in request["files"].items():
            if hashlib.sha256((work / name).read_bytes()).hexdigest() != expected:
                raise ValueError("Source file hash mismatch: " + name)
        env = os.environ.copy()
        env.update(ROS_LOG_DIR=str(work / "ros_log"), ROS_HOME=str(work / "ros_home"),
                   PYTHONDONTWRITEBYTECODE="1", CMAKE_BUILD_PARALLEL_LEVEL="2",
                   ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID="210", TMPDIR=str(work))
        def run(args, overlay=False):
            command = "source /opt/ros/humble/setup.bash && source " + shlex.quote(str(runtime / "ros/local_setup.bash"))
            if overlay:
                command += " && source " + shlex.quote(str(install / "local_setup.bash"))
            command += " && " + shlex.join(map(str, args))
            subprocess.run(["bash", "-c", command], cwd=work, env=env, check=True, timeout=180)
        run(["colcon", "--log-base", work/"logs", "build", "--base-paths", work/"flyarm_hardware_state",
             "--build-base", work/"build", "--install-base", install, "--merge-install",
             "--executor", "sequential", "--cmake-args", "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=ON"])
        run(["ctest", "--test-dir", work/"build/flyarm_hardware_state", "--output-on-failure"])
        run(["python3", work/"flyarm_hardware_state/test/probe_link_ros.py",
             install/"lib/flyarm_hardware_state/px4_link_probe"], overlay=True)
        setpoint_test=work/"flyarm_hardware_state/test/probe_setpoint_ros.py"
        if setpoint_test.is_file():
            run(["python3",setpoint_test,install/"lib/flyarm_hardware_state/px4_setpoint_probe"],overlay=True)
        manifest = dict(schema="flyarm.hardware_link_build.v1", action=action, source_sha256=digest,
                        files=request["files"], unit_tests_pass=True, mock_ros_tests_pass=True,
                        hardware_accessed=False, permanent_configuration_changes=False)
        if request.get("export_setpoint_probe") is True:
            if action!="check":raise ValueError("Executable export is RAM-only")
            binary=(install/"lib/flyarm_hardware_state/px4_setpoint_probe").read_bytes()
            print(json.dumps(dict(event="setpoint_binary",sha256=hashlib.sha256(binary).hexdigest(),
                                  base64=base64.b64encode(binary).decode(),source_sha256=digest)),flush=True)
            manifest["setpoint_mock_tests_pass"]=True
        if action == "install":
            (install / "flyarm_link_source.json").write_text(json.dumps(manifest, indent=2))
            manifest["install_path"] = str(install)
        print("HARDWARE_LINK_BUILD_READY " + json.dumps(manifest), flush=True)
    finally:
        if work.resolve().parent == Path("/dev/shm") and work.name.startswith("flyarm_hwlink_build_"):
            shutil.rmtree(work)
        print(json.dumps({"ram_work_removed": not work.exists()}), flush=True)


if __name__ == "__main__":
    main()
