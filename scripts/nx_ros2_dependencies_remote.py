"""Approved user-directory dependency build; invoked over SSH stdin, not installed.

Only compiled runtime/code is persistent. Source, build files and logs use tmpfs.
Does not open the FC, use sudo, configure networking, or edit startup/shell files.
"""
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
    install = Path.home() / "flyarm/ros2_hw_runtime/v1"
    if install.exists() and any(install.iterdir()):
        raise RuntimeError("Installation directory is not empty; refusing to overwrite")
    work = Path(tempfile.mkdtemp(prefix="flyarm_ros2_build_", dir="/dev/shm"))
    try:
        extracted = {}
        for name in ("agent", "px4_msgs"):
            item = request[name]
            data = base64.b64decode(item["zip_base64"])
            if hashlib.sha256(data).hexdigest() != item["sha256"]:
                raise RuntimeError("Archive hash mismatch: " + name)
            target = work / name
            target.mkdir()
            with zipfile.ZipFile(io.BytesIO(data)) as archive:
                for member in archive.infolist():
                    dest = (target / member.filename).resolve()
                    if not dest.is_relative_to(target.resolve()):
                        raise RuntimeError("Unsafe archive path")
                    if (member.external_attr >> 16) & 0o170000 == 0o120000:
                        raise RuntimeError("Symlinks not allowed in source archive")
                archive.extractall(target)
            roots = list(target.iterdir())
            if len(roots) != 1 or not (roots[0] / "CMakeLists.txt").is_file():
                raise RuntimeError("Unexpected source layout")
            extracted[name] = roots[0]
        env = os.environ.copy()
        env.update(ROS_LOG_DIR=str(work / "ros_logs"), ROS_HOME=str(work / "ros_home"),
                   PYTHONDONTWRITEBYTECODE="1", CMAKE_BUILD_PARALLEL_LEVEL="3")
        install.mkdir(parents=True, exist_ok=True)

        def run(argv):
            print("BUILD_STEP", shlex.join(map(str, argv)), flush=True)
            command = "source /opt/ros/humble/setup.bash && " + shlex.join(map(str, argv))
            subprocess.run(["bash", "-c", command], env=env, cwd=work, check=True)

        run(["cmake", "-S", extracted["agent"], "-B", work / "agent_build",
             "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_INSTALL_PREFIX=" + str(install / "agent"),
             "-DUAGENT_SUPERBUILD=OFF", "-DUAGENT_USE_SYSTEM_FASTDDS=ON",
             "-DUAGENT_USE_SYSTEM_FASTCDR=ON", "-DUAGENT_USE_SYSTEM_LOGGER=ON",
             "-DUAGENT_CED_PROFILE=OFF", "-DUAGENT_P2P_PROFILE=OFF",
             "-DUAGENT_SOCKETCAN_PROFILE=OFF", "-DUAGENT_BUILD_TESTS=OFF"])
        run(["cmake", "--build", work / "agent_build", "--parallel", "3"])
        run(["cmake", "--install", work / "agent_build"])
        run(["colcon", "--log-base", work / "colcon_logs", "build",
             "--base-paths", extracted["px4_msgs"], "--build-base", work / "ros_build",
             "--install-base", install / "ros", "--merge-install", "--executor", "sequential",
             "--cmake-args", "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF"])
        manifest = {name: {k: v for k, v in request[name].items() if k != "zip_base64"}
                    for name in ("agent", "px4_msgs")}
        manifest.update(installation=str(install), ros_distro="humble", system_changes=False)
        (install / "source_manifest.json").write_text(json.dumps(manifest, indent=2))
        print("DEPENDENCIES_READY", json.dumps(manifest), flush=True)
    finally:
        # work is exclusively created by this invocation under the literal tmpfs root.
        if work.resolve().parent == Path("/dev/shm") and work.name.startswith("flyarm_ros2_build_"):
            shutil.rmtree(work)


if __name__ == "__main__":
    main()
