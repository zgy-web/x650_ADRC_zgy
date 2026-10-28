"""Approved bench session: observe states and request ONE pre-arm check via DDS."""
import base64
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import shlex
import subprocess
import sys
import tempfile
import threading
import types


def main():
    request = json.load(sys.stdin)
    if request.get("approved_diagnostic_command") is not True or request.get("bench_power_disconnected") is not True:
        raise ValueError("Explicit diagnostic command and bench condition confirmation required")
    digest = request["source_sha256"]
    if not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise ValueError("Expected full source hash")
    helper = base64.b64decode(request["usb_helper_base64"], validate=True)
    if hashlib.sha256(helper).hexdigest() != request["usb_helper_sha256"]:
        raise ValueError("USB helper hash mismatch")
    usb = types.ModuleType("flyarm_temporary_usb_helper")
    exec(compile(helper, "<memory:usb_helper>", "exec"), usb.__dict__)
    root = Path.home() / "flyarm/ros2_hw_runtime/v1"
    install = root / "link_check" / digest[:12]
    marker = json.loads((install / "flyarm_link_source.json").read_text())
    if marker.get("source_sha256") != digest or not marker.get("unit_tests_pass") or not marker.get("mock_ros_tests_pass"):
        raise RuntimeError("Link check installation not qualified")
    agent_path, probe_path = root / "agent/bin/MicroXRCEAgent", install / "lib/flyarm_hardware_state/px4_link_probe"
    if not agent_path.is_file() or not probe_path.is_file():
        raise RuntimeError("Missing tested program")
    if subprocess.run(["pgrep", "-x", "MicroXRCEAgent"], stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError("An Agent is already running; do not compete for USB")
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    if request.get("usb_mode") == "fresh-mavlink":
        usb.preflight_and_switch(request)
    elif request.get("usb_mode") == "existing-dds":
        usb.emit(event="existing_dds_session", firmware_identity="prior_confirmed_session_not_requeried")
    else:
        raise ValueError("Explicit USB protocol selection required")
    duration, settle = 75.0, 15.0
    with tempfile.TemporaryDirectory(prefix="flyarm_hwlink_session_", dir="/dev/shm") as tmp:
        env = os.environ.copy()
        env.update(ROS_LOG_DIR=tmp+"/ros_logs", ROS_HOME=tmp+"/ros_home", TMPDIR=tmp,
                   PYTHONDONTWRITEBYTECODE="1", ROS_DOMAIN_ID="0",
                   LD_LIBRARY_PATH=str(root/"agent/lib")+":/opt/ros/humble/lib:"+env.get("LD_LIBRARY_PATH", ""))
        agent = probe = None
        try:
            usb.emit(event="hardware_link_started", source_sha256=digest, duration_s=duration, settle_s=settle,
                     allowed_command=401, maximum_command_count=1, flight_setpoint_publication=False,
                     source_component=request["source_component"], input_origin="physical_px4")
            agent = subprocess.Popen([str(agent_path), "serial", "--dev", usb.DEVICE, "-b", "921600", "-v", "4"],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, cwd=tmp)
            def forward():
                for line in agent.stdout:
                    usb.emit(event="agent", text=line.decode("utf-8", errors="replace").rstrip())
            thread = threading.Thread(target=forward, daemon=True)
            thread.start()
            command = ("source /opt/ros/humble/setup.bash && source " + shlex.quote(str(root/"ros/local_setup.bash")) +
                       " && source " + shlex.quote(str(install/"local_setup.bash")) + " && exec " +
                       shlex.join([str(probe_path), "--ros-args", "--disable-external-lib-logs", "--disable-rosout-logs",
                                   "-p", "mode:=diagnostic_ack", "-p", "duration_s:="+str(duration),
                                   "-p", "settle_s:="+str(settle), "-p", "source_component:="+str(request["source_component"])]))
            probe = subprocess.Popen(["bash", "-c", command], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, cwd=tmp)
            output, _ = probe.communicate(timeout=duration+15)
            with usb.OUTPUT_LOCK:
                print(output.decode("utf-8", errors="replace"), end="", flush=True)
            code = probe.returncode
            usb.emit(event="hardware_link_finished", probe_exit=code, agent_exited_early=agent.poll() is not None,
                     persistent_result_files=[], usb_recovery="Reboot disarmed FC to restore USB MAVLink")
            return code
        finally:
            usb.stop(probe)
            usb.stop(agent)
            usb.emit(event="hardware_link_cleanup", agent_stopped=agent is None or agent.poll() is not None,
                     probe_stopped=probe is None or probe.poll() is not None)


if __name__ == "__main__":
    sys.exit(main())
