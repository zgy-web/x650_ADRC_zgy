"""Explicitly approved, temporary USB MAVLink-to-XRCE state-only session.

This is a separate provisioning tool, never an extension of the read-only USB
probe's send allowlist. It changes runtime USB ownership, not saved parameters.
No ARM, Offboard, VehicleCommand or actuator publisher exists in this program.
After the session, rebooting the FC restores its configured USB MAVLink startup.
"""
import base64
import fcntl
import hashlib
import json
import os
from pathlib import Path
import resource
import re
import select
import shlex
import subprocess
import sys
import tempfile
import termios
import threading
import time
import tty
import types
import zlib

DEVICE = "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00"
SWITCH = "uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600; mavlink stop -d /dev/ttyACM0"
QUERIES = ("ver all", "uxrce_dds_client status", "mavlink status", "param show SYS_USB_AUTO")
OUTPUT_LOCK = threading.Lock()


def emit(**kwargs):
    with OUTPUT_LOCK:
        print(json.dumps(kwargs, allow_nan=False), flush=True)


class Console:
    def __init__(self, dialect):
        self.d = dialect
        self.mav = dialect.MAVLink(None, srcSystem=245, srcComponent=191)
        self.mav.robust_parsing = True
        self.hb = None
        self.hb_time = 0
        self.fd = None
        self.original = None
        self.exclusive = False

    def __enter__(self):
        self.fd = os.open(DEVICE, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            self.exclusive = True
            self.original = termios.tcgetattr(self.fd)
            tty.setraw(self.fd)
            self.read(2.5)
            self.guard()
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *unused):
        if self.fd is not None:
            try:
                if self.original is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.original)
                if self.exclusive:
                    fcntl.ioctl(self.fd, termios.TIOCNXCL)
            finally:
                os.close(self.fd)
                self.fd = None

    def guard(self):
        if self.hb is None or time.monotonic()-self.hb_time > 1.5 or self.hb.base_mode & 128:
            raise RuntimeError("Fresh disarmed FC heartbeat required")

    def read(self, seconds):
        end = time.monotonic()+seconds
        fragments = []
        while time.monotonic() < end:
            if not select.select([self.fd], [], [], .02)[0]:
                continue
            try:
                data = os.read(self.fd, 16384)
            except BlockingIOError:
                continue
            for msg in self.mav.parse_buffer(data) or []:
                if msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1:
                    self.hb = msg
                    self.hb_time = time.monotonic()
                if msg.get_type() == "SERIAL_CONTROL" and msg.device == 10 and msg.count:
                    fragments.append(bytes(msg.data[:msg.count]).decode("utf-8", errors="replace"))
        return "".join(fragments)

    def shell(self, command):
        if command not in ("", SWITCH, *QUERIES):
            raise ValueError("Console command is not in fixed provisioning allowlist")
        self.guard()
        raw = (command + "\n").encode("ascii")
        for i in range(0, len(raw), 70):
            part = raw[i:i+70]
            packet = self.d.MAVLink_serial_control_message(10, 6, 0, 0, len(part), list(part.ljust(70, b"\0"))).pack(self.mav)
            self.mav.seq = (self.mav.seq+1) & 255
            end = time.monotonic()+1
            while packet:
                if time.monotonic() > end:
                    raise TimeoutError("Serial console write")
                if select.select([], [self.fd], [], .05)[1]:
                    try:
                        packet = packet[os.write(self.fd, packet):]
                    except BlockingIOError:
                        pass
        emit(event="console_send", command=command)


def stop(process):
    if process and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


def preflight_and_switch(request):
    source = zlib.decompress(base64.b64decode(request["dialect_zlib_base64"]))
    if hashlib.sha256(source).hexdigest() != request["dialect_sha256"]:
        raise ValueError("Dialect hash mismatch")
    os.environ.pop("MAV_IGNORE_CRC", None)
    dialect = types.ModuleType("flyarm_memory_mavlink")
    sys.modules[dialect.__name__] = dialect
    exec(compile(source, "<memory:mavlink>", "exec"), dialect.__dict__)
    with Console(dialect) as console:
        console.shell("")
        console.read(.5)
        replies = {}
        for query in QUERIES:
            console.shell(query)
            replies[query] = console.read(2)
            emit(event="console_reply", command=query, text=replies[query])
        if request["expected_firmware_commit"] not in replies["ver all"]:
            raise RuntimeError("Unexpected firmware commit; do not switch")
        if "not running" not in replies["uxrce_dds_client status"]:
            raise RuntimeError("DDS client already active or status unrecognized")
        if "serial (/dev/ttyACM0" not in replies["mavlink status"]:
            raise RuntimeError("Expected USB MAVLink instance not found")
        if not re.search(r"SYS_USB_AUTO[^\r\n]*:\s*2\b", replies["param show SYS_USB_AUTO"]):
            raise RuntimeError("USB reboot recovery mode is not confirmed")
        console.shell(SWITCH)
        # The complete NSH line is queued before USB MAVLink closes. Do not send
        # any further MAVLink frame on a port that may now carry XRCE.
        time.sleep(.3)
    emit(event="usb_switch_requested", persistent_parameter_changes=False)


def main():
    request = json.load(sys.stdin)
    if request.get("approved_temporary_usb_switch") is not True:
        raise ValueError("Explicit temporary USB switch authorization is required")
    if not re.fullmatch(r"[0-9a-f]{40}", request.get("expected_firmware_commit", "")):
        raise ValueError("A full 40-character expected firmware commit is required")
    root = Path.home() / "flyarm/ros2_hw_runtime/v1"
    agent_path = root / "agent/bin/MicroXRCEAgent"
    probe_path = root / "probe/lib/flyarm_hardware_state/px4_state_probe"
    if not agent_path.is_file() or not probe_path.is_file():
        raise RuntimeError("Build and test the dependencies and probe before switching USB")
    marker = json.loads((root / "probe/flyarm_probe_source.json").read_text())
    if marker.get("build_complete") is not True:
        raise RuntimeError("Probe build/test marker is incomplete")
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    # An already switched USB must never be probed using MAVLink again. This
    # mode needs an explicitly confirmed existing DDS session from the caller.
    if request.get("observe_existing_dds") is True:
        emit(event="existing_dds_observation", firmware_identity="from_previous_switch_not_requeried",
             expected_firmware_commit=request["expected_firmware_commit"])
    else:
        preflight_and_switch(request)
    # 15 seconds for session setup/discovery, followed by 60 seconds of stats.
    # The result still fails if telemetry never arrives; no automatic bypass.
    duration_s, settle_s = 75.0, 15.0
    with tempfile.TemporaryDirectory(prefix="flyarm_state_session_", dir="/dev/shm") as tmp:
        env = os.environ.copy()
        env.update(ROS_LOG_DIR=tmp+"/ros_logs", ROS_HOME=tmp+"/ros_home",
                   PYTHONDONTWRITEBYTECODE="1", ROS_DOMAIN_ID="0",
                   LD_LIBRARY_PATH=str(root/"agent/lib")+":/opt/ros/humble/lib:"+env.get("LD_LIBRARY_PATH", ""))
        agent = None
        observer = None
        try:
            agent = subprocess.Popen([str(agent_path), "serial", "--dev", DEVICE, "-b", "921600", "-v", "4"],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, cwd=tmp)
            def forward_agent():
                for line in agent.stdout:
                    emit(event="agent", text=line.decode("utf-8", errors="replace").rstrip())
            thread = threading.Thread(target=forward_agent, daemon=True)
            thread.start()
            command = ("source /opt/ros/humble/setup.bash && source " + shlex.quote(str(root/"ros/local_setup.bash")) +
                       " && source " + shlex.quote(str(root/"probe/local_setup.bash")) + " && exec " +
                       shlex.join([str(probe_path), "--ros-args", "--disable-external-lib-logs", "--disable-rosout-logs",
                                   "-p", "duration_s:="+str(duration_s), "-p", "settle_s:="+str(settle_s)]))
            observer = subprocess.Popen(["bash", "-c", command], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, cwd=tmp)
            output, _ = observer.communicate(timeout=duration_s+15)
            with OUTPUT_LOCK:
                print(output.decode("utf-8", errors="replace"), end="", flush=True)
            result = observer.returncode
            emit(event="session_finished", probe_exit=result, agent_exited_early=agent.poll() is not None,
                 persistent_output_files=[], usb_recovery="Stop Agent and reboot the disarmed FC to restore configured MAVLink USB")
            return result
        finally:
            stop(observer)
            stop(agent)


if __name__ == "__main__":
    sys.exit(main())
