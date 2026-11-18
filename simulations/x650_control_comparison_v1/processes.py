"""Audit local Linux simulation processes and stop the owned closure bridge group."""
from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import time


def stop_process_group(process: subprocess.Popen):
    """Only use for a Popen created with start_new_session=True by this runner."""
    def send(sig):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass
    send(signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        send(signal.SIGTERM)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            send(signal.SIGKILL)
            process.wait(timeout=3)
    finally:
        # ros2 run can exit before its actual bridge child. The owned group
        # remains the cleanup authority even after the launcher has exited.
        send(signal.SIGTERM)


TRIAL_ENV = "AMSIM_X650_TRIAL_ID"


def _runtime_role(args):
    if not args:
        return None
    executable = Path(args[0]).name
    if executable in {"px4", "MicroXRCEAgent", "parameter_bridge", "rosbag2_transport"}:
        return executable
    # /usr/bin/gz uses Process.setproctitle, flattening the absolute world
    # path into argv[0]. Inspect its command prefix before taking the basename.
    if args[0].split(maxsplit=2)[:2] == ["gz", "sim"]:
        return "gazebo"
    if executable.startswith("gz-sim") or executable == "gz" and "sim" in args[1:3]:
        return "gazebo"
    if any(Path(arg).name == "gz" for arg in args[1:3]) and "sim" in args[1:4]:
        return "gazebo"
    if "amsim.runtime.sitl_supervisor" in args:
        return "sitl_supervisor"
    if "parameter_bridge" in args and "ros_gz_bridge" in args:
        return "parameter_bridge_launcher"
    if any(Path(arg).name == "ros2" for arg in args[:2]) and "bag" in args[1:4] and "record" in args:
        return "rosbag_recorder"
    if any(Path(arg).name == "run_flight.py" for arg in args[:2]):
        return "scenario"
    return None


def process_snapshot(partition, proc_root=Path("/proc")):
    """Read process identity without signalling anything or archiving full environments."""
    rows, errors = [], []
    denied = 0
    for proc in proc_root.iterdir():
        if not proc.name.isdecimal() or int(proc.name) == os.getpid():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            stat = (proc/"stat").read_text().rsplit(")", 1)[-1].split()
            if stat[0] in {"Z", "X"}:
                continue
            args = os.fsdecode((proc/"cmdline").read_bytes()).rstrip("\0").split("\0")
            role = _runtime_role(args)
            try:
                env = dict(entry.split(b"=", 1) for entry in (proc/"environ").read_bytes().split(b"\0") if b"=" in entry)
            except PermissionError:
                # Same-user protected GUI processes may be unreadable. Fail closed
                # for a recognizable simulator process with unknown partition.
                denied += 1
                if role:
                    errors.append({"pid": int(proc.name), "role": role, "error": "environment unreadable"})
                continue
            token = os.fsdecode(env.get(TRIAL_ENV.encode(), b""))
            gz_partition = os.fsdecode(env.get(b"GZ_PARTITION", b""))
            if gz_partition != partition or not (token or role):
                continue
            # pid + Linux start ticks distinguish PID reuse. pgid/sid survive reparenting.
            rows.append({"pid": int(proc.name), "ppid": int(stat[1]), "pgid": int(stat[2]),
                         "sid": int(stat[3]), "start_ticks": int(stat[19]), "role": role or "trial_child",
                         "executable": Path(args[0].split(maxsplit=1)[0]).name if args[0] else None,
                         "trial_id": token or None,
                         "partition": gz_partition, "scenario": os.fsdecode(env.get(b"AMSIM_SCENARIO_NAME", b"")),
                         "run_dir": os.fsdecode(env.get(b"AMSIM_RUN_DIR", b""))})
        except (FileNotFoundError, ProcessLookupError):
            continue  # process exited during enumeration
        except OSError as exc:
            errors.append({"pid": int(proc.name), "error": str(exc)})
    return {"wall_time_ns": time.time_ns(), "uid": os.getuid(), "partition": partition,
            "load_average": list(os.getloadavg()), "unreadable_environments": denied,
            "processes": sorted(rows, key=lambda item: item["pid"]), "errors": errors,
            "clean": not rows and not errors}


def wait_for_exit_snapshot(partition, timeout=3.):
    """Allow exiting children to finish and retain any observed transient matches."""
    deadline = time.monotonic() + timeout
    snapshot = process_snapshot(partition)
    first = snapshot
    while not snapshot["clean"] and time.monotonic() < deadline:
        time.sleep(.1)
        snapshot = process_snapshot(partition)
    if first is not snapshot:
        snapshot["initial_processes"] = first["processes"]
        snapshot["initial_errors"] = first["errors"]
    return snapshot
