"""Power-disconnected, disarmed ownership/stream-loss check; all results on PC.

The operator alone changes SA. No arming, mode-request, or parameter command.
Starts a reviewed RAM executable and temporary serial DDS, then stops both.
The original USB MAVLink path is restored only after all owned processes end.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import queue
import re
import shlex
import subprocess
import threading
import time

from px4_debug_console import DebugConsole, FIRMWARE, START_DDS, STOP_DDS, STOP_USB_MAVLINK, clean
from px4_imu_pole_check import guard
from px4_rc_switch_setup import boolean, number, fresh
from px4_motor_single_pulse import require_zero, dshot_channels

TOPICS = ("vehicle_status", "vehicle_control_mode", "offboard_control_mode", "vehicle_attitude_setpoint",
          "actuator_armed", "manual_control_switches", "manual_control_setpoint", "battery_status")
# Same USB command as the pinned cdcacm_autostart implementation.
RESTORE_USB_MAVLINK = "mavlink start -d /dev/ttyACM0 -m 2"
READS = frozenset(("ver all", "dshot status", "commander status", "mavlink status", "uxrce_dds_client status",
                   "param show SYS_USB_AUTO", "param show -q USB_MAV_MODE", "param show -q UXRCE_DDS_DOM_ID",
                   "listener timesync_status -n 1") +
                  tuple("listener " + x + " -n 1" for x in TOPICS))
SWITCHES = frozenset((START_DDS, STOP_DDS, STOP_USB_MAVLINK, RESTORE_USB_MAVLINK))
# This is a hold on the failed hardware path, not on offline contract tests.
# Remove only in a reviewed follow-up with a firmware/transport remediation and
# fresh power-disconnected acceptance evidence. There is no CLI bypass.
USB_SWITCH_INCIDENT = "docs/validation/nx_offboard_bench_20261030/README.md"


def require_live_transport_released():
    raise RuntimeError("USB MAVLink/DDS switching is quarantined after the 2026-10-30 "
                       "PX4 Hard Fault; see " + USB_SWITCH_INCIDENT)


def protocol_reply_ok(text):
    return "nsh>" in text and not re.search(
        r"\bERROR\b|\bfailed\b|\btimeout\b|\bforcing\b|Hard Fault|PANIC|Assertion failed",
        text, re.I)


class BenchConsole(DebugConsole):
    def __init__(self, port):
        super().__init__(port)
        self.switch_permit = None
        self.protocol_fault = False

    def query(self, command):
        if self.active_listener or (command not in READS and command != self.switch_permit):
            raise ValueError("Command outside approved bench scope")
        is_switch = command in SWITCHES
        self.switch_permit = None
        self.s.write((command + "\r").encode())
        text = clean(self.record(command, self.read(6)))
        if re.search(r"Hard Fault|PANIC|Assertion failed", text, re.I):
            self.protocol_fault = True
            raise RuntimeError("Flight-controller fault; further protocol writes disabled")
        if is_switch and not protocol_reply_ok(text):
            self.protocol_fault = True
            raise RuntimeError("Unconfirmed/forced protocol transition: " + command)
        if "nsh>" not in text: raise RuntimeError("Missing prompt: " + command)
        return text

    def switch(self, command):
        if command not in SWITCHES: raise ValueError("Not a temporary USB protocol operation")
        if self.protocol_fault: raise RuntimeError("Protocol fault latched; no further writes")
        require_live_transport_released()
        self.switch_permit = command
        return self.query(command)


def teardown_usb_session(c, *, stop_probe, release_agent, nx_cleaned):
    """Two-phase teardown. Never restart MAVLink after an uncertain DDS stop.

    Callbacks let tests inject lost confirmation, forced stops and cleanup
    failures without opening serial/SSH. Agent remains alive until STOP_DDS has
    completed; even on error, our NX processes are released and never restarted.
    """
    result = dict(dds_stop_graceful=False, dds_stopped=False,
                  usb_mavlink_restored=False, recovery_required=False)
    try:
        if not stop_probe():
            raise RuntimeError("NX publisher stop was not confirmed")
        if c.protocol_fault:
            raise RuntimeError("Flight-controller protocol fault already latched")
        reply = c.switch(STOP_DDS)
        if not protocol_reply_ok(reply):
            raise RuntimeError("DDS stop did not complete gracefully")
        result["dds_stop_graceful"] = True
        status = c.query("uxrce_dds_client status")
        result["dds_stopped"] = protocol_reply_ok(status) and "not running" in status
        if not result["dds_stopped"]:
            raise RuntimeError("DDS stopped state not confirmed")
    except Exception as exc:
        result["cleanup_error"] = repr(exc)
    finally:
        try:
            release_agent()
        except Exception as exc:
            result["agent_cleanup_error"] = repr(exc)
    result["nx_processes_cleaned"] = nx_cleaned()
    if ("cleanup_error" in result or "agent_cleanup_error" in result
            or not result["nx_processes_cleaned"]):
        result["recovery_required"] = True
        return result
    try:
        reply = c.switch(RESTORE_USB_MAVLINK)
        if not protocol_reply_ok(reply):
            raise RuntimeError("MAVLink restore command failed")
        status = c.query("mavlink status")
        result["usb_mavlink_restored"] = (protocol_reply_ok(status)
                                           and "serial (/dev/ttyACM0" in status)
        if not result["usb_mavlink_restored"]:
            raise RuntimeError("USB MAVLink presence not confirmed")
    except Exception as exc:
        result["cleanup_error"] = repr(exc)
        result["recovery_required"] = True
    # Presence alone is not a health/flight acceptance criterion.
    return result


def snapshot(c):
    row = {x: c.query("listener " + x + " -n 1") for x in TOPICS}
    row["dshot"] = c.query("dshot status")
    return row


def power_off_state(raw, *, allow_operator_arming=False):
    fresh(raw["vehicle_status"], 1.5)
    allowed_arms = (1, 2) if allow_operator_arming else (1,)
    if number(raw["vehicle_status"], "arming_state") not in allowed_arms or number(raw["vehicle_status"], "hil_state") != 0:
        raise ValueError("Unexpected armed/HIL state")
    fresh(raw["actuator_armed"], 1.5)
    if not allow_operator_arming and (boolean(raw["actuator_armed"], "armed") or not boolean(raw["actuator_armed"], "kill")):
        raise ValueError("Keep disarmed and SB kill engaged throughout")
    battery = clean(raw["battery_status"])
    # A USB-only boot can have no battery publisher. Physical power-off is an
    # explicit mandatory CLI attestation, never inferred from absent telemetry.
    unpublished = re.search(r"^never published$",battery,re.M) and "TOPIC:" not in battery
    if not unpublished:
        fresh(battery, 1.5)
        if boolean(battery, "connected") or not 0 <= number(battery, "voltage_v") < 1:
            raise ValueError("Power observation contradicts disconnected battery")
    fresh(raw["manual_control_setpoint"], .3)
    if not boolean(raw["manual_control_setpoint"], "valid") or number(raw["manual_control_setpoint"], "throttle") > -.95:
        raise ValueError("Keep RC valid and minimum throttle")
    fresh(raw["manual_control_switches"], 1.5)
    if not allow_operator_arming and number(raw["manual_control_switches"], "arm_switch") != 3:
        raise ValueError("Keep SF off")
    if allow_operator_arming:dshot_channels(raw["dshot"])
    else:require_zero(raw["dshot"])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("host", "identity", "known-hosts", "build", "output"): p.add_argument("--" + name, required=True)
    p.add_argument("--ssh", default="ssh"); p.add_argument("--port", default="COM13")
    p.add_argument("--scenario", choices=("manual_takeover", "stream_loss"), required=True)
    p.add_argument("--approved-offboard-bench", action="store_true", required=True)
    p.add_argument("--bench-power-disconnected", action="store_true", required=True)
    p.add_argument("--operator-arming-allowed", action="store_true", help="Separately confirmed RC arming with ESC power disconnected")
    a = p.parse_args()
    # Fail before opening COM13, SSH or creating any hardware-side process.
    require_live_transport_released()
    out = Path(a.output); out.mkdir(parents=True, exist_ok=False); build = Path(a.build)
    marker=json.loads((build/"build_report.json").read_text()); binary=json.loads((build/"binary_manifest.json").read_text())
    elf=(build/"px4_offboard_bench").read_bytes()
    if not all(marker.get(k) is True for k in ("unit_tests_pass", "mock_ros_tests_pass", "offboard_mock_tests_pass")):
        raise ValueError("Build/tests incomplete")
    if marker["source_sha256"]!=binary["source_sha256"] or hashlib.sha256(elf).hexdigest()!=binary["sha256"]:
        raise ValueError("Tested executable identity differs")
    (out/"build_identity.json").write_text(json.dumps(dict(build=marker,binary=binary),indent=2))
    c=BenchConsole(a.port); child=thread=None; events=[]; q=queue.Queue(); switched=False
    result=dict(complete=False,scenario=a.scenario,firmware=FIRMWARE,rounds=[],operator_power_off=True,
                arm_commands=0,mode_commands=0,parameter_writes=0,physical_stop_verified=False,
                operator_arming_allowed=a.operator_arming_allowed)
    def await_event(predicate, seconds, diagnose=False):
        end=time.monotonic()+seconds;next_probe=time.monotonic()+3
        while time.monotonic()<end:
            if diagnose and time.monotonic()>=next_probe:
                diag={x:c.query(x) for x in ("uxrce_dds_client status","listener timesync_status -n 1")}
                result.setdefault("dds_initialization",[]).append(diag)
                print(json.dumps(dict(event="dds_initialization",readback=diag)),flush=True)
                next_probe=time.monotonic()+5
            try:e=q.get(timeout=.2)
            except queue.Empty:
                if child.poll() is not None: break
                continue
            if e.get("event")=="failure":raise RuntimeError(e["error"])
            if predicate(e):return e
        raise TimeoutError("Expected NX event not received")
    try:
        c.open()
        if FIRMWARE not in c.query("ver all"):raise ValueError("Unexpected firmware")
        result["before_guard"]=guard(c,require_kill=True,allow_unpublished_battery=True)
        result["before"]=snapshot(c);power_off_state(result["before"])
        if number(result["before"]["vehicle_status"],"nav_state")!=1 or number(result["before"]["manual_control_switches"],"mode_slot")!=1:
            raise ValueError("Start in SA-up Altitude")
        if "not running" not in c.query("uxrce_dds_client status"):raise ValueError("Do not disturb existing DDS")
        if not re.search(r"SYS_USB_AUTO[^\r\n]*:\s*2\b",c.query("param show SYS_USB_AUTO")):raise ValueError("Unexpected USB recovery policy")
        if re.findall(r"(?m)^\s*(-?\d+)\s*$",c.query("param show -q USB_MAV_MODE"))!=["2"]:raise ValueError("Unexpected original USB MAVLink mode")
        if re.findall(r"(?m)^\s*(-?\d+)\s*$",c.query("param show -q UXRCE_DDS_DOM_ID"))!=["0"]:raise ValueError("Unexpected DDS domain")
        if "serial (/dev/ttyACM0" not in c.query("mavlink status"):raise ValueError("Expected original USB MAVLink")
        c.switch(STOP_USB_MAVLINK);switched=True
        request=dict(approved_offboard_bench=True,bench_power_disconnected=True,approved_usb_dds_restore=True,scenario=a.scenario,
                     binary_base64=base64.b64encode(elf).decode(),binary_sha256=binary["sha256"],source_sha256=binary["source_sha256"])
        source=Path(__file__).with_name("nx_offboard_bench_session_remote.py").read_text()
        cmd=[a.ssh,"-T","-o","BatchMode=yes","-o","StrictHostKeyChecking=yes","-o","IdentitiesOnly=yes","-o","ConnectTimeout=8",
             "-i",a.identity,"-o","UserKnownHostsFile="+str(Path(a.known_hosts).resolve()),"jetson@"+a.host,
             shlex.join(["env","PYTHONDONTWRITEBYTECODE=1","python3","-u","-c",source])]
        child=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        child.stdin.write((json.dumps(request)+"\n").encode());child.stdin.flush()
        def drain():
            with (out/"remote.jsonl").open("wb") as log:
                for line in child.stdout:
                    log.write(line);log.flush()
                    try:e=json.loads(line)
                    except ValueError:e=dict(event="transport_text",text=line.decode(errors="replace"))
                    events.append(e);q.put(e)
                    if e.get("event")!="agent" and e.get("payload",{}).get("event")!="bench_state":print(json.dumps(e),flush=True)
        thread=threading.Thread(target=drain,daemon=True);thread.start()
        await_event(lambda e:e.get("event")=="agent_port_open",20)
        if "ERROR" in c.switch(START_DDS):raise RuntimeError("PX4 DDS start failed")
        await_event(lambda e:e.get("payload",{}).get("event")=="ready",25,diagnose=True)
        result["pre_stream"]=snapshot(c);power_off_state(result["pre_stream"])
        child.stdin.write(b"start\n");child.stdin.flush()
        await_event(lambda e:e.get("payload",{}).get("event")=="bench_state" and e["payload"]["sent"]>=75,5)
        print("OFFBOARD_STREAM_READY_OPERATOR_MAY_SWITCH_SA",flush=True)
        deadline=time.monotonic()+70
        while child.poll() is None and time.monotonic()<deadline:
            if any(e.get("event")=="awaiting_release" for e in events):break
            row=snapshot(c);power_off_state(row,allow_operator_arming=a.operator_arming_allowed);result["rounds"].append(row)
            print(json.dumps(dict(event="fc_readback",nav=number(row["vehicle_status"],"nav_state"),
                  offboard=boolean(row["vehicle_control_mode"],"flag_control_offboard_enabled"))),flush=True)
        if not any(e.get("event")=="awaiting_release" and e.get("probe_stopped") is True for e in events):
            raise RuntimeError("NX probe did not stop")
        # Keep independent FC readback for 10 s after the NX process exits. This
        # covers delayed failsafe/auto-disarm without a running DDS publisher.
        end=time.monotonic()+10
        while time.monotonic()<end:
            row=snapshot(c);power_off_state(row,allow_operator_arming=a.operator_arming_allowed);result["rounds"].append(row)
        result["after"]=snapshot(c);power_off_state(result["after"],allow_operator_arming=a.operator_arming_allowed);result["complete"]=True
    except Exception as exc:
        result["error"]=repr(exc)
    finally:
        try:
            if switched and c.s.is_open:
                def cleaned():
                    return child is None or any(e.get("event")=="process_cleanup" and
                        e.get("agent_stopped") is True and e.get("probe_stopped") is True for e in events)
                def probe_stopped():
                    return cleaned() or any(e.get("event")=="awaiting_release" and
                        e.get("probe_stopped") is True for e in events)
                def stop_probe():
                    if probe_stopped():return True
                    if child and child.poll() is None:
                        child.stdin.write(b"stop_probe\n");child.stdin.flush()
                    deadline=time.monotonic()+5
                    while time.monotonic()<deadline and not probe_stopped():time.sleep(.05)
                    return probe_stopped()
                def release_agent():
                    if child and child.poll() is None:
                        child.stdin.write(b"release\n");child.stdin.flush();child.wait(timeout=8)
                    if thread:thread.join(3)
                result.update(teardown_usb_session(c,stop_probe=stop_probe,
                              release_agent=release_agent,nx_cleaned=cleaned))
            elif c.protocol_fault:
                result["recovery_required"]=True
                result["usb_mavlink_restored"]=False
        except Exception as exc:result["cleanup_error"]=repr(exc)
        finally:
            if child and child.poll() is None:
                child.stdin.close()
                try:child.wait(timeout=8)
                except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
            if thread:thread.join(3)
            result["nx_processes_cleaned"]=any(e.get("event")=="process_cleanup" and e.get("agent_stopped") is True and e.get("probe_stopped") is True for e in events)
            result["nx_capture_complete"]=any(e.get("event")=="complete" and e.get("passed") is True for e in events)
            c.close();result["port_closed"]=not c.s.is_open
            (out/"console.json").write_text(json.dumps(c.records,indent=2),encoding="utf-8")
            (out/"report.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    print(json.dumps({k:v for k,v in result.items() if k not in ("before","pre_stream","rounds","after")}),flush=True)
    return 0 if result["complete"] and result.get("nx_capture_complete") and result.get("usb_mavlink_restored") and result.get("dds_stopped") else 1


if __name__=="__main__":raise SystemExit(main())
