"""Approved 10-second NX payload check with independent PC Debug readback."""
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
from px4_debug_console import DebugConsole,FIRMWARE,START_DDS,STOP_USB_MAVLINK,status_fields,clean


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("host","identity","known-hosts","build","output"):p.add_argument("--"+name,required=True)
    p.add_argument("--ssh",default="ssh");p.add_argument("--port",default="COM13")
    p.add_argument("--approved-setpoint-check",action="store_true",required=True)
    p.add_argument("--bench-power-disconnected",action="store_true",required=True)
    p.add_argument("--usb-already-reserved",action="store_true",help="Resume after a recorded prior attempt stopped USB MAVLink; DDS must be stopped")
    a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=False);build=Path(a.build)
    marker=json.loads((build/"build_report.json").read_text());binary=json.loads((build/"binary_manifest.json").read_text())
    elf=(build/"px4_setpoint_probe").read_bytes()
    if not(marker.get("setpoint_mock_tests_pass") and marker.get("unit_tests_pass") and marker.get("mock_ros_tests_pass")):
        raise ValueError("Build/tests incomplete")
    if marker["source_sha256"]!=binary["source_sha256"] or hashlib.sha256(elf).hexdigest()!=binary["sha256"]:
        raise ValueError("Tested executable identity differs")
    (out/"build_identity.json").write_text(json.dumps(dict(build=marker,binary=binary),indent=2))
    console=DebugConsole(a.port,allow_switch=True);child=None;thread=None;events=[];q=queue.Queue();status={"complete":False}
    try:
        console.open()
        if FIRMWARE not in console.query("ver all"):raise RuntimeError("Unexpected FC firmware")
        status["before"]=status_fields(console.query("listener vehicle_status -n 1"))
        console.query("listener trajectory_setpoint -n 1")
        dds=console.query("uxrce_dds_client status")
        if "not running" not in dds:raise RuntimeError("Expected stopped DDS; inspect existing session before reusing")
        if not re.search(r"SYS_USB_AUTO[^\r\n]*:\s*2\b",console.query("param show SYS_USB_AUTO")):
            raise RuntimeError("USB reboot recovery not confirmed")
        # DDS has not started: this parameter may not yet be marked as "used".
        if re.findall(r"(?m)^\s*(-?\d+)\s*$",clean(console.query("param show -q UXRCE_DDS_DOM_ID")))!=["0"]:
            raise RuntimeError("Unexpected DDS domain")
        usb_mavlink="serial (/dev/ttyACM0" in console.query("mavlink status")
        if not usb_mavlink and not a.usb_already_reserved:
            raise RuntimeError("Expected USB MAVLink instance not present")
        if usb_mavlink:console.query(STOP_USB_MAVLINK)
        status["usb_was_already_reserved"]=not usb_mavlink
        request=dict(approved_setpoint_check=True,bench_power_disconnected=True,approved_usb_dds_restore=True,
                     binary_base64=base64.b64encode(elf).decode(),binary_sha256=binary["sha256"],source_sha256=binary["source_sha256"])
        source=Path(__file__).with_name("nx_setpoint_check_session_remote.py").read_text()
        args=[a.ssh,"-T","-o","BatchMode=yes","-o","StrictHostKeyChecking=yes","-o","IdentitiesOnly=yes","-o","ConnectTimeout=8",
              "-i",a.identity,"-o","UserKnownHostsFile="+str(Path(a.known_hosts).resolve()),"jetson@"+a.host,
              shlex.join(["env","PYTHONDONTWRITEBYTECODE=1","python3","-u","-c",source])]
        child=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        child.stdin.write((json.dumps(request)+"\n").encode());child.stdin.flush()
        def drain():
            with (out/"remote.jsonl").open("wb") as log:
                for line in child.stdout:
                    log.write(line);log.flush()
                    try:row=json.loads(line)
                    except ValueError:row=dict(event="transport_text",text=line.decode(errors="replace"))
                    events.append(row);q.put(row)
                    payload=row.get("payload",{})
                    if row.get("event")!="agent" and payload.get("event")!="setpoint_sent":print(json.dumps(row),flush=True)
        thread=threading.Thread(target=drain,daemon=True);thread.start()
        deadline=time.monotonic()+20;port_open=False
        while time.monotonic()<deadline:
            try:row=q.get(timeout=.2)
            except queue.Empty:
                if child.poll() is not None:break
                continue
            if row.get("event")=="agent_port_open":port_open=True;break
            if row.get("event")=="failure":break
        if not port_open:raise RuntimeError("NX Agent did not open USB")
        start_reply=console.query(START_DDS)
        if "ERROR" in start_reply:raise RuntimeError("PX4 DDS startup error")
        dds=console.query("uxrce_dds_client status")
        # This firmware reports transport type, not the device path, in status.
        if "Running," not in dds or not re.search(r"Using transport:\s+serial",dds):raise RuntimeError("Temporary serial DDS did not start")
        status["temporary_dds_started"]=True
        deadline=time.monotonic()+40;ready=False
        while time.monotonic()<deadline:
            try:row=q.get(timeout=.2)
            except queue.Empty:
                if child.poll() is not None:break
                continue
            if row.get("event")=="probe" and row.get("payload",{}).get("event")=="ready":ready=True;break
            if row.get("event")=="failure":break
        if not ready:raise RuntimeError("NX probe did not become ready")
        # The topic already exists, so start an independent live listener first.
        console.start_listener();child.stdin.write(b"start\n");child.stdin.flush()
        console.collect_listener()
        child.wait(timeout=20);thread.join(3)
        if child.returncode or not events or events[-1].get("event")!="complete":raise RuntimeError("Incomplete NX session")
        status["after"]=status_fields(console.query("listener vehicle_status -n 1"))
        status["dds_after"]=console.query("uxrce_dds_client status")
        status["complete"]=True
    except Exception as e:
        status["error"]=repr(e);raise
    finally:
        if child and child.poll() is None:
            # Closing the remote control pipe stops the owned probe and Agent.
            child.stdin.close()
            try:child.wait(timeout=8)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
        if thread:thread.join(3)
        try:console.close()
        finally:
            status["port_closed"]=not console.s.is_open
            (out/"console.json").write_text(json.dumps(console.records,indent=2),encoding="utf-8")
            (out/"session_status.json").write_text(json.dumps(status,indent=2),encoding="utf-8")
    print(json.dumps(dict(session_complete=status["complete"],output=str(out))),flush=True)


if __name__=="__main__":main()
