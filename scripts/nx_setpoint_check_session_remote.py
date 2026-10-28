"""Open the USB Agent and run a bounded probe in NX RAM, under PC coordination."""
import base64
import hashlib
import json
import os
from pathlib import Path
import resource
import select
import shlex
import subprocess
import sys
import tempfile
import threading
import time

LOCK=threading.Lock()
def emit(**r):
    with LOCK:print(json.dumps(r,allow_nan=False),flush=True)
def stop(p):
    if p and p.poll() is None:
        p.terminate()
        try:p.wait(timeout=3)
        except subprocess.TimeoutExpired:p.kill();p.wait(timeout=3)


def main():
    r=json.loads(sys.stdin.readline())
    if not(r.get("approved_setpoint_check") is True and r.get("bench_power_disconnected") is True and r.get("approved_usb_dds_restore") is True):
        raise ValueError("Explicit approved disarmed bench DDS session required")
    data=base64.b64decode(r["binary_base64"],validate=True)
    if hashlib.sha256(data).hexdigest()!=r["binary_sha256"]:raise ValueError("Binary identity mismatch")
    for name in ("MicroXRCEAgent","px4_setpoint_probe"):
        # comm names are limited to 15 chars; use the exact process basename.
        if subprocess.run(["pgrep","-x",name[:15]],stdout=subprocess.DEVNULL).returncode==0:
            raise RuntimeError("Existing process must not be disturbed: "+name)
    root=Path.home()/"flyarm/ros2_hw_runtime/v1"
    resource.setrlimit(resource.RLIMIT_CORE,(0,0));code=2
    with tempfile.TemporaryDirectory(prefix="flyarm_setpoint_",dir="/dev/shm") as tmp:
        work=Path(tmp);exe=work/"px4_setpoint_probe";exe.write_bytes(data);exe.chmod(0o700)
        env=os.environ.copy();env.update(ROS_LOG_DIR=tmp+"/ros_logs",ROS_HOME=tmp+"/ros_home",TMPDIR=tmp,
            PYTHONDONTWRITEBYTECODE="1",ROS_DOMAIN_ID="0",ROS_LOCALHOST_ONLY="0",
            LD_LIBRARY_PATH=str(root/"agent/lib")+":/opt/ros/humble/lib:"+env.get("LD_LIBRARY_PATH",""))
        agent=probe=None;threads=[];finals=[]
        try:
            emit(event="session_started",binary_sha256=r["binary_sha256"],source_sha256=r["source_sha256"],physical_px4=True,
                 persistent_deployment=False,send_window_s=10,maximum_count=200)
            agent=subprocess.Popen([str(root/"agent/bin/MicroXRCEAgent"),"serial","--dev",
                "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00","-b","921600","-v","4"],
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=env,cwd=tmp)
            command="source /opt/ros/humble/setup.bash && source "+shlex.quote(str(root/"ros/local_setup.bash"))+" && exec "+shlex.join([
                str(exe),"--ros-args","--disable-external-lib-logs","--disable-rosout-logs","-p","approved_bench_probe:=true"])
            probe=subprocess.Popen(["bash","-c",command],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=env,cwd=tmp)
            def forward(p,kind):
                for line in p.stdout:
                    text=line.decode(errors="replace").rstrip()
                    try:payload=json.loads(text)
                    except ValueError:payload={"text":text}
                    if kind=="probe" and payload.get("event")=="probe_final":finals.append(payload)
                    emit(event=kind,payload=payload)
            for p,kind in ((agent,"agent"),(probe,"probe")):
                t=threading.Thread(target=forward,args=(p,kind),daemon=True);t.start();threads.append(t)
            # The PX4 USB CDC driver can reject open() when the host is not open.
            # Confirm this exact Agent owns the port before the PC starts PX4 DDS.
            usb=Path("/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00").resolve(strict=True)
            port_deadline=time.monotonic()+5;port_open=False
            while time.monotonic()<port_deadline and agent.poll() is None:
                for fd in Path("/proc") .joinpath(str(agent.pid),"fd").iterdir():
                    try:port_open=port_open or fd.resolve(strict=True)==usb
                    except FileNotFoundError:pass
                if port_open:break
                time.sleep(.05)
            if not port_open:raise RuntimeError("Owned Agent did not open the PX4 USB device")
            emit(event="agent_port_open",device=str(usb))
            deadline=time.monotonic()+50;triggered=False
            while probe.poll() is None and time.monotonic()<deadline:
                if agent.poll() is not None:raise RuntimeError("Agent exited during test")
                if select.select([sys.stdin],[],[],.05)[0]:
                    line=sys.stdin.readline()
                    if line!="start\n" or triggered:raise RuntimeError("Control pipe ended or unexpected message")
                    probe.stdin.write(b"start\n");probe.stdin.flush();triggered=True
            if probe.poll() is None:raise TimeoutError("Probe wall deadline")
            threads[1].join(3);code=probe.returncode
            if code or len(finals)!=1 or finals[0].get("complete") is not True:raise RuntimeError("Probe did not complete")
            emit(event="session_finished",probe_exit=code,final=finals[0])
        finally:
            stop(probe);stop(agent)
            for t in threads:t.join(3)
            emit(event="process_cleanup",probe_stopped=probe is None or probe.poll() is not None,agent_stopped=agent is None or agent.poll() is not None)
    emit(event="cleanup",ram_removed=not work.exists(),persistent_files_created=[])
    emit(event="complete",passed=code==0)


if __name__=="__main__":
    try:main()
    except Exception as e:emit(event="failure",error=repr(e));sys.exit(1)
