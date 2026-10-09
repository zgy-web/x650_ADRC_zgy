"""Read-only selectable vehicle-state backend for flyarm experiments.

This is observation routing, NOT PX4 HITL or actuator routing.
"""
from __future__ import annotations
import csv
import dataclasses
import json
import pathlib
import socket
import time

@dataclasses.dataclass(frozen=True)
class State:
    source: str
    backend: str
    message: str
    source_time_us: int
    observed_time_ns: int
    values: dict

def load_config(path):
    cfg=json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    if cfg.get("backend") not in ("sitl","hardware"):
        raise ValueError("backend must be sitl or hardware")
    if cfg.get("control_enabled", False) or cfg.get("hil_injection_enabled", False):
        raise ValueError("This selector is read-only; control/HITL injection must remain disabled")
    return cfg

def read_hardware_csv(path):
    """Load a recorded hardware stream; distinguish PC, server and FC clocks."""
    out=[]
    with open(path,newline="",encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("source")!="pixhawk_windows_com3": continue
            if row.get("kind") not in ("ATTITUDE","HIGHRES_IMU","HEARTBEAT"):continue
            numeric_keys=("roll","pitch","yaw") if row["kind"]=="ATTITUDE" else ("xacc","yacc","zacc","xgyro","ygyro","zgyro") if row["kind"]=="HIGHRES_IMU" else ()
            values={k:float(row[k]) for k in numeric_keys if row.get(k)}
            out.append(State(
                "pixhawk_windows_com3", "hardware",row["kind"],
                int(row.get("fc_time_us") or 0),
                int(row.get("server_time_ns") or 0),
                {"fields":values,"pc_perf_counter_ns":int(row.get("pc_time_ns") or 0)},
            ))
    return out

def capture_hardware_live(duration_s, host="127.0.0.1", port=18765):
    """Read JSON lines over a loopback-only TCP port, forwarded via SSH.

    The connection is receive-only. No MAVLink, arm or actuator methods exist.
    """
    if host != "127.0.0.1":
        raise ValueError("hardware live receiver must bind to localhost")
    if not 1024 <= int(port) <= 65535:
        raise ValueError("invalid TCP port")
    records = []
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((host, int(port)))
        server.listen(1)
        server.settimeout(30)
        print(f"WAITING for hardware telemetry via SSH on {host}:{port}", flush=True)
        conn, _ = server.accept()
        with conn:
            conn.settimeout(1.0)
            buffer = b""
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                try:
                    chunk = conn.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buffer += chunk
                if len(buffer) > 2_000_000:
                    raise ValueError("oversized incoming telemetry buffer")
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    try:
                        row = json.loads(line)
                        if row.get("source") != "pixhawk_windows_com3":
                            continue
                        kind = row.get("kind")
                        if kind not in ("ATTITUDE", "HIGHRES_IMU", "HEARTBEAT"):
                            continue
                        fields = ("roll", "pitch", "yaw") if kind == "ATTITUDE" else ("xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro") if kind == "HIGHRES_IMU" else ()
                        values = {k: float(row[k]) for k in fields if k in row}
                        records.append(State("pixhawk_windows_com3", "hardware", kind,
                                             int(row.get("fc_time_us") or 0), time.time_ns(),
                                             {"fields": values, "pc_perf_counter_ns": int(row.get("pc_time_ns") or 0)}))
                    except (ValueError, TypeError, AttributeError):
                        continue
    return records


def capture_sitl(duration_s):
    """Read-only ROS2 PX4 subscriptions, without creating MAVLink command links."""
    import rclpy
    from px4_msgs.msg import VehicleAttitude, SensorCombined
    from rclpy.qos import qos_profile_sensor_data
    records=[]
    rclpy.init()
    node=rclpy.create_node("flyarm_backend_sitl_observer")
    def attitude(msg):
        records.append(State("px4_sitl_ros2","sitl","ATTITUDE",int(msg.timestamp_sample),time.time_ns(),{"q":[float(x) for x in msg.q]}))
    def imu(msg):
        records.append(State("px4_sitl_ros2","sitl","HIGHRES_IMU",int(msg.timestamp),time.time_ns(),{"gyro_rad":[float(x) for x in msg.gyro_rad],"accelerometer_m_s2":[float(x) for x in msg.accelerometer_m_s2]}))
    try:
        node.create_subscription(VehicleAttitude,"/fmu/out/vehicle_attitude",attitude,qos_profile_sensor_data)
        node.create_subscription(SensorCombined,"/fmu/out/sensor_combined",imu,qos_profile_sensor_data)
        end=time.monotonic()+duration_s
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return records

def summary(records,backend):
    counts={name:sum(r.message==name for r in records) for name in ("ATTITUDE","HIGHRES_IMU","HEARTBEAT")}
    return {"backend":backend,"read_only":True,"records":len(records),"counts":counts,
            "sources":sorted({r.source for r in records}),
            "source_clock_note":"FC timestamps and PC/server timestamps are NOT synchronized; no one-way latency estimate"}

def main():
    import argparse
    p=argparse.ArgumentParser()
    p.add_argument("--config",required=True)
    p.add_argument("--duration",type=float,default=5.0)
    p.add_argument("--hardware-csv")
    p.add_argument("--hardware-live",action="store_true")
    p.add_argument("--port",type=int,default=18765)
    p.add_argument("--output")
    args=p.parse_args()
    cfg=load_config(args.config)
    if cfg["backend"]=="sitl":
        records=capture_sitl(args.duration)
    else:
        if args.hardware_live:
            records=capture_hardware_live(args.duration, port=args.port)
        else:
            filepath=args.hardware_csv or cfg.get("hardware_csv")
            if not filepath: raise ValueError("hardware mode requires --hardware-csv or --hardware-live")
            records=read_hardware_csv(filepath)
    result=summary(records,cfg["backend"])
    print(json.dumps(result,indent=2))
    if args.output:
        pathlib.Path(args.output).write_text(json.dumps({"summary":result,"records":[dataclasses.asdict(r) for r in records]},indent=2))
    if not records:raise SystemExit("No telemetry received")
if __name__=="__main__":main()
