"""PC-only, one-shot six-face accelerometer calibration with local live display.

The sole mutating command is 'commander calibrate accel'. PX4 performs and
saves the calibration. Start requires the operator's local button; no retry,
motor, arm, level, quick-calibration, parameter-set, reboot or NX command exists.
"""
import argparse
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import secrets
import threading
import time

from px4_debug_console import FIRMWARE, clean
from px4_imu_pole_check import ImuConsole, PARAMETERS, guard
from px4_rc_switch_setup import read_parameter

COMMAND = "commander calibrate accel"
ORDER = ("down", "left", "up", "right", "front", "back")
LABELS = dict(down="正放：起落架朝下", up="倒置：起落架朝上", back="机头朝上、机尾朝下",
              front="机头朝下、机尾朝上", left="左侧朝下", right="右侧朝下")
TERMINAL = {"done", "failed", "cancelled", "unknown"}


class Progress:
    def __init__(self):
        self.phase = "prepared"
        self.done = []
        self.measuring = None
        self.latest = "正放已准备；点击开始后，按此页实时提示换面。"
        self.started = False

    def feed(self, text):
        for line in clean(text).splitlines():
            if "[cal]" not in line:
                continue
            self.latest = line[line.index("[cal]"):]
            if self.phase in TERMINAL:
                continue
            if "[cal] calibration started: 2 accel" in line:
                self.started = True
                self.phase = "running"
            if not self.started:
                continue
            m = re.search(r"\[cal\] (down|up|back|front|left|right) (?:orientation detected|side done)", line)
            if m:
                side = m.group(1)
                if "side done" in line:
                    if side not in self.done:
                        self.done.append(side)
                    self.measuring = None
                elif side not in self.done:
                    self.measuring = side
            if "[cal] calibration done: accel" in line:
                self.phase = "done" if set(self.done) == set(ORDER) else "unknown"
            elif "[cal] calibration failed:" in line:
                self.phase = "failed"
            elif "[cal] calibration cancelled" in line:
                self.phase = "cancelled"

    def data(self):
        pending = next((side for side in ORDER if side not in self.done), None)
        if self.phase == "done":
            prompt = "六面采集完成。请恢复正放、保持静止，等待回读与复测。"
        elif self.phase in TERMINAL:
            prompt = "校准未确认成功。请恢复正放，动力继续关闭，返回聊天；不要自行重复启动。"
        elif self.measuring:
            prompt = "正在采集：" + LABELS[self.measuring] + "。保持静止！"
        elif len(self.done) == 6:
            prompt = "六面已采集，等待飞控计算及保存结果。"
        elif self.phase == "checking":
            prompt = "正在核对未解锁、停机、零输出和原校准参数，请保持正放。"
        elif self.phase == "prepared":
            prompt = "保持正放，点击下方开始。"
        else:
            prompt = ("这一面已完成，请换到：" if self.done else "请保持：") + LABELS[pending]
        return dict(phase=self.phase, done=list(self.done), measuring=self.measuring,
                    latest=self.latest, prompt=prompt, sides=LABELS, order=list(ORDER))


class Session:
    def __init__(self, output, port, *, operator_power_off, operator_ready):
        if not operator_power_off or not operator_ready:
            raise ValueError("Physical power-off and six-face operator readiness required")
        self.out = Path(output)
        self.out.mkdir(parents=True, exist_ok=False)
        self.port = port
        self.lock = threading.RLock()
        self.progress = Progress()
        self.claimed = False
        self.token = secrets.token_urlsafe(24)
        self.last_ui_poll = 0.0
        self.report = dict(kind="real_px4_six_face_accelerometer_calibration", firmware=FIRMWARE,
                           utc_created=datetime.now(timezone.utc).isoformat(), operator_power_off=True,
                           operator_ready=True, command=COMMAND, command_attempts=0,
                           nx_accessed=False, motor_commands=0, arm_commands=0,
                           calibration_success=False, post_readback_complete=False,
                           bias_acceptance_passed=False, flight_ready=False,
                           source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        self.persist()

    def persist(self):
        with self.lock:
            state = {**self.progress.data(), "report": self.report,
                     "server_unix": time.time(), "claimed": self.claimed}
            tmp = self.out / "status.tmp.json"
            tmp.write_text(json.dumps(state, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
            tmp.replace(self.out / "status.json")

    def start(self):
        with self.lock:
            if self.claimed:
                return False
            if time.monotonic() - self.last_ui_poll > 3:
                return False
            self.claimed = True
            self.progress.phase = "checking"
            self.persist()
        threading.Thread(target=self.run, daemon=True).start()
        return True

    def run(self):
        c = ImuConsole(self.port)
        stream = bytearray()
        tail = ""
        try:
            c.open()
            if FIRMWARE not in clean(c.query("ver all")):
                raise ValueError("Unexpected firmware")
            self.report["before"] = guard(c, require_kill=True)
            self.report["before_parameters"] = {p: read_parameter(c, p) for p in PARAMETERS}
            if self.report["before_parameters"]["MOT_POLE_COUNT"] != 24:
                raise ValueError("Expected verified 24-pole configuration")
            self.report["precommand"] = guard(c, require_kill=True)
            if time.monotonic() - self.last_ui_poll > 3:
                raise ValueError("Live display is not connected; command not sent")
            # Clear/log any earlier console message before the single authorized command.
            c.record("<precalibration drain>", c.read(.2, False))
            self.report["command_attempts"] = 1
            self.report["command_utc"] = datetime.now(timezone.utc).isoformat()
            self.persist()  # Record intention before sending; never retry an uncertain write.
            c.s.write((COMMAND + "\r").encode("ascii"))
            end = time.monotonic() + 900
            with (self.out / "calibration_stream.bin").open("xb") as rawfile:
                while time.monotonic() < end:
                    chunk = c.read(.15, False)
                    if chunk:
                        stream.extend(chunk)
                        rawfile.write(chunk)
                        rawfile.flush()
                        tail += chunk.decode(errors="replace")
                        lines = tail.split("\n")
                        tail = lines.pop()
                        with self.lock:
                            self.progress.feed("\n".join(lines))
                    self.persist()
                    if self.progress.phase in TERMINAL:
                        break
                else:
                    with self.lock:
                        self.progress.phase = "unknown"
                        self.progress.latest = "观察超时：飞控校准状态尚未确认，不自动重发。"
            c.record(COMMAND, bytes(stream))
            self.report["calibration_success"] = self.progress.phase == "done"
            self.report["completed_sides"] = list(self.progress.done)
            # After an explicit terminal message only, return to read-only console work.
            if self.progress.phase in {"done", "failed", "cancelled"}:
                time.sleep(.8)
                c.s.write(b"\r")
                response = c.record("<enter after calibration>", c.read(3))
                if "nsh>" not in response:
                    raise ValueError("Console prompt not recovered")
                self.report["after_parameters"] = {p: read_parameter(c, p) for p in PARAMETERS}
                self.report["after"] = guard(c, require_kill=True)
                self.report["post_readback_complete"] = True
                self.report["parameter_changes"] = {p: [old, self.report["after_parameters"][p]]
                                                    for p, old in self.report["before_parameters"].items()
                                                    if old != self.report["after_parameters"][p]}
        except Exception as exc:
            self.report["error"] = f"{type(exc).__name__}: {exc}"
            with self.lock:
                self.progress.phase = "unknown" if self.report["command_attempts"] else "failed"
                self.progress.latest = str(exc)
        finally:
            c.close()
            self.report["port_closed"] = not c.s.is_open
            self.report["worker_finished"] = True
            (self.out / "console.json").write_text(json.dumps(c.records, indent=2, ensure_ascii=False), encoding="utf-8")
            self.report["console_sha256"] = hashlib.sha256((self.out / "console.json").read_bytes()).hexdigest()
            self.report["stream_sha256"] = hashlib.sha256(stream).hexdigest()
            self.persist()


def handler_for(session, origin):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def reply(self, status, data, content_type="application/json; charset=utf-8"):
            if not isinstance(data, bytes):
                data = json.dumps(data, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        def do_GET(self):
            if self.headers.get("Host") != origin.split("://")[1]:
                return self.reply(403, {"error": "host"})
            if self.path == "/":
                page = Path(__file__).with_suffix(".html").read_text(encoding="utf-8")
                return self.reply(200, page.replace("__TOKEN__", session.token).encode(), "text/html; charset=utf-8")
            if self.path == "/state":
                with session.lock:
                    session.last_ui_poll = time.monotonic()
                    data = {**session.progress.data(), "claimed": session.claimed,
                            "finished": session.report.get("worker_finished", False),
                            "readback": session.report["post_readback_complete"],
                            "server_unix": time.time()}
                return self.reply(200, data)
            self.reply(404, {"error": "not found"})
        def do_POST(self):
            if (self.path != "/start" or self.headers.get("Origin") != origin or
                    self.headers.get("Host") != origin.split("://")[1] or
                    self.headers.get("X-Flyarm-Session") != session.token):
                return self.reply(403, {"error": "invalid local session"})
            accepted = session.start()
            self.reply(202 if accepted else 409, {"accepted": accepted})
    return Handler


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM13")
    parser.add_argument("--http-port", type=int, default=8766)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--esc-power-off", action="store_true")
    parser.add_argument("--operator-ready", action="store_true")
    args = parser.parse_args()
    session = Session(args.output, args.port, operator_power_off=args.esc_power_off, operator_ready=args.operator_ready)
    origin = f"http://127.0.0.1:{args.http_port}"
    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), handler_for(session, origin))
    (session.out / "server.json").write_text(json.dumps({"url": origin, "port": args.port}), encoding="utf-8")
    print(origin, flush=True)
    server.serve_forever()
