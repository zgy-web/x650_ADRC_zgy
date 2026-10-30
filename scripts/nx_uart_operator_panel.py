"""PC-only, loopback operator panel for one approved unarmed stream-stop test.

GET never starts hardware access. An explicit local browser POST starts the same
bounded, previously tested C++ executable. No firmware/configuration change,
arming, mode command, USB protocol change, or additional NX installation exists.
Closing the page does not extend or cancel the executable's 60-second TX limit.
"""
import argparse
import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import secrets
import subprocess
import sys
import threading
import time


ROOT = Path(__file__).resolve().parents[1]
HTML = ROOT / 'hardware/nx/mavlink_uart/operator_panel.html'


def scalar(raw, key):
    values = re.findall(r'^\s*' + re.escape(key) + r':\s*(\S+)', raw, re.M)
    return values[0] if len(values) == 1 else None


def restored(report):
    raw = report.get('final_raw', {})
    return (report.get('complete') is True and report.get('port_closed') is True
            and scalar(raw.get('vehicle_status', ''), 'arming_state') == '1'
            and scalar(raw.get('vehicle_status', ''), 'nav_state') == '1'
            and scalar(raw.get('actuator_armed', ''), 'kill') == 'True'
            and scalar(raw.get('manual_control_switches', ''), 'arm_switch') == '3'
            and scalar(raw.get('manual_control_switches', ''), 'mode_slot') == '1'
            and report.get('before', {}).get('kill') is True
            and bool(report.get('before', {}).get('outputs'))
            and all(v == 0 for v in report['before']['outputs'].values()))


class Session:
    def __init__(self, output, build, identity, known_hosts, ssh):
        self.output, self.build = output, build
        self.identity, self.known_hosts, self.ssh = identity, known_hosts, ssh
        self.lock = threading.Lock()
        self.state = dict(phase='idle', hint='保持 SA 上档、SF负100%、SB中档、油门最低。就位后点击开始。',
                          seen_offboard=False, signal_lost=None, nav_state=None, records=0,
                          capture_complete=False, restored=False, ready_deadline=None,
                          requires_power_disconnected=True, mpc_running=False)
        self.used = False
        self.restore_used = False
        self.children = []

    def set(self, **kw):
        with self.lock:
            self.state.update(kw)

    def snapshot(self):
        with self.lock:
            s = dict(self.state)
        end = s.pop('ready_deadline')
        s['action_seconds_left'] = max(0, round(end-time.monotonic())) if end else None
        return s

    def start(self, action):
        with self.lock:
            if action == 'start' and not self.used and self.state['phase'] == 'idle':
                self.used = True
                self.state.update(phase='starting', hint='正在核对真实状态；暂时保持开关不动。')
                target = self.run
            elif action == 'restore' and self.state['phase'] in ('finished', 'failed') and not self.restore_used:
                self.restore_used = True
                self.state.update(phase='restoring', hint='正在回读恢复后的状态，请保持机架与开关静止。')
                target = self.finish
            else:
                return False
        threading.Thread(target=target, daemon=True).start()
        return True

    def command(self, script, *args):
        return [sys.executable, '-X', 'utf8', '-B', str(ROOT/'scripts'/script), *map(str, args)]

    def launch(self, name, cmd, on_event):
        process = subprocess.Popen(cmd, cwd=ROOT, stdin=subprocess.DEVNULL,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        self.children.append(process)
        def pump():
            with (self.output/(name+'.console.log')).open('wb') as log:
                for line in process.stdout:
                    log.write(line)
                    log.flush()
                    try:
                        event = json.loads(line)
                    except ValueError:
                        continue
                    on_event(event)
        thread = threading.Thread(target=pump, daemon=True)
        thread.start()
        return process, thread

    def run(self):
        observer = worker = None
        try:
            self.output.mkdir(parents=True, exist_ok=False)
            (self.output/'operator_start.json').write_text(json.dumps(dict(
                utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                action='explicit_loopback_button', test='unarmed_stream_stop',
                power_disconnected_confirmed=True, automatic_arm=False,
                nx_changes=False, max_nx_transmit_seconds=60), indent=2))
            ready = threading.Event()
            def observe(event):
                if event.get('event') == 'independent_readback_ready':
                    ready.set()
                if event.get('event') == 'fc_mode':
                    self.set(nav_state=event['nav_state'], armed=event['arming_state'] != 1,
                             actual_kill=event['kill'])
            observer, observer_pump = self.launch('debug', self.command('observe_px4_uart_bench.py',
                '--port', 'COM13', '--output', self.output/'debug', '--seconds', '90',
                '--power-disconnected'), observe)
            deadline = time.monotonic()+18
            while not ready.wait(.1):
                if observer.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('独立记录器未通过初态检查；没有启动NX发送。')
            def nx(event):
                if event.get('event') == 'ready':
                    self.set(phase='ready', hint='现在将 SA 拨到下档并保持。不要操作 SF；不必回聊天确认。',
                             ready_deadline=time.monotonic()+50)
                if event.get('event') == 'result':
                    self.set(nx_result=event, ready_deadline=None)
            worker, worker_pump = self.launch('nx', self.command('run_nx_mavlink_uart_bench.py',
                '--build', self.build, '--output', self.output/'nx',
                '--identity', self.identity, '--known-hosts', self.known_hosts, '--ssh', self.ssh,
                '--host', '192.168.30.145', '--case', 'stream_stop', '--baud', '921600',
                '--seconds', '75', '--power-disconnected'), nx)
            rows_path = self.output/'debug/rounds.jsonl'
            last = None
            end = time.monotonic()+100
            while time.monotonic() < end and (observer.poll() is None or worker.poll() is None):
                if rows_path.exists():
                    lines = rows_path.read_text(encoding='utf-8').splitlines()
                    try:
                        row = json.loads(lines[-1]) if lines else None
                    except ValueError:  # Writer may currently be appending its last line.
                        row = None
                    if row and row['stamp'] != last:
                        last = row['stamp']
                        lost = scalar(row['raw']['failsafe_flags'], 'offboard_control_signal_lost') == 'True'
                        with self.lock:
                            seen = self.state['seen_offboard'] or row['nav_state'] == 14
                            phase = self.state['phase']
                        self.set(nav_state=row['nav_state'], armed=row['arming_state'] != 1,
                                 signal_lost=lost, seen_offboard=seen, records=len(lines))
                        if seen:
                            self.set(phase='observing', ready_deadline=None,
                                hint=('已检测到断流标志；继续保持SA下档，等待本轮记录结束。' if lost else
                                      '已进入Offboard；程序约3秒后主动停发。保持SA下档，等待记录。'))
                        elif phase == 'ready' and self.snapshot()['action_seconds_left'] == 0:
                            self.set(phase='observing', ready_deadline=None,
                                     hint='操作窗口已结束，请保持原开关位置，等待记录收尾；本轮不自动重试。')
                time.sleep(.25)
            if observer.poll() is None or worker.poll() is None:
                raise RuntimeError('记录器尚未完成，请保持动力断开；不再启动任何发送。')
            observer_pump.join(timeout=2)
            worker_pump.join(timeout=2)
            capture = json.loads((self.output/'nx/completion.json').read_text())
            report = json.loads((self.output/'debug/report.json').read_text(encoding='utf-8'))
            if not capture['capture_complete'] or not report['complete']:
                raise RuntimeError('本轮采集未完整完成，不能判为通过。')
            self.set(phase='finished', capture_complete=True, ready_deadline=None,
                     hint='记录结束。请将SA回上档、SB拨回下档，SF保持负100%、油门最低、动力断开；然后点“已恢复，回读确认”。')
        except Exception as exc:
            # Existing children remain intrinsically bounded; never kill SSH and
            # assume that doing so has stopped a remote control stream.
            for child in (worker, observer):
                if child and child.poll() is None:
                    try:
                        child.wait(timeout=110)
                    except subprocess.TimeoutExpired:
                        self.set(phase='active_error', hint='进程状态未确认。保持动力断开、SF负100%，请通知我检查。', error=str(exc))
                        return
            self.set(phase='failed', ready_deadline=None, error=str(exc),
                     hint='本轮未通过，发送会话已结束。请恢复SA上档、SB下档、SF负100%和油门最低，再点恢复回读。')
        finally:
            if self.output.exists():
                (self.output/'panel_state.json').write_text(json.dumps(self.snapshot(), ensure_ascii=False, indent=2), encoding='utf-8')

    def finish(self):
        try:
            process, pump = self.launch('restore', self.command('observe_px4_uart_bench.py',
                '--port', 'COM13', '--output', self.output/'restore', '--seconds', '3',
                '--power-disconnected'), lambda event: None)
            process.wait(timeout=35)
            pump.join(timeout=2)
            r = json.loads((self.output/'restore/report.json').read_text(encoding='utf-8'))
            if not restored(r):
                raise RuntimeError('恢复状态未全部匹配，请保持动力断开并通知我；不要自行解锁。')
            self.set(phase='restored', restored=True, nav_state=1, armed=False, actual_kill=True,
                     signal_lost=scalar(r['final_raw']['failsafe_flags'], 'offboard_control_signal_lost') == 'True',
                     hint='恢复回读通过：未解锁、Altitude、实际停机标志有效、输出为零。请保持动力断开，我会分析本轮结果。')
        except Exception as exc:
            self.set(phase='restore_failed', error=str(exc), hint='恢复回读尚未通过，请保持动力断开并通知我。')
        finally:
            (self.output/'panel_state.json').write_text(json.dumps(self.snapshot(), ensure_ascii=False, indent=2), encoding='utf-8')


def handler(session, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def reply(self, code, body, content='application/json; charset=utf-8'):
            data = body.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', content)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Frame-Options', 'DENY')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        def local_origin(self):
            expected = '127.0.0.1:'+str(self.server.server_port)
            return (self.headers.get('Host') == expected
                    and self.headers.get('Origin') == 'http://'+expected
                    and self.headers.get('X-Operator-Token') == token)
        def do_GET(self):
            if self.path == '/':
                self.reply(200, HTML.read_text(encoding='utf-8').replace('__TOKEN__', token), 'text/html; charset=utf-8')
            elif self.path == '/status':
                self.reply(200, json.dumps(session.snapshot(), ensure_ascii=False))
            else:
                self.reply(404, '{}')
        def do_POST(self):
            if not self.local_origin():
                self.reply(403, '{"error":"local operator action required"}')
            elif self.path not in ('/start', '/restore'):
                self.reply(404, '{}')
            elif session.start(self.path[1:]):
                self.reply(202, '{"accepted":true}')
            else:
                self.reply(409, '{"error":"action unavailable in current phase"}')
    return Handler


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('output', 'build', 'identity', 'known-hosts'):
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--ssh', required=True)
    a = p.parse_args()
    if a.output.exists():
        raise ValueError('Use a new capture directory')
    session = Session(a.output.resolve(), a.build.resolve(), a.identity.resolve(), a.known_hosts.resolve(), a.ssh)
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler(session, secrets.token_urlsafe(32)))
    print(json.dumps(dict(event='operator_panel_ready', url='http://127.0.0.1:'+str(server.server_port)+'/',
                          no_hardware_started=True, output=str(a.output.resolve()))), flush=True)
    server.serve_forever(poll_interval=.5)


if __name__ == '__main__':
    main()
