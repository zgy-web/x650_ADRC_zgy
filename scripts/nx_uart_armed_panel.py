"""One operator-armed, power-disconnected UART test; no automatic arm/mode.

Preflight must pass with SF off, SA up and SB released before any NX stream.
The existing C++ zero-thrust executable remains bounded and unchanged. A page
failure never extends the source's 60 s limit. All records remain on this PC.
"""
import argparse
from datetime import datetime, timezone
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading
import time

from nx_uart_operator_panel import ROOT, Session, handler, scalar
from prepare_px4_armed_uart import collect

RESTORE = '先将SF拨回负100%，SA回上档，再将SB拨下档；保持油门最低、动力断开。'


class ArmedFlow:
    """Guidance only; each transition uses independently observed FC state."""
    def __init__(self, case):
        if case not in ('manual_exit', 'stream_stop'):
            raise ValueError('Two explicitly separated armed cases only')
        self.case = case
        self.ready_at = self.armed_at = self.offboard_at = None
        self.finished = False
        self.qualified_observation = False
        self.phase = 'starting'

    def ready(self, now):
        self.ready_at = now
        self.phase = 'warmup'
        return dict(phase='warmup', hint='零推力流已启动；先保持SF负100%、SA上档，等待绿色提示。')

    def stop(self, reason):
        self.finished = True
        self.phase = 'return_switches'
        return dict(phase=self.phase, error=reason, hint='本轮停止推进。'+RESTORE)

    def tick(self, row, now):
        if self.finished or self.ready_at is None:
            return {}
        if now-self.ready_at >= 48:
            return self.stop('操作窗口结束；不能重新解锁或重入本轮。')
        if row is None:
            return {}
        sw = row['raw']['manual_control_switches']
        sf, sa, kill_request = (scalar(sw, x) for x in ('arm_switch', 'mode_slot', 'kill_switch'))
        armed = row['arming_state'] == 2
        lost = scalar(row['raw']['failsafe_flags'], 'offboard_control_signal_lost') == 'True'
        if row['kill'] or kill_request != '3':
            return self.stop('停机开关在测试中介入；本轮不计入接管或断流验收。')
        if scalar(row['raw']['manual_control_setpoint'], 'valid') != 'True':
            return self.stop('遥控输入无效。')
        if self.armed_at is not None and not armed:
            return self.stop('完成目标步骤前已锁定；可能达到地面自动锁定时间，需要分析。')
        if self.phase == 'warmup':
            if armed or sf != '3' or sa != '1':
                return self.stop('绿色提示前开关或解锁状态已变化。')
            if now-self.ready_at >= 1.5:
                self.phase = 'arm'
                return dict(phase='arm', hint='第1步：SF拨到正100%。手准备在SA旁；下一条提示出现后立即拨SA下档。SB保持中档。')
        elif self.phase == 'arm':
            if armed and row['nav_state'] == 1 and sf == '1' and sa == '1':
                self.armed_at = now
                self.phase = 'enter_offboard'
                return dict(phase=self.phase, hint='第2步：已确认解锁，现在立即拨SA下档！SF保持正100%、SB保持中档，不必回聊天。')
            if row['nav_state'] != 1 or sa != '1':
                return self.stop('尚未确认解锁即改变模式；本轮顺序不成立。')
        elif self.phase == 'enter_offboard':
            if armed and row['nav_state'] == 14 and sf == '1' and sa == '6':
                self.offboard_at = now
                self.phase = 'hold'
                return dict(phase='hold', hint=('已确认已解锁Offboard，保持开关，等待接管提示。'
                    if self.case == 'manual_exit' else
                    '已确认已解锁Offboard。保持SA下档、SF正100%、SB中档，等待程序停流和自动退出。'))
            # Do not impose a second, four-second human-response deadline on
            # top of PX4's unchanged ground auto-disarm. The next actual
            # disarmed sample still aborts this one-shot sequence above.
        elif self.phase in ('hold', 'manual_exit'):
            if self.case == 'manual_exit':
                if row['nav_state'] == 14 and self.phase == 'hold' and now-self.offboard_at >= .8:
                    self.phase = 'manual_exit'
                    return dict(phase='manual_exit', hint='第3步：现在拨SA回上档，执行人工接管！SF暂时保持正100%、SB保持中档，等待回读。')
                if self.phase == 'manual_exit' and row['nav_state'] == 1 and sf == '1' and sa == '1':
                    self.finished = self.qualified_observation = True
                    self.phase = 'return_switches'
                    return dict(phase=self.phase, hint='已观察到解锁状态下SA接回Altitude。'+RESTORE)
                if row['nav_state'] != 14:
                    return self.stop('未按SA接管步骤观察到预期模式退出。')
            elif row['nav_state'] != 14:
                if lost and sf == '1' and sa == '6':
                    self.finished = self.qualified_observation = True
                    self.phase = 'return_switches'
                    return dict(phase=self.phase, hint='已观察到保持SA下档时的已解锁自动退出；最终结论还需复核日志。'+RESTORE)
                return self.stop('模式退出伴随其他状态变化，不能作为单独断流验收。')
        return {}


class ArmedSession(Session):
    html = ROOT/'hardware/nx/mavlink_uart/armed_operator_panel.html'

    def __init__(self, output, build, identity, known_hosts, ssh, case):
        super().__init__(output, build, identity, known_hosts, ssh)
        self.case = case
        self.flow = ArmedFlow(case)
        self.set(case=case, hint='初始条件：动力断开、SF负100%、SA上档、SB中档、油门最低。点击后先重新核对预检。',
                 armed=False, actual_kill=None, qualified_observation=False)

    def set(self, **kw):
        # PC-only evidence of guidance timing; this does not claim that a
        # browser displayed a prompt or that the operator saw it.
        with self.lock:
            changed = any(kw.get(k, self.state.get(k)) != self.state.get(k)
                          for k in ('phase', 'hint', 'error'))
            self.state.update(kw)
            if self.output.exists() and changed:
                event = dict(pc_monotonic_ns=time.monotonic_ns(),
                             **{k:self.state.get(k) for k in
                                ('phase', 'hint', 'error', 'armed', 'nav_state', 'records')})
                with (self.output/'guidance.jsonl').open('a', encoding='utf-8') as f:
                    f.write(json.dumps(event, ensure_ascii=False)+'\n')

    def run(self):
        observer = worker = None
        try:
            self.output.mkdir(parents=True, exist_ok=False)
            (self.output/'operator_start.json').write_text(json.dumps(dict(
                utc=datetime.now(timezone.utc).isoformat(), test='armed_'+self.case,
                action='explicit_loopback_button', power_disconnected_confirmed=True,
                automatic_arm=False, nx_config_changes=False), indent=2))
            preflight = collect('COM13', self.output/'preflight')
            if not preflight['complete'] or not preflight['ready']:
                raise RuntimeError('当前初态预检未通过；没有启动NX发送，请查看preflight/report.json。')
            ready = threading.Event()
            def observe(event):
                if event.get('event') == 'independent_readback_ready':
                    ready.set()
                if event.get('event') == 'fc_mode':
                    self.set(nav_state=event['nav_state'], armed=event['arming_state'] == 2,
                             actual_kill=event['kill'])
            observer, opump = self.launch('debug', self.command('observe_px4_uart_bench.py',
                '--port', 'COM13', '--output', self.output/'debug', '--seconds', '90',
                '--power-disconnected', '--allow-operator-arming'), observe)
            end = time.monotonic()+18
            while not ready.wait(.1):
                if observer.poll() is not None or time.monotonic() > end:
                    raise RuntimeError('独立记录器未通过初态检查，没有启动NX发送。')
            def nx(event):
                if event.get('event') == 'ready':
                    self.set(**self.flow.ready(time.monotonic()))
                    self.set(ready_deadline=time.monotonic()+48)
                if event.get('event') == 'result':
                    self.set(nx_result=event, ready_deadline=None)
            worker, wpump = self.launch('nx', self.command('run_nx_mavlink_uart_bench.py',
                '--build', self.build, '--output', self.output/'nx', '--identity', self.identity,
                '--known-hosts', self.known_hosts, '--ssh', self.ssh,
                '--host', '192.168.30.145', '--case', self.case, '--baud', '921600',
                '--seconds', '75', '--power-disconnected', '--configure-bench-streams'), nx)
            rows_path = self.output/'debug/rounds.jsonl'
            last = None
            last_row_at = time.monotonic()
            end = time.monotonic()+100
            while time.monotonic() < end and (observer.poll() is None or worker.poll() is None):
                if rows_path.exists():
                    lines = rows_path.read_text(encoding='utf-8').splitlines()
                    try:
                        row = json.loads(lines[-1]) if lines else None
                    except ValueError:
                        row = None
                    if row and row['stamp'] != last:
                        last = row['stamp']
                        last_row_at = time.monotonic()
                        self.set(nav_state=row['nav_state'], armed=row['arming_state'] == 2,
                                 actual_kill=row['kill'], records=len(lines),
                                 seen_offboard=self.snapshot()['seen_offboard'] or row['nav_state'] == 14,
                                 signal_lost=scalar(row['raw']['failsafe_flags'], 'offboard_control_signal_lost') == 'True')
                        self.set(**self.flow.tick(row, time.monotonic()))
                        self.set(qualified_observation=self.flow.qualified_observation)
                if self.flow.ready_at is not None and not self.flow.finished and time.monotonic()-last_row_at > 3:
                    self.set(**self.flow.stop('独立飞控状态超过3秒未更新，不再依据旧提示操作。'))
                time.sleep(.1)
            if observer.poll() is None or worker.poll() is None:
                raise RuntimeError('进程尚未完成；保持动力断开。')
            opump.join(timeout=2)
            wpump.join(timeout=2)
            capture = json.loads((self.output/'nx/completion.json').read_text())
            report = json.loads((self.output/'debug/report.json').read_text(encoding='utf-8'))
            if not capture['capture_complete'] or not report['complete']:
                raise RuntimeError('采集或清理不完整，不能判为通过。')
            self.set(phase='finished', capture_complete=True,
                     ready_deadline=None,
                     hint='本轮记录已结束。'+RESTORE+'开关已恢复后，点击“恢复回读”。')
        except Exception as exc:
            self.set(hint='本轮停止推进。'+RESTORE, error=str(exc))
            for child in (worker, observer):
                if child and child.poll() is None:
                    try:
                        child.wait(timeout=110)
                    except Exception:
                        self.set(phase='active_error', hint='进程状态未确认。'+RESTORE)
                        return
            self.set(phase='failed', hint='没有可接受的完整结果。'+RESTORE+'然后点击恢复回读。')
        finally:
            if self.output.exists():
                (self.output/'panel_state.json').write_text(json.dumps(self.snapshot(), ensure_ascii=False, indent=2), encoding='utf-8')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('output', 'build', 'identity', 'known-hosts'):
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--ssh', required=True)
    p.add_argument('--case', required=True, choices=('manual_exit', 'stream_stop'))
    a = p.parse_args()
    if a.output.exists():
        raise ValueError('Use a new capture directory')
    session = ArmedSession(a.output.resolve(), a.build.resolve(), a.identity.resolve(),
                           a.known_hosts.resolve(), a.ssh, a.case)
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler(session, secrets.token_urlsafe(32)))
    print(json.dumps(dict(event='armed_panel_ready', case=a.case,
                         url='http://127.0.0.1:'+str(server.server_port)+'/',
                         no_hardware_started=True)), flush=True)
    server.serve_forever(poll_interval=.5)


if __name__ == '__main__':
    main()
