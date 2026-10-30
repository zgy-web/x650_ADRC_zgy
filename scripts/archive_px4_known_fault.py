"""Archive one identified USB-switch fault after explicit operator approval.

Default prints the exact plan without opening COM13. No deletion, fault-check
disable, reboot, arming, mode command or NX access. Only the already backed-up
file may move; any additional root fault or changed source aborts the operation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

from prepare_px4_armed_uart import PreflightConsole
from px4_debug_console import FIRMWARE, clean
from px4_imu_pole_check import guard
from px4_rc_switch_setup import number, fresh, read_parameter

SOURCE = '/fs/microsd/fault_2026_10_29_09_01_06.log'
DIRECTORY = '/fs/microsd/flyarm_fault_archive_20261030'
TARGET = DIRECTORY + '/fault_2026_10_29_09_01_06.log'
SIZE = 16923
SHA256 = 'd97375d97750fd7a8f93ec4979f5789ffe16a0aa9750f77ca5056b6ce154b9bb'
WRITES = ('mkdir '+DIRECTORY, 'mv '+SOURCE+' '+TARGET)


def body_from_cat(text, path):
    prefix = 'cat '+path+'\n'
    if not text.startswith(prefix) or not text.endswith('nsh> '):
        raise ValueError('Incomplete fault-file capture')
    data = text[len(prefix):-len('nsh> ')].encode('utf-8')
    if len(data) != SIZE or hashlib.sha256(data).hexdigest() != SHA256:
        raise ValueError('Fault file differs from the independently backed-up incident')
    return data


class ArchiveConsole(PreflightConsole):
    def __init__(self, port):
        super().__init__(port)
        self.permit = None

    def query(self, command):
        allowed = {'ls -l /fs/microsd', 'ls -l '+DIRECTORY, 'cat '+SOURCE, 'cat '+TARGET}
        if command in WRITES:
            if command != self.permit:
                raise ValueError('Write has not passed the immediate state guard')
            self.permit = None
        elif command not in allowed:
            return super().query(command)
        self.s.write((command+'\r').encode('ascii'))
        raw = self.read(9, False) if command.startswith('cat ') else self.read(6)
        text = clean(self.record(command, raw))
        if 'nsh>' not in text or re.search(r'Hard Fault escalation|PANIC!!!|Assertion failed', text):
            raise RuntimeError('Console interrupted')
        return text


def safe(c):
    state = guard(c, require_kill=True, allow_unpublished_battery=True)
    sw = c.query('listener manual_control_switches -n 1')
    rc = c.query('listener manual_control_setpoint -n 1')
    fresh(sw, 1.5)
    fresh(rc, .5)
    if (tuple(number(sw, x) for x in ('arm_switch', 'mode_slot', 'kill_switch')) != (3, 1, 1)
            or number(rc, 'throttle') > -.95):
        raise ValueError('Require SF off, SA up, SB down and minimum throttle')
    return state


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backup', type=Path)
    p.add_argument('--output', type=Path)
    p.add_argument('--port', default='COM13')
    p.add_argument('--archive-approved', action='store_true')
    p.add_argument('--power-disconnected', action='store_true')
    a = p.parse_args()
    plan = dict(source=SOURCE, target=TARGET, bytes=SIZE, sha256=SHA256,
                commands=WRITES, deletion=False, fault_check_remains_enabled=True,
                nx_accessed=False, reboot_required_after_archive=True,
                scope='Power-disconnected UART bench only; USB fault remains unresolved')
    if not a.archive_approved:
        print(json.dumps(plan, indent=2))
        return 0
    if not a.power_disconnected or not a.backup or not a.output:
        raise ValueError('Approval, physical power-off, backup and new output required')
    data = a.backup.read_bytes()
    if len(data) != SIZE or hashlib.sha256(data).hexdigest() != SHA256:
        raise ValueError('Local backup incomplete or changed')
    a.output.mkdir(parents=True, exist_ok=False)
    c = ArchiveConsole(a.port)
    report = dict(plan=plan, complete=False, approved=True, writes=[])
    try:
        c.open()
        if FIRMWARE not in c.query('ver all'):
            raise ValueError('Unexpected firmware')
        report['before'] = safe(c)
        listing = c.query('ls -l /fs/microsd')
        faults = re.findall(r'\b(fault_[^\s/]+\.log)\b', listing)
        if faults != [SOURCE.rsplit('/', 1)[1]] or DIRECTORY.rsplit('/', 1)[1]+'/' in listing:
            raise ValueError('Unexpected fault inventory or archive already exists')
        body_from_cat(c.query('cat '+SOURCE), SOURCE)
        if read_parameter(c, 'COM_ARM_HFLT_CHK') != 1:
            raise ValueError('Fault detection must stay enabled')
        for command in WRITES:
            safe(c)
            c.permit = command
            attempt = dict(command=command, response_received=False)
            report['writes'].append(attempt)
            answer = c.query(command)
            attempt['response_received'] = True
            attempt['response'] = answer
            if re.search(r'failed|error|not found|exists|denied', answer, re.I):
                raise ValueError('Archive operation did not succeed')
        body_from_cat(c.query('cat '+TARGET), TARGET)
        listing = c.query('ls -l /fs/microsd')
        if re.search(r'\bfault_[^\s/]+\.log\b', listing):
            raise ValueError('Root fault file remains or new fault appeared')
        report['target_listing'] = c.query('ls -l '+DIRECTORY)
        report['after'] = safe(c)
        report['fault_check'] = read_parameter(c, 'COM_ARM_HFLT_CHK')
        if report['fault_check'] != 1:
            raise ValueError('Fault check unexpectedly changed')
        report['complete'] = True
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        c.close()
        report['port_closed'] = not c.s.is_open
        (a.output/'console.json').write_text(json.dumps(c.records, indent=2), encoding='utf-8')
        (a.output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report))
    return 0 if report['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
