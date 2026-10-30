"""Withdrawn eight-parameter proposal; powered-disarmed guards retained for audit.

Default prints the plan without opening serial. A powered, props-off bench is
permitted: every write requires fresh disarmed, actual kill, SF-off, SA-upper,
minimum-throttle, zero-output and healthy supply observations. No command to
arm, start a motor, enter a mode, reboot, change NX or switch USB exists here.
This does not authorize flight or prove loaded control performance.
The user withdrew this proposal on 2026-10-30. Live application is disabled;
the historical flow is retained for offline tests and audit, not future use.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time

from px4_debug_console import DebugConsole, FIRMWARE, clean, status_fields
from px4_first_flight_readonly import PARAMETERS, READS
from px4_motor_single_pulse import require_zero
from px4_motor_telemetry_check import telemetry_snapshot
from px4_rc_switch_setup import boolean, fresh, number, read_parameter


# Limits first, then the two inactive SA slots. Never rewrite the active upper slot.
APPLY_ENABLED = False  # User chose original configuration; no live reapplication.
PLAN = (
    ('MPC_MAN_TILT_MAX', 35, 20), ('MPC_MAN_Y_MAX', 150, 60),
    ('MPC_Z_VEL_MAX_UP', 3, 1.0), ('MPC_Z_VEL_MAX_DN', 1.5, .6),
    ('MPC_TKO_SPEED', 1.5, 1.0), ('MPC_LAND_SPEED', .7, .6),
    ('COM_FLTMODE4', 2, 1), ('COM_FLTMODE6', 7, 8),
)
TARGETS = {name: target for name, old, target in PLAN}
KEEP = {'COM_FLTMODE1': 1, 'RC_MAP_ARM_SW': 5, 'RC_MAP_FLTMODE': 6,
        'RC_MAP_KILL_SW': 7, 'COM_ARM_HFLT_CHK': 1, 'CBRK_SUPPLY_CHK': 0}
DEPENDENCIES = ('MPC_Z_VEL_ALL', 'SYS_VEHICLE_RESP')
# COM_FLTMODE uses 8 for Stabilized; VehicleStatus.nav_state uses 15.
SLOT_NAV = {1: 1, 4: 1, 6: 15}
WRITES = frozenset(f'param set {n} {v:g}' for n, _, v in PLAN) | {'param save'}


def same(a, b):
    return a is not None and math.isfinite(a) and math.isclose(a, b, abs_tol=1e-5, rel_tol=0)


def save_json(path, value):
    with path.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())


def validate_dependencies(parameters):
    for name, expected in KEEP.items():
        if not same(parameters.get(name), expected):
            raise ValueError('Unexpected retained parameter: '+name)
    for name in DEPENDENCIES:
        value = parameters.get(name)
        if value is None or not math.isfinite(value) or value >= 0:
            raise ValueError('Overall override is not disabled: '+name)


def validate_baseline(parameters):
    validate_dependencies(parameters)
    for name, old, _ in PLAN:
        if not same(parameters.get(name), old):
            raise ValueError('Initial value changed; review before writing: '+name)


def validate_delta(before, after):
    validate_dependencies(after)
    for name in PARAMETERS:
        expected = TARGETS.get(name, before[name])
        if not same(after.get(name), expected):
            raise ValueError('Final inventory differs from approved scope: '+name)
    return {n: {'before': before[n], 'after': after[n]} for n in PARAMETERS
            if not same(before[n], after[n])}


def assess_guard(raw, slot=1, expected_nav=1):
    fresh(raw['vehicle_status'], 1.5)
    state = status_fields(raw['vehicle_status'])
    if state['nav_state'] != expected_nav:
        raise ValueError('Actual navigation state does not match the required mode')
    armed = raw['actuator_armed']
    fresh(armed, 1.5)
    if any(boolean(armed, k) for k in ('armed', 'prearmed', 'termination', 'in_esc_calibration_mode')) or not boolean(armed, 'kill'):
        raise ValueError('Require disarmed actuators and actual kill=True')
    rc = raw['manual_control_setpoint']
    fresh(rc, .5)
    if not boolean(rc, 'valid') or number(rc, 'data_source') != 1:
        raise ValueError('Fresh valid RC required')
    if not -1.001 <= number(rc, 'throttle') <= -.95 or any(abs(number(rc, k)) > .05 for k in ('roll', 'pitch', 'yaw')):
        raise ValueError('Require minimum throttle and centered other axes')
    switches = raw['manual_control_switches']
    fresh(switches, 1.5)
    sw = {k: number(switches, k) for k in ('arm_switch', 'mode_slot', 'kill_switch')}
    if (sw['arm_switch'], sw['mode_slot'], sw['kill_switch']) != (3, slot, 1):
        raise ValueError('Require SF off, expected SA slot and SB kill')
    power = raw['system_power']
    fresh(power, .5)
    volts = number(power, 'voltage5v_v')
    if not 4.7 <= volts <= 5.4 or number(power, 'periph_5v_oc') != 0 or number(power, 'hipower_5v_oc') != 0:
        raise ValueError('Invalid 5 V power observation')
    if number(power, 'brick_valid') not in (1, 2, 3) and number(power, 'usb_valid') != 1:
        raise ValueError('No observed valid POWER or USB supply')
    outputs = require_zero(raw['dshot'])
    battery = raw['battery_status']
    fresh(battery, 1)
    connected = boolean(battery, 'connected')
    result = dict(state=state, actual_kill=True, switches=sw, outputs=outputs,
                  voltage5v_v=volts, brick_valid=number(power, 'brick_valid'),
                  usb_connected=number(power, 'usb_connected'), battery_connected=connected)
    if connected:
        if number(battery, 'warning') != 0 or number(battery, 'cell_count') != 6 or not 18 <= number(battery, 'voltage_v') <= 25.3:
            raise ValueError('Unexpected connected 6S battery state')
        result['battery_voltage_v'] = number(battery, 'voltage_v')
        result['esc_telemetry'] = telemetry_snapshot(raw['dshot'], raw['esc_status'])
    return result


def guard(c, slot=1):
    raw = {topic: c.query('listener '+topic+' -n 1') for topic in
           ('vehicle_status', 'actuator_armed', 'manual_control_setpoint',
            'manual_control_switches', 'system_power', 'battery_status')}
    raw['dshot'] = c.query('dshot status')
    if boolean(raw['battery_status'], 'connected'):
        raw['esc_status'] = c.query('listener esc_status -n 1')
    return assess_guard(raw, slot, SLOT_NAV[slot])


class FirstFlightSetupConsole(DebugConsole):
    def __init__(self, port, *, execute=False):
        super().__init__(port)
        self.execute = execute

    def _send(self, command):
        self.s.write((command+'\r').encode('ascii'))
        text = clean(self.record(command, self.read(6)))
        if 'nsh>' not in text or re.search(r'Hard Fault|\bPANIC\b|Assertion failed|\bERROR\b|command not found', text, re.I):
            raise RuntimeError('Console failure after '+command)
        return text

    def query(self, command):
        if command not in READS or self.active_listener:
            raise ValueError('Only fixed first-flight reads are allowed')
        return self._send(command)

    def write_fixed(self, command):
        if not self.execute or command not in WRITES or self.active_listener:
            raise ValueError('Write outside approved eight-parameter scope')
        # Check dependencies before the final live state guard, on every write.
        validate_dependencies({p: read_parameter(self, p) for p in (*KEEP, *DEPENDENCIES)})
        state = guard(self)
        text = self._send(command)
        if re.search(r'\bfailed\b|\bWARN\b', text, re.I):
            raise RuntimeError('Write response requires review: '+command)
        return dict(guard=state, response=text)


def run(port, output, *, execute=False, props_removed=False, verify_slot=None):
    if execute and not APPLY_ENABLED:
        raise ValueError('Proposal withdrawn by user; live application disabled')
    if execute and (not props_removed or verify_slot is not None):
        raise ValueError('Apply requires confirmed removed props and no verification mode')
    if not execute and verify_slot not in SLOT_NAV:
        raise ValueError('Select apply or a fixed read-only verification slot')
    output.mkdir(parents=True, exist_ok=False)
    c = FirstFlightSetupConsole(port, execute=execute)
    report = dict(complete=False, flight_ready=False, utc=datetime.now(timezone.utc).isoformat(),
                  firmware=FIRMWARE, plan=PLAN, props_removed_confirmed=props_removed,
                  parameter_attempts=[], parameter_writes=0, parameter_save_verified=False,
                  powercycle_persistence_verified=False, arm_commands=0, motor_commands=0,
                  explicit_mode_commands=0, nx_accessed=False, verify_slot=verify_slot,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        c.open()
        if FIRMWARE not in c.query('ver all'):
            raise ValueError('Unexpected firmware')
        report['before_state'] = guard(c, verify_slot or 1)
        report['before_parameters'] = {p: read_parameter(c, p) for p in PARAMETERS}
        if execute:
            validate_baseline(report['before_parameters'])
            save_json(output/'rollback_values.json', {n: report['before_parameters'][n] for n in TARGETS})
            save_json(output/'before_parameters.json', report['before_parameters'])
            for name, old, target in PLAN:
                if not same(read_parameter(c, name), old):
                    raise ValueError('Parameter changed during setup: '+name)
                attempt = {'name': name, 'target': target, 'readback_verified': False}
                report['parameter_attempts'].append(attempt)
                save_json(output/'report.json', report)
                attempt['dispatch'] = c.write_fixed(f'param set {name} {target:g}')
                report['parameter_writes'] += 1
                value = read_parameter(c, name)
                attempt['readback'] = value
                if not same(value, target):
                    raise ValueError('Write/readback mismatch: '+name)
                attempt['readback_verified'] = True
            report['after_parameters'] = {p: read_parameter(c, p) for p in PARAMETERS}
            report['changed_parameters'] = validate_delta(report['before_parameters'], report['after_parameters'])
            report['save'] = c.write_fixed('param save')
            # Read the full inventory again after save; save is not a power-cycle proof.
            report['saved_parameters'] = {p: read_parameter(c, p) for p in PARAMETERS}
            validate_delta(report['before_parameters'], report['saved_parameters'])
            report['parameter_save_verified'] = True
        else:
            validate_dependencies(report['before_parameters'])
            for name, target in TARGETS.items():
                if not same(report['before_parameters'][name], target):
                    raise ValueError('Configuration is not the applied first-flight group: '+name)
        report['after_state'] = guard(c, verify_slot or 1)
        if report['after_state']['state']['timestamp'] <= report['before_state']['state']['timestamp']:
            raise ValueError('FC time did not advance')
        report['complete'] = True
    except Exception as exc:
        report['error'] = str(exc)
        report['partial_write_policy'] = 'Stop; preserve attempts and readbacks; no automatic rollback or retry'
    finally:
        c.close()
        report['port_closed'] = not c.s.is_open
        save_json(output/'console.json', c.records)
        report['console_sha256'] = hashlib.sha256((output/'console.json').read_bytes()).hexdigest()
        save_json(output/'report.json', report)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM13')
    p.add_argument('--output', type=Path)
    action = p.add_mutually_exclusive_group()
    action.add_argument('--execute', action='store_true')
    action.add_argument('--verify-slot', type=int, choices=tuple(SLOT_NAV))
    p.add_argument('--props-removed', action='store_true')
    a = p.parse_args()
    if not a.execute and a.verify_slot is None:
        print(json.dumps(dict(plan=PLAN, status='withdrawn', application_enabled=APPLY_ENABLED,
                              hardware_opened=False), indent=2))
        return 0
    if a.output is None:
        p.error('--output required for apply or read-only verification')
    r = run(a.port, a.output, execute=a.execute, props_removed=a.props_removed, verify_slot=a.verify_slot)
    print(json.dumps({k: r[k] for k in ('complete', 'parameter_writes', 'parameter_save_verified',
                     'port_closed', 'changed_parameters', 'after_state', 'error') if k in r}, indent=2), flush=True)
    return 0 if r['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
