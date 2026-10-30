"""PC-side preflight snapshot for operator-armed, physically unpowered UART tests.

Only fixed diagnostic reads and one RUN_PREARM_CHECKS request are permitted.
No arming, mode, parameter, calibration, reboot or NX operation is implemented.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import time

from px4_debug_console import DebugConsole, FIRMWARE, clean
from px4_imu_pole_check import guard
from px4_rc_switch_setup import boolean, fresh, number, read_parameter

PARAMETERS = ('COM_DISARM_PRFLT', 'COM_DISARM_LAND', 'COM_KILL_DISARM',
              'COM_OF_LOSS_T', 'COM_OBL_RC_ACT', 'COM_RC_OVERRIDE',
              'COM_ARM_WO_GPS', 'CBRK_USB_CHK', 'COM_ARM_HFLT_CHK',
              'COM_CPU_MAX', 'COM_RAM_MAX', 'COM_FLTMODE1', 'COM_FLTMODE6')
TOPICS = ('vehicle_status', 'actuator_armed', 'manual_control_switches',
          'manual_control_setpoint', 'health_report', 'failsafe_flags',
          'battery_status', 'vehicle_land_detected', 'cpuload')
READS = frozenset(('ver all', 'commander status', 'dshot status', 'dmesg') +
                 tuple('listener '+x+' -n 1' for x in TOPICS) +
                 tuple('param show -q '+x for x in PARAMETERS))


class PreflightConsole(DebugConsole):
    def query(self, command):
        if self.active_listener or command not in READS | {'commander check'}:
            raise ValueError('Fixed preflight diagnostics only')
        self.s.write((command+'\r').encode('ascii'))
        # dmesg contains old prompts; do not mistake them for the live response.
        raw = self.read(2, False) if command == 'dmesg' else self.read(6)
        if command == 'commander check':
            raw += self.read(.6, False)  # Preflight event messages may follow prompt.
        text = clean(self.record(command, raw))
        if 'nsh>' not in text:
            raise RuntimeError('No console prompt after '+command)
        return text


def assess(raw):
    status, armed = raw['vehicle_status'], raw['actuator_armed']
    rc, sw, health = raw['manual_control_setpoint'], raw['manual_control_switches'], raw['health_report']
    # health_report is event-published: its age alone is not a failure. Fresh
    # vehicle_status provides the current selected-mode preflight result.
    for topic, age in ((status, 1.5), (armed, 1.5), (rc, .5), (sw, 1.5)):
        fresh(topic, age)
    checks = dict(
        disarmed=number(status, 'arming_state') == 1 and not boolean(armed, 'armed'),
        altitude_mode=number(status, 'nav_state') == 1,
        no_hil_or_failsafe=number(status, 'hil_state') == 0 and not boolean(status, 'failsafe'),
        preflight_pass=boolean(status, 'pre_flight_checks_pass'),
        altitude_can_arm=bool(int(number(health, 'can_arm_mode_flags')) & (1 << 1)),
        kill_released=not boolean(armed, 'kill'),
        no_actuator_override=not any(boolean(armed, x) for x in
            ('prearmed', 'lockdown', 'termination', 'in_esc_calibration_mode')),
        rc_valid_minimum_throttle=boolean(rc, 'valid') and number(rc, 'throttle') <= -.95,
        sticks_centered=all(abs(number(rc, x)) <= .05 for x in ('roll', 'pitch', 'yaw')),
        sf_off_sa_up_sb_released=tuple(int(number(sw, x)) for x in
            ('arm_switch', 'mode_slot', 'kill_switch')) == (3, 1, 3),
    )
    return dict(ready=all(checks.values()), checks=checks,
                failed_checks=[k for k, v in checks.items() if not v],
                health_error_flags=int(number(health, 'health_error_flags')),
                arming_check_error_flags=int(number(health, 'arming_check_error_flags')))


def collect(port, output):
    output.mkdir(parents=True, exist_ok=False)
    c = PreflightConsole(port)
    report = dict(complete=False, ready=False, utc=datetime.now(timezone.utc).isoformat(),
                  firmware=FIRMWARE, operator_power_disconnected=True,
                  arm_commands=0, mode_commands=0, parameter_writes=0, nx_accessed=False)
    try:
        c.open()
        if FIRMWARE not in c.query('ver all'):
            raise ValueError('Unexpected firmware')
        report['before'] = guard(c, allow_unpublished_battery=True)
        report['parameters'] = {p: read_parameter(c, p) for p in PARAMETERS}
        report['refresh'] = c.query('commander check')
        report['preflight_refresh_requests'] = 1
        time.sleep(.3)
        report['dmesg'] = c.query('dmesg')
        report['raw'] = {x: c.query('listener '+x+' -n 1') for x in TOPICS}
        report['assessment'] = assess(report['raw'])
        report['after'] = guard(c, allow_unpublished_battery=True)
        if report['after']['state']['timestamp'] <= report['before']['state']['timestamp']:
            raise ValueError('Flight-controller time failed to advance')
        report['ready'] = report['assessment']['ready']
        report['complete'] = True
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        c.close()
        report['port_closed'] = not c.s.is_open
        (output/'console.json').write_text(json.dumps(c.records, indent=2), encoding='utf-8')
        (output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM13')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--power-disconnected', action='store_true', required=True)
    a = p.parse_args()
    report = collect(a.port, a.output)
    print(json.dumps(report), flush=True)
    return 0 if report['complete'] and report['ready'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
