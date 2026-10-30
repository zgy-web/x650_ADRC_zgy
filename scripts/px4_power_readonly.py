"""Fixed read-only, disarmed POWER/USB observation; no flight authorization.

Run a combined-power baseline, then (only after the operator unplugs USB) a
power-only window. Serial data and results are saved only on the PC. A baseline
from another computer/boot/session must not be used for uptime comparison.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import time

from px4_debug_console import FIRMWARE
from px4_first_flight_readonly import FirstFlightConsole, parked, observed
from px4_rc_switch_setup import boolean, field, fresh, number

TOPICS = ('system_power', 'battery_status', 'manual_control_setpoint',
          'manual_control_switches')


def clock_interval(raw, start, end):
    stamp = fresh(raw, .5)
    age = re.findall(r'^\s*timestamp:\s*\d+\s*\(([^ ]+) seconds ago\)', raw, re.M)
    if len(age) != 1 or end < start:
        raise ValueError('Invalid clock observation')
    age = float(age[0])
    # The message print time is bounded by the serial command/response interval.
    return [start-stamp/1e6-age, end-stamp/1e6-age]


def consistent_clock(a, b, tolerance=.25):
    return max(a[0], b[0]) <= min(a[1], b[1])+tolerance


def assessment(raw, phase, standby, times):
    power, rc, sw = (raw[x] for x in
                     ('system_power', 'manual_control_setpoint', 'manual_control_switches'))
    fresh(power, .5)
    fresh(rc, .5)
    fresh(sw, 1.5)
    if not standby['kill']:
        raise ValueError('Actual kill flag is not active')
    if (not boolean(rc, 'valid') or number(rc, 'throttle') > -.95 or
        number(rc, 'throttle') < -1.001 or
        any(abs(number(rc, k)) > .05 for k in ('roll', 'pitch', 'yaw'))):
        raise ValueError('Require valid RC, minimum throttle and centered other axes')
    if tuple(number(sw, k) for k in ('arm_switch', 'mode_slot', 'kill_switch')) != (3, 1, 1):
        raise ValueError('Require SF off, SA upper and SB kill')
    values = {k:number(power, k) for k in ('voltage5v_v','brick_valid','usb_connected',
              'usb_valid','periph_5v_oc','hipower_5v_oc')}
    usb = 1 if phase == 'combined' else 0
    checks = dict(
        brick_detected=values['brick_valid'] in (1, 2, 3),
        usb_matches_phase=values['usb_connected'] == usb and values['usb_valid'] == usb,
        rail_within_pinned_px4_limits=4.7 <= values['voltage5v_v'] <= 5.4,
        no_reported_overcurrent=values['periph_5v_oc'] == 0 and values['hipower_5v_oc'] == 0,
    )
    # Battery monitor readiness is independent of proving an auxiliary 5V supply.
    battery = observed(raw['battery_status'], ('connected','voltage_v','cell_count',
                       'remaining','warning','current_a','source'))
    try:
        fresh(raw['battery_status'], 1.5)
        battery_ok = (boolean(raw['battery_status'], 'connected') and
                      number(raw['battery_status'], 'cell_count') == 6 and
                      number(raw['battery_status'], 'warning') == 0 and
                      0 <= number(raw['battery_status'], 'remaining') <= 1 and
                      0 < number(raw['battery_status'], 'voltage_v') <= 26)
    except ValueError:
        battery_ok = False
    return dict(power=values, checks=checks, power_ok=all(checks.values()),
                battery=battery, battery_monitor_usable=battery_ok,
                clock_interval=clock_interval(power, *times))


def load_baseline(path):
    r = json.loads(path.read_text(encoding='utf-8'))
    if (r.get('phase') != 'combined' or not r.get('complete') or
        not r.get('window_verified') or not r.get('port_closed') or
        r.get('firmware') != FIRMWARE or r.get('observed_duration_s', 0) < 10):
        raise ValueError('Need a completed combined-power baseline of at least 10 seconds')
    return r


def collect(port, output, phase, seconds, baseline_path=None):
    if phase not in ('combined', 'power_only') or not 10 <= seconds <= 120:
        raise ValueError('Use combined/power_only and a 10–120 second window')
    if phase == 'power_only' and (baseline_path is None or seconds < 60):
        raise ValueError('Power-only proof requires a baseline and at least 60 seconds')
    baseline = load_baseline(baseline_path) if phase == 'power_only' else None
    output.mkdir(parents=True, exist_ok=False)
    c = FirstFlightConsole(port)
    report = dict(complete=False, window_verified=False, independent_power_verified=False, flight_ready=False,
                  phase=phase, firmware=FIRMWARE, seconds_requested=seconds,
                  utc=datetime.now(timezone.utc).isoformat(), samples=[],
                  arm_commands=0, mode_commands=0, parameter_writes=0, motor_commands=0,
                  nx_accessed=False, operator_disconnect_required=(phase == 'power_only'),
                  scope='Sampled idle POWER validation; not a transient/loaded-flight power test',
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    if baseline:
        report['baseline_sha256'] = hashlib.sha256(baseline_path.read_bytes()).hexdigest()
    try:
        c.open()
        if FIRMWARE not in c.query('ver all'):
            raise ValueError('Unexpected firmware')
        first_end = None
        reference = baseline['samples'][-1]['assessment']['clock_interval'] if baseline else None
        previous_end = baseline['samples'][-1]['host_end_s'] if baseline else None
        start = time.monotonic()
        while True:
            if time.monotonic()-start > seconds+15:
                raise ValueError('Observation could not finish in its bounded window')
            standby = parked(c)
            raw, timing = {}, None
            for topic in TOPICS:
                begin = time.monotonic()
                raw[topic] = c.query('listener '+topic+' -n 1')
                end = time.monotonic()
                if topic == 'system_power': timing = (begin, end)
            result = assessment(raw, phase, standby, timing)
            end = time.monotonic()
            sample = dict(host_end_s=end, standby=standby, raw=raw, assessment=result)
            report['samples'].append(sample)
            if previous_end is not None and end <= previous_end:
                raise ValueError('Host monotonic clock did not advance; baseline not comparable')
            if not result['power_ok']:
                raise ValueError('Power conditions not met: '+', '.join(
                    k for k,v in result['checks'].items() if not v))
            if reference is not None and not consistent_clock(reference, result['clock_interval']):
                raise ValueError('FC uptime/host clock relationship changed; possible reboot')
            if reference is None: reference = result['clock_interval']
            if first_end is None: first_end = end
            previous_end = end
            report['observed_duration_s'] = end-first_end
            if end-first_end >= seconds: break
            time.sleep(.5)
        report['after'] = parked(c)
        if not report['after']['kill']:
            raise ValueError('Kill flag changed at end of observation')
        if report['after']['state']['timestamp'] <= report['samples'][-1]['standby']['state']['timestamp']:
            raise ValueError('FC time did not advance at end of observation')
        report['complete'] = report['window_verified'] = True
        report['independent_power_verified'] = phase == 'power_only'
        report['battery_monitor_usable_all_samples'] = all(
            x['assessment']['battery_monitor_usable'] for x in report['samples'])
        volts = [x['assessment']['power']['voltage5v_v'] for x in report['samples']]
        report['rail_min_max_v'] = [min(volts), max(volts)]
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        c.close()
        report['port_closed'] = not c.s.is_open
        (output/'console.json').write_text(json.dumps(c.records,indent=2),encoding='utf-8')
        (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM13')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--phase', choices=('combined','power_only'), required=True)
    p.add_argument('--seconds', type=int, required=True)
    p.add_argument('--baseline', type=Path)
    a = p.parse_args()
    r = collect(a.port, a.output, a.phase, a.seconds, a.baseline)
    print(json.dumps({k:v for k,v in r.items() if k not in ('samples','after')},indent=2),flush=True)
    return 0 if r['complete'] and r['window_verified'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
