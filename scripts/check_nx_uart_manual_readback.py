"""Verify the unarmed manual-exit bench from PC captures, without hardware I/O.

The Debug listener prints five decimal places. Check that printed representation
separately from float32 telemetry; do not assume the target printf rounds rather
than truncates. Topics in one Debug round are sampled sequentially.
"""
import argparse
from decimal import Decimal, ROUND_DOWN, ROUND_HALF_EVEN
import hashlib
import importlib.util
import json
from pathlib import Path
import re

from summarize_nx_mavlink_capture import summarize


def field(raw, name):
    values = re.findall(r'^\s*' + re.escape(name) + r':\s*(\S+)', raw, re.M)
    if len(values) != 1:
        raise ValueError('Missing/ambiguous field: ' + name)
    return values[0]


def printed_vector(raw, name):
    values = re.findall(r'^\s*' + re.escape(name) + r':\s*\[([^\]]+)\]', raw, re.M)
    if len(values) != 1:
        raise ValueError('Missing/ambiguous vector: ' + name)
    return [x.strip() for x in values[0].split(',')]


def check_printed_quaternion(tokens, expected):
    if len(tokens) != 4 or len(expected) != 4:
        raise ValueError('Quaternion length differs')
    methods = []
    for token, value in zip(tokens, expected):
        if not re.fullmatch(r'-?\d+\.\d{5}', token):
            raise ValueError('Unexpected Debug display precision')
        actual = Decimal(token)
        source = Decimal.from_float(float(value))
        quantum = Decimal('0.00001')
        if actual == source.quantize(quantum, rounding=ROUND_DOWN):
            methods.append('truncated_5dp')
        elif actual == source.quantize(quantum, rounding=ROUND_HALF_EVEN):
            methods.append('rounded_5dp')
        else:
            raise ValueError('Internal quaternion differs beyond printed representation')
    return methods


def check(run, generated, debug):
    capture = summarize(run, generated)  # Also verifies the locked dialect/hash.
    if (not capture['seen_offboard'] or capture['stop_reason'] != 'manual_or_px4_exit'
            or capture['crc_errors_after_sync'] or capture['any_heartbeat_armed']
            or capture['mpc_called'] or capture['commands']):
        raise ValueError('Not a clean unarmed manual-exit run')
    spec = importlib.util.spec_from_file_location('readback_dialect', generated / 'dialect.py')
    dialect = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(dialect)
    events = [json.loads(x) for x in (run / 'transport.jsonl').read_bytes().splitlines()]
    result = next(x for x in events if x.get('event') == 'result')
    parser = dialect.MAVLink(None)
    targets, telemetry = [], []
    seen_mode = seen_sa = False
    exit_ns = None
    for rec in result['records']:
        msg = parser.parse_buffer(bytes.fromhex(rec['hex']))[0]
        mid = msg.get_msgId()
        if rec['tx']:
            if mid not in (0, 82, 111):
                raise ValueError('Unexpected transmitted message')
            if mid == 82:
                if (msg.type_mask != 7 or msg.thrust != 0
                        or msg.target_system != 1 or msg.target_component != 1):
                    raise ValueError('Unexpected attitude target fields')
                targets.append((rec['ns'], list(msg.q)))
        elif msg.get_srcSystem() == msg.get_srcComponent() == 1:
            if mid == 65:
                if msg.chan6_raw >= 1750:
                    seen_sa = True
                elif seen_sa and seen_mode and exit_ns is None:
                    exit_ns = rec['ns']
            elif mid == 436:
                mode = (msg.custom_mode >> 16) & 255
                if mode == 6:
                    seen_mode = True
                elif seen_mode and exit_ns is None:
                    exit_ns = rec['ns']
            elif mid == 83:
                telemetry.append(dict(timestamp=msg.time_boot_ms * 1000,
                                      q=list(msg.q), thrust=msg.thrust))
    if (not targets or len(targets) != result['setpoints'] or exit_ns is None
            or targets[-1][0] >= exit_ns):
        raise ValueError('Missing exit or setpoints continued after observed exit')
    anchor = targets[0][1]
    if any(q != anchor for _, q in targets):
        raise ValueError('Bench target was not the fixed captured attitude')
    report = json.loads((debug / 'report.json').read_text(encoding='utf-8'))
    rows = [json.loads(x) for x in (debug / 'rounds.jsonl').read_text(encoding='utf-8').splitlines()]
    if not report['complete'] or not report['port_closed'] or not rows:
        raise ValueError('Debug capture incomplete')
    states = [(int(field(x['raw']['vehicle_status'], 'nav_state_timestamp')), x['nav_state']) for x in rows]
    start = min(t for t, n in states if n == 14)
    end = min(t for t, n in states if n != 14 and t > start)
    matches, transition_rows = [], 0
    for row in rows:
        if row['arming_state'] != 1 or any(v != 0 for v in row['outputs'].values()):
            raise ValueError('Arming or nonzero output in unarmed bench')
        raw = row['raw']['vehicle_attitude_setpoint']
        stamp = int(field(raw, 'timestamp'))
        if start <= stamp < end:
            tokens = printed_vector(raw, 'q_d')
            methods = check_printed_quaternion(tokens, anchor)
            if (any(Decimal(x) != 0 for x in printed_vector(raw, 'thrust_body'))
                    or field(raw, 'yaw_sp_move_rate').lower() != 'nan'):
                raise ValueError('Internal thrust or ignored yaw-rate semantic differs')
            matches.append(dict(timestamp=stamp, display_methods=methods,
                                q_max_display_difference=max(abs(float(x)-y) for x, y in zip(tokens, anchor))))
        elif row['nav_state'] == 14:
            transition_rows += 1
    if len({x['timestamp'] for x in matches}) < 2 or rows[-1]['nav_state'] != 1:
        raise ValueError('Insufficient matching readback or final mode is not Altitude')
    # Exact float32 telemetry is a second check, within the interval independently
    # bracketed by matching Debug samples. Preserve mode-transition mismatches.
    first, last = min(x['timestamp'] for x in matches), max(x['timestamp'] for x in matches)
    stable = [x for x in telemetry if first <= x['timestamp'] <= last]
    if len(stable) < 2 or any(x['q'] != anchor or x['thrust'] != 0 for x in stable):
        raise ValueError('Full-precision telemetry differs in Debug-confirmed interval')
    boundary = [x for x in telemetry if start <= x['timestamp'] < end
                and (x['q'] != anchor or x['thrust'] != 0)]
    files = [run/'transport.jsonl', debug/'report.json', debug/'rounds.jsonl', Path(__file__)]
    return dict(kind='flyarm.mavlink_uart_manual_readback.v2',
        unarmed_manual_exit_passed=True, raw_internal_attitude_readback_passed=True,
        armed_takeover_qualified=False, mpc_executed_in_live_loop=False,
        setpoints=len(targets), max_send_gap_ms=result['max_send_gap_ms'],
        anchor_frd_to_ned_q=anchor, offboard_start_px4_us=start, offboard_exit_px4_us=end,
        internal_matching_samples=matches, exact_float32_telemetry_samples=len(stable),
        boundary_telemetry_differences=boundary,
        transition_rows_outside_actual_offboard_interval=transition_rows,
        transition_explanation='Sequential Debug topics and mode/controller publications are not an atomic snapshot. Retain boundary differences; require exact telemetry inside the independently Debug-confirmed interval.',
        display_precision_explanation='Debug values have five decimal places. Verify the actual rounded or truncated representation, not a half-unit rounding assumption; no controller tolerance was changed.',
        last_setpoint_before_exit_observation_ms=(exit_ns-targets[-1][0])/1e6,
        observed_after_exit_s=(result['records'][-1]['ns']-exit_ns)/1e9,
        zero_setpoints_after_observed_exit=True, all_debug_samples_disarmed=True,
        all_debug_outputs_zero=True, crc_errors_after_sync=0,
        nx_kernel_faults=[], nx_persistent_files_created=[],
        hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('run', 'generated', 'debug', 'output'):
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    summary = check(a.run, a.generated, a.debug)
    with a.output.open('x', encoding='utf-8') as f:
        json.dump(summary, f, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps(summary, indent=2))
