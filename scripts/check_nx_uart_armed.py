"""Offline verification of operator-armed, power-disconnected UART evidence.

Require actual armed mode transitions, independent original PX4 fields, and
unconfounded RC inputs. Never classify ground auto-disarm as armed fallback.
"""
import argparse
from decimal import Decimal
import hashlib
import importlib.util
import json
from pathlib import Path

from check_nx_uart_manual_readback import field, printed_vector, check_printed_quaternion
from nx_uart_operator_panel import restored
from summarize_nx_mavlink_capture import summarize


def qualified_transition(rows, case):
    active = []
    for i, row in enumerate(rows):
        sw = row['raw']['manual_control_switches']
        flags = row['raw']['failsafe_flags']
        controls = (field(sw, 'arm_switch'), field(sw, 'mode_slot'), field(sw, 'kill_switch'))
        if row['arming_state'] == 2 and row['nav_state'] == 14 and not row['kill'] and controls == ('1', '6', '3'):
            if field(flags, 'offboard_control_signal_lost') == 'False':
                active.append(i)
    if not active:
        raise ValueError('No independent armed, valid Offboard interval')
    first = active[0]
    for i in range(first+1, len(rows)):
        row = rows[i]
        sw = row['raw']['manual_control_switches']
        controls = (field(sw, 'arm_switch'), field(sw, 'mode_slot'), field(sw, 'kill_switch'))
        if row['arming_state'] != 2 or row['kill'] or controls[0] != '1' or controls[2] != '3':
            raise ValueError('Disarm or kill intervened before qualified exit')
        if row['nav_state'] == 14:
            if case == 'stream_stop' and controls[1] != '6':
                raise ValueError('SA moved before automatic fallback')
            continue
        if case == 'manual_exit':
            if row['nav_state'] != 1 or controls[1] != '1':
                raise ValueError('Not an armed SA return to Altitude')
        else:
            flags = row['raw']['failsafe_flags']
            if controls[1] != '6' or field(flags, 'offboard_control_signal_lost') != 'True':
                raise ValueError('No unchanged-SA Offboard-loss exit')
            if field(flags, 'manual_control_signal_lost') != 'False':
                raise ValueError('RC loss confounds target-stream loss')
            if field(row['raw']['vehicle_status'], 'failsafe') != 'True':
                raise ValueError('No independently confirmed failsafe transition')
        return first, i
    raise ValueError('No independent armed exit')


def post_exit_observations(rows, after):
    """Separate an armed re-entry from PX4's disarmed intended-mode display.

    The pinned PX4 failsafe framework selects Action::None after disarm, so
    SA still down can expose nav_state=14 with no valid Offboard signal. This
    is recorded explicitly, never treated as resumed NX control. The full
    wire capture separately enforces no further target transmission.
    """
    unarmed = []
    disarmed_seen = False
    for row in rows[after+1:]:
        if row['arming_state'] not in (1, 2):
            raise ValueError('Unknown post-exit arming state')
        if row['arming_state'] == 1:
            disarmed_seen = True
        elif disarmed_seen:
            raise ValueError('Second arming attempt after exit')
        if row['nav_state'] != 14:
            continue
        raw = row['raw']
        if (row['arming_state'] != 1
                or field(raw['actuator_armed'], 'armed') != 'False'
                or field(raw['manual_control_switches'], 'arm_switch') != '3'
                or field(raw['failsafe_flags'], 'offboard_control_signal_lost') != 'True'
                or field(raw['vehicle_status'], 'nav_state_user_intention') != '14'):
            raise ValueError('Armed or signal-valid Offboard re-entry after exit')
        unarmed.append(dict(px4_timestamp_us=row['stamp'], nav_state=14,
                            arming_state=1, sf='off', offboard_signal_lost=True,
                            sa_mode_slot=field(raw['manual_control_switches'], 'mode_slot')))
    return unarmed


def check(directory, generated, case):
    run = directory/'nx'
    capture = summarize(run, generated)
    reason = 'manual_or_px4_exit' if case == 'manual_exit' else 'planned_stream_stop'
    if (not capture['seen_offboard'] or capture['stop_reason'] != reason
            or capture['crc_errors_after_sync'] or not capture['any_heartbeat_armed']
            or capture['mpc_called']):
        raise ValueError('Not a complete armed UART case')
    request = json.loads((run/'request.json').read_text())
    if request['case'] != case or not request['power_disconnected']:
        raise ValueError('Case or power-isolation attestation differs')
    spec = importlib.util.spec_from_file_location('armed_dialect', generated/'dialect.py')
    dialect = importlib.util.module_from_spec(spec); spec.loader.exec_module(dialect)
    events = [json.loads(x) for x in (run/'transport.jsonl').read_bytes().splitlines()]
    result = next(x for x in events if x.get('event') == 'result')
    parser = dialect.MAVLink(None)
    targets, telemetry, rc = [], [], []
    entry = exit_ns = None
    command_count = 0
    for rec in result['records']:
        msg = parser.parse_buffer(bytes.fromhex(rec['hex']))[0]
        mid = msg.get_msgId()
        if rec['tx']:
            if mid not in (0, 82, 111, 76):
                raise ValueError('Unexpected transmitted message')
            if mid == 76:
                allowed = {(31, 20000), (65, 50000), (436, 100000), (230, 100000),
                           (245, 500000), (1, 500000), (147, 500000), (83, 50000)}
                if (msg.command != 511 or (msg.param1, msg.param2) not in allowed
                        or msg.target_system != 1 or msg.target_component != 1
                        or any(getattr(msg, 'param'+str(n)) != 0 for n in range(3, 8))):
                    raise ValueError('TX command outside approved temporary frequency requests')
                command_count += 1
            if mid == 82:
                if (msg.type_mask != 7 or msg.thrust != 0 or msg.target_system != 1 or msg.target_component != 1):
                    raise ValueError('Unexpected attitude target fields')
                targets.append((rec['ns'], list(msg.q)))
        elif msg.get_srcSystem() == msg.get_srcComponent() == 1:
            if mid == 65:
                rc.append((rec['ns'], msg.chan5_raw, msg.chan6_raw, msg.chan7_raw))
                if case == 'manual_exit' and entry is not None and msg.chan6_raw < 1750 and exit_ns is None:
                    exit_ns = rec['ns']
            elif mid == 436:
                mode = (msg.custom_mode >> 16) & 255
                if mode == 6 and entry is None:
                    entry = rec['ns']
                elif mode != 6 and entry is not None and exit_ns is None:
                    exit_ns = rec['ns']
            elif mid == 83:
                telemetry.append((msg.time_boot_ms*1000, list(msg.q), msg.thrust))
    if command_count != (8 if request['configure'] else 0):
        raise ValueError('Unexpected stream configuration command count')
    if not targets or entry is None or exit_ns is None or targets[-1][0] >= exit_ns:
        raise ValueError('Targets did not stop before observed exit')
    anchor = targets[0][1]
    if len(targets) != result['setpoints'] or any(q != anchor for _, q in targets):
        raise ValueError('Not a fixed initial attitude')
    if (result['records'][-1]['ns']-targets[-1][0])/1e9 < 10:
        raise ValueError('Insufficient stopped observation')
    if case == 'stream_stop':
        if not 2.98 <= (targets[-1][0]-entry)/1e9 <= 3.04:
            raise ValueError('Planned stop differs from three-second condition')
        during = [x for x in rc if entry <= x[0] <= exit_ns]
        if not during or any(sf < 1750 or sa < 1750 or sb > 1750 for _, sf, sa, sb in during):
            raise ValueError('High-rate RC shows intervention before automatic exit')
    report = json.loads((directory/'debug/report.json').read_text(encoding='utf-8'))
    rows = [json.loads(x) for x in (directory/'debug/rounds.jsonl').read_text(encoding='utf-8').splitlines()]
    if not report['complete'] or not report['port_closed'] or not report['operator_arming_allowed']:
        raise ValueError('Independent armed capture incomplete')
    first, after = qualified_transition(rows, case)
    start_stamp = int(field(rows[first]['raw']['vehicle_status'], 'nav_state_timestamp'))
    end_stamp = int(field(rows[after]['raw']['vehicle_status'], 'nav_state_timestamp'))
    matches = []
    for row in rows:
        raw = row['raw']['vehicle_attitude_setpoint']
        stamp = int(field(raw, 'timestamp'))
        if start_stamp <= stamp < end_stamp:
            tokens = printed_vector(raw, 'q_d')
            methods = check_printed_quaternion(tokens, anchor)
            if any(Decimal(x) != 0 for x in printed_vector(raw, 'thrust_body')) or field(raw, 'yaw_sp_move_rate').lower() != 'nan':
                raise ValueError('Internal target semantics differ')
            matches.append(dict(timestamp=stamp, display_methods=methods))
    if len({x['timestamp'] for x in matches}) < 2:
        raise ValueError('Need two independent original-target samples')
    low, high = min(x['timestamp'] for x in matches), max(x['timestamp'] for x in matches)
    exact = [x for x in telemetry if low <= x[0] <= high]
    if len(exact) < 2 or any(q != anchor or thrust != 0 for _, q, thrust in exact):
        raise ValueError('Full float32 target telemetry differs')
    restore = json.loads((directory/'restore/report.json').read_text(encoding='utf-8'))
    if not restored(restore):
        raise ValueError('Safe final restoration not confirmed')
    unarmed_mode_restore = post_exit_observations(rows, after)
    preflight = json.loads((directory/'preflight/report.json').read_text())
    if not preflight['complete'] or not preflight['ready']:
        raise ValueError('Initial selected-mode preflight not passed')
    files = [run/'transport.jsonl', directory/'debug/rounds.jsonl', directory/'debug/report.json',
             directory/'preflight/report.json', directory/'restore/report.json', Path(__file__)]
    return dict(kind='flyarm.armed_uart_bench.v2', case=case, passed=True,
        power_disconnected=True, automatic_arm_commands=0, automatic_mode_commands=0,
        persistent_parameter_writes=0, temporary_stream_requests=command_count,
        mpc_executed=False, flight_qualified=False, safe_final_restoration=True,
        actual_exit_nav_state=rows[after]['nav_state'], actual_armed_through_exit=True,
        nx_target_stream_restarted=False, armed_offboard_reentry=False,
        unarmed_offboard_mode_observations=unarmed_mode_restore,
        offboard_start_px4_us=start_stamp, offboard_exit_px4_us=end_stamp,
        offboard_duration_s=(end_stamp-start_stamp)/1e6,
        setpoints=len(targets), max_send_gap_ms=result['max_send_gap_ms'],
        anchor_frd_to_ned_q=anchor, internal_matching_samples=matches,
        exact_float32_telemetry_samples=len(exact), crc_errors_after_sync=0,
        observed_after_last_target_s=(result['records'][-1]['ns']-targets[-1][0])/1e9,
        last_target_to_exit_observation_s=(exit_ns-targets[-1][0])/1e9,
        timing_note='NX receipt times and sequential Debug samples include sampling/telemetry delay; not a precise failsafe response-time measurement.',
        tested_binary_sha256=capture['tested_binary_sha256'],
        nx_kernel_faults=capture['kernel_faults'], nx_persistent_files_created=[],
        hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory', type=Path, required=True)
    p.add_argument('--generated', type=Path, required=True)
    p.add_argument('--case', choices=('manual_exit', 'stream_stop'), required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    summary = check(a.directory, a.generated, a.case)
    with a.output.open('x', encoding='utf-8') as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
