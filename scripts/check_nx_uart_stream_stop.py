"""Check planned target-stream interruption and restoration from PC evidence.

An unarmed Offboard-loss flag is not an armed failsafe qualification. This script
reports actual mode behaviour instead of requiring or inventing automatic exit.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

from check_nx_uart_manual_readback import field
from nx_uart_operator_panel import restored
from summarize_nx_mavlink_capture import summarize


def check(directory, generated):
    run, debug = directory/'nx', directory/'debug'
    capture = summarize(run, generated)
    if (not capture['seen_offboard'] or capture['stop_reason'] != 'planned_stream_stop'
            or capture['crc_errors_after_sync'] or capture['any_heartbeat_armed']
            or capture['mpc_called'] or capture['commands']):
        raise ValueError('Not a clean unarmed planned-stop capture')
    spec = importlib.util.spec_from_file_location('stream_stop_dialect', generated/'dialect.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    events = [json.loads(x) for x in (run/'transport.jsonl').read_bytes().splitlines()]
    result = next(x for x in events if x.get('event') == 'result')
    parser = module.MAVLink(None)
    tx, offboard_ns = [], None
    for rec in result['records']:
        msg = parser.parse_buffer(bytes.fromhex(rec['hex']))[0]
        if rec['tx']:
            if msg.get_msgId() not in (0, 82, 111):
                raise ValueError('Unexpected transmitted message')
            if msg.get_msgId() == 82:
                if msg.thrust != 0 or msg.type_mask != 7 or msg.target_system != 1 or msg.target_component != 1:
                    raise ValueError('Unexpected attitude target')
                tx.append((rec['ns'], list(msg.q)))
        elif (msg.get_msgId() == 436 and msg.get_srcSystem() == msg.get_srcComponent() == 1
                and ((msg.custom_mode >> 16) & 255) == 6 and offboard_ns is None):
            offboard_ns = rec['ns']
    if not tx or offboard_ns is None or any(q != tx[0][1] for _, q in tx):
        raise ValueError('Missing observed Offboard or nonconstant bench target')
    duration = (tx[-1][0]-offboard_ns)/1e9
    # 20 ms scheduling quantization around the fixed three-second condition.
    if not 2.98 <= duration <= 3.04:
        raise ValueError('Stop timing differs from three-second test condition')
    after_stop = (result['records'][-1]['ns']-tx[-1][0])/1e9
    if after_stop < 10:
        raise ValueError('Insufficient observation after last target')
    report = json.loads((debug/'report.json').read_text(encoding='utf-8'))
    rows = [json.loads(x) for x in (debug/'rounds.jsonl').read_text(encoding='utf-8').splitlines()]
    if not report['complete'] or not report['port_closed'] or not rows:
        raise ValueError('Independent readback incomplete')
    records = []
    for row in rows:
        if row['arming_state'] != 1 or any(v != 0 for v in row['outputs'].values()):
            raise ValueError('Unexpected arming/output in unarmed test')
        flags, mode = row['raw']['failsafe_flags'], row['raw']['offboard_control_mode']
        records.append(dict(nav_state=row['nav_state'],
            status_stamp=int(field(row['raw']['vehicle_status'], 'timestamp')),
            flags_stamp=int(field(flags, 'timestamp')),
            ocm_stamp=int(field(mode, 'timestamp')),
            lost=field(flags, 'offboard_control_signal_lost') == 'True'))
    active = [x for x in records if x['nav_state'] == 14 and not x['lost']]
    if not active:
        raise ValueError('No independently observed valid Offboard period')
    lost = [x for x in records if x['flags_stamp'] > active[0]['flags_stamp'] and x['lost']]
    if not lost:
        raise ValueError('No subsequent independent loss flag')
    final_ocm_stamp = records[-1]['ocm_stamp']
    if any(x['ocm_stamp'] != final_ocm_stamp for x in lost):
        raise ValueError('Offboard control stream resumed after observed loss')
    if lost[-1]['flags_stamp']-lost[0]['flags_stamp'] < 10_000_000:
        raise ValueError('Loss observation too short')
    restoration = json.loads((directory/'restore/report.json').read_text(encoding='utf-8'))
    if not restored(restoration):
        raise ValueError('Safe operator restoration not confirmed')
    restored_rows = [json.loads(x) for x in (directory/'restore/rounds.jsonl').read_text(encoding='utf-8').splitlines()]
    if not restored_rows or any(x['arming_state'] != 1 or x['nav_state'] != 1 or not x['kill']
                               or any(v != 0 for v in x['outputs'].values()) for x in restored_rows):
        raise ValueError('Restoration observations inconsistent')
    sources = dict(
        unarmed_action_selection='https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/failsafe/framework.cpp#L404-L420',
        offboard_loss_detection='https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp#L36-L62')
    files = [run/'transport.jsonl', debug/'rounds.jsonl', debug/'report.json',
             directory/'restore/report.json', directory/'restore/rounds.jsonl', Path(__file__)]
    return dict(kind='flyarm.unarmed_uart_stream_stop.v1',
        planned_stop_and_loss_detection_passed=True, armed_failsafe_qualified=False,
        automatic_mode_exit_observed=any(x['nav_state'] != 14 for x in lost),
        unarmed_mode_states_after_loss=sorted({x['nav_state'] for x in lost}),
        explanation='This firmware selects Action::None while unarmed; loss detection is verified but armed automatic fallback remains untested.',
        setpoints=len(tx), observed_offboard_to_last_target_s=duration,
        recorded_after_last_target_s=after_stop, max_send_gap_ms=result['max_send_gap_ms'],
        last_px4_offboard_control_mode_stamp_us=final_ocm_stamp,
        first_lost_flag_stamp_us=lost[0]['flags_stamp'],
        first_lost_sample_age_s=(lost[0]['flags_stamp']-final_ocm_stamp)/1e6,
        loss_timing_note='Sequential Debug sampling gives an observation time, not the exact instant when the flag changed.',
        continuous_loss_observation_s=(lost[-1]['flags_stamp']-lost[0]['flags_stamp'])/1e6,
        target_stream_resumed=False, all_debug_samples_disarmed=True, all_debug_outputs_zero=True,
        restored_disarmed_altitude_kill_true_outputs_zero=True,
        preflight_pass_after_restore=field(restoration['final_raw']['vehicle_status'], 'pre_flight_checks_pass') == 'True',
        crc_errors_after_sync=0, nx_kernel_faults=[], nx_persistent_files_created=[],
        mpc_executed_in_live_loop=False, firmware_sources=sources,
        hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('directory', 'generated', 'output'):
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    result = check(a.directory, a.generated)
    with a.output.open('x', encoding='utf-8') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps(result, indent=2))
