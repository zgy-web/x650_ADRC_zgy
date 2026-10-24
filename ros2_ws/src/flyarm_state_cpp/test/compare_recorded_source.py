"""Offline C++ source parity against a completed original run, never a flight.

The old bag contains position/attitude CDR but no angular-rate/flags CDR. Those
two are explicitly checked from preserved original raw-field/receipt snapshots.
CSV visibility is held to each recorded selected joint and last sent-packet
prefix, so this test does not retroactively reveal a packet flushed later.
"""
import argparse
import base64
import csv
import hashlib
import json
from pathlib import Path
import sqlite3
import subprocess

import numpy as np
from rclpy.serialization import deserialize_message
from flyarm_control_interfaces.msg import ControlSnapshot
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude


def identity(path):
    raw = path.read_bytes()
    return {"path": str(path.resolve()), "bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest()}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--executable', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    run = args.run
    source_paths = [run/'isolated_live/feedback_sources.json', run/'isolated_live/recording/diagnostics.jsonl',
                    run/'hover_reference.json', run/'estimated_feedback.json', run/'ts20_state.csv',
                    run/'ts20_packets.csv', run/'bag/bag_0.db3']
    identities = [identity(path) for path in source_paths]
    feedback = {float(r['sim_s']): r['feedback'] for r in json.loads(source_paths[0].read_text())}
    reference = json.loads(source_paths[2].read_text())['px4_reference_at_bind']
    extra = json.loads(source_paths[3].read_text())['extra_reset_counters_at_bind']
    bound = {**reference, **extra}
    packets = list(csv.DictReader(source_paths[5].open()))
    packet_times = np.array([float(p['send_s']) for p in packets])
    period = float(packet_times[1]-packet_times[0])
    delay = float(packets[0]['apply_s'])-float(packets[0]['send_s'])
    snapshots = []
    with source_paths[1].open() as f:
        for line in f:
            row = json.loads(line)
            if 'snapshot_cdr' in row:
                snapshots.append(deserialize_message(base64.b64decode(row['snapshot_cdr']['base64']), ControlSnapshot))
    if not snapshots:
        raise ValueError('No original admitted snapshot CDR')
    db = sqlite3.connect('file:'+str(source_paths[-1])+'?mode=ro', uri=True)
    bag = {}
    for name, cls in [('position', VehicleLocalPosition), ('attitude', VehicleAttitude)]:
        topic = '/fmu/out/vehicle_local_position_v1' if name == 'position' else '/fmu/out/vehicle_attitude'
        ident = db.execute('SELECT id FROM topics WHERE name=?', (topic,)).fetchone()[0]
        records = {}
        for (raw,) in db.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (ident,)):
            msg = deserialize_message(raw, cls)
            records[int(msg.timestamp)] = msg
        bag[name] = records
    db.close()
    topic_order = ['position', 'attitude', 'angular_velocity', 'estimator_flags', 'clock']
    origin_keys = {'lat': 'ref_lat', 'lon': 'ref_lon', 'alt': 'ref_alt', 'ref_timestamp': 'ref_timestamp',
                   'xy_global': 'xy_global', 'z_global': 'z_global', 'xy_reset': 'xy_reset_counter',
                   'z_reset': 'z_reset_counter', 'heading_reset': 'heading_reset_counter',
                   'vxy_reset': 'vxy_reset_counter', 'vz_reset': 'vz_reset_counter', 'quat_reset': 'quat_reset_counter'}
    cases = []
    raw_bag_matches = {'position': 0, 'attitude': 0}
    for s in snapshots:
        fb = feedback[s.control_time_s]
        raw = fb['px4_snapshot']
        o = {**raw['origin'], **raw['extra_reset_counters']}
        available_packet = s.packet_send_time_s[s.packet_count-1] + (s.control_time_s-s.task_time_s)
        packet_index = int(np.argmin(abs(packet_times-available_packet)))
        assert abs(packet_times[packet_index]-available_packet) <= 1e-10
        values = {'now': s.control_time_s, 'task': s.task_time_s,
                  'wall': fb['callbacks']['clock']['received_wall_s'] + fb['callbacks']['clock']['wall_age_s'],
                  'state_cutoff': s.joint_time_s, 'packet_cutoff': packet_times[packet_index]}
        for prefix, doc in [('b_', bound), ('o_', o)]:
            values.update({prefix+k: float(doc[key]) for k, key in origin_keys.items()})
        for prefix, name in [('p', 'position_ned_m'), ('v', 'velocity_ned_m_s'), ('a', 'acceleration_ned_m_s2'),
                             ('quat', 'attitude_wxyz'), ('omega', 'angular_velocity_frd_rad_s')]:
            values.update({prefix+str(i): v for i, v in enumerate(raw[name])})
        values.update({k: float(v) for k, v in raw['valid'].items()})
        values['dead_reckoning'] = float(raw['dead_reckoning'])
        values.update({k: float(v) for k, v in raw['estimator_flags'].items()})
        for i, name in enumerate(topic_order):
            r = fb['callbacks'][name]
            values.update({f'rc{i}_seq': r['sequence'], f'rc{i}_sim': r['received_sim_s'], f'rc{i}_wall': r['received_wall_s']})
        for name in ('position', 'attitude'):
            stamp = fb['px4_raw_timestamps'][name]['timestamp']
            m = bag[name].get(stamp)
            if m is None:
                # A best-effort recorder may miss a raw packet. Its absence is
                # accounted for, never filled with a different-time message.
                continue
            if name == 'position':
                for fields, key in [(('x', 'y', 'z'), 'position_ned_m'), (('vx', 'vy', 'vz'), 'velocity_ned_m_s'),
                                    (('ax', 'ay', 'az'), 'acceleration_ned_m_s2')]:
                    assert list(map(lambda n: float(getattr(m, n)), fields)) == raw[key]
                for key in ('ref_lat', 'ref_lon', 'ref_alt', 'ref_timestamp', 'xy_reset_counter', 'z_reset_counter', 'heading_reset_counter'):
                    assert getattr(m, key) == o[key]
            else:
                assert list(m.q) == raw['attitude_wxyz'] and m.quat_reset_counter == o['quat_reset_counter']
            raw_bag_matches[name] += 1
        cases.append(values)
    fixture = args.output/'original_selected_inputs.csv'
    with fixture.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(cases[0]));writer.writeheader();writer.writerows(cases)
    result = args.output/'cpp_converted.jsonl'
    completed = subprocess.run([str(args.executable), str(source_paths[4]), str(source_paths[5]), str(fixture),
                                repr(period), repr(delay), str(result)], capture_output=True, text=True)
    (args.output/'conversion_stdout.txt').write_text(completed.stdout+completed.stderr)
    if completed.returncode:
        raise RuntimeError(completed.stderr)
    converted = [json.loads(line) for line in result.read_text().splitlines()]
    assert len(converted) == len(snapshots)
    maxima = {}
    for i, (c, s) in enumerate(zip(converted, snapshots)):
        expected = {'p': s.position_enu, 'v': s.velocity_enu, 'a': s.acceleration_enu, 'R': s.rotation_row_major,
                    'omega': s.omega_body_flu, 'state_time': s.state_time_s, 'joint_time': s.joint_time_s,
                    'q': s.q, 'dq': s.dq, 'r': s.reference_r, 'reference_time': s.reference_time_s, 'bus_origin': s.bus_origin_s,
                    'commands': [[s.command_time_s[j], *s.command_velocity[2*j:2*j+2]] for j in range(s.command_count)],
                    'packets': [[s.packet_send_time_s[j], *s.packet_q_ref[2*j:2*j+2], *s.packet_dq_ref[2*j:2*j+2],
                                 *s.packet_tau_ff[2*j:2*j+2], *s.packet_kp[2*j:2*j+2], *s.packet_kd[2*j:2*j+2]] for j in range(s.packet_count)]}
        assert c['index'] == i
        for name, e in expected.items():
            a, b = np.asarray(c[name]), np.asarray(e)
            assert a.shape == b.shape, (i, name, a.shape, b.shape)
            error = float(np.max(np.abs(a-b)))
            maxima[name] = max(maxima.get(name, 0.), error)
            assert error <= (1e-14 if name == 'R' else 1e-12), (i, name, error)
    report = {'passed': True, 'scope': 'Offline source conversion parity at original selected visibility; no ROS scheduling/flight/latency claim',
              'snapshots': len(snapshots), 'maximum_absolute_error': maxima, 'raw_bag_exact_matches': raw_bag_matches,
              'raw_bag_missing_by_topic': {k: len(snapshots)-v for k, v in raw_bag_matches.items()},
              'angular_and_flags_evidence': 'Original preserved raw fields/receipt metadata; these topics absent from old ROS bag',
              'visibility': 'state prefix ends at original selected joint stamp; MIT prefix ends at last original actual packet',
              'source_identities': identities, 'executable': identity(args.executable), 'fixture': identity(fixture),
              'result': identity(result), 'script': identity(Path(__file__))}
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps({k: report[k] for k in ('passed', 'snapshots', 'maximum_absolute_error', 'raw_bag_exact_matches')}, indent=2))


if __name__ == '__main__':
    main()
