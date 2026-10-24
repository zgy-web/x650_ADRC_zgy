"""Reproducible offline auditor tests against an existing synthetic ROS probe.

This starts no ROS node, worker, simulator or actuator. The source CDR is original
software-probe evidence. Worker fixtures below are explicitly artificial and
must not be described as an actual ingress/worker transport qualification.
"""
import argparse
import json
from pathlib import Path
import shutil

from audit_x650_cpp_source import audit_source, identity, load_ros, require, wire_records


def run(probe, output):
    from rclpy.serialization import serialize_message
    probe, output = Path(probe), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    report = json.loads((probe / 'report.json').read_text())
    require(report['passed'] and len(report['cases']) == 4, 'Expected completed four-case synthetic ROS probe')
    reference = dict(px4_source_contract_commit='d6f12ad1c4f70ad3230afd7d86e971421e02fef4',
        frame_mode='raw_ev_world', target_frozen=True, fit_estimator_translation=False,
        px4_reference_at_bind=dict(ref_lat=47., ref_lon=8., ref_alt=500., ref_timestamp=100,
            xy_global=True, z_global=True, xy_reset_counter=0, z_reset_counter=0, heading_reset_counter=0))
    extra = dict(vxy_reset_counter=0, vz_reset_counter=0, quat_reset_counter=0)
    expected = dict(configuration_sha256='1' * 64, position_gains=[1., 1., 1.],
                    bus_period_s=.005, delay_s=.005)
    normal = probe / 'normal'
    decode = load_ros()
    records = list(wire_records(normal / 'evidence/source_wire.cdr', decode))
    live = next(record.message for record in records if record.kind == 6)
    fixture = output / 'artificial_worker_fixture'
    worker = fixture / 'native_records'
    worker.mkdir(parents=True)
    (fixture / 'worker_live_start.cdr').write_bytes(serialize_message(live))
    prior, phases = live.origin_s, []
    for sequence, record in enumerate((r for r in records if r.kind == 7), start=1):
        snapshot = record.message
        snapshot.sequence = sequence
        snapshot.dt_s = snapshot.control_time_s - prior
        prior = snapshot.control_time_s
        path = worker / f'{sequence:06d}.snapshot.cdr'
        path.write_bytes(serialize_message(snapshot))
        (worker / f'{sequence:06d}.json').write_text(json.dumps(dict(sequence=sequence, snapshot_cdr=identity(path))))
        phases.append([sequence, snapshot.issued_monotonic_ns, record.wall + 1, 1])
    (fixture / 'diagnostic_sender.json').write_text(json.dumps(dict(complete=True, failed=False,
        dropped=0, offered=len(phases), written=len(phases))))
    (fixture / 'worker_timing.json').write_text(json.dumps(dict(complete=True, dropped=0,
        columns=['sequence', 'snapshot_issued_ns', 'callback_entry_ns', 'record_offered'], rows=phases)))

    def check(directory=normal, **kwargs):
        return audit_source(directory / 'evidence', directory / 'state.csv', directory / 'packets.csv',
                            reference, extra, expected_binding=expected, **kwargs)

    positive = check(worker_directory=worker)
    require(positive['passed'] and positive['original_worker_snapshots_verified'] == len(phases),
            'Normal software source evidence rejected')
    (output / 'normal_report.json').write_text(json.dumps(positive, indent=2, allow_nan=False))
    negative = {}

    def rejects(name, action):
        try:
            action()
        except ValueError as error:
            negative[name] = repr(error)
        else:
            raise AssertionError('Invalid evidence accepted: ' + name)

    for name, mutate in [('joint_q', lambda s: s.q.__setitem__(0, s.q[0] + .01)),
            ('packet_tau', lambda s: s.packet_tau_ff.__setitem__(0, 1.)),
            ('unused_history', lambda s: s.command_time_s.__setitem__(63, 1.)),
            ('source_sequence', lambda s: setattr(s, 'sequence', s.sequence + 1))]:
        def changed(kind, raw):
            message = decode(kind, raw)
            if kind == 7:
                mutate(message)
            return message
        rejects(name, lambda: check(decode=changed))
    for name in ('bad_binding_hash', 'duplicate_binding', 'unbound_start'):
        rejects(name, lambda: check(probe / name))
    truncated = output / 'truncated_worker_fixture'
    shutil.copytree(fixture, truncated)
    (truncated / 'native_records' / f'{len(phases):06d}.snapshot.cdr').unlink()
    rejects('missing_worker_tail', lambda: check(worker_directory=truncated / 'native_records'))
    changed_worker = output / 'mutated_worker_fixture'
    shutil.copytree(fixture, changed_worker)
    path = changed_worker / 'native_records/000001.snapshot.cdr'
    message = decode(7, path.read_bytes())
    message.q[0] += .001
    path.write_bytes(serialize_message(message))
    # Recompute the artificial digest too: contents still must match the source.
    (path.parent / '000001.json').write_text(json.dumps(dict(sequence=1, snapshot_cdr=identity(path))))
    rejects('worker_value_changed_with_updated_digest', lambda: check(worker_directory=path.parent))
    summary = dict(passed=True, positive_cases=1, negative_cases_rejected=negative,
        scope='Existing actual synthetic-ROS source CDR, explicitly artificial worker linkage fixtures; no new ROS node/worker/plant',
        source_probe_report=identity(probe / 'report.json'), audit_script=identity(Path(__file__).with_name('audit_x650_cpp_source.py')),
        probe_script=identity(Path(__file__)), normal_report=identity(output / 'normal_report.json'))
    (output / 'report.json').write_text(json.dumps(summary, indent=2, allow_nan=False))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = run(args.probe, args.output)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
