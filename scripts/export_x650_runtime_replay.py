#!/usr/bin/env python3
"""Export a hash-bound actual ABI-2 prefix for original C++ memory replay.

This is a new adapter for the newer runtime record schema. The established
x650_cpp_recovery_sequence.cpp can reconstruct arbitrary prefix lengths, so no
Python plan/proposal is used to synthesize controller memory. Proposals remain
comparison oracles only. Neither simulator nor actuator publisher is started.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from export_x650_recovery_recording import identity, plain, write_new


def require(ok, message):
    if not ok:
        raise ValueError(message)


def export(run, output, last_sequence):
    from flyarm_control_interfaces.msg import LiveStart, ControlSnapshot, ControlProposal
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict

    run, output = Path(run).resolve(strict=True), Path(output).resolve()
    require(type(last_sequence) is int and 1 <= last_sequence <= 20000,
            'Explicit bounded actual prefix length required')
    require(not output.exists(), 'Evidence output must be a new directory')
    require(not output.is_relative_to(run), 'Derived evidence must remain outside the original run')
    source, originals = run/'isolated_live', []

    def bound(path, expected=None):
        receipt = identity(path)
        if expected is not None:
            require(all(receipt[k] == expected[k] for k in ('bytes', 'sha256')),
                    'Original source receipt differs: '+str(path))
        originals.append(receipt)
        return receipt

    def decode(path, cls):
        return plain(message_to_ordereddict(deserialize_message(Path(path).read_bytes(), cls)))

    startup_id = bound(source/'worker_startup.json')
    startup = json.loads(Path(startup_id['path']).read_text())
    require(startup['core_abi'] == 2, 'Actual ABI-2 runtime recording required')
    manifest = startup['startup_manifest']
    libraries = {}
    for role, key, loaded in [('core', 'core_library', 'loaded_core_path'),
                             ('optimization', 'optimization_library', 'loaded_optimization_path'),
                             ('predictor', 'predictor_library', 'loaded_predictor_path')]:
        item = manifest[key]
        actual = identity(item['path'])
        require(actual['sha256'] == item['sha256']
            and str(Path(actual['path']).resolve()) == startup[loaded],
            'Actually loaded original library changed: '+role)
        libraries[role] = actual
        originals.append(actual)
    start_id, json_start_id = bound(source/'worker_live_start.cdr'), bound(source/'live_start.json')
    start = decode(start_id['path'], LiveStart)
    saved_start = json.loads(Path(json_start_id['path']).read_text())
    for field, value in start.items():
        if field not in saved_start:
            continue
        saved = saved_start[field]
        if field == 'configuration_sha256' and isinstance(saved, str):
            saved = list(bytes.fromhex(saved))
        require(saved == value, 'CDR/JSON LiveStart differs: '+field)
    require(start['session_id'] == startup['session_id'] and start['epoch'] == startup['epoch']
        and bytes(start['configuration_sha256']).hex() == startup['configuration_sha256'],
        'Actual LiveStart/startup identity differs')
    record_paths = sorted((source/'native_records').glob('[0-9]*.json'))
    require(len(record_paths) >= last_sequence, 'Actual native prefix is incomplete')
    selected = record_paths[:last_sequence]
    output.mkdir(parents=True)
    rows, recovered = [], []
    for sequence, path in enumerate(selected, 1):
        record_id = bound(path)
        record = json.loads(path.read_text())
        require(record['sequence'] == sequence and path.stem == f'{sequence:06d}',
                'Actual prefix sequence missing/reordered')
        decoded = {}
        for kind, cls in [('snapshot', ControlSnapshot), ('proposal', ControlProposal)]:
            receipt = bound(record[kind+'_cdr']['path'], record[kind+'_cdr'])
            decoded[kind+'_identity'] = receipt
            value = decode(receipt['path'], cls)
            require(value['sequence'] == sequence and all(value[field] == start[field]
                for field in ('session_id', 'epoch', 'configuration_sha256')),
                'Actual '+kind+' binding/sequence differs')
            decoded[kind] = value
        failed = bool(record['error'])
        require((decoded['proposal']['status'] != 0) is failed
            and (not failed or sequence == last_sequence), 'Native outcome/prefix rejection differs')
        d = record.get('result', {}).get('diagnostics', record.get('rejected_diagnostics', {}))
        if d.get('recovery_triggered') is True:
            recovered.append(dict(sequence=sequence, accepted=not failed,
                                  reason=d['recovery_reason']))
        row_path = output/f'{sequence:06d}.json'
        write_new(row_path, dict(sequence=sequence, record_identity=record_id, **decoded))
        rows.append(dict(sequence=sequence, **identity(row_path)))
    require(all(identity(item['path']) == item for item in originals),
            'Original source changed during export')
    result = dict(kind='x650_original_cpp_recording_export_v1', schema_version=1,
        config=manifest['config'], trajectory=manifest['trajectory'], live_start=start, rows=rows,
        startup_identity=startup_id, live_start_cdr_identity=start_id,
        live_start_json_identity=json_start_id, exporter_identity=identity(__file__),
        source_run_directory=str(run), actual_core_abi=2, original_libraries=libraries,
        original_recovery_observations=recovered, originals_verified_unchanged=True,
        controller_memory_reconstructed=False, physical_validation_claimed=False)
    write_new(output/'recording.json', result)
    return dict(row_count=len(rows), recording=identity(output/'recording.json'),
                original_recovery_observations=recovered)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--last-sequence', type=int, required=True)
    args = parser.parse_args()
    print(json.dumps(export(args.run, args.output, args.last_sequence), indent=2))
