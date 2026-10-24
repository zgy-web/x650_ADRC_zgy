"""Read-only verification of this archived A/B/A2 evidence; no simulation."""
from pathlib import Path
import hashlib
import json
import zipfile


def main():
    root = Path(__file__).resolve().parent
    manifest = json.loads((root / 'raw_archive_manifest.json').read_text())
    archive = root / manifest['archive']
    assert archive.stat().st_size == manifest['archive_bytes']
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == manifest['archive_sha256']
    with zipfile.ZipFile(archive) as packed:
        assert set(packed.namelist()) == set(manifest['entries'])
        for name, expected in manifest['entries'].items():
            data = packed.read(name)
            assert len(data) == expected['bytes'], name
            assert hashlib.sha256(data).hexdigest() == expected['sha256'], name
            if (root / name).exists():
                assert (root / name).read_bytes() == data, name
        reports = [json.loads(packed.read(name)) for name in
                   ('A_no_sim.json', 'B_unarmed.json', 'A2_no_sim.json')]
    for report in reports:
        assert report['completed'] == report['expected_count'] == len(report['rows']) == 1056
        assert report['all_accepted'] and report['max_command_error'] == 0
        assert len(report['command_max_errors']) == 1056
        assert not any(report['command_max_errors'])
        assert report['affinity'] == report['cpu_begin_values'] == report['cpu_end_values'] == [87]
        assert report['engine_above_20ms'] == report['engine_plus_diagnostic_above_20ms'] == 0
        for key in ('sources', 'library', 'environment'):
            assert report[key] == reports[0][key], key
    placement = json.loads((root / 'placement_probe/child_final.json').read_text())
    last_probe = max(row[1] for row in placement['callbacks'])
    first_b = reports[1]['rows'][0][reports[1]['columns'].index('callback_entry_ns')]
    assert last_probe < first_b
    comparison = json.loads((root / 'comparison.json').read_text())
    assert comparison['probe_last_callback_ns'] == last_probe
    assert comparison['B_first_timed_ns'] == first_b
    assert comparison['gap_seconds'] == (first_b - last_probe) / 1e9
    status = json.loads((root / 'background/status.json').read_text())
    assert status['stage'] == 'completed_background_observation'
    assert status['released'] and status['ground_window_verified']
    assert status['runtime_file_absent'] and status['remaining_owned_processes'] == []
    assert (root / 'background/unarmed_load_v5.exit').read_text().strip() == '0'
    first_a2 = reports[2]['rows'][0][reports[2]['columns'].index('callback_entry_ns')]
    assert status['ended_monotonic_ns'] < first_a2
    for name in ('ground_before.json', 'ground_window.json'):
        ground = json.loads((root / 'background' / name).read_text())
        assert ground['passed'] and ground['ground_disarmed_verified']
        assert ground['commands_published'] == ground['parameters_written'] == 0
    receipt = json.loads((root / 'retrieval_receipt.json').read_text())
    for name, expected in receipt['files'].items():
        data = (root / name).read_bytes()
        assert len(data) == expected['bytes'], name
        assert hashlib.sha256(data).hexdigest() == expected['sha256'], name
    print(json.dumps({'archive_entries_verified': len(manifest['entries']),
                      'full_replay_rows_verified': 3 * 1056,
                      'exact_commands': True, 'placement_overlap': False,
                      'background_cleanup_before_A2': True,
                      'flight_acceptance': False}, indent=2))


if __name__ == '__main__':
    main()
