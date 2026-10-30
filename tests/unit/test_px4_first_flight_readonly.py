"""No hardware: reject active states, partial evidence and unsafe commands."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts'))
import px4_first_flight_readonly as m


@pytest.mark.parametrize('command', ['commander arm', 'commander check', 'commander takeoff',
    'param set SYS_HAS_GPS 0', 'param save', 'commander calibrate accel',
    'mavlink stop-all', 'listener vehicle_status -n 1; commander arm',
    'ina226 stop', 'battery_status start', 'adc stop'])
def test_mutating_or_arbitrary_commands_rejected(command):
    c = m.FirstFlightConsole('unused')
    with pytest.raises(ValueError):
        c.query(command)
    assert not c.s.is_open


def test_unknown_and_stale_localization_do_not_become_valid():
    r = m.assess({}, {}, {})
    assert len(r['positioning_blockers']) == 6
    assert r['gps']['fix_type'] is None
    assert r['power']['brick_valid'] is None
    valid = '\n'.join(k+': True' for k in r['local_position_validity'])
    r = m.assess({'vehicle_local_position':'timestamp: 5000000 (10 seconds ago)\n'+valid}, {}, {})
    assert r['positioning_blockers'] == ['local position missing or stale']
    assert r['flight_ready'] is False
    r = m.assess({'vehicle_local_position':'timestamp: 5000000 (0.01 seconds ago)\n'+valid}, {}, {})
    assert r['positioning_blockers'] == []
    assert r['flight_ready'] is False  # a valid topic alone never clears a vehicle for flight


@pytest.mark.parametrize('fault', [None, 'firmware', 'armed', 'nonzero', 'stalled', 'midread'])
def test_incomplete_capture_is_preserved_and_port_closed(monkeypatch, tmp_path, fault):
    class Fake:
        def __init__(self, port):
            self.s = SimpleNamespace(is_open=False)
            self.records = []
            self.count = 0
        def open(self): self.s.is_open = True
        def close(self): self.s.is_open = False
        def query(self, cmd):
            assert cmd in m.READS
            self.records.append({'command': cmd})
            if cmd == 'ver all': return 'other' if fault == 'firmware' else m.FIRMWARE
            if cmd == 'listener vehicle_status -n 1':
                self.count += 1
                t = 1000000 if fault == 'stalled' else self.count*1000000
                return (f'timestamp: {t} (0.01 seconds ago)\narming_state: {2 if fault=="armed" else 1}\n'
                        'nav_state: 1\nhil_state: 0\nfailsafe: False\n')
            if cmd == 'listener actuator_armed -n 1':
                return ('timestamp: 1000000 (0.01 seconds ago)\narmed: False\nprearmed: False\n'
                        'kill: True\ntermination: False\nin_esc_calibration_mode: False\n')
            if cmd == 'dshot status':
                return 'Outputs initialized: yes\nOutputs used: 0xf\n'+'\n'.join(
                    f'Channel {i}: func: {101+i if i<4 else 0}, value: {109 if fault=="nonzero" and i==0 else 0}, '
                    'failsafe: 0, disarmed: 0, min: 109, max: 1999, center: 65535' for i in range(9))
            if cmd == 'logger status' and fault == 'midread': raise OSError('lost connection')
            return 'never published\nnsh> '
    monkeypatch.setattr(m, 'FirstFlightConsole', Fake)
    out = tmp_path/'new'
    r = m.collect('unused', out)
    assert r['complete'] is (fault is None)
    assert r['port_closed'] and r['flight_ready'] is False
    assert r['parameter_writes'] == r['arm_commands'] == r['mode_commands'] == r['motor_commands'] == 0
    assert json.loads((out/'report.json').read_text()) == json.loads(json.dumps(r))
    if fault in ('firmware', 'armed', 'nonzero'):
        assert not r['parameters']


def test_existing_evidence_is_not_overwritten(monkeypatch, tmp_path):
    monkeypatch.setattr(m, 'FirstFlightConsole', lambda port: pytest.fail('must not touch hardware'))
    with pytest.raises(FileExistsError): m.collect('unused', tmp_path)
