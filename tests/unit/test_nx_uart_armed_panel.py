import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts'))
from nx_uart_armed_panel import ArmedFlow, ArmedSession
from prepare_px4_armed_uart import assess
from archive_px4_known_fault import ArchiveConsole, body_from_cat, SOURCE, WRITES


def row(armed=False, nav=1, sf='3', sa='1', kill=False, lost=False):
    return dict(arming_state=2 if armed else 1, nav_state=nav, kill=kill,
        raw=dict(manual_control_switches=f'arm_switch: {sf}\nmode_slot: {sa}\nkill_switch: {1 if kill else 3}\n',
                 failsafe_flags=f'offboard_control_signal_lost: {lost}\n',
                 manual_control_setpoint='valid: True\n'))


def enter(case):
    f = ArmedFlow(case)
    f.ready(10)
    assert f.tick(row(), 11) == {}
    assert f.tick(row(), 11.6)['phase'] == 'arm'
    assert f.tick(row(sf='1'), 11.7) == {}  # SF alone is not actual arming.
    assert f.tick(row(True, sf='1'), 12)['phase'] == 'enter_offboard'
    assert f.tick(row(True, 14, '1', '6'), 13)['phase'] == 'hold'
    return f


def test_actual_arming_then_manual_takeover():
    f = enter('manual_exit')
    assert f.tick(row(True, 14, '1', '6'), 14)['phase'] == 'manual_exit'
    assert f.tick(row(True, 1, '1', '1'), 15)['phase'] == 'return_switches'
    assert f.qualified_observation
    assert f.tick(row(), 16) == {}  # No second attempt after completing.


def test_actual_armed_automatic_fallback_requires_unchanged_rc():
    f = enter('stream_stop')
    assert f.tick(row(True, 14, '1', '6', lost=True), 17) == {}
    assert f.tick(row(True, 1, '1', '6', lost=True), 18)['phase'] == 'return_switches'
    assert f.qualified_observation


@pytest.mark.parametrize('bad', [
    row(False, 1, '1', '6', lost=True),  # Ground auto-disarm, not armed exit.
    row(True, 1, '1', '1', lost=True),   # SA changed: not automatic.
    row(True, 1, '3', '6', lost=True),   # SF changed: not automatic.
    row(True, 1, '1', '6', kill=True, lost=True),
    row(True, 1, '1', '6', lost=False),
])
def test_confounded_exit_never_qualifies(bad):
    f = enter('stream_stop')
    assert 'error' in f.tick(bad, 17)
    assert f.finished and not f.qualified_observation


def test_early_action_timeout_and_latched_abort():
    f = ArmedFlow('manual_exit'); f.ready(10)
    assert 'error' in f.tick(row(True, sf='1'), 11)
    assert f.tick(row(), 12) == {}
    f = ArmedFlow('stream_stop'); f.ready(10)
    assert 'error' in f.tick(row(), 59)


def test_human_sa_delay_uses_real_disarm_not_extra_four_second_deadline():
    f = ArmedFlow('manual_exit'); f.ready(10)
    assert f.tick(row(), 11.6)['phase'] == 'arm'
    assert f.tick(row(True, sf='1'), 12)['phase'] == 'enter_offboard'
    assert f.tick(row(True, sf='1'), 16.5) == {}
    assert not f.finished
    assert f.tick(row(True, 14, '1', '6'), 17)['phase'] == 'hold'
    assert f.tick(row(False, 14, '1', '6'), 18)['phase'] == 'return_switches'
    assert not f.qualified_observation
    assert f.tick(row(True, 14, '1', '6'), 19) == {}  # No second arming attempt.


def test_guidance_records_transitions_once_without_hardware(tmp_path):
    output = tmp_path/'run'
    session = ArmedSession(output, Path('build'), Path('key'), Path('known'), 'ssh', 'manual_exit')
    assert not output.exists()  # Opening the page still creates no capture.
    output.mkdir()
    session.set(phase='arm', hint='SF')
    session.set(phase='arm', hint='SF')
    session.set(phase='enter_offboard', hint='SA')
    events = [json.loads(x) for x in (output/'guidance.jsonl').read_text(encoding='utf-8').splitlines()]
    assert [x['phase'] for x in events] == ['arm', 'enter_offboard']
    assert events[1]['pc_monotonic_ns'] >= events[0]['pc_monotonic_ns']


def preflight_raw():
    def fresh(fields):
        return 'timestamp: 10000000 (0.1 seconds ago)\n' + fields
    return dict(vehicle_status=fresh('arming_state: 1\nnav_state: 1\nhil_state: 0\nfailsafe: False\npre_flight_checks_pass: True\n'),
        actuator_armed=fresh('armed: False\nprearmed: False\nlockdown: False\ntermination: False\nin_esc_calibration_mode: False\nkill: False\n'),
        manual_control_setpoint=fresh('valid: True\nthrottle: -1.0\nroll: 0\npitch: 0\nyaw: 0\n'),
        manual_control_switches=fresh('arm_switch: 3\nmode_slot: 1\nkill_switch: 3\n'),
        health_report='timestamp: 1 (200 seconds ago)\ncan_arm_mode_flags: 114563\nhealth_error_flags: 0\narming_check_error_flags: 17956864\n')


def test_preflight_uses_selected_mode_and_actual_kill_not_global_error_bits():
    raw = preflight_raw()
    assert assess(raw)['ready']  # Errors for unavailable position modes remain recorded.
    raw['actuator_armed'] = raw['actuator_armed'].replace('kill: False', 'kill: True')
    assert not assess(raw)['ready']
    raw = preflight_raw()
    raw['health_report'] = raw['health_report'].replace('can_arm_mode_flags: 114563', 'can_arm_mode_flags: 0')
    assert not assess(raw)['ready']
    raw = preflight_raw()
    raw['vehicle_status'] = raw['vehicle_status'].replace('pre_flight_checks_pass: True', 'pre_flight_checks_pass: False')
    assert not assess(raw)['ready']


def test_stale_real_status_is_rejected_even_when_preflight_pass_is_true():
    raw = preflight_raw(); raw['vehicle_status'] = raw['vehicle_status'].replace('0.1 seconds', '8 seconds')
    with pytest.raises(ValueError, match='Stale'):
        assess(raw)


def test_failed_preflight_never_launches_nx_or_observer(tmp_path, monkeypatch):
    output = tmp_path/'new'
    session = ArmedSession(output, Path('build'), Path('key'), Path('known'), 'ssh', 'manual_exit')
    monkeypatch.setattr('nx_uart_armed_panel.collect', lambda *args:dict(complete=True, ready=False))
    def unexpected(*args):
        pytest.fail('Failed preflight launched a child')
    monkeypatch.setattr(session, 'launch', unexpected)
    session.run()
    assert session.snapshot()['phase'] == 'failed'
    assert not session.children


def test_archive_rejects_different_or_partial_fault_without_writes():
    with pytest.raises(ValueError):
        body_from_cat('cat '+SOURCE+'\npartial\nnsh> ', SOURCE)
    c = ArchiveConsole('COM_DO_NOT_OPEN')
    for command in WRITES:
        with pytest.raises(ValueError, match='guard'):
            c.query(command)
    with pytest.raises(ValueError, match='diagnostics'):
        c.query('param set COM_ARM_HFLT_CHK 0')
    assert not c.s.is_open
