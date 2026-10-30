"""Offline checks of the bounded, powered-but-disarmed parameter procedure."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import px4_first_flight_setup as m


def raw(slot=1, nav=1, stamp=100000000):
    t = f'timestamp: {stamp} (0.01 seconds ago)\n'
    dshot = 'Outputs initialized: yes\nOutputs used: 0xf\n' + '\n'.join(
        f'Channel {i}: func: {101+i if i<4 else 0}, value: 0, failsafe: 0, disarmed: 0, min: 109, max: 1999, center: 65535'
        for i in range(9)) + '\nNumber of successful ESC frames: 100\nNumber of timeouts: 0\nNumber of CRC errors: 0\n'
    esc = t + 'esc_count: 4\n'
    for i in range(4):
        esc += (f'\n  esc[{i}] (esc_report):\n    timestamp: {stamp-i*2500} (0.04 seconds ago)\n'
                f'    actuator_function: {101+i}\n    esc_rpm: 0\n    esc_voltage: 24.3\n'
                '    esc_current: 0.1\n    esc_temperature: 30\n')
    return dict(vehicle_status=t+f'arming_state: 1\nnav_state: {nav}\nhil_state: 0\nfailsafe: False\n',
                actuator_armed=t+'armed: False\nprearmed: False\ntermination: False\nin_esc_calibration_mode: False\nkill: True\n',
                manual_control_setpoint=t+'valid: True\ndata_source: 1\nthrottle: -1\nroll: 0\npitch: 0\nyaw: 0\n',
                manual_control_switches=t+f'arm_switch: 3\nmode_slot: {slot}\nkill_switch: 1\n',
                system_power=t+'voltage5v_v: 5.1\nbrick_valid: 1\nusb_valid: 0\nusb_connected: 0\nperiph_5v_oc: 0\nhipower_5v_oc: 0\n',
                battery_status=t+'connected: True\nwarning: 0\ncell_count: 6\nvoltage_v: 24.3\n',
                dshot=dshot, esc_status=esc)


def parameters():
    return {**dict.fromkeys(m.PARAMETERS, 0), **m.KEEP,
            'MPC_Z_VEL_ALL': -3, 'SYS_VEHICLE_RESP': -.4,
            **{n: old for n, old, target in m.PLAN}}


def test_battery_power_is_allowed_with_actual_kill_and_zero_outputs():
    a = m.assess_guard(raw())
    assert a['battery_connected'] and a['actual_kill'] and a['usb_connected'] == 0
    assert all(v == 0 for v in a['outputs'].values())


def test_battery_error_named_numeric_fields_are_not_console_errors(monkeypatch):
    c = m.FirstFlightSetupConsole('unused')
    c.s = SimpleNamespace(write=lambda _: None)
    monkeypatch.setattr(c, 'read', lambda _: b'prediction_error: -0.00034\ninterface_error: 0\nmax_error: 0\nnsh> ')
    assert 'interface_error: 0' in c.query('listener battery_status -n 1')
    monkeypatch.setattr(c, 'read', lambda _: b'ERROR [listener] failed\nnsh> ')
    with pytest.raises(RuntimeError): c.query('listener battery_status -n 1')


@pytest.mark.parametrize('topic,old,new', [
    ('vehicle_status', 'arming_state: 1', 'arming_state: 2'),
    ('vehicle_status', 'failsafe: False', 'failsafe: True'),
    ('actuator_armed', 'kill: True', 'kill: False'),
    ('actuator_armed', 'prearmed: False', 'prearmed: True'),
    ('manual_control_switches', 'arm_switch: 3', 'arm_switch: 1'),
    ('manual_control_switches', 'kill_switch: 1', 'kill_switch: 3'),
    ('manual_control_switches', 'mode_slot: 1', 'mode_slot: 6'),
    ('manual_control_setpoint', 'throttle: -1', 'throttle: 0'),
    ('manual_control_setpoint', '0.01 seconds ago', '0.8 seconds ago'),
    ('manual_control_setpoint', 'valid: True', 'valid: False'),
    ('system_power', 'voltage5v_v: 5.1', 'voltage5v_v: nan'),
    ('system_power', 'periph_5v_oc: 0', 'periph_5v_oc: 1'),
    ('battery_status', 'warning: 0', 'warning: 3'),
    ('dshot', 'value: 0', 'value: 109'),
    ('esc_status', 'esc_rpm: 0', 'esc_rpm: 100'),
    ('esc_status', '0.04 seconds ago', '2 seconds ago'),
])
def test_unsafe_or_untrusted_state_blocks_before_write(topic, old, new):
    data = raw(); data[topic] = data[topic].replace(old, new)
    with pytest.raises(ValueError): m.assess_guard(data)


def test_actual_stabilized_state_is_15_not_configuration_value_8():
    assert m.assess_guard(raw(6, 15), 6, 15)['state']['nav_state'] == 15
    with pytest.raises(ValueError): m.assess_guard(raw(6, 8), 6, 15)


@pytest.mark.parametrize('name,value', [('MPC_Z_VEL_ALL', 2), ('SYS_VEHICLE_RESP', .4),
                                      ('MPC_MAN_TILT_MAX', 10), ('RC_MAP_KILL_SW', 0)])
def test_changed_baseline_is_not_silently_overwritten(name, value):
    p = parameters(); p[name] = value
    with pytest.raises(ValueError): m.validate_baseline(p)


@pytest.mark.parametrize('command', ['commander arm', 'param set COM_FLTMODE6 7',
                                    'param set MPC_MAN_TILT_MAX 60', 'param reset_all',
                                    'param save; reboot', 'mavlink stop -d /dev/ttyACM0'])
def test_command_scope_cannot_send_unapproved_mutations(command, monkeypatch):
    c = m.FirstFlightSetupConsole('unused', execute=True)
    monkeypatch.setattr(c, '_send', lambda _: pytest.fail('must not send'))
    with pytest.raises(ValueError): c.query(command)
    with pytest.raises(ValueError): c.write_fixed(command)


def test_no_execute_or_props_confirmation_cannot_open_hardware(tmp_path, monkeypatch):
    monkeypatch.setattr(m, 'FirstFlightSetupConsole', lambda *a, **k: pytest.fail('must not open'))
    with pytest.raises(ValueError): m.run('unused', tmp_path/'a', execute=True)
    with pytest.raises(ValueError): m.run('unused', tmp_path/'b')
    monkeypatch.setattr(sys, 'argv', ['setup'])
    assert m.main() == 0


class FakeConsole(m.FirstFlightSetupConsole):
    instances = []
    fail_readback = False
    kill_lost_after_first = False
    def __init__(self, port, *, execute=False):
        self.s = SimpleNamespace(is_open=False)
        self.records = []; self.active_listener = False; self.execute = execute
        self.params = parameters(); self.t = 100000000; self.writes = []
        self.instances.append(self)
    def open(self): self.s.is_open = True
    def close(self): self.s.is_open = False
    def _send(self, cmd):
        self.records.append({'command': cmd}); self.t += 10000
        if cmd == 'ver all': return m.FIRMWARE
        if cmd.startswith('param show -q '): return str(self.params[cmd.split()[-1]])
        if cmd.startswith('param set '):
            _, _, name, value = cmd.split(); self.writes.append(cmd)
            self.params[name] = float(value)
            if self.fail_readback: self.params[name] += 1
            return 'nsh> '
        if cmd == 'param save': self.writes.append(cmd); return 'nsh> '
        data = raw(stamp=self.t)
        if self.kill_lost_after_first and self.writes:
            data['actuator_armed'] = data['actuator_armed'].replace('kill: True', 'kill: False')
        return data['dshot' if cmd == 'dshot status' else cmd.split()[1]]


def test_exact_eight_writes_saved_readback_and_backup(tmp_path, monkeypatch):
    monkeypatch.setattr(m, 'APPLY_ENABLED', True)  # Historical flow, fake hardware only.
    monkeypatch.setattr(m, 'FirstFlightSetupConsole', FakeConsole)
    r = m.run('unused', tmp_path/'ok', execute=True, props_removed=True)
    assert r['complete'] and r['parameter_save_verified'] and r['port_closed']
    assert r['parameter_writes'] == 8 and len(r['changed_parameters']) == 8
    assert set(FakeConsole.instances[-1].writes) == m.WRITES
    assert json.loads((tmp_path/'ok/rollback_values.json').read_text()) == {n: old for n, old, _ in m.PLAN}
    assert not r['powercycle_persistence_verified'] and not r['flight_ready']


@pytest.mark.parametrize('fault', ['fail_readback', 'kill_lost_after_first'])
def test_partial_failure_preserved_and_never_retried_or_auto_rolled_back(tmp_path, monkeypatch, fault):
    monkeypatch.setattr(m, 'APPLY_ENABLED', True)
    monkeypatch.setattr(m, 'FirstFlightSetupConsole', FakeConsole)
    monkeypatch.setattr(FakeConsole, fault, True)
    r = m.run('unused', tmp_path/'bad', execute=True, props_removed=True)
    assert not r['complete'] and r['port_closed'] and not r['parameter_save_verified']
    assert len(FakeConsole.instances[-1].writes) == 1
    assert (tmp_path/'bad/rollback_values.json').exists()
    assert json.loads((tmp_path/'bad/report.json').read_text())['error']


def test_out_of_scope_drift_rejected_even_when_all_targets_match():
    before = parameters(); after = {**before, **m.TARGETS, 'MC_ROLL_P': 42}
    with pytest.raises(ValueError): m.validate_delta(before, after)


def test_withdrawn_proposal_is_disabled_even_with_props_confirmation(tmp_path, monkeypatch):
    monkeypatch.setattr(m, 'FirstFlightSetupConsole', lambda *a, **k: pytest.fail('must not open'))
    with pytest.raises(ValueError, match='withdrawn'):
        m.run('unused', tmp_path/'disabled', execute=True, props_removed=True)


def test_restore_is_exact_and_leaves_unknown_auto_value_untouched(tmp_path, monkeypatch):
    import px4_first_flight_restore as restore
    class FakeRestore(FakeConsole):
        def __init__(self, port, **kwargs):
            super().__init__(port, **kwargs)
            self.params.update({n: current for n, (current, _) in restore.RESTORE.items()})
            self.params.update({'MPC_Z_V_AUTO_UP': 1, 'MPC_Z_V_AUTO_DN': 1.5})
        def query(self, command):
            if command in {f'param show -q {n}' for n in restore.AUTO} | {f'param show {n}' for n in restore.AUTO}:
                return self._send(command)
            return super().query(command)
        def write_fixed(self, command):
            assert command in restore.ALLOWED
            return dict(guard=m.guard(self), response=self._send(command))
        def _send(self, command):
            if command.startswith('param show MPC_'): return str(self.params[command.split()[-1]])
            return super()._send(command)
    monkeypatch.setattr(restore, 'RestoreConsole', FakeRestore)
    backup = tmp_path/'before.json'; backup.write_text(json.dumps(parameters()))
    r = restore.restore('unused', tmp_path/'restore', backup)
    assert r['complete'] and r['port_closed'] and not r['remaining_inventory_differences']
    assert r['auto_after']['MPC_Z_V_AUTO_UP'] == 1 and r['automatic_values_history_unknown']
    assert set(FakeRestore.instances[-1].writes) == restore.ALLOWED
