"""Restore only the three observed changes from the cancelled 2026-10-30 setup.

Uses the actual pre-write backup, not generic defaults. Read automatic ascent/
descent values too: these were missing from the earlier inventory, so their
historical values cannot be asserted by this tool. Never arms or changes modes.
"""
import argparse
import hashlib
import json
from pathlib import Path

from px4_debug_console import FIRMWARE
from px4_first_flight_setup import (FirstFlightSetupConsole, PARAMETERS, READS,
    guard, read_parameter, same, save_json, validate_dependencies)

RESTORE = {'MPC_MAN_TILT_MAX': (20, 35), 'MPC_MAN_Y_MAX': (60, 150),
           'MPC_Z_VEL_MAX_UP': (1, 3)}
AUTO = ('MPC_Z_V_AUTO_UP', 'MPC_Z_V_AUTO_DN')
ALLOWED = frozenset(f'param set {p} {v:g}' for p, (_, v) in RESTORE.items()) | {'param save'}


class RestoreConsole(FirstFlightSetupConsole):
    def query(self, command):
        if command in {f'param show -q {n}' for n in AUTO} | {f'param show {n}' for n in AUTO}:
            if self.active_listener: raise ValueError('Active listener')
            return self._send(command)
        return super().query(command)

    def write_fixed(self, command):
        if not self.execute or command not in ALLOWED or self.active_listener:
            raise ValueError('Outside fixed three-value restoration')
        state = guard(self)
        return dict(guard=state, response=self._send(command))


def restore(port, output, backup):
    before = json.loads(backup.read_text())
    if set(before) != set(PARAMETERS): raise ValueError('Incomplete backup')
    validate_dependencies(before)
    for n, (_, value) in RESTORE.items():
        if not same(before[n], value): raise ValueError('Backup conflicts with reviewed restore: '+n)
    output.mkdir(parents=True, exist_ok=False)
    c = RestoreConsole(port, execute=True)
    r = dict(complete=False, flight_ready=False, parameter_attempts=[], arm_commands=0,
             motor_commands=0, mode_commands=0, nx_accessed=False,
             backup=str(backup), backup_sha256=hashlib.sha256(backup.read_bytes()).hexdigest(),
             powercycle_persistence_verified=False, automatic_values_history_unknown=True)
    try:
        c.open()
        if FIRMWARE not in c.query('ver all'): raise ValueError('Unexpected firmware')
        r['before_state'] = guard(c)
        r['before_parameters'] = {p: read_parameter(c, p) for p in PARAMETERS}
        r['auto_before'] = {p: read_parameter(c, p) for p in AUTO}
        r['auto_description'] = {p: c.query('param show '+p) for p in AUTO}
        for p in PARAMETERS:
            allowed = RESTORE[p] if p in RESTORE else (before[p],)
            if not any(same(r['before_parameters'][p], v) for v in allowed):
                raise ValueError('Unexpected current value: '+p)
        save_json(output/'report.json', r)
        for p, (_, value) in RESTORE.items():
            if same(r['before_parameters'][p], value): continue
            attempt = dict(name=p, target=value)
            r['parameter_attempts'].append(attempt)
            save_json(output/'report.json', r)
            attempt['dispatch'] = c.write_fixed(f'param set {p} {value:g}')
            attempt['readback'] = read_parameter(c, p)
            if not same(attempt['readback'], value): raise ValueError('Restore readback failed: '+p)
        r['save'] = c.write_fixed('param save')
        r['after_parameters'] = {p: read_parameter(c, p) for p in PARAMETERS}
        r['remaining_inventory_differences'] = {p: dict(original=before[p], actual=r['after_parameters'][p])
            for p in PARAMETERS if not same(before[p], r['after_parameters'][p])}
        r['auto_after'] = {p: read_parameter(c, p) for p in AUTO}
        r['after_state'] = guard(c)
        if r['remaining_inventory_differences']: raise ValueError('Inventory not restored')
        r['complete'] = True
    except Exception as exc:
        r['error'] = str(exc)
    finally:
        c.close(); r['port_closed'] = not c.s.is_open
        save_json(output/'console.json', c.records)
        save_json(output/'report.json', r)
    return r


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM13')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--backup', type=Path, required=True)
    p.add_argument('--execute', action='store_true')
    a = p.parse_args()
    if not a.execute:
        print(json.dumps(dict(plan=RESTORE, hardware_opened=False))); return 0
    r = restore(a.port, a.output, a.backup)
    print(json.dumps({k: r[k] for k in ('complete', 'remaining_inventory_differences',
        'auto_before', 'auto_after', 'after_state', 'port_closed', 'error') if k in r}, indent=2))
    return 0 if r['complete'] else 2


if __name__ == '__main__': raise SystemExit(main())
