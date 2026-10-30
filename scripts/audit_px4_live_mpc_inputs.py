"""Read-only PC audit before a future real-state whole-body MPC shadow run.

No NX access, state injection, parameter writes, arming or setpoint publication.
This bounded snapshot is not a clock-alignment or continuous-stream qualification.
Joint feedback and applied-command history cannot be verified from PX4 alone.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

from prepare_px4_armed_uart import PreflightConsole, READS
from px4_debug_console import FIRMWARE, clean
from px4_imu_pole_check import guard
from px4_rc_switch_setup import fresh
from nx_uart_operator_panel import scalar


TOPICS = ('vehicle_status', 'actuator_armed', 'failsafe_flags',
          'vehicle_local_position', 'vehicle_attitude', 'vehicle_odometry')
EXTRA = frozenset('listener '+name+' -n 1' for name in TOPICS)


class AuditConsole(PreflightConsole):
    def query(self, command):
        if command not in READS | EXTRA or self.active_listener:
            raise ValueError('Read-only fixed input audit')
        if command not in EXTRA:
            return super().query(command)
        self.s.write((command+'\r').encode('ascii'))
        text = clean(self.record(command, self.read(6)))
        if 'nsh>' not in text or re.search(r'Hard Fault|PANIC|Assertion failed', text, re.I):
            raise RuntimeError('Input readback interrupted')
        return text


def assess(raw):
    p = raw['vehicle_local_position']
    validity = {k:scalar(p, k) for k in
                ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid', 'heading_good_for_control')}
    source_fresh = {}
    for key in ('vehicle_attitude', 'vehicle_local_position', 'vehicle_odometry'):
        try:
            fresh(raw[key], 1.5)
            source_fresh[key] = True
        except ValueError:
            source_fresh[key] = False
    blockers = [k+' is not valid' for k, v in validity.items() if v != 'True']
    blockers += [k+' is missing or stale' for k,v in source_fresh.items() if not v]
    # These conditions require the complete live adapter, not a zero-filled
    # snapshot or an assumption based on the flight-controller topics above.
    blockers += ['real joint angles and velocities not verified by this PX4-only audit',
                 'applied joint-command history not verified',
                 'live MAVLink world-acceleration/reset adapter not qualified',
                 'cross-device time alignment not qualified']
    return dict(whole_body_shadow_ready=False, blockers=blockers,
                local_position_validity=validity, source_fresh=source_fresh,
                attitude_invalid=scalar(raw['failsafe_flags'], 'attitude_invalid'),
                local_position_invalid=scalar(raw['failsafe_flags'], 'local_position_invalid'),
                local_velocity_invalid=scalar(raw['failsafe_flags'], 'local_velocity_invalid'),
                joint_feedback_verified=False, applied_history_verified=False,
                mpc_executed=False, substituted_states=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', default='COM13')
    parser.add_argument('--power-disconnected', action='store_true', required=True)
    a = parser.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    c = AuditConsole(a.port)
    report = dict(complete=False, nx_accessed=False, parameter_writes=0,
                  arm_commands=0, mode_commands=0, setpoints=0,
                  power_disconnected_confirmed=True, firmware=FIRMWARE,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        c.open()
        if FIRMWARE not in c.query('ver all'):
            raise ValueError('Unexpected firmware')
        report['before'] = guard(c, require_kill=True, allow_unpublished_battery=True)
        report['raw'] = {topic:c.query('listener '+topic+' -n 1') for topic in TOPICS}
        report['assessment'] = assess(report['raw'])
        report['after'] = guard(c, require_kill=True, allow_unpublished_battery=True)
        if report['after']['state']['timestamp'] <= report['before']['state']['timestamp']:
            raise ValueError('Flight-controller time did not advance')
        report['complete'] = True
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        c.close()
        report['port_closed'] = not c.s.is_open
        (a.output/'console.json').write_text(json.dumps(c.records, indent=2), encoding='utf-8')
        (a.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('raw','before','after')}, indent=2))
    return 0 if report['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
