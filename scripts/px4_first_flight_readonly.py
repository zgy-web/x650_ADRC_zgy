"""PC-only fixed read-only inventory for the first X650 flight.

This is an inventory, not an authorization to fly. It never changes parameters,
refreshes pre-arm checks, arms, changes mode, or starts a calibration/motor test.
Physical propellers, power wiring, flight area and pilot are not observable here.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re

from px4_debug_console import DebugConsole, FIRMWARE, clean, status_fields
from px4_motor_preflight_readonly import parse_parameter
from px4_motor_single_pulse import require_zero
from px4_rc_switch_setup import boolean, field, fresh
from px4_imu_pole_check import assess_bias


PARAMETERS = (
    'SYS_AUTOSTART', 'SYS_HAS_GPS', 'GPS_1_CONFIG', 'GPS_2_CONFIG',
    'SENS_BOARD_ROT', 'SENS_BOARD_X_OFF', 'SENS_BOARD_Y_OFF', 'SENS_BOARD_Z_OFF',
    'EKF2_MULTI_IMU', 'EKF2_ABL_LIM', 'EKF2_GPS_CTRL', 'EKF2_EV_CTRL', 'EKF2_HGT_REF',
    'SENS_MAG_MODE', 'CAL_MAG0_ID', 'CAL_MAG0_ROT', 'CAL_MAG0_PRIO',
    'CAL_MAG1_ID', 'CAL_MAG1_ROT', 'CAL_MAG1_PRIO',
    'CA_AIRFRAME', 'CA_ROTOR_COUNT', 'MOT_POLE_COUNT', 'DSHOT_MIN', 'DSHOT_3D_ENABLE',
    'THR_MDL_FAC', 'MPC_THR_HOVER', 'MPC_USE_HTE', 'MPC_THR_MIN', 'MPC_THR_MAX',
    'MPC_TKO_SPEED', 'MPC_LAND_SPEED', 'MPC_TILTMAX_AIR', 'MPC_TILTMAX_LND',
    'MPC_Z_VEL_MAX_UP', 'MPC_Z_VEL_MAX_DN', 'MPC_VEL_MANUAL', 'MPC_ACC_HOR_MAX',
    'MPC_MAN_TILT_MAX', 'MPC_MAN_Y_MAX', 'MPC_Z_VEL_ALL', 'SYS_VEHICLE_RESP',
    'MPC_TKO_RAMP_T', 'MPC_ALT_MODE', 'MPC_THR_CURVE', 'MPC_MANTHR_MIN',
    'MC_ROLL_P', 'MC_PITCH_P', 'MC_YAW_P',
    'MC_ROLLRATE_P', 'MC_ROLLRATE_I', 'MC_ROLLRATE_D',
    'MC_PITCHRATE_P', 'MC_PITCHRATE_I', 'MC_PITCHRATE_D',
    'MC_YAWRATE_P', 'MC_YAWRATE_I', 'MC_YAWRATE_D',
    'IMU_GYRO_CUTOFF', 'IMU_DGYRO_CUTOFF', 'IMU_GYRO_DNF_EN',
    'BAT1_SOURCE', 'BAT1_N_CELLS', 'BAT1_CAPACITY', 'BAT1_V_CHARGED', 'BAT1_V_EMPTY',
    'BAT_LOW_THR', 'BAT_CRIT_THR', 'BAT_EMERGEN_THR', 'COM_LOW_BAT_ACT',
    'COM_POWER_COUNT', 'SENS_EN_INA226', 'SENS_EN_INA228', 'COM_RC_IN_MODE',
    'COM_RC_LOSS_T', 'NAV_RCL_ACT', 'COM_RCL_EXCEPT', 'COM_FAIL_ACT_T',
    'COM_DL_LOSS_T', 'NAV_DLL_ACT', 'COM_RC_OVERRIDE', 'COM_OBL_RC_ACT',
    'COM_ARM_WO_GPS', 'COM_ARM_HFLT_CHK', 'COM_DISARM_LAND', 'COM_DISARM_PRFLT',
    'COM_KILL_DISARM', 'CBRK_IO_SAFETY', 'CBRK_USB_CHK', 'CBRK_SUPPLY_CHK',
    'RC_MAP_ARM_SW', 'RC_MAP_KILL_SW', 'RC_MAP_FLTMODE',
    'SDLOG_MODE', 'SDLOG_PROFILE', 'GF_ACTION', 'GF_MAX_HOR_DIST', 'GF_MAX_VER_DIST',
) + tuple('COM_FLTMODE'+str(i) for i in range(1, 7)) + tuple(
    'PWM_AUX_FUNC'+str(i) for i in range(1, 5))
TOPICS = (
    'vehicle_status', 'actuator_armed', 'manual_control_setpoint', 'manual_control_switches',
    'input_rc', 'health_report', 'failsafe_flags', 'vehicle_land_detected',
    'vehicle_local_position', 'vehicle_global_position', 'home_position',
    'vehicle_attitude', 'sensor_gps', 'sensor_selection', 'estimator_selector_status',
    'battery_status', 'system_power', 'esc_status',
)
STATUS = ('ver all', 'commander status', 'dshot status', 'gps status',
          'sensors status', 'ekf2 status', 'logger status', 'mavlink status',
          'battery_status status', 'adc status', 'ina220 status',
          'ina226 status', 'ina228 status', 'ina238 status')
READS = frozenset(STATUS + tuple('listener '+t+' -n 1' for t in TOPICS) +
                  tuple('listener estimator_sensor_bias -i '+str(i)+' -n 1' for i in range(3)) +
                  tuple('param show -q '+p for p in PARAMETERS))


class FirstFlightConsole(DebugConsole):
    def query(self, command):
        if command not in READS or self.active_listener:
            raise ValueError('Only fixed first-flight inventory reads are allowed')
        self.s.write((command+'\r').encode('ascii'))
        text = clean(self.record(command, self.read(6)))
        if 'nsh>' not in text or re.search(r'Hard Fault|PANIC|Assertion failed', text, re.I):
            raise RuntimeError('Read-only console interrupted')
        return text


def parked(c):
    raw = c.query('listener vehicle_status -n 1')
    fresh(raw, 1.5)
    state = status_fields(raw)
    armed = c.query('listener actuator_armed -n 1')
    fresh(armed, 1.5)
    if any(boolean(armed, k) for k in ('armed', 'prearmed', 'termination', 'in_esc_calibration_mode')):
        raise ValueError('Not an inactive bench')
    return dict(state=state, kill=boolean(armed, 'kill'),
                outputs=require_zero(c.query('dshot status')))


def observed(raw, keys):
    """Unknown or unparseable fields stay unknown, without assuming zero/False."""
    values = {}
    for key in keys:
        try:
            values[key] = field(raw, key)
        except (ValueError, TypeError):
            values[key] = None
    return values


def assess(raw, parameters, biases):
    validity = observed(raw.get('vehicle_local_position', ''),
                        ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid', 'heading_good_for_control'))
    try:
        fresh(raw.get('vehicle_local_position', ''), 1.5)
        position_fresh = True
    except ValueError:
        position_fresh = False
    blockers = [k+' not valid' for k,v in validity.items() if v != 'True']
    if not position_fresh:
        blockers.append('local position missing or stale')
    return dict(
        flight_ready=False,
        scope='read-only inventory; no automatic flight authorization',
        position_fresh=position_fresh, local_position_validity=validity,
        positioning_blockers=blockers,
        gps=observed(raw.get('sensor_gps', ''), ('fix_type','satellites_used','eph','epv','vel_ned_valid')),
        power=observed(raw.get('system_power', ''), ('voltage5v_v','usb_connected','brick_valid','servo_valid')),
        battery=observed(raw.get('battery_status', ''), ('connected','voltage_v','cell_count','remaining','warning','source')),
        rc=observed(raw.get('manual_control_setpoint', ''), ('valid','throttle')),
        switches=observed(raw.get('manual_control_switches', ''), ('arm_switch','mode_slot','kill_switch')),
        estimator_biases=biases,
        mode_parameters={k:parameters.get(k) for k in ('COM_FLTMODE1','COM_FLTMODE4','COM_FLTMODE6')},
        circuit_breakers={k:parameters.get(k) for k in ('CBRK_IO_SAFETY','CBRK_USB_CHK','CBRK_SUPPLY_CHK')},
        motor_poles=parameters.get('MOT_POLE_COUNT'),
        physical_checks_not_observable=['flight configuration and actual takeoff mass',
            'propeller orientation/fastening and structural inspection',
            'power redundancy and stability with external USB disconnected',
            'pilot, clear flight area, weather, and on-site operating permissions',
            'actual loaded attitude/vibration and landing behavior'],
    )


def collect(port, output):
    output.mkdir(parents=True, exist_ok=False)
    c = FirstFlightConsole(port)
    report = dict(complete=False, flight_ready=False, parameter_writes=0,
                  arm_commands=0, mode_commands=0, motor_commands=0, nx_accessed=False,
                  utc=datetime.now(timezone.utc).isoformat(), firmware=FIRMWARE,
                  parameters={}, raw={}, biases={},
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        c.open()
        if FIRMWARE not in c.query('ver all'):
            raise ValueError('Unexpected firmware')
        report['before'] = parked(c)
        report['status'] = {cmd:c.query(cmd) for cmd in STATUS[1:]}
        report['parameters'] = {p:parse_parameter(c.query('param show -q '+p)) for p in PARAMETERS}
        report['raw'] = {topic:c.query('listener '+topic+' -n 1') for topic in TOPICS}
        for i in range(3):
            text = c.query('listener estimator_sensor_bias -i '+str(i)+' -n 1')
            report['raw']['estimator_sensor_bias_'+str(i)] = text
            try:
                report['biases'][str(i)] = assess_bias(text)
            except (ValueError, TypeError) as exc:
                report['biases'][str(i)] = dict(unknown=str(exc))
        report['assessment'] = assess(report['raw'], report['parameters'], report['biases'])
        report['after'] = parked(c)
        if report['after']['state']['timestamp'] <= report['before']['state']['timestamp']:
            raise ValueError('Flight-controller time did not advance')
        report['unknown_parameters'] = [p for p in PARAMETERS if report['parameters'][p] is None]
        report['complete'] = True
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        c.close()
        report['port_closed'] = not c.s.is_open
        (output/'console.json').write_text(json.dumps(c.records, indent=2), encoding='utf-8')
        report['console_sha256'] = hashlib.sha256((output/'console.json').read_bytes()).hexdigest()
        (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM13')
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    r = collect(a.port, a.output)
    print(json.dumps({k:r[k] for k in ('complete','flight_ready','port_closed','assessment','error','unknown_parameters') if k in r}, indent=2))
    return 0 if r['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
