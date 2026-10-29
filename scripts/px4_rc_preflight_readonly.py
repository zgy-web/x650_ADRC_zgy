"""Read-only PC inventory for real PX4 RC takeover and failsafe preparation.

Requires a disarmed vehicle. No arming, mode command, parameter write, motor
command, or NX connection is implemented. Missing topics/parameters stay unknown.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from px4_debug_console import DebugConsole, FIRMWARE, clean, status_fields
from px4_motor_preflight_readonly import parse_parameter
from px4_motor_single_pulse import require_zero

PARAMETERS = (
    "COM_RC_IN_MODE", "COM_RC_LOSS_T", "NAV_RCL_ACT", "COM_RCL_EXCEPT", "COM_FAIL_ACT_T",
    "COM_RC_OVERRIDE", "COM_RC_STICK_OV", "COM_OF_LOSS_T", "COM_OBL_RC_ACT",
    "COM_DL_LOSS_T", "NAV_DLL_ACT", "COM_DLL_EXCEPT", "COM_KILL_DISARM",
    "COM_DISARM_LAND", "COM_DISARM_PRFLT", "COM_DISARM_MAN", "COM_ARM_SWISBTN",
    "COM_RC_ARM_HYST", "COM_PREARM_MODE", "COM_ARM_WO_GPS", "CBRK_IO_SAFETY",
    "RC_MAP_ROLL", "RC_MAP_PITCH", "RC_MAP_YAW", "RC_MAP_THROTTLE",
    "RC_MAP_FLTMODE", "RC_MAP_MODE_SW", "RC_MAP_KILL_SW", "RC_MAP_ARM_SW",
    "RC_MAP_RETURN_SW", "RC_MAP_OFFB_SW", "RC_MAP_POSCTL_SW", "RC_MAP_LOITER_SW",
    "RC_KILLSW_TH", "RC_KILLSWITCH_TH", "RC_ARMSWITCH_TH", "RC_FAILS_THR", "RC_INPUT_PROTO", "RC_PORT_CONFIG",
    "RC_MAP_TERM_SW", "RC_MAP_FLTM_BTN", "RC_CHAN_CNT",
    "MOT_POLE_COUNT", "DSHOT_TEL_CFG", "DSHOT_BIDIR_EN",
    "RC_CRSF_PRT_CFG", "RC_CRSF_TEL_EN", "RC_DZ", "UXRCE_DDS_CFG",
    "MAV_0_CONFIG", "MAV_1_CONFIG", "MAV_2_CONFIG", "MAV_0_MODE", "MAV_1_MODE", "MAV_2_MODE",
    "SER_TEL1_BAUD", "SER_TEL2_BAUD", "SER_TEL3_BAUD",
) + tuple(f"COM_FLTMODE{i}" for i in range(1, 7)) + tuple(
    f"RC{i}_{field}" for i in range(1, 19) for field in ("MIN", "MAX", "TRIM", "DZ", "REV"))
STATUS_COMMANDS = (
    "ver all", "listener vehicle_status -n 1", "listener actuator_armed -n 1",
    "listener input_rc -n 1", "listener rc_channels -n 1",
    "listener manual_control_setpoint -n 1", "listener manual_control_switches -n 1",
    "listener failsafe_flags -n 1", "listener vehicle_control_mode -n 1",
    "listener vehicle_land_detected -n 1", "listener vehicle_status_flags -n 1",
    "listener esc_status -n 1", "listener battery_status -n 1", "dshot status",
    "rc_input status", "rc_update status", "manual_control status", "px4io status",
    "crsf_rc status", "mavlink status", "uxrce_dds_client status",
)
ALLOWED = frozenset(STATUS_COMMANDS + tuple(f"param show -q {x}" for x in PARAMETERS))


class RcReadConsole(DebugConsole):
    def query(self, command):
        if command not in ALLOWED or self.active_listener:
            raise ValueError("Only fixed RC inventory reads are allowed")
        self.s.write((command + "\r").encode("ascii"))
        response = self.record(command, self.read(6))
        if "nsh>" not in response:
            raise RuntimeError("No console prompt after " + command)
        return response


def collect(port, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    c = RcReadConsole(port)
    report = dict(schema_version=1, kind="real_px4_readonly_rc_inventory",
                  utc_started=datetime.now(timezone.utc).isoformat(), port=port,
                  firmware_expected=FIRMWARE, firmware_verified=False,
                  read_complete=False, parameters={}, status_text={}, motor_commands=0,
                  parameter_writes=0, arm_commands=0, mode_commands=0, nx_accessed=False,
                  rc_takeover_verified=False, armed_failsafe_verified=False,
                  physical_stop_verified=False, flight_ready=False,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        c.open()
        if FIRMWARE not in clean(c.query("ver all")):
            raise ValueError("Unexpected firmware")
        report["firmware_verified"] = True
        report["before"] = status_fields(c.query("listener vehicle_status -n 1"))
        report["before_outputs"] = require_zero(c.query("dshot status"))
        for command in STATUS_COMMANDS[2:]:
            report["status_text"][command] = clean(c.query(command))
        for name in PARAMETERS:
            report["parameters"][name] = parse_parameter(c.query("param show -q " + name))
        report["after_outputs"] = require_zero(c.query("dshot status"))
        report["after"] = status_fields(c.query("listener vehicle_status -n 1"))
        if report["after"]["timestamp"] <= report["before"]["timestamp"]:
            raise ValueError("FC timestamp did not advance")
        report["read_complete"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        c.close()
        report["unknown_parameters"] = [x for x in PARAMETERS if report["parameters"].get(x) is None]
        report["port_closed"] = not c.s.is_open
        (output / "console.json").write_text(json.dumps(c.records, indent=2, ensure_ascii=False), encoding="utf-8")
        report["console_sha256"] = hashlib.sha256((output / "console.json").read_bytes()).hexdigest()
        (output / "inventory.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM13")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = collect(args.port, args.output)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["read_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
