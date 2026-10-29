"""PC-side, fixed read-only NSH inventory for rotor commissioning.

This does not configure outputs, calibrate ESCs, arm, or spin motors. Physical
wiring and rotation must be checked separately. Results stay on the PC.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re

from px4_debug_console import DebugConsole, FIRMWARE, clean, status_fields


PARAMETERS = (
    "SYS_AUTOSTART", "CA_AIRFRAME", "CA_ROTOR_COUNT", "SENS_BOARD_ROT",
    "SENS_BOARD_X_OFF", "SENS_BOARD_Y_OFF", "SENS_BOARD_Z_OFF",
    "SYS_USE_IO", "COM_MOT_TEST_EN", "CBRK_IO_SAFETY", "DSHOT_MIN",
    "DSHOT_3D_ENABLE", "DSHOT_BIDIR_EN", "DSHOT_TEL_CFG", "MOT_POLE_COUNT",
    "PWM_AUX_REV", "THR_MDL_FAC", "CA_R_REV",
) + tuple(f"CA_ROTOR{i}_{field}" for i in range(4)
          for field in ("PX", "PY", "PZ", "AX", "AY", "AZ", "KM")) + tuple(
    f"PWM_{bank}_FUNC{i}" for bank in ("MAIN", "AUX") for i in range(1, 9)
) + tuple(f"PWM_AUX_TIM{i}" for i in range(5))
STATUS_COMMANDS = (
    "ver all", "listener vehicle_status -n 1", "listener actuator_armed -n 1",
    "control_allocator status", "pwm_out status", "px4io status", "dshot status",
    "listener actuator_motors -n 1", "listener esc_status -n 1",
)
ALLOWED = frozenset(STATUS_COMMANDS + tuple(f"param show -q {name}" for name in PARAMETERS))


def parse_parameter(text):
    """Missing/error/ambiguous responses remain unknown, never become zero."""
    text = clean(text)
    if re.search(r"\b(?:ERROR|WARN|not found|not running|unknown)\b", text, re.I):
        return None
    values = re.findall(r"^\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*$", text, re.M)
    if len(values) != 1:
        return None
    value = float(values[0])
    if not math.isfinite(value):
        return None
    return int(value) if value.is_integer() else value


class RotorReadConsole(DebugConsole):
    def query(self, command):
        if command not in ALLOWED or self.active_listener:
            raise ValueError("Only fixed rotor inventory read commands are allowed")
        self.s.write((command + "\r").encode("ascii"))
        text = self.record(command, self.read(6))
        if "nsh>" not in text:
            raise RuntimeError("No prompt after " + command)
        return text


def output_assignments(parameters):
    """List observed assignments; never infer physical ESC wiring from them."""
    return {f"{bank}{i}": parameters.get(f"PWM_{bank}_FUNC{i}")
            for bank in ("MAIN", "AUX") for i in range(1, 9)}


def collect(port, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    console = RotorReadConsole(port)
    report = dict(
        schema_version=1, kind="physical_px4_readonly_rotor_inventory",
        utc_started=datetime.now(timezone.utc).isoformat(), port=port,
        firmware_expected=FIRMWARE, read_complete=False, parameters={},
        status_text={}, physical_wiring_verified=False, rotation_verified=False,
        flight_ready=False, parameter_writes=0, actuator_commands=0,
        nx_accessed=False, source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    )
    try:
        console.open()
        firmware = console.query("ver all")
        if FIRMWARE not in clean(firmware):
            raise RuntimeError("Firmware does not match the reviewed PX4 build")
        report["firmware_verified"] = True
        report["before"] = status_fields(console.query("listener vehicle_status -n 1"))
        for command in STATUS_COMMANDS[2:]:
            report["status_text"][command] = clean(console.query(command))
        for name in PARAMETERS:
            report["parameters"][name] = parse_parameter(console.query(f"param show -q {name}"))
        report["after"] = status_fields(console.query("listener vehicle_status -n 1"))
        if report["after"]["timestamp"] <= report["before"]["timestamp"]:
            raise RuntimeError("FC status timestamp did not advance")
        report["read_complete"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        console.close()
        report["port_closed"] = not console.s.is_open
        report["unknown_parameters"] = [n for n in PARAMETERS if report["parameters"].get(n) is None]
        report["observed_output_functions"] = output_assignments(report["parameters"])
        (output / "console.json").write_text(json.dumps(console.records, indent=2, ensure_ascii=False), encoding="utf-8")
        report["console_sha256"] = hashlib.sha256((output / "console.json").read_bytes()).hexdigest()
        (output / "inventory.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM13")
    parser.add_argument("--output", type=Path, required=True, help="New directory on the connected PC")
    args = parser.parse_args()
    report = collect(args.port, args.output)
    print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
    return 0 if report["read_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
