"""Fixed, user-confirmed 6S/30000 mAh metadata correction; no protection bypass.

Only BAT1_N_CELLS=6, BAT1_CAPACITY=30000 and param save may be written.
Require disarmed, actual kill, zero output, valid RC, SF off, minimum throttle.
Set cell count first while capacity remains unknown, then verify live SOC before
enabling capacity integration. No arm, mode, motor, calibration or NX actions.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

from px4_debug_console import FIRMWARE, clean, status_fields
from px4_rc_powered_stop_observer import PoweredReadConsole, snapshot, READS
from px4_rc_switch_setup import boolean, fresh, number, field, read_parameter
from px4_motor_single_pulse import require_zero

PARAMETERS = ("BAT1_N_CELLS", "BAT1_CAPACITY", "BAT1_SOURCE", "BAT1_V_EMPTY", "BAT1_V_CHARGED",
              "BAT_LOW_THR", "BAT_CRIT_THR", "BAT_EMERGEN_THR", "COM_ARM_BAT_MIN")
WRITES = frozenset(("param set BAT1_N_CELLS 6", "param set BAT1_CAPACITY 30000", "param save"))
READ_ONLY = READS | frozenset("param show -q " + name for name in PARAMETERS)


class MetadataConsole(PoweredReadConsole):
    def __init__(self, port):
        super().__init__(port)
        self.permit = False

    def query(self, command):
        write = command in WRITES
        if self.active_listener or (command not in READ_ONLY and not (write and self.permit)):
            raise ValueError("Outside fixed battery metadata scope")
        if write:
            self.permit = False
        self.s.write((command + "\r").encode("ascii"))
        text = clean(self.record(command, self.read(6)))
        if "nsh>" not in text:
            raise RuntimeError("No prompt after " + command)
        return text


def validate_parked(row):
    raw = row["raw"]
    fresh(raw["vehicle_status"], 1.5)
    status_fields(raw["vehicle_status"])
    require_zero(raw["dshot"])
    fresh(raw["actuator_armed"], 1.5)
    if not boolean(raw["actuator_armed"], "kill") or any(boolean(raw["actuator_armed"], x) for x in
            ("armed", "prearmed", "lockdown", "termination", "in_esc_calibration_mode")):
        raise ValueError("Require disarmed actual kill")
    fresh(raw["input_rc"], .25)
    if boolean(raw["input_rc"], "rc_lost") or boolean(raw["input_rc"], "rc_failsafe"):
        raise ValueError("RC is invalid")
    fresh(raw["manual_control_switches"], 1.5)
    if tuple(int(field(raw["manual_control_switches"], x)) for x in ("arm_switch", "mode_slot", "kill_switch")) != (3, 1, 1):
        raise ValueError("SF/SA/SB differs from parked state")
    fresh(raw["manual_control_setpoint"], .3)
    if not boolean(raw["manual_control_setpoint"], "valid") or number(raw["manual_control_setpoint"], "throttle") > -.95:
        raise ValueError("Require valid minimum throttle")


def validate_battery(text):
    fresh(text, 1.5)
    if (not boolean(text, "connected") or number(text, "cell_count") != 6 or
            not 22 <= number(text, "voltage_v") <= 25.5 or
            not .3 <= number(text, "remaining") <= 1 or number(text, "warning") != 0):
        raise ValueError("Live 6S battery estimate has not recovered")


def run(port, output, execute=False):
    if not execute:
        return dict(plan=["BAT1_N_CELLS=6", "validate live SOC", "BAT1_CAPACITY=30000", "param save"], writes=0)
    out = Path(output);out.mkdir(parents=True, exist_ok=False)
    c = MetadataConsole(port)
    r = dict(kind="fixed_6s_30000mah_metadata", complete=False, write_attempts=[], guard_rows=[],
             utc_started=datetime.now(timezone.utc).isoformat(), firmware=FIRMWARE,
             source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), nx_accessed=False)
    try:
        c.open()
        if FIRMWARE not in c.query("ver all"):
            raise ValueError("Unexpected firmware")
        r["before"] = {p: read_parameter(c, p) for p in PARAMETERS}
        if r["before"]["BAT1_N_CELLS"] not in (0, 6) or r["before"]["BAT1_CAPACITY"] not in (-1, 30000):
            raise ValueError("Original battery metadata differs")
        def write(command):
            row = snapshot(c);r["guard_rows"].append(row);validate_parked(row)
            r["write_attempts"].append(command);c.permit = True
            return c.query(command)
        if r["before"]["BAT1_N_CELLS"] == 0:
            write("param set BAT1_N_CELLS 6")
        if read_parameter(c, "BAT1_N_CELLS") != 6:
            raise ValueError("Cell count readback mismatch")
        r["battery_samples"] = []
        deadline = time.monotonic() + 12
        while True:
            text = c.query("listener battery_status -n 1");r["battery_samples"].append(text)
            try:
                validate_battery(text);break
            except ValueError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(1)
        if r["before"]["BAT1_CAPACITY"] == -1:
            write("param set BAT1_CAPACITY 30000")
        r["after"] = {p: read_parameter(c, p) for p in PARAMETERS}
        if r["after"]["BAT1_N_CELLS"] != 6 or r["after"]["BAT1_CAPACITY"] != 30000:
            raise ValueError("Metadata readback mismatch")
        if any(r["before"][p] != r["after"][p] for p in PARAMETERS[2:]):
            raise ValueError("Unrelated battery setting changed")
        write("param save")
        r["final_battery"] = c.query("listener battery_status -n 1")
        validate_battery(r["final_battery"])
        r["complete"] = True
    except Exception as e:
        r["error"] = repr(e)
    finally:
        c.close();r["port_closed"] = not c.s.is_open
        (out / "console.json").write_text(json.dumps(c.records, indent=2), encoding="utf-8")
        (out / "report.json").write_text(json.dumps(r, indent=2), encoding="utf-8")
    return r


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="COM13");p.add_argument("--output", type=Path)
    p.add_argument("--execute", action="store_true");a = p.parse_args()
    if a.execute and not a.output:p.error("--output required for execution")
    r = run(a.port, a.output, a.execute)
    print(json.dumps({k: v for k, v in r.items() if k not in ("guard_rows", "battery_samples")}, indent=2))
    raise SystemExit(0 if not a.execute or r["complete"] else 1)
