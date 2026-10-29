"""Configure one stopped DShot ESC; default is a plan, never spins a motor.

Uses the same fresh bench preflight as the one-second pulse. reverse/normal are
volatile relative-direction commands; save persists the ESC's current settings.
NSH success proves dispatch, not ESC acceptance or physical rotation direction.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import time

from px4_debug_console import clean, status_fields
from px4_motor_single_pulse import ATTESTATIONS, PulseConsole, check_armed, plan, require_zero


def direction_command(motor, action):
    plan(motor)
    if action not in ("reverse", "normal", "save"):
        raise ValueError("Only single-ESC reverse, normal, or save is allowed")
    # dshot -m uses a physical output index, unlike actuator_test's function ID.
    # The inherited preflight requires AUX1..4 == Motor1..4 before this is valid.
    return f"dshot {action} -m {motor}"


class DirectionConsole(PulseConsole):
    def configure(self, action):
        command = direction_command(self.plan["motor"], action)
        if not self._permit or time.monotonic() > self._permit_until:
            raise ValueError("Fresh stopped-output preflight is required")
        self._permit = False
        self.s.write((command + "\r").encode("ascii"))
        text = self.record(command, self.read(3))
        if "nsh>" not in text or re.search(r"\b(?:ERROR|WARN|unknown|not found|Usage)\b", clean(text), re.I):
            raise RuntimeError("DShot command dispatch did not succeed")
        return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor", type=int, choices=range(1, 5), required=True)
    parser.add_argument("--action", choices=("reverse", "normal", "save"), required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--port", default="COM13")
    parser.add_argument("--output", type=Path)
    for name in ATTESTATIONS:
        parser.add_argument("--" + name.replace("_", "-"), action="store_true")
    args = parser.parse_args()
    command = direction_command(args.motor, args.action)
    if not args.execute:
        print(json.dumps(dict(command=command, persistent=args.action == "save", motor_pulse=False)))
        return 0
    if args.output is None or not all(getattr(args, name) for name in ATTESTATIONS):
        parser.error("Execution requires a new PC output directory and all bench attestations")
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(utc_started=datetime.now(timezone.utc).isoformat(), command=command,
                  command_attempted=False, command_dispatch_success=False, motor_pulse=False,
                  persistent_save_requested=args.action == "save", esc_acceptance_verified=False,
                  physical_direction_verified=False, output_zero_verified=False,
                  nx_accessed=False, fc_parameter_writes=0,
                  attestations={name: getattr(args, name) for name in ATTESTATIONS},
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    console = DirectionConsole(args.port, args.motor)
    try:
        console.open()
        report["preflight"] = console.preflight()
        report["command_attempted"] = True
        console.configure(args.action)
        report["command_dispatch_success"] = True
        time.sleep(.5)
        report["after_outputs"] = require_zero(console.query("dshot status"))
        report["output_zero_verified"] = True
        check_armed(console.query("listener actuator_armed -n 1"))
        report["after"] = status_fields(console.query("listener vehicle_status -n 1"))
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        console.close()
        report["port_closed"] = not console.s.is_open
        (args.output / "console.json").write_text(json.dumps(console.records, indent=2, ensure_ascii=False), encoding="utf-8")
        report["console_sha256"] = hashlib.sha256((args.output / "console.json").read_bytes()).hexdigest()
        (args.output / "direction_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["command_dispatch_success"] and report["output_zero_verified"] and "error" not in report else 1


if __name__ == "__main__":
    raise SystemExit(main())
