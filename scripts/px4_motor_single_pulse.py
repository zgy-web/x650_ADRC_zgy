"""One bounded, minimum-output, props-off pulse through the PC FMU console.

Default: print a plan without opening any device. --execute requires all bench
attestations. Never arms, changes parameters, configures the NX, or tests a second
motor automatically. The reviewed FC firmware owns the 1 s timeout. PC-side
checks and a best-effort stop supplement it; they cannot guarantee physical stop.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import time

from px4_debug_console import FIRMWARE, clean, status_fields
from px4_motor_preflight_readonly import RotorReadConsole, parse_parameter
from px4_motor_telemetry_check import telemetry_snapshot, assess_samples


LOCATIONS = {1: "front-right CCW", 2: "rear-left CCW", 3: "front-left CW", 4: "rear-right CW"}
EXPECTED = {
    "SYS_AUTOSTART": 4001, "CA_AIRFRAME": 0, "CA_ROTOR_COUNT": 4,
    "SENS_BOARD_ROT": 0, "PWM_AUX_TIM0": -3, "PWM_AUX_REV": 0,
    "CA_R_REV": 0, "THR_MDL_FAC": 0, "DSHOT_3D_ENABLE": 0,
    "DSHOT_MIN": .055, "COM_MOT_TEST_EN": 1,
    "DSHOT_BIDIR_EN": 0, "DSHOT_TEL_CFG": 103, "CBRK_IO_SAFETY": 22027,
    **{f"PWM_AUX_FUNC{i}": 100 + i if i <= 4 else 0 for i in range(1, 9)},
    **{f"PWM_MAIN_FUNC{i}": 0 for i in range(1, 9)},
}
ATTESTATIONS = ("props_removed", "frame_secured", "operator_ready", "power_connected", "wiring_confirmed")


def plan(motor):
    if type(motor) is not int or motor not in LOCATIONS:
        raise ValueError("Select exactly one motor from 1 to 4")
    return dict(
        motor=motor, location_viewed_from_above=LOCATIONS[motor], aux_port=motor,
        firmware=FIRMWARE, value=0, fc_timeout_seconds=1,
        pulse=f"actuator_test set -m {motor} -v 0 -t 1",
        stop=f"actuator_test set -m {motor} -v -1 -t 0.1",
        value_meaning="0 is minimum RUNNING output, not stop; DShot mixer value 109/1999",
        safety_note="IO safety bypass is present; operator must be able to disconnect ESC power",
        no_automatic_retry=True, flight_ready=False,
    )


def integer_field(text, field):
    values = re.findall(r"^\s*" + re.escape(field) + r":\s*(\d+)\s*$", clean(text), re.M)
    if len(values) != 1:
        raise ValueError("Missing or ambiguous field: " + field)
    return int(values[0])


def check_armed(text):
    for field in ("armed", "prearmed", "lockdown", "kill", "termination", "in_esc_calibration_mode"):
        values = re.findall(r"^\s*" + field + r":\s*(\S+)\s*$", clean(text), re.M)
        if values != ["False"]:
            raise ValueError("Unexpected actuator_armed flag: " + field)


def check_parameters(parameters):
    differences = {name: {"expected": value, "actual": parameters.get(name)}
                   for name, value in EXPECTED.items() if parameters.get(name) != value}
    if differences:
        raise ValueError("Configuration differs from this reviewed bench plan: " + json.dumps(differences))


def dshot_channels(text):
    text = clean(text)
    if not re.search(r"Outputs initialized:\s*yes", text) or not re.search(r"Outputs used:\s*0xf\s*$", text, re.M):
        raise ValueError("DShot four-channel driver is not initialized as expected")
    pattern = (r"^Channel (\d+): func:\s*(\d+), value:\s*(\d+), failsafe:\s*(\d+), "
               r"disarmed:\s*(\d+), min:\s*(\d+), max:\s*(\d+), center:\s*(\d+)\s*$")
    rows = [tuple(map(int, row)) for row in re.findall(pattern, text, re.M)]
    if len(rows) != 9 or {row[0] for row in rows} != set(range(9)):
        raise ValueError("Missing, duplicate, or unexpected DShot channels")
    channels = {}
    for index, function, value, failsafe, disarmed, minimum, maximum, center in rows:
        if function != (101 + index if index < 4 else 0) or (failsafe, disarmed, minimum, maximum) != (0, 0, 109, 1999):
            raise ValueError("DShot mapping or limits changed on channel " + str(index))
        channels[index] = value
    return channels


def require_zero(text):
    channels = dshot_channels(text)
    if any(channels.values()):
        raise ValueError("DShot outputs are not all zero")
    return channels


def pulse_telemetry(pulse_text, esc_text, motor):
    """Use FC source timestamps, not serial arrival time, for startup evidence.

    UART readout may finish after the pulse. Only source samples inside the
    requested window are labelled in-window. This is diagnostic evidence, not
    a calibrated RPM/current measurement or a physical direction observation.
    """
    def number(text, name):
        values = re.findall(r"^\s*" + re.escape(name) + r":\s*(\S+)", clean(text), re.M)
        if len(values) != 1:
            raise ValueError("Missing or ambiguous telemetry field: " + name)
        value = float(values[0])
        if not math.isfinite(value):
            raise ValueError("Nonfinite telemetry field: " + name)
        return value

    try:
        message = {k: number(pulse_text, k) for k in ("timestamp", "function", "value", "action", "timeout_ms")}
        if message["timestamp"] <= 0 or (message["function"], message["value"], message["action"], message["timeout_ms"]) != (100 + motor, 0, 1, 1000):
            raise ValueError("Actuator-test message does not match this pulse")
        blocks = re.split(r"\n\s*esc\[(\d+)\] \(esc_report\):", clean(esc_text))
        samples = {}
        for offset in range(1, len(blocks), 2):
            index = int(blocks[offset])
            if index >= 4:
                continue
            if index in samples:
                raise ValueError("Duplicate ESC sample")
            values = {k: number(blocks[offset + 1], k) for k in
                      ("timestamp", "actuator_function", "esc_rpm", "esc_voltage", "esc_current", "esc_temperature")}
            if values["actuator_function"] != 101 + index:
                raise ValueError("ESC sample mapping differs from reviewed channel assignment")
            values["relative_to_pulse_start_ms"] = (values["timestamp"] - message["timestamp"]) / 1000
            values["inside_requested_pulse_window"] = 0 <= values["relative_to_pulse_start_ms"] <= message["timeout_ms"]
            samples[index] = values
        if set(samples) != set(range(4)):
            raise ValueError("Not all four ESC samples are present")
        return dict(available=True, pulse_message=message, samples=samples, calibrated=False)
    except (ValueError, TypeError) as exc:
        return dict(available=False, reason=str(exc), calibrated=False)


class PulseConsole(RotorReadConsole):
    """Separate mutation entry point; ordinary query() remains read-only."""
    def __init__(self, port, motor):
        super().__init__(port)
        self.plan = plan(motor)
        self.pulse_attempted = False
        self.stop_attempted = False
        self._permit = False

    def preflight(self):
        self._permit = False
        firmware = clean(self.query("ver all"))
        if FIRMWARE not in firmware:
            raise ValueError("Unexpected firmware")
        before = status_fields(self.query("listener vehicle_status -n 1"))
        parameters = {name: parse_parameter(self.query("param show -q " + name)) for name in EXPECTED}
        check_parameters(parameters)
        check_armed(self.query("listener actuator_armed -n 1"))
        if integer_field(self.query("listener actuator_motors -n 1"), "reversible_flags") != 0:
            raise ValueError("Reversible motor flags are set")
        dshot_before = self.query("dshot status")
        require_zero(dshot_before)
        telemetry_before = telemetry_snapshot(dshot_before, self.query("listener esc_status -n 1"))
        # vehicle_status is slower than the mixer; allow one normal publication.
        time.sleep(.6)
        after = status_fields(self.query("listener vehicle_status -n 1"))
        if after["timestamp"] <= before["timestamp"]:
            raise ValueError("FC status did not advance")
        check_armed(self.query("listener actuator_armed -n 1"))
        dshot_after = self.query("dshot status")
        require_zero(dshot_after)
        telemetry_after = telemetry_snapshot(dshot_after, self.query("listener esc_status -n 1"))
        telemetry_check = assess_samples([telemetry_before, telemetry_after])
        # The telemetry read takes time; finish on a fresh stopped-output check.
        require_zero(self.query("dshot status"))
        self._permit_until = time.monotonic() + 1
        self._permit = True
        return dict(parameters=parameters, before=before, after=after, telemetry=telemetry_check,
                    telemetry_observations=[telemetry_before, telemetry_after])

    def send_pulse(self):
        if not self._permit or self.pulse_attempted or time.monotonic() > self._permit_until:
            raise ValueError("No fresh preflight permit, or pulse already attempted")
        self._permit = False
        # Set before write: a write failure may still have delivered bytes.
        self.pulse_attempted = True
        return self._command(self.plan["pulse"])

    def stop(self):
        if not self.pulse_attempted or self.stop_attempted:
            return None
        self.stop_attempted = True
        return self._command(self.plan["stop"])

    def _command(self, command):
        if command not in (self.plan["pulse"], self.plan["stop"]) or self.active_listener:
            raise ValueError("Only this one bounded pulse and its bounded stop are permitted")
        self.s.write((command + "\r").encode("ascii"))
        text = self.record(command, self.read(2), host_monotonic=time.monotonic())
        if "nsh>" not in text or re.search(r"\b(?:ERROR|WARN|not found|unknown|Usage)\b", clean(text), re.I):
            raise RuntimeError("No successful console response to bounded command")
        return text


def run(port, output, motor, attestations, console_factory=PulseConsole, sleep=time.sleep):
    if any(attestations.get(name) is not True for name in ATTESTATIONS):
        raise ValueError("All physical bench attestations are required before opening the port")
    selected = plan(motor)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    report = dict(schema_version=1, utc_started=datetime.now(timezone.utc).isoformat(),
                  plan=selected, attestations=attestations, pulse_attempted=False,
                  output_observed=False, automatic_return_to_zero_observed=False,
                  output_zero_verified=False, physical_motor_confirmed=False,
                  rotation_confirmed=False, parameter_writes=0, flight_ready=False,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    console = console_factory(port, motor)
    try:
        console.open()
        report["preflight"] = console.preflight()
        console.send_pulse()
        sleep(.08)
        during = dshot_channels(console.query("dshot status"))
        if any(value for index, value in during.items() if index != motor - 1):
            raise RuntimeError("Unexpected non-selected output; stopping")
        if during[motor - 1] not in (0, 109):
            raise RuntimeError("Selected output exceeds the reviewed minimum; stopping")
        report["during_outputs"] = during
        report["output_observed"] = during[motor - 1] == 109
        report["actuator_test_message"] = clean(console.query("listener actuator_test -n 1"))
        report["during_esc_status"] = clean(console.query("listener esc_status -n 1"))
        report["startup_telemetry"] = pulse_telemetry(report["actuator_test_message"], report["during_esc_status"], motor)
        sleep(1.2)
        report["after_timeout_outputs"] = require_zero(console.query("dshot status"))
        report["automatic_return_to_zero_observed"] = report["output_observed"]
        report["output_zero_verified"] = True
        check_armed(console.query("listener actuator_armed -n 1"))
        report["after"] = status_fields(console.query("listener vehicle_status -n 1"))
        report["esc_status"] = clean(console.query("listener esc_status -n 1"))
    except BaseException as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if console.pulse_attempted:
            try:
                console.stop()
                sleep(.2)
                report["final_outputs"] = require_zero(console.query("dshot status"))
                report["output_zero_verified"] = True
            except BaseException as exc:
                report["stop_check_error"] = f"{type(exc).__name__}: {exc}"
                report["output_zero_verified"] = False
        console.close()
        report.update(pulse_attempted=console.pulse_attempted, stop_attempted=console.stop_attempted,
                      port_closed=not console.s.is_open)
        report["console_check_passed"] = bool(report["output_observed"] and report["automatic_return_to_zero_observed"]
                                              and report["output_zero_verified"] and "error" not in report
                                              and "stop_check_error" not in report)
        (output / "console.json").write_text(json.dumps(console.records, indent=2, ensure_ascii=False), encoding="utf-8")
        report["console_sha256"] = hashlib.sha256((output / "console.json").read_bytes()).hexdigest()
        (output / "pulse_report.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor", type=int, choices=range(1, 5), default=1)
    parser.add_argument("--port", default="COM13")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    for name in ATTESTATIONS:
        parser.add_argument("--" + name.replace("_", "-"), action="store_true")
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps(plan(args.motor), indent=2))
        return 0
    if args.output is None:
        parser.error("--execute requires a new --output directory on the PC")
    attestations = {name: getattr(args, name) for name in ATTESTATIONS}
    if not all(attestations.values()):
        parser.error("--execute requires every physical bench attestation")
    report = run(args.port, args.output, args.motor, attestations)
    print(json.dumps(report, indent=2))
    return 0 if report["console_check_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
