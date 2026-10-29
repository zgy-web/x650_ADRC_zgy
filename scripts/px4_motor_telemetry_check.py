"""Read-only, stopped-motor telemetry check on the PC FMU Debug connection.

The 250 ms age limit is a bench diagnostic criterion, not a PX4 specification.
Counters are compared over the observation window; historical errors are retained.
Neither the UART online mask nor the armed mask proves fresh ESC data.
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
from px4_motor_preflight_readonly import RotorReadConsole

MAX_AGE_S = .250


def _number(text, name):
    values = re.findall(r"^\s*" + re.escape(name) + r":\s*(\S+)", text, re.M)
    if len(values) != 1:
        raise ValueError("Missing or ambiguous field: " + name)
    value = float(values[0])
    if not math.isfinite(value):
        raise ValueError("Nonfinite field: " + name)
    return value


def _timestamp(text):
    matches = re.findall(r"^\s*timestamp:\s*(\d+)\s+\(([-+\d.eE]+) seconds ago\)", text, re.M)
    if len(matches) != 1:
        raise ValueError("Missing or ambiguous timestamp and age")
    stamp, age = int(matches[0][0]), float(matches[0][1])
    if stamp <= 0 or not math.isfinite(age) or not 0 <= age <= MAX_AGE_S:
        raise ValueError("Telemetry timestamp is zero, invalid, or older than 250 ms")
    return dict(timestamp_us=stamp, age_s=age)


def telemetry_snapshot(dshot_text, esc_text):
    """Require four fresh stopped ESC slots; does not prove their physical IDs."""
    counters = {}
    for key, label in (("success", "successful ESC frames"), ("timeouts", "timeouts"), ("crc_errors", "CRC errors")):
        values = re.findall(r"Number of " + label + r":\s*(\d+)\s*$", clean(dshot_text), re.M)
        if len(values) != 1:
            raise ValueError("Missing or ambiguous telemetry counter: " + key)
        counters[key] = int(values[0])
    parts = re.split(r"\n\s*esc\[(\d+)\] \(esc_report\):", clean(esc_text))
    header = _timestamp(parts[0])
    if _number(parts[0], "esc_count") != 4:
        raise ValueError("Expected four configured ESCs")
    samples = {}
    for offset in range(1, len(parts), 2):
        index = int(parts[offset])
        if index >= 4:
            continue
        if index in samples:
            raise ValueError("Duplicate ESC slot")
        block = parts[offset + 1]
        values = {name: _number(block, name) for name in
                  ("actuator_function", "esc_rpm", "esc_voltage", "esc_current", "esc_temperature")}
        if values["actuator_function"] != 101 + index or values["esc_rpm"] != 0:
            raise ValueError("Unexpected ESC function or nonzero stopped-motor RPM")
        if not 0 < values["esc_voltage"] <= 30:
            raise ValueError("Invalid voltage for this 6S bench setup")
        samples[index] = {**_timestamp(block), **values}
    if set(samples) != set(range(4)):
        raise ValueError("Missing ESC slot")
    return dict(counters=counters, header=header, samples=samples, calibrated=False)


def assess_samples(snapshots):
    if len(snapshots) < 2:
        raise ValueError("At least two observations are required")
    for previous, current in zip(snapshots, snapshots[1:]):
        a, b = previous["counters"], current["counters"]
        if b["success"] <= a["success"]:
            raise ValueError("Successful telemetry frame count did not advance")
        if b["crc_errors"] != a["crc_errors"] or b["timeouts"] != a["timeouts"]:
            raise ValueError("CRC/timeout counters changed during the observation window")
        if current["header"]["timestamp_us"] <= previous["header"]["timestamp_us"]:
            raise ValueError("ESC status timestamp did not advance")
        for index in range(4):
            if current["samples"][index]["timestamp_us"] <= previous["samples"][index]["timestamp_us"]:
                raise ValueError("An individual ESC timestamp did not advance")
    return dict(observed_healthy=True, successful_frames_added=snapshots[-1]["counters"]["success"] - snapshots[0]["counters"]["success"],
                crc_errors_added=0, timeouts_added=0, max_age_s=MAX_AGE_S,
                scope="Only this stopped-motor observation window; not a flight-readiness result")


def collect(port, output):
    from px4_motor_single_pulse import require_zero, check_armed
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    c = RotorReadConsole(port)
    report = dict(utc_started=datetime.now(timezone.utc).isoformat(), kind="readonly_stopped_esc_telemetry",
                  samples=[], observed_healthy=False, motor_commands=0, parameter_writes=0, nx_accessed=False,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        c.open()
        if FIRMWARE not in clean(c.query("ver all")):
            raise ValueError("Unexpected firmware")
        parsed = []
        for i in range(3):
            check_armed(c.query("listener actuator_armed -n 1"))
            row = dict(status=status_fields(c.query("listener vehicle_status -n 1")))
            row["dshot"] = clean(c.query("dshot status"))
            row["outputs"] = require_zero(row["dshot"])
            row["esc_status"] = clean(c.query("listener esc_status -n 1"))
            report["samples"].append(row)
            parsed.append(telemetry_snapshot(row["dshot"], row["esc_status"]))
            if i < 2:
                time.sleep(.5)
        report["assessment"] = assess_samples(parsed)
        report["observed_healthy"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        c.close()
        (output / "console.json").write_text(json.dumps(c.records, indent=2, ensure_ascii=False), encoding="utf-8")
        report["console_sha256"] = hashlib.sha256((output / "console.json").read_bytes()).hexdigest()
        (output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM13")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = collect(args.port, args.output)
    print(json.dumps({k: v for k, v in result.items() if k != "samples"}, indent=2))
    return 0 if result["observed_healthy"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
