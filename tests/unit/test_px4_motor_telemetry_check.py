"""Reject frozen/shifted telemetry even when firmware masks say all online."""
from copy import deepcopy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from px4_motor_telemetry_check import telemetry_snapshot, assess_samples


def fixture(stamp=1000000, success=100):
    dshot = (f"Number of successful ESC frames: {success}\n"
             "Number of timeouts: 123\nNumber of CRC errors: 456\n")
    esc = (f"timestamp: {stamp} (0.003 seconds ago)\nesc_count: 4\n"
           "esc_online_flags: 15 (0b1111)\nesc_armed_flags: 15 (0b1111)\n")
    for i in range(4):
        esc += (f"\n  esc[{i}] (esc_report):\n    timestamp: {stamp-i*2500} (0.06 seconds ago)\n"
                f"    actuator_function: {101+i}\n    esc_rpm: 0\n    esc_voltage: 19.5\n"
                "    esc_current: 0.1\n    esc_temperature: 30\n")
    return dshot, esc


def test_advancing_window_ignores_historical_errors_only():
    a = telemetry_snapshot(*fixture())
    b = telemetry_snapshot(*fixture(2000000, 500))
    result = assess_samples([a, b])
    assert result["observed_healthy"]
    assert result["successful_frames_added"] == 400
    assert result["crc_errors_added"] == 0


@pytest.mark.parametrize("replacement", [
    ("0.003 seconds ago", "251.497 seconds ago"),
    ("0.06 seconds ago", "1.0 seconds ago"),
    ("0.06 seconds ago", "-0.01 seconds ago"),
    ("actuator_function: 102", "actuator_function: 103"),
    ("esc_rpm: 0", "esc_rpm: 142"),
    ("esc_voltage: 19.5", "esc_voltage: nan"),
    ("esc_voltage: 19.5", "esc_voltage: 0"),
    ("esc_count: 4", "esc_count: 3"),
    ("esc[3] (esc_report):", "esc[2] (esc_report):"),
])
def test_bad_telemetry_is_not_rehabilitated_by_online_flags(replacement):
    d, e = fixture()
    with pytest.raises(ValueError):
        telemetry_snapshot(d, e.replace(*replacement, 1))


@pytest.mark.parametrize("fault", ["crc", "timeout", "counter_reset", "frozen_frames", "frozen_header", "frozen_slot"])
def test_window_failure(fault):
    a = telemetry_snapshot(*fixture())
    b = telemetry_snapshot(*fixture(2000000, 500))
    if fault == "crc":
        b["counters"]["crc_errors"] += 1
    elif fault == "timeout":
        b["counters"]["timeouts"] += 1
    elif fault == "counter_reset":
        b["counters"]["crc_errors"] = 0
    elif fault == "frozen_frames":
        b["counters"]["success"] = a["counters"]["success"]
    elif fault == "frozen_header":
        b["header"] = deepcopy(a["header"])
    else:
        b["samples"][1] = deepcopy(a["samples"][1])
    with pytest.raises(ValueError):
        assess_samples([a, b])


def test_missing_counter_or_observation_is_unknown():
    d, e = fixture()
    with pytest.raises(ValueError):
        telemetry_snapshot(d.replace("Number of CRC errors: 456", ""), e)
    with pytest.raises(ValueError):
        assess_samples([telemetry_snapshot(d, e)])


@pytest.mark.parametrize("fault", [None, "crc", "frozen_slot"])
def test_real_preflight_never_grants_permit_for_bad_telemetry(monkeypatch, fault):
    import px4_motor_single_pulse as pulse
    c = pulse.PulseConsole("unused", 2)
    status_reads, telemetry_reads, dshot_reads = 0, 0, 0

    def query(command):
        nonlocal status_reads, telemetry_reads, dshot_reads
        if command == "ver all":
            return pulse.FIRMWARE
        if command.startswith("param show -q "):
            return str(pulse.EXPECTED[command.split()[-1]])
        if command == "listener vehicle_status -n 1":
            status_reads += 1
            return f"timestamp: {status_reads*1000000}\narming_state: 1\nnav_state: 4\nhil_state: 0\nfailsafe: False\n"
        if command == "listener actuator_armed -n 1":
            return "\n".join(f"{x}: False" for x in ("armed", "prearmed", "lockdown", "kill", "termination", "in_esc_calibration_mode"))
        if command == "listener actuator_motors -n 1":
            return "reversible_flags: 0"
        if command == "dshot status":
            dshot_reads += 1
            counters, _ = fixture(success=100*dshot_reads)
            if fault == "crc" and dshot_reads > 1:
                counters = counters.replace("errors: 456", "errors: 457")
            return "Outputs initialized: yes\nOutputs used: 0xf\n" + "\n".join(
                f"Channel {i}: func: {101+i if i<4 else 0}, value: 0, failsafe: 0, disarmed: 0, min: 109, max: 1999, center: 65535"
                for i in range(9)) + "\n" + counters
        if command == "listener esc_status -n 1":
            telemetry_reads += 1
            _, esc = fixture(stamp=telemetry_reads*1000000)
            if fault == "frozen_slot" and telemetry_reads == 2:
                esc = esc.replace("timestamp: 1997500", "timestamp: 997500")
            return esc
        raise AssertionError(command)

    monkeypatch.setattr(c, "query", query)
    monkeypatch.setattr(pulse.time, "sleep", lambda _: None)
    if fault:
        with pytest.raises(ValueError):
            c.preflight()
        assert not c._permit
    else:
        result = c.preflight()
        assert result["telemetry"]["observed_healthy"] and c._permit
    assert not c.s.is_open
    assert not c.pulse_attempted
