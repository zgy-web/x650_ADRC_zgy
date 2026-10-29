"""Bounded actuator scope, live-data checks and cleanup, without physical IO."""
from pathlib import Path
import sys
from types import SimpleNamespace
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import px4_motor_single_pulse as pulse


def dshot(values=None):
    values = values or {}
    lines = ["Outputs initialized: yes", "Outputs used: 0xf"]
    for i in range(9):
        lines.append(f"Channel {i}: func: {101+i if i<4 else 0}, value: {values.get(i, 0)}, "
                     "failsafe: 0, disarmed: 0, min: 109, max: 1999, center: 65535")
    return "\n".join(lines)


ARMED = "\n".join(f"{key}: False" for key in
                  ("armed", "prearmed", "lockdown", "kill", "termination", "in_esc_calibration_mode"))
STATUS = "timestamp: 1000000\narming_state: 1\nnav_state: 4\nhil_state: 0\nfailsafe: False\n"


@pytest.mark.parametrize("motor", [0, 5, True, "1; commander arm"])
def test_motor_rejected(motor):
    with pytest.raises(ValueError):
        pulse.plan(motor)


@pytest.mark.parametrize("change", [{"PWM_AUX_REV": 1}, {"CA_R_REV": 1}, {"DSHOT_MIN": .1},
                                    {"PWM_AUX_FUNC2": 103, "PWM_AUX_FUNC3": 102},
                                    {"PWM_MAIN_FUNC1": 101}, {"DSHOT_3D_ENABLE": 1}])
def test_changed_configuration_rejected(change):
    with pytest.raises(ValueError):
        pulse.check_parameters({**pulse.EXPECTED, **change})


def test_live_feedback_checks():
    pulse.check_parameters(pulse.EXPECTED)
    assert pulse.require_zero(dshot()) == dict.fromkeys(range(9), 0)
    assert pulse.dshot_channels(dshot({0: 109}))[0] == 109
    with pytest.raises(ValueError):
        pulse.require_zero(dshot({0: 109}))
    with pytest.raises(ValueError):
        pulse.dshot_channels(dshot().replace("min: 109", "min: 500", 1))
    with pytest.raises(ValueError):
        pulse.dshot_channels(dshot() + "\n" + dshot().splitlines()[2])
    with pytest.raises(ValueError):
        pulse.integer_field("never published", "reversible_flags")
    pulse.check_armed(ARMED)
    with pytest.raises(ValueError):
        pulse.check_armed(ARMED.replace("armed: False", "armed: True", 1))


def test_no_pulse_without_fresh_permit_or_second_pulse(monkeypatch):
    console = pulse.PulseConsole("unused", 1)
    sent = []
    monkeypatch.setattr(console, "_command", sent.append)
    with pytest.raises(ValueError):
        console.send_pulse()
    console._permit = True
    console._permit_until = time.monotonic() - 1
    with pytest.raises(ValueError):
        console.send_pulse()
    console._permit_until = time.monotonic() + 1
    console.send_pulse()
    with pytest.raises(ValueError):
        console.send_pulse()
    console.stop()
    console.stop()
    assert sent == ["actuator_test set -m 1 -v 0 -t 1", "actuator_test set -m 1 -v -1 -t 0.1"]
    assert not console.s.is_open


@pytest.mark.parametrize("fault", [None, "wrong_motor", "no_output", "pulse_write_failed", "post_nonzero", "preflight_failed"])
def test_run_cleanup_and_evidence(tmp_path, fault):
    class Fake:
        def __init__(self, port, motor):
            self.s = SimpleNamespace(is_open=False)
            self.records = []
            self.pulse_attempted = self.stop_attempted = False
            self.dshot_count = 0

        def open(self):
            self.s.is_open = True

        def preflight(self):
            if fault == "preflight_failed":
                raise ValueError("changed parameters")
            return {"synthetic": True}

        def send_pulse(self):
            self.pulse_attempted = True
            if fault == "pulse_write_failed":
                raise OSError("unknown delivery")

        def stop(self):
            self.stop_attempted = True

        def close(self):
            self.s.is_open = False

        def query(self, command):
            if command == "dshot status":
                self.dshot_count += 1
                if self.stop_attempted:
                    return dshot()
                if fault == "post_nonzero":
                    return dshot({0: 109})
                if self.dshot_count == 1:
                    return dshot({1: 109} if fault == "wrong_motor" else {} if fault == "no_output" else {0: 109})
                return dshot()
            if command == "listener actuator_armed -n 1":
                return ARMED
            if command == "listener vehicle_status -n 1":
                return STATUS
            if command == "listener esc_status -n 1":
                return "never published"
            if command == "listener actuator_test -n 1":
                return "timestamp: 1000000\nfunction: 101\nvalue: 0\naction: 1\ntimeout_ms: 1000\n"
            raise AssertionError(command)

    result = pulse.run("unused", tmp_path / "new", 1,
                       dict.fromkeys(pulse.ATTESTATIONS, True), console_factory=Fake, sleep=lambda _: None)
    assert result["console_check_passed"] is (fault is None)
    assert result["stop_attempted"] is (fault != "preflight_failed")
    assert result["pulse_attempted"] is (fault != "preflight_failed")
    assert result["port_closed"]
    assert not result["physical_motor_confirmed"]
    assert not result["rotation_confirmed"]
    assert not result["flight_ready"]


def test_missing_physical_attestation_does_not_open_port(tmp_path):
    def forbidden(*args):
        pytest.fail("must not open hardware")
    with pytest.raises(ValueError):
        pulse.run("unused", tmp_path / "new", 1, {}, console_factory=forbidden)
    assert not (tmp_path / "new").exists()


def test_startup_telemetry_uses_source_time_and_keeps_stale_samples_visible():
    command = "timestamp: 1000000\nfunction: 102\nvalue: 0\naction: 1\ntimeout_ms: 1000\n"
    snapshots = "esc_status\n" + "".join(
        f"\n  esc[{i}] (esc_report):\n    timestamp: {stamp} (0.002 seconds ago)\n"
        f"    actuator_function: {101+i}\n    esc_rpm: {rpm}\n    esc_voltage: 19.4\n"
        "    esc_current: 1.2\n    esc_temperature: 30\n"
        for i, stamp, rpm in ((0, 990000, 0), (1, 1300000, 400), (2, 2100000, 0), (3, 1400000, 0)))
    r = pulse.pulse_telemetry(command, snapshots, 2)
    assert r["available"]
    assert not r["samples"][0]["inside_requested_pulse_window"]
    assert r["samples"][1]["inside_requested_pulse_window"]
    assert r["samples"][1]["relative_to_pulse_start_ms"] == 300
    assert r["samples"][1]["esc_rpm"] == 400
    assert not r["samples"][2]["inside_requested_pulse_window"]
    assert not pulse.pulse_telemetry(command.replace("function: 102", "function: 103"), snapshots, 2)["available"]
    assert not pulse.pulse_telemetry(command, "never published", 2)["available"]
    assert not pulse.pulse_telemetry(command, snapshots.replace("esc_voltage: 19.4", "esc_voltage: nan", 1), 2)["available"]
