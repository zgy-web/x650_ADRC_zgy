import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import px4_imu_pole_check as m


BIAS = """timestamp: 1000000 (0.01 seconds ago)
accel_device_id: 2818066
gyro_device_id: 2818066
accel_bias: [-0.39951, 0.4, -0.08274]
accel_bias_variance: [0.00029, 0.0003, 0.00001]
accel_bias_limit: 0.4
accel_bias_valid: True
accel_bias_stable: True
gyro_bias: [-0.00201, -0.0002, 0.00272]
gyro_bias_variance: [0, 0, 0]
gyro_bias_limit: 0.15
gyro_bias_valid: True
gyro_bias_stable: True
"""


def test_real_snapshot_warns_on_xy_and_not_gyro():
    r = m.assess_bias(BIAS)
    assert r["accel"]["exceeds"] == [True, True, False]
    assert r["accel"]["thresholds"][0] == pytest.approx(.3510881591)
    assert r["gyro"]["exceeds"] == [False, False, False]


def test_uncertain_or_invalid_bias_is_not_declared_a_warning():
    high_variance = BIAS.replace("0.00029, 0.0003", "0.04, 0.04")
    assert m.assess_bias(high_variance)["accel"]["exceeds"] == [False] * 3
    invalid = BIAS.replace("accel_bias_valid: True", "accel_bias_valid: False")
    assert m.assess_bias(invalid)["accel"]["exceeds"] is None


@pytest.mark.parametrize("replacement", ["2.1", "nan", "-1"])
def test_stale_or_invalid_age_is_unknown(replacement):
    with pytest.raises(ValueError):
        m.assess_bias(BIAS.replace("0.01 seconds", replacement + " seconds"))


@pytest.mark.parametrize("command", [
    "commander arm", "commander calibrate accel", "param set EKF2_ABL_LIM 1",
    "param set MOT_POLE_COUNT 14", "actuator_test set -m 1 -v 0 -t 1",
    "param set MOT_POLE_COUNT 24\rcommander arm", "reboot",
])
def test_unrelated_mutations_rejected_even_with_pole_permission(command):
    c = m.ImuConsole("COM13", allow_pole_write=True)
    with pytest.raises(ValueError):
        c.query(command)
    assert not c.s.is_open


@pytest.mark.parametrize("command", sorted(m.WRITES))
def test_readonly_cannot_write(command):
    c = m.ImuConsole("COM13")
    with pytest.raises(ValueError):
        c.query(command)
    assert not c.s.is_open


def test_only_two_fixed_writes_are_exposed():
    assert m.WRITES == {"param set MOT_POLE_COUNT 24", "param save"}
    assert not m.READS.intersection(m.WRITES)


def test_no_physical_attestation_no_serial_or_output(tmp_path):
    with pytest.raises(ValueError):
        m.run("COM13", tmp_path / "not_created", apply_poles=True)
    assert not (tmp_path / "not_created").exists()


@pytest.mark.parametrize("case", ["wrong_firmware", "powered", "wrong_poles", "failed_readback"])
def test_failed_guards_do_not_save_or_retry_write(monkeypatch, tmp_path, case):
    calls = []

    class FakeConsole:
        def __init__(self, *a, **kw):
            self.records = []
            self.s = type("Serial", (), {"is_open": False})()
        def open(self):
            self.s.is_open = True
        def close(self):
            self.s.is_open = False
        def query(self, cmd):
            calls.append(cmd)
            if cmd == "ver all":
                return "unknown" if case == "wrong_firmware" else m.FIRMWARE
            return "nsh> "

    def fake_guard(*a, **kw):
        if case == "powered":
            raise ValueError("power on")
        return {"state": {"timestamp": 1000000}}

    def fake_param(c, p):
        if p == "MOT_POLE_COUNT":
            return 12 if case == "wrong_poles" else 14
        return 0

    monkeypatch.setattr(m, "ImuConsole", FakeConsole)
    monkeypatch.setattr(m, "guard", fake_guard)
    monkeypatch.setattr(m, "read_parameter", fake_param)
    r = m.run("unused", tmp_path / case, operator_power_off=True, apply_poles=True, rounds=1)
    assert not r["complete"] and r["port_closed"]
    assert "param save" not in calls
    assert calls.count("param set MOT_POLE_COUNT 24") == (1 if case == "failed_readback" else 0)
    assert json.loads((tmp_path / case / "report.json").read_text())["error"]
