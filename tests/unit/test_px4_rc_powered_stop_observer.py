"""Read-only scope, startup interlocks and moving/stale telemetry separation."""
from copy import deepcopy
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import px4_rc_powered_stop_observer as m
from px4_motor_telemetry_check import telemetry_snapshot
from test_px4_motor_telemetry_check import fixture
from test_px4_motor_single_pulse import dshot, ARMED, STATUS


@pytest.mark.parametrize("command", ["commander arm", "commander disarm", "param set COM_RC_OVERRIDE 3",
                                     "actuator_test set -m 1 -v 0 -t 1", "uxrce_dds_client stop"])
def test_readonly_whitelist(command):
    c = m.PoweredReadConsole("unused")
    with pytest.raises(ValueError):
        c.query(command)
    assert not c.s.is_open


def parked_row():
    counters, esc = fixture()
    stamp = "timestamp: 1000000 (0.01 seconds ago)\n"
    return dict(raw={
        "vehicle_status": STATUS,
        "actuator_armed": stamp + ARMED.replace("kill: False", "kill: True"),
        "input_rc": stamp + "timestamp_last_signal: 999900\nrc_lost: False\nrc_failsafe: False\n",
        "manual_control_switches": stamp + "arm_switch: 3\nkill_switch: 1\nmode_slot: 1\n",
        "manual_control_setpoint": stamp + "data_source: 1\nvalid: True\nroll: 0\npitch: 0\nyaw: 0\nthrottle: -1\n",
        "dshot": dshot() + "\n" + counters, "esc_status": esc})


@pytest.mark.parametrize("topic,before,after", [
    ("actuator_armed", "kill: True", "kill: False"),
    ("actuator_armed", "armed: False", "armed: True"),
    ("input_rc", "rc_lost: False", "rc_lost: True"),
    ("input_rc", "timestamp_last_signal: 999900", "timestamp_last_signal: 0"),
    ("manual_control_switches", "arm_switch: 3", "arm_switch: 1"),
    ("manual_control_setpoint", "throttle: -1", "throttle: 0"),
    ("manual_control_setpoint", "roll: 0", "roll: 1"),
    ("manual_control_setpoint", "0.01 seconds ago", "5 seconds ago"),
    ("esc_status", "esc_rpm: 0", "esc_rpm: 800"),
    ("esc_status", "esc_voltage: 19.5", "esc_voltage: 12"),
    ("esc_status", "0.06 seconds ago", "2 seconds ago"),
])
def test_start_interlocks(topic, before, after):
    row = parked_row()
    assert m.parked(row)["samples"][0]["esc_rpm"] == 0
    bad = deepcopy(row)
    bad["raw"][topic] = bad["raw"][topic].replace(before, after, 1)
    with pytest.raises(ValueError):
        m.parked(bad)


def test_motion_opt_in_does_not_relax_freshness_or_accept_negative_rpm():
    d, e = fixture()
    moving = e.replace("esc_rpm: 0", "esc_rpm: 800")
    with pytest.raises(ValueError):
        telemetry_snapshot(d, moving)
    assert telemetry_snapshot(d, moving, require_stopped=False)["samples"][0]["esc_rpm"] == 800
    for bad in (moving.replace("0.06 seconds ago", "5 seconds ago"), moving.replace("esc_rpm: 800", "esc_rpm: -800")):
        with pytest.raises(ValueError):
            telemetry_snapshot(d, bad, require_stopped=False)


def test_attestations_before_io(tmp_path):
    with pytest.raises(ValueError):
        m.run("unused", tmp_path / "case", seconds=60, attestations={})
    assert not (tmp_path / "case").exists()


def test_battery_profile_requires_configured_noncritical_estimate():
    row = parked_row()
    row["raw"]["esc_status"] = row["raw"]["esc_status"].replace("19.5", "24.4")
    row["raw"]["battery_status"] = "timestamp: 1000000 (0.01 seconds ago)\nconnected: True\ncell_count: 6\nremaining: 0.8\nwarning: 0\nvoltage_v: 24.5\n"
    m.parked(row, "6s_battery")
    row["raw"]["battery_status"] = row["raw"]["battery_status"].replace("warning: 0", "warning: 3")
    with pytest.raises(ValueError):m.parked(row, "6s_battery")
