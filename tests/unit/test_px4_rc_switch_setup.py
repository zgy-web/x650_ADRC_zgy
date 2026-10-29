"""No serial hardware: verify setup order, command bounds and failure paths."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import px4_rc_switch_setup as setup


def input_text():
    return ("timestamp: 1000000 (0.001 seconds ago)\ntimestamp_last_signal: 999980\n"
            "input_source: 14\nchannel_count: 16\nrc_lost: False\nrc_failsafe: False\n"
            "values: [1502, 1500, 1001, 1499, 1011, 1011, 1988, 1011, 1011, 1011, 1326, 1496, 1000, 1000, 2000, 2000, 0, 0]\n")


@pytest.mark.parametrize("old,new", [
    ("0.001 seconds", "1.001 seconds"), ("999980", "1"),
    ("rc_lost: False", "rc_lost: True"), ("rc_failsafe: False", "rc_failsafe: True"),
    ("input_source: 14", "input_source: 10"), ("channel_count: 16", "channel_count: 4"),
    ("1502, 1500", "2000, 1500"), ("1001, 1499", "2000, 1499"),
    ("1011, 1011, 1988", "1988, 1011, 1988"),
    ("1011, 1011, 1988", "1011, 1503, 1988"),
    ("1011, 1011, 1988", "1011, 1011, 1011"),
    ("values: [", "values: [nan,"),
])
def test_unsafe_or_stale_input_rejected(old, new):
    with pytest.raises(ValueError):
        setup.validate_inputs(input_text().replace(old, new))


def test_fixed_command_permissions():
    for execute in (False, True):
        c = setup.SetupConsole("unused", execute=execute)
        for cmd in ("commander arm", "commander disarm", "reboot", "param set RC_MAP_ARM_SW 9",
                    "actuator_test set -m 1 -v 0 -t 1", "param set RC_MAP_KILL_SW 0",
                    "param set RC_MAP_KILL_SW 7\rcommander arm"):
            with pytest.raises(ValueError):
                c.query(cmd)
        assert not c.s.is_open
    with pytest.raises(ValueError):
        setup.SetupConsole("unused").query("param set RC_MAP_KILL_SW 7")


def test_confirmation_required_before_any_io(tmp_path):
    with pytest.raises(ValueError, match="power-off"):
        setup.apply("unused", tmp_path / "never_created")
    assert not (tmp_path / "never_created").exists()


def zero_outputs():
    return "Outputs initialized: yes\nOutputs used: 0xf\n" + "\n".join(
        f"Channel {i}: func: {101+i if i<4 else 0}, value: 0, failsafe: 0, disarmed: 0, min: 109, max: 1999, center: 65535"
        for i in range(9))


@pytest.mark.parametrize("fault", [None, "wrong_config", "no_kill", "second_write", "battery", "open", "warning_once", "resume_kill"])
def test_fixed_apply_and_partial_failures(monkeypatch, tmp_path, fault):
    instances = []

    class Fake:
        def __init__(self, port, execute):
            assert execute is True
            self.s = SimpleNamespace(is_open=False)
            self.records = []
            self.params = dict(setup.EXPECTED)
            self.warning_seen = False
            if fault == "wrong_config":
                self.params["RC_MAP_ARM_SW"] = 8
            if fault == "resume_kill":
                self.params["RC_MAP_KILL_SW"] = 7
            self.writes = []
            instances.append(self)

        def open(self):
            if fault == "open":
                raise OSError("port unavailable")
            self.s.is_open = True

        def close(self):
            self.s.is_open = False

        def query(self, command):
            assert command in setup.READ_ALLOWED | setup.WRITE_ALLOWED
            self.records.append(dict(command=command))
            kill = self.params["RC_MAP_KILL_SW"] == 7 and fault != "no_kill"
            stamp = "timestamp: 1000000 (0.01 seconds ago)\n"
            if command == "ver all":
                return setup.FIRMWARE
            if command == "listener vehicle_status -n 1":
                return stamp + f"arming_state: 1\nnav_state: {1 if self.params['RC_MAP_FLTMODE'] else 2}\nhil_state: 0\nfailsafe: False\n"
            if command == "dshot status":
                return zero_outputs()
            if command == "listener battery_status -n 1":
                return stamp + f"connected: {fault == 'battery'}\nvoltage_v: 0\n"
            if command == "listener input_rc -n 1":
                return input_text()
            if command == "listener manual_control_setpoint -n 1":
                return stamp + "valid: True\ndata_source: 1\nroll: 0\npitch: 0\nyaw: 0\nthrottle: -1\n"
            if command == "listener manual_control_switches -n 1":
                return stamp + f"arm_switch: {3 if self.params['RC_MAP_ARM_SW'] else 0}\nkill_switch: {1 if kill else 0}\nmode_slot: {1 if self.params['RC_MAP_FLTMODE'] else 0}\n"
            if command == "listener actuator_armed -n 1":
                return stamp + f"armed: False\nprearmed: False\nkill: {kill}\ntermination: False\nin_esc_calibration_mode: False\n"
            if command.startswith("param show -q "):
                if fault == "warning_once" and self.writes and not self.warning_seen:
                    self.warning_seen = True
                    return "WARN [health_and_arming_checks] Preflight Fail: Kill switch engaged\n7\nnsh> "
                return str(self.params[command.split()[-1]]) + "\nnsh> "
            if command.startswith("param set "):
                self.writes.append(command)
                if fault == "second_write" and len(self.writes) == 2:
                    raise OSError("link interrupted")
                _, _, name, value = command.split()
                self.params[name] = int(value)
                return "nsh> "
            if command == "param save":
                return "Parameters saved\nnsh> "
            raise AssertionError(command)

    ticks = iter(range(1000))
    monkeypatch.setattr(setup.time, "monotonic", lambda: next(ticks))
    monkeypatch.setattr(setup.time, "sleep", lambda seconds: None)
    monkeypatch.setattr(setup, "SetupConsole", Fake)
    result = setup.apply("unused", tmp_path / "record", operator_power_off=True, resume=fault == "resume_kill")
    assert result["complete"] is (fault in (None, "warning_once", "resume_kill"))
    assert result["port_closed"]
    assert not instances[0].s.is_open
    assert result["arm_commands"] == result["motor_commands"] == 0
    assert result["rc_takeover_verified"] is result["flight_ready"] is False
    disk = json.loads((tmp_path / "record" / "setup_report.json").read_text())
    assert disk == json.loads(json.dumps(result))
    writes = instances[0].writes
    if fault in (None, "warning_once"):
        assert writes == [f"param set {n} {v}" for n, v in setup.PLAN]
        assert result["after"]["actuator_kill"]
    elif fault == "resume_kill":
        assert writes == [f"param set {n} {v}" for n, v in setup.PLAN[1:]]
        assert result["previously_applied_prefix"] == 1
    elif fault in ("wrong_config", "battery", "open"):
        assert writes == []
    elif fault == "no_kill":
        assert writes == ["param set RC_MAP_KILL_SW 7"]
    elif fault == "second_write":
        assert writes == ["param set RC_MAP_KILL_SW 7", "param set COM_FLTMODE1 1"]
        assert instances[0].params["RC_MAP_KILL_SW"] == 7


def test_duplicate_record_rejected_before_io(monkeypatch, tmp_path):
    monkeypatch.setattr(setup, "SetupConsole", lambda *a, **k: pytest.fail("unexpected hardware object"))
    with pytest.raises(FileExistsError):
        setup.apply("unused", tmp_path, operator_power_off=True)


def test_switch_enum_matches_vendored_px4_message():
    source = (Path(__file__).resolve().parents[2] / "vendor/px4_msgs/msg/ManualControlSwitches.msg").read_text()
    import re
    for name, value in (("ON", setup.SWITCH_ON), ("OFF", setup.SWITCH_OFF)):
        assert int(re.search(r"SWITCH_POS_" + name + r"\s*=\s*(\d+)", source)[1]) == value


def test_resume_rejects_hole_and_conflicting_configuration():
    p = dict(setup.EXPECTED)
    p["RC_MAP_ARM_SW"] = 5  # Arm mapping without the verified kill prefix.
    with pytest.raises(ValueError):
        setup.applied_prefix(p, True)
    p = dict(setup.EXPECTED)
    p["RC_MAP_KILL_SW"] = 7
    with pytest.raises(ValueError):
        setup.applied_prefix(p, False)
    assert setup.applied_prefix(p, True) == 1


def test_ambiguous_read_does_not_turn_into_write(monkeypatch):
    calls = []
    class Fake:
        def query(self, command):
            calls.append(command)
            return "WARN [health_and_arming_checks] busy\n7\nnsh> "
    monkeypatch.setattr(setup.time, "sleep", lambda _: None)
    with pytest.raises(ValueError, match="remains unknown"):
        setup.read_parameter(Fake(), "RC_MAP_KILL_SW")
    assert calls == ["param show -q RC_MAP_KILL_SW"] * 3
