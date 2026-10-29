"""Fixed read scope, partial evidence and cleanup; no physical serial access."""
from pathlib import Path
import json
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import px4_rc_preflight_readonly as rc


@pytest.mark.parametrize("command", [
    "commander arm", "commander disarm", "commander mode manual",
    "param set RC_CRSF_PRT_CFG 102", "param set MOT_POLE_COUNT 24",
    "param save", "crsf_rc start -d /dev/ttyS4", "mavlink stop-all",
    "actuator_test set -m 1 -v 0 -t 1", "dshot reverse -m 2", "reboot",
    "param show -q COM_RC_OVERRIDE; commander arm",
    "listener input_rc -n 1\rcommander arm", "listener input_rc -n 10000",
])
def test_only_fixed_reads_allowed(command):
    console = rc.RcReadConsole("unused")
    with pytest.raises(ValueError):
        console.query(command)
    assert not console.s.is_open


def test_query_rejects_active_listener_and_missing_prompt(monkeypatch):
    console = rc.RcReadConsole("unused")
    written = []
    monkeypatch.setattr(console.s, "write", written.append)
    monkeypatch.setattr(console, "read", lambda seconds: b"incomplete")
    console.active_listener = True
    with pytest.raises(ValueError):
        console.query("crsf_rc status")
    assert written == []
    console.active_listener = False
    with pytest.raises(RuntimeError, match="No console prompt"):
        console.query("crsf_rc status")
    assert written == [b"crsf_rc status\r"]


def zero_outputs(nonzero=False):
    return "Outputs initialized: yes\nOutputs used: 0xf\n" + "\n".join(
        f"Channel {i}: func: {101+i if i<4 else 0}, value: {109 if nonzero and i==0 else 0}, "
        "failsafe: 0, disarmed: 0, min: 109, max: 1999, center: 65535"
        for i in range(9))


@pytest.mark.parametrize("fault", [None, "open", "firmware", "armed", "nonzero", "stalled", "midread"])
def test_inventory_is_observation_not_acceptance(monkeypatch, tmp_path, fault):
    instances = []

    class Fake:
        def __init__(self, port):
            self.s = SimpleNamespace(is_open=False)
            self.records = []
            self.status_count = 0
            instances.append(self)

        def open(self):
            if fault == "open":
                raise OSError("COM port unavailable")
            self.s.is_open = True

        def close(self):
            self.s.is_open = False

        def query(self, command):
            assert command in rc.ALLOWED
            self.records.append({"command": command})
            if command == "ver all":
                return "unexpected" if fault == "firmware" else rc.FIRMWARE
            if command == "listener vehicle_status -n 1":
                self.status_count += 1
                stamp = 1000000 if fault == "stalled" else 1000000 * self.status_count
                return (f"timestamp: {stamp}\narming_state: {2 if fault == 'armed' else 1}\n"
                        "nav_state: 4\nhil_state: 0\nfailsafe: False\n")
            if command == "dshot status":
                return zero_outputs(fault == "nonzero")
            if command == "listener input_rc -n 1":
                if fault == "midread":
                    raise OSError("lost serial connection")
                return "timestamp_last_signal: 0\nrc_lost: True\nrc_total_frame_count: 0\n"
            if command == "listener manual_control_setpoint -n 1":
                return "timestamp: 0\nvalid: False\n"
            if command == "param show -q RC_MAP_KILL_SW":
                return "0\nnsh> "
            if command.startswith("param show -q "):
                return "ERROR [param] Parameter not found\nnsh> "
            return "never published\nnsh> "

    monkeypatch.setattr(rc, "RcReadConsole", Fake)
    result = rc.collect("unused", tmp_path / "new")
    assert result["read_complete"] is (fault is None)
    assert result["port_closed"]
    assert not instances[0].s.is_open
    assert json.loads((tmp_path / "new" / "inventory.json").read_text()) == json.loads(json.dumps(result))
    assert len(result["console_sha256"]) == 64
    assert result["motor_commands"] == result["parameter_writes"] == 0
    assert result["arm_commands"] == result["mode_commands"] == 0
    assert not result["nx_accessed"]
    for key in ("rc_takeover_verified", "armed_failsafe_verified", "physical_stop_verified", "flight_ready"):
        assert result[key] is False
    if fault is None:
        # Successful inventory with invalid RC is not a healthy RC result.
        assert "rc_lost: True" in result["status_text"]["listener input_rc -n 1"]
        assert result["parameters"]["RC_MAP_KILL_SW"] == 0
        assert result["parameters"]["RC_CRSF_PRT_CFG"] is None
        assert "RC_CRSF_PRT_CFG" in result["unknown_parameters"]
    if fault in ("firmware", "armed", "nonzero"):
        assert not result["parameters"]


def test_existing_record_is_never_overwritten(monkeypatch, tmp_path):
    def forbidden(port):
        pytest.fail("must not construct hardware console")
    monkeypatch.setattr(rc, "RcReadConsole", forbidden)
    with pytest.raises(FileExistsError):
        rc.collect("unused", tmp_path)
