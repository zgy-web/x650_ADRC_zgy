"""Read-only scope and invalid/stale data handling of the real RC observer."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import px4_rc_safety_observer as m


@pytest.mark.parametrize("command", ["commander arm", "commander disarm", "commander mode offboard",
                                      "param set COM_RC_OVERRIDE 3", "actuator_test set -m 1 -v 0 -t 1"])
def test_no_mutation_command_can_reach_serial(command):
    c = m.ObserverConsole("unused")
    with pytest.raises(ValueError):
        c.query(command)
    assert not c.s.is_open


@pytest.mark.parametrize("off,seconds", [(False, 30), (True, 0), (True, 91)])
def test_scope_rejected_before_output_or_serial(tmp_path, off, seconds):
    out = tmp_path / "case"
    with pytest.raises(ValueError):
        m.run("unused", out, seconds=seconds, operator_power_off=off)
    assert not out.exists()


def test_invalid_manual_sample_preserves_invalidity_despite_old_throttle():
    text = "timestamp: 100\ndata_source: 1\nvalid: False\nthrottle: -1.00000\n"
    result = m.summarize({"manual_control_setpoint": text})
    assert result["manual_control_setpoint"]["valid"] is False


def test_unknown_is_not_reported_as_safe():
    result = m.summarize({"actuator_armed": "never published", "input_rc": "truncated"})
    assert "unknown" in result["actuator_armed"]
    assert "unknown" in result["input_rc"]
