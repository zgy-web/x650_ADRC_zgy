"""Fail-closed serial scope and missing-parameter handling; no hardware access."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import px4_motor_preflight_readonly as motor


@pytest.mark.parametrize("response, expected", [
    ("param show -q PWM_AUX_FUNC1\r\n101\r\nnsh> ", 101),
    ("\x1b[0m-0.050000\r\nnsh> ", -.05),
    ("0\r\nnsh> ", 0),
    ("ERROR [param] Parameter not found\nnsh> ", None),
    ("param show -q PWM_AUX_FUNC1\nnsh> ", None),
    ("101\n102\nnsh> ", None),
    ("nan\nnsh> ", None),
])
def test_parameter_response(response, expected):
    assert motor.parse_parameter(response) == expected


@pytest.mark.parametrize("command", [
    "param set PWM_AUX_TIM0 -3", "commander arm", "actuator_test set -m 1 -v .1",
    "dshot reverse -m 1", "reboot", "param show -q SYS_AUTOSTART; commander arm",
])
def test_mutations_rejected_before_serial_write(command):
    console = motor.RotorReadConsole("unused")
    with pytest.raises(ValueError):
        console.query(command)
    assert not console.s.is_open


def test_missing_assignment_is_not_disabled_assignment():
    assignments = motor.output_assignments({"PWM_MAIN_FUNC1": 0, "PWM_AUX_FUNC1": 101})
    assert assignments["MAIN1"] == 0
    assert assignments["AUX1"] == 101
    assert assignments["AUX2"] is None
