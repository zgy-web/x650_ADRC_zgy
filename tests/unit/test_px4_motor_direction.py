"""Direction commands never omit the channel and require a fresh stopped check."""
from pathlib import Path
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from px4_motor_direction import DirectionConsole, direction_command


@pytest.mark.parametrize("motor, action", [(0, "reverse"), (5, "save"), (True, "reverse"),
                                         (1, "3d_on"), (1, "reverse; commander arm")])
def test_reject_invalid_scope(motor, action):
    with pytest.raises(ValueError):
        direction_command(motor, action)


def test_channel_is_always_explicit():
    for motor in range(1, 5):
        for action in ("reverse", "normal", "save"):
            assert direction_command(motor, action) == f"dshot {action} -m {motor}"


def test_requires_fresh_permit_before_write():
    console = DirectionConsole("unused", 1)
    with pytest.raises(ValueError):
        console.configure("reverse")
    console._permit = True
    console._permit_until = time.monotonic() - 1
    with pytest.raises(ValueError):
        console.configure("save")
    assert not console.s.is_open
