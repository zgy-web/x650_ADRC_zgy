import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import px4_rc_battery_metadata as m
from test_px4_rc_powered_stop_observer import parked_row

def test_parked_state_and_unknown_not_safe():
    row = parked_row()
    row["raw"]["vehicle_status"] = row["raw"]["vehicle_status"].replace("timestamp: 1000000", "timestamp: 1000000 (0.01 seconds ago)")
    m.validate_parked(row)
    row["raw"]["actuator_armed"] = "never published"
    with pytest.raises(ValueError):m.validate_parked(row)

@pytest.mark.parametrize("command", ["param set BAT1_N_CELLS 4", "param set CBRK_SUPPLY_CHK 894281", "commander arm", "param save"])
def test_fixed_write_scope(command):
    c = m.MetadataConsole("unused")
    with pytest.raises(ValueError):c.query(command)
    assert not c.s.is_open

@pytest.mark.parametrize("replacement", [("cell_count: 6", "cell_count: 0"), ("remaining: 0.8", "remaining: -1"),
    ("warning: 0", "warning: 3"), ("voltage_v: 24.5", "voltage_v: 19.5"), ("0.01 seconds", "5 seconds")])
def test_requires_real_recovered_battery(replacement):
    text = "timestamp: 1000000 (0.01 seconds ago)\nconnected: True\ncell_count: 6\nremaining: 0.8\nwarning: 0\nvoltage_v: 24.5\n"
    m.validate_battery(text)
    with pytest.raises(ValueError):m.validate_battery(text.replace(*replacement))
