import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from check_nx_uart_manual_readback import check_printed_quaternion


def test_actual_nuttx_truncated_float32_display_is_not_a_control_mismatch():
    q = [0.7848355174064636, 0.025781013071537018, -0.01460971962660551, -0.6189952492713928]
    assert check_printed_quaternion(['0.78483', '0.02578', '-0.01460', '-0.61899'], q) == ['truncated_5dp']*4


@pytest.mark.parametrize('tokens', [
    ['0.78483', '0.02578', '-0.01459', '-0.61899'],  # A wrong target is not accepted by a broad tolerance.
    ['0.78483', '-0.01460', '0.02578', '-0.61899'],  # Axis swap.
    ['0.7848', '0.02578', '-0.01460', '-0.61899'],   # Less evidence precision.
    ['0.78483', '0.02578', 'nan', '-0.61899'],
])
def test_rejects_wrong_or_insufficient_readback(tokens):
    with pytest.raises(ValueError):
        check_printed_quaternion(tokens, [0.7848355174064636, 0.025781013071537018, -0.01460971962660551, -0.6189952492713928])
