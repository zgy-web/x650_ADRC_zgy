import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts'))
from check_nx_uart_armed import qualified_transition, post_exit_observations


def sample(nav=14, armed=2, sf=1, sa=6, kill=False, lost=False, failsafe=False, rc_lost=False):
    return dict(stamp=1000000, nav_state=nav, arming_state=armed, kill=kill,
        raw=dict(manual_control_switches=f'arm_switch: {sf}\nmode_slot: {sa}\nkill_switch: {1 if kill else 3}\n',
                 failsafe_flags=f'offboard_control_signal_lost: {lost}\nmanual_control_signal_lost: {rc_lost}\n',
                 actuator_armed=f'armed: {armed == 2}\n',
                 vehicle_status=f'failsafe: {failsafe}\nnav_state_user_intention: 14\n'))


def test_successful_armed_manual_and_automatic_exits():
    assert qualified_transition([sample(), sample(nav=1, sa=1)], 'manual_exit') == (0, 1)
    assert qualified_transition([sample(), sample(lost=True), sample(nav=1, lost=True, failsafe=True)], 'stream_stop') == (0, 2)


@pytest.mark.parametrize('bad', [
    sample(nav=1, armed=1, lost=True, failsafe=True),
    sample(nav=1, sf=3, lost=True, failsafe=True),
    sample(nav=1, sa=1, lost=True, failsafe=True),
    sample(nav=1, kill=True, lost=True, failsafe=True),
    sample(nav=1, lost=False, failsafe=True),
    sample(nav=1, lost=True, failsafe=False),
    sample(nav=1, lost=True, failsafe=True, rc_lost=True),
])
def test_unqualified_automatic_exit_rejected(bad):
    with pytest.raises(ValueError):
        qualified_transition([sample(), bad], 'stream_stop')


def test_unarmed_or_invalid_offboard_does_not_establish_armed_interval():
    for active in (sample(armed=1), sample(lost=True), sample(kill=True)):
        with pytest.raises(ValueError, match='No independent armed'):
            qualified_transition([active, sample(nav=1, lost=True, failsafe=True)], 'stream_stop')


def test_unarmed_intended_mode_is_recorded_without_calling_it_resumed_control():
    rows = [sample(nav=1, lost=True, failsafe=True),
            sample(nav=14, armed=1, sf=3, lost=True), sample(nav=1, armed=1, sf=3, sa=1, kill=True, lost=True)]
    result = post_exit_observations(rows, 0)
    assert len(result) == 1 and result[0]['arming_state'] == 1
    assert result[0]['offboard_signal_lost'] is True


@pytest.mark.parametrize('bad', [sample(), sample(armed=1, sf=3, lost=False), sample(armed=1, sf=1, lost=True)])
def test_true_or_unproven_reentry_remains_rejected(bad):
    with pytest.raises(ValueError, match='re-entry'):
        post_exit_observations([sample(nav=1), bad], 0)


def test_rearming_after_disarm_is_not_a_single_attempt():
    with pytest.raises(ValueError, match='Second arming'):
        post_exit_observations([sample(nav=1), sample(nav=1, armed=1, sf=3), sample(nav=1)], 0)
