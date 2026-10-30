import copy,importlib.util,json,sys
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from nx_mavlink_bandwidth import calculate
from prepare_px4_uart_profile import ProfileConsole,PARAMS
from observe_px4_uart_bench import ReadbackConsole


@pytest.fixture
def profile():return json.loads((ROOT/'configs/hardware/nx_mavlink_uart_v1.json').read_text())


def test_full_duplex_accounting(profile):
    r=calculate(profile,'mpc')
    assert r['capacity_bytes_s_per_direction']==92160
    assert r['directions']['px4_to_nx']['periodic_bytes_s']==22681
    assert r['directions']['nx_to_px4']['periodic_bytes_s']==3571
    assert r['budget_fits'] and not r['measured']


def test_existing_profile_cannot_sustain_new_streams(profile):
    profile['baud']=57600;profile['px4_tx_budget_bytes_s']=1200
    assert not calculate(profile,'bench')['budget_fits']
    assert not calculate(profile,'mpc')['budget_fits']


def test_signature_budget_not_payload_truncation(profile):
    unsigned=calculate(profile,'mpc');signed=calculate(profile,'mpc',True)
    assert signed['budget_fits']
    for d in unsigned['directions']:
        count=sum(m['hz'] for m in unsigned['directions'][d]['messages'])
        assert signed['directions'][d]['periodic_bytes_s']-unsigned['directions'][d]['periodic_bytes_s']==13*count


def test_duplicate_message_rejected(profile):
    profile['messages'].append(copy.deepcopy(profile['messages'][0]))
    with pytest.raises(ValueError):calculate(profile,'bench')


def test_no_parameter_write_in_readonly_mode():
    c=ProfileConsole('COM13',False)
    with pytest.raises(ValueError):c.command('param set SER_TEL1_BAUD 921600')
    assert not c.s.is_open


@pytest.mark.parametrize('command',['commander arm','reboot','mavlink stop -d /dev/ttyACM0',
                                   'param set COM_ARM_WO_GPS 1','param set MAV_0_RATE 999999'])
def test_write_whitelist_has_no_control_or_usb_mutation(command):
    c=ProfileConsole('COM13',True)
    with pytest.raises(ValueError):c.command(command)


def test_prepared_parameters_match_reviewed_profile(profile):
    assert profile['px4_parameters_proposed']==PARAMS
    assert profile['control']['per_setpoint_ack_required'] is False
    assert len(profile['full_mpc_state_gaps'])==4


def test_unknown_stage_rejected(profile):
    with pytest.raises(ValueError):calculate(profile,'flight')


@pytest.mark.parametrize('command',['commander arm','reboot','mavlink stop -d /dev/ttyACM0',
                                   'param set SER_TEL1_BAUD 921600','motor_test start'])
def test_independent_observer_cannot_mutate_flight_controller(command):
    c=ReadbackConsole('COM13')
    with pytest.raises(ValueError):c.query(command)
    assert not c.s.is_open
