"""Hardware runner rejects unintended writes and connected-power observations."""
from pathlib import Path
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"scripts"))
import run_nx_offboard_bench as m
from test_px4_motor_single_pulse import dshot


def row():
    t="timestamp: 1234567 (0.01 seconds ago)\n"
    return dict(vehicle_status=t+"arming_state: 1\nhil_state: 0\n",
        actuator_armed=t+"armed: False\nkill: True\n",
        battery_status=t+"connected: False\nvoltage_v: 0\n",
        manual_control_setpoint=t+"valid: True\nthrottle: -1\n",
        manual_control_switches=t+"arm_switch: 3\n",dshot=dshot())


@pytest.mark.parametrize("cmd",["commander arm", "commander mode offboard", "param set COM_OF_LOSS_T 0", "reboot", "actuator_test set -m 1 -v 1"])
def test_no_arm_mode_parameter_or_actuator_command(cmd):
    c=m.BenchConsole("unused")
    with pytest.raises(ValueError):c.query(cmd)
    with pytest.raises(ValueError):c.switch(cmd)
    assert not c.s.is_open


@pytest.mark.parametrize("allow",[False,True])
@pytest.mark.parametrize("topic,before,after",[
    ("battery_status","connected: False","connected: True"),
    ("battery_status","voltage_v: 0","voltage_v: 24.5"),
    ("battery_status","0.01 seconds ago","2 seconds ago"),
    ("manual_control_setpoint","throttle: -1","throttle: 0"),
    ("manual_control_setpoint","valid: True","valid: False"),
    ("vehicle_status","hil_state: 0","hil_state: 1"),
])
def test_no_connected_power_or_bad_input_even_with_operator_arming(allow,topic,before,after):
    good=row();m.power_off_state(good,allow_operator_arming=allow)
    good[topic]=good[topic].replace(before,after)
    with pytest.raises(ValueError):m.power_off_state(good,allow_operator_arming=allow)


def test_operator_arming_must_be_explicit_and_still_power_disconnected():
    r=row();r["vehicle_status"]=r["vehicle_status"].replace("arming_state: 1","arming_state: 2")
    r["actuator_armed"]=r["actuator_armed"].replace("armed: False","armed: True").replace("kill: True","kill: False")
    r["manual_control_switches"]=r["manual_control_switches"].replace("arm_switch: 3","arm_switch: 1")
    r["dshot"]=dshot({0:109,1:109,2:109,3:109})
    with pytest.raises(ValueError):m.power_off_state(r)
    m.power_off_state(r,allow_operator_arming=True)


def test_only_exact_unpublished_battery_is_accepted_for_attested_usb_boot():
    r=row();r["battery_status"]="listener battery_status -n 1\nnever published\nnsh> "
    m.power_off_state(r)
    for bad in ("", "timeout", "TOPIC: battery_status\nnever published"):
        r["battery_status"]=bad
        with pytest.raises(ValueError):m.power_off_state(r)


def test_incident_quarantine_blocks_all_real_protocol_writes():
    c=m.BenchConsole("unused")
    for command in m.SWITCHES:
        with pytest.raises(RuntimeError,match="quarantined"):
            c.switch(command)
        assert not c.s.is_open
    with pytest.raises(RuntimeError,match="Hard Fault"):
        m.require_live_transport_released()


class FakeTransport:
    def __init__(self, *, forced=False, dds_running=False, restore_error=False,
                 fault=False, missing_prompt=False):
        self.calls=[]
        self.protocol_fault=fault
        self.forced=forced
        self.dds_running=dds_running
        self.restore_error=restore_error
        self.missing_prompt=missing_prompt

    def switch(self,command):
        self.calls.append(command)
        if command==m.STOP_DDS and self.forced:
            return "ERROR [uxrce_dds_client] timeout, forcing stop\nnsh> "
        if command==m.STOP_DDS and self.missing_prompt:return ""
        if command==m.RESTORE_USB_MAVLINK and self.restore_error:return "ERROR init failed\nnsh> "
        return command+"\nnsh> "

    def query(self,command):
        self.calls.append(command)
        if command=="uxrce_dds_client status":
            return ("running" if self.dds_running else "not running")+"\nnsh> "
        return "transport protocol: serial (/dev/ttyACM0 @ 2000000)\nnsh> "


def cleanup_fixture(c, *, publisher_stopped=True, cleaned=True, release_raises=False):
    def stop_probe():
        c.calls.append("stop_probe")
        return publisher_stopped
    def release_agent():
        c.calls.append("release_agent")
        if release_raises:raise RuntimeError("Lost Agent cleanup confirmation")
    return m.teardown_usb_session(c,stop_probe=stop_probe,release_agent=release_agent,
                                  nx_cleaned=lambda:cleaned)


def test_teardown_keeps_agent_alive_until_client_stopped():
    c=FakeTransport();r=cleanup_fixture(c)
    assert c.calls==["stop_probe",m.STOP_DDS,"uxrce_dds_client status",
                     "release_agent",m.RESTORE_USB_MAVLINK,"mavlink status"]
    assert r["dds_stop_graceful"] and r["dds_stopped"] and r["usb_mavlink_restored"]
    assert not r["recovery_required"]


@pytest.mark.parametrize("failure",["forced","dds_running","missing_prompt","fault",
                                    "publisher_running","agent_unconfirmed","agent_release_failed"])
def test_uncertain_teardown_never_restarts_usb_mavlink(failure):
    c=FakeTransport(**{failure:True}) if failure in ("forced","dds_running","missing_prompt","fault") else FakeTransport()
    r=cleanup_fixture(c,publisher_stopped=failure!="publisher_running",
        cleaned=failure!="agent_unconfirmed",release_raises=failure=="agent_release_failed")
    assert "release_agent" in c.calls
    assert m.RESTORE_USB_MAVLINK not in c.calls
    assert not r["usb_mavlink_restored"] and r["recovery_required"]
    if failure in ("fault","publisher_running"):assert m.STOP_DDS not in c.calls


def test_restore_error_cannot_be_masked_by_later_instance_presence():
    c=FakeTransport(restore_error=True);r=cleanup_fixture(c)
    assert not r["usb_mavlink_restored"] and r["recovery_required"]
    assert "mavlink status" not in c.calls


def test_real_recorded_force_stop_is_rejected():
    import json
    root=Path(__file__).resolve().parents[2]
    records=json.loads((root/"docs/validation/nx_offboard_bench_20261030/protocol_failure_records.json").read_text())
    assert not m.protocol_reply_ok(records["forced_dds_stop"]["text"])
    assert not m.protocol_reply_ok(records["hardfault_on_mavlink_stop"]["text"])


def test_hardfault_seen_even_in_readonly_query_latches_future_writes():
    c=m.BenchConsole("unused")
    class Serial:
        def write(self,data):pass
    c.s=Serial()
    c.read=lambda duration:b"arm_hardfault: Hard Fault\nnsh> "
    with pytest.raises(RuntimeError,match="Flight-controller fault"):
        c.query("mavlink status")
    assert c.protocol_fault
    with pytest.raises(RuntimeError,match="latched"):
        c.switch(m.RESTORE_USB_MAVLINK)
