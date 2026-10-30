"""Offline acceptance boundaries for independent-power evidence."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
import px4_power_readonly as m


def data(usb=0, brick=1, volts=5.05, overcurrent=0):
    t='timestamp: 100000000 (0.01 seconds ago)\n'
    return dict(system_power=t+f'voltage5v_v: {volts}\nbrick_valid: {brick}\n'
                f'usb_connected: {usb}\nusb_valid: {usb}\nperiph_5v_oc: {overcurrent}\nhipower_5v_oc: 0\n',
                manual_control_setpoint=t+'valid: True\nthrottle: -1\nroll: 0\npitch: 0\nyaw: 0\n',
                manual_control_switches=t+'arm_switch: 3\nmode_slot: 1\nkill_switch: 1\n',
                battery_status='never published\n')


def test_usb_removed_does_not_prove_battery_monitor_calibrated():
    r=m.assessment(data(),'power_only',{'kill':True},(1000,1000.1))
    assert r['power_ok'] and not r['battery_monitor_usable']
    assert r['battery']['voltage_v'] is None


@pytest.mark.parametrize('changed',[
    {'usb':1}, {'brick':0}, {'brick':4}, {'volts':4.6}, {'volts':5.5}, {'overcurrent':1}])
def test_reject_power_claim_when_usb_remains_or_supply_is_invalid(changed):
    r=m.assessment(data(**changed),'power_only',{'kill':True},(1000,1000.1))
    assert not r['power_ok']


@pytest.mark.parametrize('topic,key,value',[
    ('manual_control_setpoint','throttle: -1','throttle: 0'),
    ('manual_control_setpoint','valid: True','valid: False'),
    ('manual_control_switches','arm_switch: 3','arm_switch: 1'),
    ('system_power','0.01 seconds ago','2 seconds ago'),
    ('system_power','voltage5v_v: 5.05','voltage5v_v: nan')])
def test_reject_nonparked_or_untrusted_readback(topic,key,value):
    raw=data();raw[topic]=raw[topic].replace(key,value)
    with pytest.raises(ValueError):m.assessment(raw,'power_only',{'kill':True},(1000,1000.1))


def test_reject_kill_request_without_actual_kill():
    with pytest.raises(ValueError):m.assessment(data(),'combined',{'kill':False},(1000,1000.1))


def test_uptime_increasing_alone_cannot_hide_a_reboot_between_samples():
    # First: FC uptime 100 s. Later: 150 s, but 200 s elapsed on the PC.
    # Simple increasing-timestamp comparisons would incorrectly accept this.
    a=m.clock_interval(data()['system_power'],1000,1000.1)
    raw=data()['system_power'].replace('100000000','150000000')
    b=m.clock_interval(raw,1200,1200.1)
    assert not m.consistent_clock(a,b)
    continuous=raw.replace('150000000','300000000')
    assert m.consistent_clock(a,m.clock_interval(continuous,1200,1200.1))


def test_bad_baseline_and_short_window_cannot_touch_serial(tmp_path,monkeypatch):
    monkeypatch.setattr(m,'FirstFlightConsole',lambda port:pytest.fail('must not open hardware'))
    path=tmp_path/'bad.json';path.write_text(json.dumps({'phase':'combined','complete':False}))
    with pytest.raises(ValueError):m.collect('unused',tmp_path/'a','power_only',60,path)
    with pytest.raises(ValueError):m.collect('unused',tmp_path/'b','power_only',10,path)
    assert not (tmp_path/'a').exists() and not (tmp_path/'b').exists()


def test_serial_failure_preserves_partial_evidence_and_closes_port(tmp_path,monkeypatch):
    class Fake:
        def __init__(self,port):self.s=SimpleNamespace(is_open=False);self.records=[]
        def open(self):self.s.is_open=True;raise RuntimeError('no reply')
        def close(self):self.s.is_open=False
    monkeypatch.setattr(m,'FirstFlightConsole',Fake)
    r=m.collect('unused',tmp_path/'new','combined',10)
    assert not r['complete'] and not r['window_verified'] and not r['independent_power_verified'] and r['port_closed']
    assert json.loads((tmp_path/'new/report.json').read_text())['error']=='no reply'


def test_combined_window_alone_is_not_independent_power_proof(tmp_path,monkeypatch):
    clock={'t':1000.,'usb':1}
    def stamp():return int((clock['t']-900)*1e6)
    class Fake:
        def __init__(self,port):self.s=SimpleNamespace(is_open=False);self.records=[]
        def open(self):self.s.is_open=True
        def close(self):self.s.is_open=False
        def query(self,cmd):
            clock['t']+=.02
            if cmd=='ver all':return m.FIRMWARE
            topic=cmd.split()[1]
            return data(usb=clock['usb'])[topic].replace('100000000',str(stamp()))
    def parked(c):
        clock['t']+=.02
        return {'kill':True,'state':{'timestamp':stamp()},'outputs':{'0':0}}
    def sleep(seconds):clock['t']+=seconds
    monkeypatch.setattr(m,'FirstFlightConsole',Fake)
    monkeypatch.setattr(m,'parked',parked)
    monkeypatch.setattr(m.time,'monotonic',lambda:clock['t'])
    monkeypatch.setattr(m.time,'sleep',sleep)
    a=m.collect('unused',tmp_path/'a','combined',10)
    assert a['window_verified'] and a['complete'] and not a['independent_power_verified']
    clock['t']+=5;clock['usb']=0  # operator disconnect, same FC boot
    b=m.collect('unused',tmp_path/'b','power_only',60,tmp_path/'a/report.json')
    assert b['independent_power_verified'] and b['observed_duration_s']>=60
    assert b['port_closed'] and not b['flight_ready']
    assert not b['battery_monitor_usable_all_samples']  # no invented battery reading
