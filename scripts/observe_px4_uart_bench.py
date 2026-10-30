"""Independent, bounded COM13 readback for the UART bench; PC files only.

No USB switching, parameter writes, arming, mode or actuator commands exist.
A completed capture is not a passed Offboard/physical acceptance test.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import re
import time

from px4_debug_console import DebugConsole,FIRMWARE,clean
from px4_imu_pole_check import guard
from px4_rc_switch_setup import boolean,number,fresh
from px4_motor_single_pulse import dshot_channels

TOPICS=('vehicle_status','vehicle_control_mode','offboard_control_mode',
        'vehicle_attitude_setpoint','actuator_armed','manual_control_switches',
        'manual_control_setpoint','failsafe_flags')
READS=frozenset(('ver all','mavlink status','dshot status','commander status')+
               tuple('listener '+x+' -n 1' for x in TOPICS+('battery_status','health_report','trajectory_setpoint')))


class ReadbackConsole(DebugConsole):
    def query(self,command):
        if command not in READS or self.active_listener:raise ValueError('Fixed read commands only')
        self.s.write((command+'\r').encode('ascii'))
        text=clean(self.record(command,self.read(6)))
        if 'nsh>' not in text or re.search(r'Hard Fault|PANIC|Assertion failed',text,re.I):
            raise RuntimeError('Flight-controller readback interrupted or faulted')
        return text


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port',default='COM13')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=80)
    p.add_argument('--power-disconnected',action='store_true',required=True)
    p.add_argument('--allow-operator-arming',action='store_true')
    a=p.parse_args()
    if not 1<=a.seconds<=90:raise ValueError('1..90 second window required')
    a.output.mkdir(parents=True,exist_ok=False);c=ReadbackConsole(a.port)
    result=dict(complete=False,physical_acceptance=False,operator_power_off=True,
                operator_arming_allowed=a.allow_operator_arming,firmware=FIRMWARE,
                utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rounds=0,
                parameter_writes=0,mode_commands=0,arm_commands=0,nx_accessed=False,
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    try:
        c.open()
        if FIRMWARE not in c.query('ver all'):raise ValueError('Unexpected firmware')
        result['before']=guard(c,allow_unpublished_battery=True)
        initial=c.query('listener manual_control_setpoint -n 1');fresh(initial,.5)
        switches=c.query('listener manual_control_switches -n 1')
        if not boolean(initial,'valid') or number(initial,'throttle')>-.95:
            raise ValueError('Valid RC at minimum throttle required')
        if number(switches,'arm_switch')!=3 or number(switches,'mode_slot')!=1:
            raise ValueError('Start with SF off and SA up')
        result['mavlink_before']=c.query('mavlink status')
        print(json.dumps(dict(event='independent_readback_ready',before=result['before'])),flush=True)
        last_stamp=0;previous=None;end=time.monotonic()+a.seconds
        with (a.output/'rounds.jsonl').open('w',encoding='utf-8') as f:
            while time.monotonic()<end:
                raw={x:c.query('listener '+x+' -n 1') for x in TOPICS}
                raw['dshot']=c.query('dshot status');stamp=fresh(raw['vehicle_status'],1.5)
                armed=int(number(raw['vehicle_status'],'arming_state'));nav=int(number(raw['vehicle_status'],'nav_state'))
                row=dict(pc_monotonic_ns=time.monotonic_ns(),raw=raw,stamp=stamp,arming_state=armed,nav_state=nav,
                         offboard=boolean(raw['vehicle_control_mode'],'flag_control_offboard_enabled'),
                         kill=boolean(raw['actuator_armed'],'kill'),outputs=dshot_channels(raw['dshot']))
                f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');f.flush();result['rounds']+=1
                if stamp<last_stamp or number(raw['vehicle_status'],'hil_state')!=0:
                    raise ValueError('FC reboot or unexpected HIL state')
                last_stamp=stamp
                if armed not in ((1,2) if a.allow_operator_arming else (1,)):
                    raise ValueError('Unexpected arming state for selected observation')
                compact=dict(event='fc_mode',arming_state=armed,nav_state=nav,offboard=row['offboard'],kill=row['kill'])
                if compact!=previous:print(json.dumps(compact),flush=True);previous=compact
                time.sleep(.05)
        result['final_raw']={x:c.query('listener '+x+' -n 1') for x in TOPICS}
        result['final_raw']['dshot']=c.query('dshot status');result['mavlink_after']=c.query('mavlink status')
        result['complete']=True
    except Exception as e:result['error']=str(e)
    finally:
        c.close();result['port_closed']=not c.s.is_open
        (a.output/'console.json').write_text(json.dumps(c.records,indent=2,ensure_ascii=False),encoding='utf-8')
        (a.output/'report.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('final_raw','mavlink_before','mavlink_after')}),flush=True)
    return 0 if result['complete'] else 2


if __name__=='__main__':raise SystemExit(main())
