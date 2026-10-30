"""Read/prepare or explicitly apply the five reviewed TELEM1 parameters.

No reboot, USB restart, control setpoint, arming or mode command is implemented.
Changing parameters requires separate operator approval; default is read-only.
"""
import argparse,datetime,hashlib,json
from pathlib import Path
from px4_debug_console import DebugConsole,FIRMWARE,clean,status_fields
from px4_motor_single_pulse import require_zero
from px4_motor_preflight_readonly import parse_parameter

PARAMS={'SER_TEL1_BAUD':921600,'MAV_0_MODE':1,'MAV_0_RATE':30000,'MAV_0_FLOW_CTRL':0,'MAV_0_FORWARD':0}
READS={'ver all','listener vehicle_status -n 1','dshot status','mavlink status','param show -q MAV_0_CONFIG'} | {'param show -q '+k for k in PARAMS}


class ProfileConsole(DebugConsole):
    def __init__(self,port,apply):super().__init__(port);self.apply=apply
    def command(self,cmd):
        writes={'param set '+k+' '+str(v) for k,v in PARAMS.items()}
        if cmd not in READS and not(self.apply and cmd in writes):raise ValueError('Unapproved command')
        self.s.write((cmd+'\r').encode('ascii'));result=self.record(cmd,self.read(6))
        if 'nsh>' not in result:raise RuntimeError('No console prompt')
        return clean(result)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port',default='COM13')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--apply-approved-profile',action='store_true')
    p.add_argument('--power-disconnected',action='store_true');a=p.parse_args()
    if a.apply_approved_profile and not a.power_disconnected:raise ValueError('Operator power-isolation confirmation required')
    a.output.mkdir(parents=True,exist_ok=False)
    r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),apply_requested=a.apply_approved_profile,
           proposed=PARAMS,original={},readback={},written=[],reboot_sent=False,arm_commands=0,mode_commands=0,setpoints=0,
           nx_configuration_changed=False,complete=False,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    c=ProfileConsole(a.port,a.apply_approved_profile)
    try:
        c.open()
        if FIRMWARE not in c.command('ver all'):raise ValueError('Unexpected firmware')
        r['status']=status_fields(c.command('listener vehicle_status -n 1'))
        r['outputs']=require_zero(c.command('dshot status'))
        if parse_parameter(c.command('param show -q MAV_0_CONFIG'))!=101:raise ValueError('MAV_0 is not TELEM1')
        r['mavlink_before']=c.command('mavlink status')
        if '/dev/ttyS6' not in r['mavlink_before']:raise ValueError('Expected UART instance absent')
        for name in PARAMS:
            v=parse_parameter(c.command('param show -q '+name))
            if v is None or int(v)!=v:raise ValueError('Original parameter not an integer')
            r['original'][name]=int(v)
        # The rollback values are saved on PC BEFORE the first mutation.
        (a.output/'original_parameters.json').write_text(json.dumps(r['original'],indent=2))
        if a.apply_approved_profile:
            for name,value in PARAMS.items():
                c.command('param set '+name+' '+str(value));r['written'].append(name)
                actual=parse_parameter(c.command('param show -q '+name));r['readback'][name]=actual
                if actual!=value:raise ValueError('Parameter readback mismatch: '+name)
            r['reboot_required']=True
        r['final_status']=status_fields(c.command('listener vehicle_status -n 1'))
        r['final_outputs']=require_zero(c.command('dshot status'));r['complete']=True
    except Exception as e:r['error']=str(e)
    finally:
        c.close();r['port_closed']=not c.s.is_open
        (a.output/'console.json').write_text(json.dumps(c.records,indent=2,ensure_ascii=False),encoding='utf-8')
        (a.output/'report.json').write_text(json.dumps(r,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(r,indent=2,ensure_ascii=False))
    return 0 if r['complete'] else 2


if __name__=='__main__':raise SystemExit(main())
