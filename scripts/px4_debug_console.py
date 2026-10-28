"""Bounded fixed-command NSH observer; logs stay on the connected PC."""
import base64
import re
import time
import serial

FIRMWARE="d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
START_DDS="uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600"
STOP_USB_MAVLINK="mavlink stop -d /dev/ttyACM0"
STOP_DDS="uxrce_dds_client stop"
LISTENER="listener trajectory_setpoint -n 150 -r 10"
READ_COMMANDS=("ver all","uxrce_dds_client status","mavlink status","param show SYS_USB_AUTO","param show -q UXRCE_DDS_DOM_ID",
               "listener vehicle_status -n 1","listener trajectory_setpoint -n 1")

def clean(text):return re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]","",text).replace("\r","")
def status_fields(text):
    text=clean(text);out={}
    for key in ("timestamp","arming_state","nav_state","hil_state","failsafe"):
        values=re.findall(r"^\s*"+key+r":\s*(\S+)",text,re.M)
        if len(values)!=1:raise ValueError("Missing or ambiguous FC status field: "+key)
        if key=="failsafe":
            if values[0] not in ("True","False"):raise ValueError("Invalid FC failsafe flag")
            out[key]=values[0]=="True"
        else:out[key]=int(values[0])
    if out["timestamp"]<=0 or out["arming_state"]!=1 or not 0<=out["nav_state"]<31 or out["nav_state"]==14 or out["hil_state"] or out["failsafe"]:
        raise ValueError("FC does not satisfy disarmed bench conditions")
    return out


class DebugConsole:
    def __init__(self,port,allow_switch=False):
        self.allow_switch=allow_switch;self.records=[];self.active_listener=False
        self.s=serial.Serial(port=None,baudrate=57600,bytesize=8,parity="N",stopbits=1,timeout=.1,write_timeout=1,
                             xonxoff=False,rtscts=False,dsrdtr=False)
        self.s.dtr=False;self.s.rts=False;self.s.port=port
    def record(self,command,raw,**extra):
        text=raw.decode(errors="replace");self.records.append(dict(command=command,text=text,raw_base64=base64.b64encode(raw).decode(),**extra));return text
    def read(self,seconds,prompt=True):
        end=time.monotonic()+seconds;b=bytearray()
        while time.monotonic()<end:
            b.extend(self.s.read(min(max(self.s.in_waiting,1),4096)))
            if prompt and b"nsh>" in b:break
        return bytes(b)
    def open(self):
        self.s.open();self.record(None,self.read(.3,False));self.s.write(b"\r")
        text=self.record("<enter>",self.read(3))
        if "nsh>" not in text:raise RuntimeError("No verified NSH prompt")
    def query(self,command):
        allowed=READ_COMMANDS+((START_DDS,STOP_USB_MAVLINK,STOP_DDS) if self.allow_switch else ())
        if command not in allowed or self.active_listener:raise ValueError("Command not allowed in this console state")
        self.s.write((command+"\r").encode());text=self.record(command,self.read(6))
        if "nsh>" not in text:raise RuntimeError("No prompt after "+command)
        return text
    def start_listener(self):
        if self.active_listener:raise RuntimeError("Listener already active")
        self.s.write((LISTENER+"\r").encode());self.active_listener=True
    def collect_listener(self):
        if not self.active_listener:raise RuntimeError("No owned listener")
        raw=self.read(18);interrupted=b"nsh>" not in raw
        if interrupted:
            self.s.write(b"\x03");raw+=self.read(3)
        self.active_listener=False
        text=self.record(LISTENER,raw,interrupted=interrupted)
        if "nsh>" not in text:raise RuntimeError("Owned listener did not release console")
        return text
    def close(self):
        if self.s.is_open:
            if self.active_listener:
                self.s.write(b"\x03");self.record("<cancel_owned_listener>",self.read(3));self.active_listener=False
            self.s.close()
