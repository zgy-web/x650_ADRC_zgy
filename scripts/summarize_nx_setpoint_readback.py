"""Check independent PX4 console fields against bounded NX test publications."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
from px4_debug_console import FIRMWARE,LISTENER,clean

def pattern(i):
    s=(1.,-1.,.5)[i]
    return dict(position=[-.25*s,.125*s,-.5*s],velocity=[-.0625*s,.03125*s,-.015625*s],
                acceleration=[.125*s,.0625*s,.03125*s],yaw=math.pi/2,jerk=[math.nan]*3,yawspeed=math.nan)

def parse_frames(text):
    parts=re.split(r"(?m)^[ \t]*timestamp:[ \t]*(\d+)[^\n]*\n",clean(text));out=[]
    for j in range(1,len(parts),2):
        row=dict(timestamp=int(parts[j]));body=parts[j+1]
        for name in ("position","velocity","acceleration","jerk"):
            m=re.search(r"(?m)^\s*"+name+r":\s*\[([^\]]*)\]",body)
            if not m:raise ValueError("Incomplete console field: "+name)
            vals=[float(x.strip()) for x in m[1].split(",")]
            if len(vals)!=3:raise ValueError("Wrong vector dimension")
            row[name]=vals
        for name in ("yaw","yawspeed"):
            m=re.search(r"(?m)^\s*"+name+r":\s*(\S+)",body)
            if not m:raise ValueError("Incomplete console field: "+name)
            row[name]=float(m[1])
        out.append(row)
    if not out:raise ValueError("No actual internal console frames")
    return out

def matches(row,expected,tolerance):
    for key in expected:
        if key not in row:return False
        a=row[key] if isinstance(row[key],list) else [row[key]]
        b=expected[key] if isinstance(expected[key],list) else [expected[key]]
        if len(a)!=len(b):return False
        for x,y in zip(a,b):
            if isinstance(x,bool) or not isinstance(x,(float,int)):return False
            if math.isnan(y):
                if not math.isnan(x):return False
            elif not math.isfinite(x) or abs(x-y)>tolerance:return False
    return True

def check_frames(frames,before,after,baseline=None):
    counts=[0,0,0];matched=[];unmatched=0;seen={}
    # Interleaved internal traffic may only be excluded when it matches the
    # independently captured pre-send NaN-position mode (yaw can evolve).
    baseline_fields=None
    if baseline and all(math.isnan(x) for x in baseline["position"]) and math.isfinite(baseline["yaw"]):
        baseline_fields={k:baseline[k] for k in pattern(0) if k!="yaw"}
    for f in frames:
        if f["timestamp"] in seen:
            prior={key:seen[f["timestamp"]][key] for key in pattern(0)}
            if not matches(f,prior,0):raise ValueError("Conflicting payload at identical FC timestamp")
            continue
        seen[f["timestamp"]]=f
        i=next((i for i in range(3) if matches(f,pattern(i),5.1e-6)),None)
        if i is None:
            if baseline_fields is None or not math.isfinite(f["yaw"]) or not matches(f,baseline_fields,5.1e-6):
                raise ValueError("Unexpected internal payload; not a test pattern or captured baseline")
            unmatched+=1;continue
        if not before<f["timestamp"]<after:raise ValueError("Matched pattern has stale/future FC timestamp")
        if matched and (i<matched[-1]["pattern"] or f["timestamp"]<=matched[-1]["timestamp"]):raise ValueError("Pattern ordering regressed")
        counts[i]+=1;matched.append(dict(timestamp=f["timestamp"],pattern=i))
    if min(counts)<2:raise ValueError("Fewer than two independent new frames for each A/B/C pattern: "+str(counts))
    return dict(matched_counts=counts,matched=matched,unmatched_internal_frames=unmatched,
                baseline_matched_internal_frames=unmatched,unexpected_internal_frames=0,
                unique_internal_frames=len(seen),printed_field_tolerance=5.1e-6)

def summarize(root):
    status=json.loads((root/"session_status.json").read_text());console=json.loads((root/"console.json").read_text())
    events=[json.loads(x) for x in (root/"remote.jsonl").read_text().splitlines()]
    identity=json.loads((root/"build_identity.json").read_text())
    versions=[x["text"] for x in console if x.get("command")=="ver all"]
    if len(versions)!=1 or FIRMWARE not in versions[0]:raise ValueError("Missing verified FC firmware identity")
    if not(status.get("complete") is True and status.get("port_closed") is True):raise ValueError("Incomplete console session")
    for key in ("before","after"):
        s=status[key]
        if s["arming_state"]!=1 or s["nav_state"]==14 or s["hil_state"] or s["failsafe"]:raise ValueError("Unsafe FC status")
    if not events or events[-1].get("event")!="complete" or events[-1].get("passed") is not True or any(x.get("event")=="failure" for x in events):
        raise ValueError("Missing successful NX completion")
    starts=[x for x in events if x.get("event")=="session_started"]
    if len(starts)!=1 or starts[0].get("binary_sha256")!=identity["binary"]["sha256"] or starts[0].get("source_sha256")!=identity["binary"]["source_sha256"] or starts[0].get("physical_px4") is not True:
        raise ValueError("Runtime identity mismatch")
    cleanup=[x for x in events if x.get("event")=="cleanup"]
    processes=[x for x in events if x.get("event")=="process_cleanup"]
    if len(cleanup)!=1 or cleanup[0].get("ram_removed") is not True or cleanup[0].get("persistent_files_created")!=[]:
        raise ValueError("RAM cleanup missing")
    if len(processes)!=1 or not processes[0].get("agent_stopped") or not processes[0].get("probe_stopped"):raise ValueError("Process cleanup missing")
    payloads=[x["payload"] for x in events if x.get("event")=="probe"]
    sent=[x for x in payloads if x.get("event")=="setpoint_sent"]
    finals=[x for x in payloads if x.get("event")=="probe_final"]
    if len(finals)!=1 or finals[0].get("complete") is not True or not 100<=len(sent)<=200 or finals[0]["sent"]!=len(sent):raise ValueError("Incomplete send window")
    f=finals[0]
    if any(f[x] for x in ("armed_seen","offboard_seen","hil_seen","status_fault","arm_requests_sent","offboard_requests_sent","actuator_commands_sent")):
        raise ValueError("Unsafe or out-of-scope output")
    counts=[0,0,0]
    for j,row in enumerate(sent):
        if row["seq"]!=j+1 or not 0<=row["elapsed_s"]<10:raise ValueError("Publication count/window differs")
        if j and (row["timestamp"]<=sent[j-1]["timestamp"] or row["elapsed_s"]<=sent[j-1]["elapsed_s"]):raise ValueError("Publication time regressed")
        i=row["pattern"];expected_i=0 if row["elapsed_s"]<3 else (1 if row["elapsed_s"]<6 else 2)
        if i!=expected_i:raise ValueError("Pattern duration differs")
        converted=dict(row,jerk=[math.nan if x is None else x for x in row["jerk"]],yawspeed=math.nan if row["yawspeed"] is None else row["yawspeed"])
        if row.get("null_represents_nan") is not True or not matches(converted,pattern(i),1e-7):raise ValueError("NX publication fields differ")
        counts[i]+=1
    if counts!=f["pattern_counts"] or min(counts)<10:raise ValueError("Pattern send coverage incomplete")
    streams=[x for x in console if x.get("command")==LISTENER]
    if len(streams)!=1:raise ValueError("Missing independent internal observer")
    initial=[x for x in console if x.get("command")=="listener trajectory_setpoint -n 1"]
    if len(initial)!=1:raise ValueError("Missing independent pre-send baseline")
    baseline=parse_frames(initial[0]["text"])
    if len(baseline)!=1:raise ValueError("Ambiguous baseline")
    result=check_frames(parse_frames(streams[0]["text"]),status["before"]["timestamp"],status["after"]["timestamp"],baseline[0])
    result.update(passed=True,kind="flyarm.px4_setpoint_readback.v1",sent_count=len(sent),sent_pattern_counts=counts,
                  binary_sha256=identity["binary"]["sha256"],source_sha256=identity["binary"]["source_sha256"],firmware_commit=FIRMWARE,
                  transport_fields_verified=True,control_execution_verified=False,mpc_live_validation=False,
                  offboard_takeover_tested=False,actuators_tested=False,latency_measured=False,
                  evidence=[dict(file=p.name,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sorted(root.glob('*.json*')) if p.name!='summary.json'])
    return result

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--run",type=Path,required=True);a=p.parse_args()
    r=summarize(a.run);(a.run/"summary.json").write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
