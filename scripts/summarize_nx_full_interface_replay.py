"""Fail-closed summary of the RAM-only full interface replay evidence."""
import argparse
import hashlib
import json
import math
from pathlib import Path


FLAGS=("historical_baseline_match","facade_vs_direct_exact","prepare_did_not_commit",
       "commit_memory_exact","cdr_setpoint_fields_exact","joint_output_identity","passed")


def checked_result(result,item,libraries,ids,*,wire_required=False):
    if result.get("kind")!="x650_hardware_interface_replay_result_v1" or result.get("complete") is not True:
        raise ValueError("Incomplete native report")
    if result.get("input_sha256")!=item["sha256"] or result.get("source_sha256")!=item["source_sha256"]:
        raise ValueError("Input identity differs")
    if result.get("physical_validation") is not False or result.get("raw_cdr_recording") is not False or result.get("publishers_created")!=0:
        raise ValueError("Replay scope changed")
    if wire_required and (result.get("mavlink_wire_replay") is not True or result.get("serial_devices_opened")!=0):
        raise ValueError("Missing offline MAVLink replay evidence")
    for key,reportkey in (("core","loaded_core"),("optimization","loaded_optimizer"),("predictor","loaded_predictor")):
        if result.get(reportkey,{}).get("sha256")!=libraries[key]["sha256"]:
            raise ValueError("Loaded library differs")
    rows=result.get("rows",[])
    if result.get("row_count")!=item["count"] or len(rows)!=item["count"] or [r.get("id") for r in rows]!=ids:
        raise ValueError("Missing, duplicated or reordered rows")
    for row in rows:
        if wire_required and row.get("mavlink_setpoint_fields_exact") is not True:
            raise ValueError("MAVLink field validation missing or failed")
        if any(row.get(flag) is not True for flag in FLAGS) or "error" in row:
            raise ValueError("Failed replay row: "+str(row.get("id")))
        for key in ("adapter_state_max_error","historical_baseline_max_error","quantized_input_command_difference","quantized_input_position_reference_difference"):
            v=row.get(key)
            if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<0:
                raise ValueError("Invalid numeric evidence")
        if row["adapter_state_max_error"]>2e-12:raise ValueError("State conversion differs")
        axes=row.get("quantized_input_command_absolute_differences",[])
        if len(axes)!=5 or not all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) and x>=0 for x in axes):
            raise ValueError("Per-axis quantization evidence missing")
        if max(axes)!=row["quantized_input_command_difference"]:raise ValueError("Quantization maximum differs")
        msg=row.get("expected_setpoint",{})
        if not isinstance(msg.get("timestamp"),int) or msg["timestamp"]<=0:raise ValueError("Missing output timestamp")
        for name in ("position","velocity","acceleration"):
            values=msg.get(name,[])
            if len(values)!=3 or not all(isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x) for x in values):
                raise ValueError("Missing finite setpoint fields")
        if msg.get("jerk")!=[None]*3 or "yawspeed" not in msg or msg["yawspeed"] is not None or msg.get("null_represents_nan") is not True:
            raise ValueError("NaN field mask changed")
        if not isinstance(msg.get("yaw"),(int,float)) or not math.isfinite(msg["yaw"]):raise ValueError("Invalid yaw")
    if result.get("passed") is not True:raise ValueError("Native report failed")
    return dict(name=item["name"],rows=len(rows),passed=True,
                adapter_state_max_error=max(r["adapter_state_max_error"] for r in rows),
                historical_baseline_max_error=max(r["historical_baseline_max_error"] for r in rows),
                quantized_input_command_difference_max=max(r["quantized_input_command_difference"] for r in rows),
                quantized_input_position_reference_difference_max=max(r["quantized_input_position_reference_difference"] for r in rows),
                quantized_command_absolute_difference_max_by_axis=[max(r["quantized_input_command_absolute_differences"][i] for r in rows) for i in range(5)],
                quantized_command_axis_units=["m/s","m/s","m/s","rad/s","rad/s"],
                quantized_command_difference_above_1e_minus_7_count=sum(r["quantized_input_command_difference"]>1e-7 for r in rows))


def summarize(run,inputs):
    request=json.loads((run/"request_manifest.json").read_text())
    declaration=json.loads((inputs/"manifest.json").read_text())
    items=request.get("inputs",[])
    if not items or items!=declaration.get("inputs") or any(x.get("count",0)<=0 for x in items):
        raise ValueError("Requested input coverage differs")
    if len({x["name"] for x in items})!=len(items) or sum(x["count"] for x in items)!=declaration.get("row_count"):
        raise ValueError("Duplicated or incomplete batch declaration")
    events=[json.loads(x) for x in (run/"transport.jsonl").read_text().splitlines()]
    if not events or events[-1].get("event")!="complete" or events[-1].get("passed") is not True:
        raise ValueError("Missing successful session end")
    expected_results=[dict(name=x["name"],count=x["count"],passed=True) for x in items]
    if events[-1].get("results")!=expected_results or any(e.get("event")=="failure" for e in events):
        raise ValueError("Final batch coverage differs or failed phase exists")
    cleanup=[e for e in events if e.get("event")=="cleanup"]
    if len(cleanup)!=1 or cleanup[0].get("ram_removed") is not True or cleanup[0].get("libraries_unchanged") is not True or cleanup[0].get("persistent_files_created")!=[]:
        raise ValueError("Cleanup/library audit missing")
    build=[e for e in events if e.get("event")=="build_identity"]
    if len(build)!=1 or build[0].get("source_archive_sha256")!=request["archive_sha256"]:
        raise ValueError("Build identity missing")
    if (build[0].get("bound_header_sha256")!=request["files"]["bound_native/mpc_core.hpp"] or
        build[0].get("math_libraries")!={k:v["sha256"] for k,v in request["libraries"].items()} or
        build[0].get("ram_only") is not True):
        raise ValueError("Build header or libraries differ")
    artifact_events=[e for e in events if e.get("event")=="artifact"]
    artifacts={e["name"]:e for e in artifact_events}
    expected_artifacts={"configure.log","build.log","ctest.log"}|{x["name"]+".log" for x in items}
    if len(artifacts)!=len(artifact_events) or set(artifacts)!=expected_artifacts:
        raise ValueError("Missing or duplicated phase artifacts")
    for name,event in artifacts.items():
        if Path(name).name!=name or event.get("exit_code")!=0:
            raise ValueError("Invalid phase result")
        if hashlib.sha256((run/name).read_bytes()).hexdigest()!=event.get("sha256"):
            raise ValueError("Phase artifact changed")
    groups=[];evidence=[]
    for item in request["inputs"]:
        input_raw=(inputs/item["file"]).read_bytes()
        if hashlib.sha256(input_raw).hexdigest()!=item["sha256"]:raise ValueError("Local input changed")
        data=json.loads(input_raw);path=run/(item["name"]+".log");raw=path.read_bytes()
        if artifacts.get(path.name,{}).get("sha256")!=hashlib.sha256(raw).hexdigest():raise ValueError("Native output changed")
        groups.append(checked_result(json.loads(raw),item,request["libraries"],[c["id"] for c in data["recorded"]["cases"]],
                                     wire_required=request.get("mavlink_wire_replay",False)))
        evidence.append(dict(file=path.name,sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw)))
    return dict(kind="x650_full_interface_replay_summary_v1",passed=True,row_count=sum(g["rows"] for g in groups),
                groups=groups,build=build[0],cleanup=cleanup[0],evidence=evidence,
                physical_validation=False,raw_cdr_recording=False,publishers_created=0,
                mavlink_wire_replay=request.get("mavlink_wire_replay",False),
                controller_parameters_changed=False,
                limits=["Reconstructed messages from saved simulation states; not captured raw PX4 CDR.",
                        "Float32 input quantization changes the input: differences against unquantized historical outputs are reported separately.",
                        "Fixed snapshots restore authentic Memory; this does not certify a continuous live controller session.",
                        "No USB/DDS network delivery, PX4 internal receipt, real joint bus, live clock alignment, takeover or real-time qualification."])


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--run",type=Path,required=True)
    p.add_argument("--inputs",type=Path,required=True);p.add_argument("--output",type=Path,required=True)
    a=p.parse_args();result=summarize(a.run,a.inputs)
    a.output.write_text(json.dumps(result,indent=2),encoding="utf-8");print(json.dumps(result,indent=2))
