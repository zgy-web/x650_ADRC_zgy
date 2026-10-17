#!/usr/bin/env python3
"""Compare packaged outputs with saved commands from actual historical flights.

Read-only input replay: this never launches PX4/Gazebo and never modifies a run.
Unlike the native-rate presentation NPZ, control_trace.csv includes the exact
measured velocity, angular velocity, timestamps and period used by the original
controller. Replay starts at the first prestream row so the angular acceleration
filter history is retained through the task start.

Usage: python tests/replay_control_trace.py --case /absolute/original/run --output report.json
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from flyarm_pid_ff import (  # noqa: E402
    ArmParameters, ControlState, EndpointTarget, FigureEight,
    PidFeedforwardController, frozen_pid_parameters,
)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xyz(row, prefix, axes="xyz"):
    return np.array([float(row[prefix+axis]) for axis in axes])


def pair(row, first, second):
    return np.array([float(row[first]), float(row[second])])


def replay(case, tolerance=2e-10):
    case = Path(case).resolve()
    meta_path, trace_path, model_path = (case/name for name in ("metadata.yaml","control_trace.csv","model_snapshot.sdf"))
    metadata = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
    if not metadata.get("complete"):
        raise ValueError(f"Historical run is not complete: {case}")
    requested = metadata["requested_parameters"]
    if requested != frozen_pid_parameters():
        raise ValueError(f"Historical run does not use this package's frozen PID parameters: {case}")
    initial = ControlState(
        metadata["reference_base_enu_m"], metadata["reference_rotation"],
        np.zeros(3), np.zeros(3), np.zeros(2), np.zeros(2), 0.,
    )
    trajectory = FigureEight(center=metadata["reference_center_enu_m"], **metadata["trajectory"])
    controller = PidFeedforwardController(initial, arm_parameters=ArmParameters.from_sdf(model_path), mode=metadata["mode"])
    max_error = {}
    worst = []
    counts = {"all":0,"hold":0,"task":0}
    with trace_path.open(newline="",encoding="utf-8") as stream:
        for index,row in enumerate(csv.DictReader(stream)):
            sample = ControlState(
                xyz(row,"base_p"),xyz(row,"base_q","wxyz"),xyz(row,"v"),xyz(row,"body_w"),
                pair(row,"q1","q2"),pair(row,"qd1","qd2"),float(row["truth_stamp_s"]),
            )
            t,dt=float(row["t_s"]),float(row["control_dt_s"])
            in_task=bool(int(row["in_task"]))
            if in_task:
                command=controller.step(t,dt,sample,EndpointTarget(*trajectory.sample(t)))
            else:
                command=controller.hold(sample)
            setpoint=command.px4_setpoint(xyz(row,"px4_p"),xyz(row,"px4_v"))
            comparisons={
                "position_reference":(command.position_reference,xyz(row,"base_ref_")),
                "velocity_feedforward":(command.velocity_feedforward,xyz(row,"base_v_ref_")),
                "nominal_acceleration":(command.nominal_acceleration,xyz(row,"base_a_ref_")),
                "arm_acceleration":(command.arm_acceleration,xyz(row,"ff_a")),
                "acceleration_feedforward":(command.acceleration_feedforward,xyz(row,"cmd_a")),
                "base_velocity_target":(command.base_velocity_target,xyz(row,"base_cmd_v_")),
                "joint_velocity":(command.joint_velocity,pair(row,"qd1_cmd","qd2_cmd")),
                "joint_reference":(command.reference["q_ref"],pair(row,"q1_ref","q2_ref")),
                "joint_velocity_reference":(command.reference["qd_ref"],pair(row,"qd1_ref","qd2_ref")),
                "joint_acceleration_reference":(command.reference["qdd_ref"],pair(row,"qdd1_ref","qdd2_ref")),
                "tip_position":(command.tip_position,xyz(row,"tip_")),
                "tip_target":(command.tip_target,xyz(row,"tip_ref_")),
                "px4_position":(setpoint.position,xyz(row,"ned_sp_")),
                "px4_velocity":(setpoint.velocity,xyz(row,"ned_sv_")),
                "px4_acceleration":(setpoint.acceleration,xyz(row,"ned_sa_")),
            }
            for name,(actual,expected) in comparisons.items():
                actual,expected=np.asarray(actual),np.asarray(expected)
                if not np.isfinite(actual).all() or not np.isfinite(expected).all():
                    raise ValueError(f"Nonfinite replay field {name}, row {index}")
                error=float(np.max(np.abs(actual-expected)))
                if error>max_error.get(name,-1.):
                    max_error[name]=error
                if error>tolerance:
                    worst.append({"field":name,"row":index,"task_time_s":t,"max_abs_error":error})
                    worst=sorted(worst,key=lambda item:item["max_abs_error"],reverse=True)[:12]
            counts["all"]+=1
            counts["task" if in_task else "hold"]+=1
    if not counts["hold"] or not counts["task"]:
        raise ValueError("Replay must include prestream/hold and task rows; do not supply a cropped CSV")
    return {
        "case":str(case),"mode":metadata["mode"],"experiment":metadata["experiment"],
        "replicate":metadata["replicate"],"passed":all(value<=tolerance for value in max_error.values()),
        "samples":counts,"absolute_tolerance":tolerance,"max_abs_error_by_field":max_error,
        "largest_failures":worst,
        "input_sha256":{path.name:sha256(path) for path in (meta_path,trace_path,model_path)},
        "scope":"Actual recorded control inputs replayed chronologically, including hold filter history; command equivalence only, not a new simulation flight.",
    }


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case",type=Path,action="append",required=True)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--tolerance",type=float,default=2e-10)
    args=parser.parse_args()
    if not np.isfinite(args.tolerance) or args.tolerance<=0:
        parser.error("--tolerance must be finite and positive")
    reports=[replay(case,args.tolerance) for case in args.case]
    result={"passed":all(item["passed"] for item in reports),"cases":reports}
    text=json.dumps(result,indent=2,allow_nan=False)+"\n"
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(text,encoding="utf-8")
    print(text,end="")
    return 0 if result["passed"] else 1


if __name__=="__main__":
    raise SystemExit(main())
