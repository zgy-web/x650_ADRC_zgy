#!/usr/bin/env python3

import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np

from rosbag2_py import (
    SequentialReader,
    StorageOptions,
    ConverterOptions,
)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


if len(sys.argv) != 2:
    raise SystemExit(
        "Usage: analyze_motion_simtime.py <motion_result_dir>"
    )

folder = Path(sys.argv[1]).resolve()

result = json.loads(
    (folder / "result.json").read_text()
)

events = result["events"]


def exact(text):
    m = [
        e for e in events
        if e["event"] == text
    ]

    if len(m) != 1:
        raise RuntimeError(
            f"event {text!r}: count={len(m)}"
        )

    return float(
        m[0]["sim_time_latest_truth"]
    )


def prefix(text):
    m = [
        e for e in events
        if e["event"].startswith(text)
    ]

    if len(m) != 1:
        raise RuntimeError(
            f"event prefix {text!r}: count={len(m)}"
        )

    return float(
        m[0]["sim_time_latest_truth"]
    )


t_pre0 = exact(
    "Pre-motion stationary control hold"
)

t_pre1 = prefix(
    "Pre-motion stationary control hold finished:"
)

t_motion0 = exact(
    "MOTION BEGIN: q1 0deg -> -5deg -> 0deg"
)

t_target = exact(
    "MOTION TARGET REACHED: q1=-5deg"
)

t_return = exact(
    "MOTION RETURN BEGIN: q1 -5deg -> 0deg"
)

t_motion1 = exact(
    "MOTION END: returned to reviewed q1=0 pose"
)

t_post0 = exact(
    "Post-motion stabilization hold"
)

t_post1 = prefix(
    "Post-motion stabilization hold finished:"
)


windows = {
    "pre": (
        t_pre0,
        t_pre1,
    ),

    "outbound": (
        t_motion0,
        t_target,
    ),

    "target_hold": (
        t_target,
        t_return,
    ),

    "return": (
        t_return,
        t_motion1,
    ),

    "post": (
        t_post0,
        t_post1,
    ),
}


wanted = {
    "/clock",
    "/ground_truth/odometry",
    "/arm/joint_states",
    "/fmu/out/vehicle_local_position_v1",
}


reader = SequentialReader()

reader.open(
    StorageOptions(
        uri=str(folder / "bag"),
        storage_id="sqlite3",
    ),
    ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    ),
)


types = {
    t.name: t.type
    for t in reader.get_all_topics_and_types()
}


missing = wanted - set(types)

if missing:
    raise RuntimeError(
        "Missing topics: "
        + ", ".join(sorted(missing))
    )


msg_types = {
    topic: get_message(types[topic])
    for topic in wanted
}


raw = {
    topic: []
    for topic in wanted
}


while reader.has_next():

    topic, payload, bag_ns = reader.read_next()

    if topic not in wanted:
        continue

    msg = deserialize_message(
        payload,
        msg_types[topic],
    )

    raw[topic].append(
        (
            bag_ns / 1e9,
            msg,
        )
    )


clock_bag = np.asarray(
    [
        t
        for t, _ in raw["/clock"]
    ],
    dtype=float,
)

clock_sim = np.asarray(
    [
        m.clock.sec
        + m.clock.nanosec * 1e-9
        for _, m in raw["/clock"]
    ],
    dtype=float,
)


def to_sim(topic):

    bag_t = np.asarray(
        [
            t
            for t, _ in raw[topic]
        ],
        dtype=float,
    )

    sim_t = np.interp(
        bag_t,
        clock_bag,
        clock_sim,
    )

    return [
        (
            float(t),
            msg,
        )
        for t, (_, msg)
        in zip(
            sim_t,
            raw[topic],
        )
    ]


data = {
    topic: to_sim(topic)
    for topic in (
        "/ground_truth/odometry",
        "/arm/joint_states",
        "/fmu/out/vehicle_local_position_v1",
    )
}


def samples(topic, bounds):

    lo, hi = bounds

    return [
        msg
        for t, msg in data[topic]
        if lo <= t <= hi
    ]


def mean(x):
    return sum(x) / len(x)


def p2p(x):
    return max(x) - min(x)


def rms(x):
    return math.sqrt(
        sum(v*v for v in x) / len(x)
    )


def quat_rp(q):

    sinr = 2 * (
        q.w*q.x
        + q.y*q.z
    )

    cosr = 1 - 2 * (
        q.x*q.x
        + q.y*q.y
    )

    roll = math.atan2(
        sinr,
        cosr,
    )

    sinp = 2 * (
        q.w*q.y
        - q.z*q.x
    )

    sinp = max(
        -1,
        min(1, sinp),
    )

    pitch = math.asin(sinp)

    return (
        math.degrees(roll),
        math.degrees(pitch),
    )


summary = {
    "experiment":
        result["experiment"],

    "model_sha256":
        result["model_sha256"],

    "time_basis":
        "Gazebo /clock simulation time",

    "windows": {},
}


for name, bounds in windows.items():

    truth = samples(
        "/ground_truth/odometry",
        bounds,
    )

    joints = samples(
        "/arm/joint_states",
        bounds,
    )

    px4 = samples(
        "/fmu/out/vehicle_local_position_v1",
        bounds,
    )

    if not truth or not joints or not px4:
        raise RuntimeError(
            f"empty window {name}"
        )


    x = [
        m.pose.pose.position.x
        for m in truth
    ]

    y = [
        m.pose.pose.position.y
        for m in truth
    ]

    z = [
        m.pose.pose.position.z
        for m in truth
    ]

    mx = mean(x)
    my = mean(y)

    r = [
        math.hypot(
            xx - mx,
            yy - my,
        )
        for xx, yy
        in zip(x, y)
    ]

    roll = []
    pitch = []

    for m in truth:
        rr, pp = quat_rp(
            m.pose.pose.orientation
        )

        roll.append(rr)
        pitch.append(pp)


    px = [m.x for m in px4]
    py = [m.y for m in px4]
    pz = [m.z for m in px4]

    pmx = mean(px)
    pmy = mean(py)

    pr = [
        math.hypot(
            xx - pmx,
            yy - pmy,
        )
        for xx, yy
        in zip(px, py)
    ]


    q1 = []
    dq1 = []
    q2 = []

    for m in joints:

        if (
            "arm_joint_1" not in m.name
            or
            "arm_joint_2" not in m.name
        ):
            continue

        i1 = m.name.index(
            "arm_joint_1"
        )

        i2 = m.name.index(
            "arm_joint_2"
        )

        if (
            i1 >= len(m.position)
            or i1 >= len(m.velocity)
            or i2 >= len(m.position)
        ):
            continue

        q1.append(
            math.degrees(
                m.position[i1]
            )
        )

        dq1.append(
            m.velocity[i1]
        )

        q2.append(
            math.degrees(
                m.position[i2]
            )
        )


    summary["windows"][name] = {

        "sim_start_s":
            bounds[0],

        "sim_end_s":
            bounds[1],

        "duration_s":
            bounds[1] - bounds[0],

        "ground_truth": {
            "samples":
                len(truth),

            "z_mean_m":
                mean(z),

            "z_p2p_m":
                p2p(z),

            "z_std_m":
                statistics.pstdev(z),

            "xy_rms_about_mean_m":
                rms(r),

            "xy_max_about_mean_m":
                max(r),

            "roll_p2p_deg":
                p2p(roll),

            "pitch_p2p_deg":
                p2p(pitch),

            "max_abs_roll_deg":
                max(
                    abs(v)
                    for v in roll
                ),

            "max_abs_pitch_deg":
                max(
                    abs(v)
                    for v in pitch
                ),
        },

        "px4": {
            "z_p2p_m":
                p2p(pz),

            "xy_rms_about_mean_m":
                rms(pr),
        },

        "joints": {
            "q1_min_deg":
                min(q1),

            "q1_max_deg":
                max(q1),

            "q1_final_deg":
                q1[-1],

            "q1_max_abs_velocity_rad_s":
                max(
                    abs(v)
                    for v in dq1
                ),

            "q2_max_abs_deg":
                max(
                    abs(v)
                    for v in q2
                ),
        },
    }


out = (
    folder
    / "analysis_motion_simtime_v1.json"
)

out.write_text(
    json.dumps(
        summary,
        indent=2,
    )
)


print()
print(
    "============================================================"
)

print(
    " MOTION V1 SIMULATION-TIME ANALYSIS"
)

print(
    "============================================================"
)


for name in (
    "pre",
    "outbound",
    "target_hold",
    "return",
    "post",
):

    w = summary["windows"][name]

    gt = w["ground_truth"]
    j = w["joints"]

    print()
    print(
        f"[{name.upper()}] "
        f"{w['sim_start_s']:.3f}"
        f"->{w['sim_end_s']:.3f}s "
        f"({w['duration_s']:.3f}s)"
    )

    print(
        f"  GT z_p2p       = "
        f"{gt['z_p2p_m']:.6f} m"
    )

    print(
        f"  GT z_std       = "
        f"{gt['z_std_m']:.6f} m"
    )

    print(
        f"  GT xy_rms      = "
        f"{gt['xy_rms_about_mean_m']:.6f} m"
    )

    print(
        f"  GT roll_p2p    = "
        f"{gt['roll_p2p_deg']:.6f} deg"
    )

    print(
        f"  GT pitch_p2p   = "
        f"{gt['pitch_p2p_deg']:.6f} deg"
    )

    print(
        f"  q1 min/max     = "
        f"{j['q1_min_deg']:.6f} / "
        f"{j['q1_max_deg']:.6f} deg"
    )

    print(
        f"  q2 maxabs      = "
        f"{j['q2_max_abs_deg']:.9f} deg"
    )


print()
print("Saved:", out)
