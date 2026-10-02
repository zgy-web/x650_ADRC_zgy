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
        "Usage: analyze_baseline_simtime.py <baseline_dir>"
    )


folder = Path(sys.argv[1]).resolve()
bag_dir = folder / "bag"
result_path = folder / "result.json"

if not bag_dir.is_dir():
    raise SystemExit(f"Missing bag: {bag_dir}")

if not result_path.is_file():
    raise SystemExit(f"Missing result.json: {result_path}")


result = json.loads(result_path.read_text())
events = result["events"]


# ============================================================
# Event helpers
# ============================================================

def event_exact(text):
    matches = [
        e for e in events
        if e["event"] == text
    ]

    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one exact event {text!r}, "
            f"got {len(matches)}"
        )

    value = matches[0]["sim_time_latest_truth"]

    if value is None:
        raise RuntimeError(
            f"Event has no simulation time: {text}"
        )

    return float(value)


def event_prefix(prefix):
    matches = [
        e for e in events
        if e["event"].startswith(prefix)
    ]

    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one event prefix {prefix!r}, "
            f"got {len(matches)}"
        )

    value = matches[0]["sim_time_latest_truth"]

    if value is None:
        raise RuntimeError(
            f"Event has no simulation time: {prefix}"
        )

    return float(value)


windows = {
    "pre": (
        event_exact(
            "Pre-baseline stabilization hold"
        ),
        event_prefix(
            "Pre-baseline stabilization hold finished:"
        ),
    ),

    "baseline": (
        event_exact(
            "STATIONARY UNDERFOLD BASELINE INTERVAL BEGIN"
        ),
        event_exact(
            "STATIONARY UNDERFOLD BASELINE INTERVAL END"
        ),
    ),

    "post": (
        event_exact(
            "Post-baseline stabilization hold"
        ),
        event_prefix(
            "Post-baseline stabilization hold finished:"
        ),
    ),
}


# ============================================================
# Read bag
# ============================================================

wanted = {
    "/clock",
    "/ground_truth/odometry",
    "/arm/joint_states",
    "/fmu/out/vehicle_local_position_v1",
}


reader = SequentialReader()

reader.open(
    StorageOptions(
        uri=str(bag_dir),
        storage_id="sqlite3",
    ),
    ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    ),
)


topic_types = {
    t.name: t.type
    for t in reader.get_all_topics_and_types()
}


missing = wanted - set(topic_types)

if missing:
    raise RuntimeError(
        "Bag really is missing required topics: "
        + ", ".join(sorted(missing))
    )


msg_types = {
    topic: get_message(topic_types[topic])
    for topic in wanted
}


raw_data = {
    topic: []
    for topic in wanted
}


while reader.has_next():

    topic, raw, bag_ns = reader.read_next()

    if topic not in wanted:
        continue

    msg = deserialize_message(
        raw,
        msg_types[topic],
    )

    raw_data[topic].append(
        (
            bag_ns / 1e9,
            msg,
        )
    )


print()
print("============================================================")
print(" BAG MESSAGE COUNTS")
print("============================================================")

for topic in sorted(wanted):
    print(
        f"{topic:40s} "
        f"{len(raw_data[topic])}"
    )


# ============================================================
# Establish bag-time -> Gazebo simulation-time mapping
# ============================================================

clock_bag_t = np.asarray(
    [
        t
        for t, _ in raw_data["/clock"]
    ],
    dtype=np.float64,
)

clock_sim_t = np.asarray(
    [
        msg.clock.sec
        + msg.clock.nanosec * 1e-9
        for _, msg in raw_data["/clock"]
    ],
    dtype=np.float64,
)


if len(clock_bag_t) < 2:
    raise RuntimeError(
        "Not enough /clock samples."
    )


if np.any(
    np.diff(clock_bag_t) <= 0
):
    raise RuntimeError(
        "Bag receipt timestamps for /clock "
        "are not strictly increasing."
    )


if np.any(
    np.diff(clock_sim_t) < 0
):
    raise RuntimeError(
        "Gazebo /clock moved backwards."
    )


print()
print("============================================================")
print(" CLOCK MAPPING")
print("============================================================")

print(
    f"bag time : "
    f"{clock_bag_t[0]:.6f} -> "
    f"{clock_bag_t[-1]:.6f}"
)

print(
    f"sim time : "
    f"{clock_sim_t[0]:.6f} -> "
    f"{clock_sim_t[-1]:.6f}"
)


def bag_to_sim_time(bag_times):

    bag_times = np.asarray(
        bag_times,
        dtype=np.float64,
    )

    return np.interp(
        bag_times,
        clock_bag_t,
        clock_sim_t,
    )


# Convert every required topic to simulation time.
data = {}

for topic in (
    "/ground_truth/odometry",
    "/arm/joint_states",
    "/fmu/out/vehicle_local_position_v1",
):

    bag_times = [
        t
        for t, _ in raw_data[topic]
    ]

    sim_times = bag_to_sim_time(
        bag_times
    )

    data[topic] = [
        (
            float(sim_t),
            msg,
        )
        for sim_t, (_, msg)
        in zip(
            sim_times,
            raw_data[topic],
        )
    ]


# ============================================================
# Math
# ============================================================

def window_samples(topic, bounds):

    lo, hi = bounds

    return [
        msg
        for t, msg in data[topic]
        if lo <= t <= hi
    ]


def mean(x):
    return sum(x) / len(x)


def rms(x):
    return math.sqrt(
        sum(v*v for v in x) / len(x)
    )


def p2p(x):
    return max(x) - min(x)


def quat_to_roll_pitch(q):

    x = q.x
    y = q.y
    z = q.z
    w = q.w

    sinr_cosp = 2.0 * (
        w*x + y*z
    )

    cosr_cosp = 1.0 - 2.0 * (
        x*x + y*y
    )

    roll = math.atan2(
        sinr_cosp,
        cosr_cosp,
    )

    sinp = 2.0 * (
        w*y - z*x
    )

    sinp = max(
        -1.0,
        min(1.0, sinp),
    )

    pitch = math.asin(
        sinp
    )

    return (
        math.degrees(roll),
        math.degrees(pitch),
    )


# ============================================================
# Analysis
# ============================================================

summary = {
    "experiment_dir": str(folder),

    "model_sha256":
        result.get("model_sha256"),

    "ulog_sha256":
        result.get("ulog_sha256"),

    "time_basis":
        "Gazebo /clock simulation time",

    "windows": {},
}


for name, bounds in windows.items():

    truth = window_samples(
        "/ground_truth/odometry",
        bounds,
    )

    joints = window_samples(
        "/arm/joint_states",
        bounds,
    )

    px4 = window_samples(
        "/fmu/out/vehicle_local_position_v1",
        bounds,
    )


    if (
        not truth
        or not joints
        or not px4
    ):
        raise RuntimeError(
            f"Empty data in window {name}: "
            f"truth={len(truth)}, "
            f"joints={len(joints)}, "
            f"px4={len(px4)}"
        )


    # --------------------------------------------------------
    # Ground truth
    # --------------------------------------------------------

    tx = [
        m.pose.pose.position.x
        for m in truth
    ]

    ty = [
        m.pose.pose.position.y
        for m in truth
    ]

    tz = [
        m.pose.pose.position.z
        for m in truth
    ]


    mx = mean(tx)
    my = mean(ty)


    radial_about_mean = [
        math.hypot(
            x - mx,
            y - my,
        )
        for x, y
        in zip(tx, ty)
    ]


    x0 = tx[0]
    y0 = ty[0]

    radial_from_start = [
        math.hypot(
            x - x0,
            y - y0,
        )
        for x, y
        in zip(tx, ty)
    ]


    roll = []
    pitch = []

    for m in truth:

        r, p = quat_to_roll_pitch(
            m.pose.pose.orientation
        )

        roll.append(r)
        pitch.append(p)


    # --------------------------------------------------------
    # PX4 estimate
    # --------------------------------------------------------

    px = [
        m.x
        for m in px4
    ]

    py = [
        m.y
        for m in px4
    ]

    pz = [
        m.z
        for m in px4
    ]


    pmx = mean(px)
    pmy = mean(py)


    pradial = [
        math.hypot(
            x - pmx,
            y - pmy,
        )
        for x, y
        in zip(px, py)
    ]


    # --------------------------------------------------------
    # Joints
    # --------------------------------------------------------

    joint_metrics = {}

    for joint_name in (
        "arm_joint_1",
        "arm_joint_2",
    ):

        q = []
        dq = []

        for msg in joints:

            if joint_name not in msg.name:
                continue

            i = msg.name.index(
                joint_name
            )

            if (
                i >= len(msg.position)
                or
                i >= len(msg.velocity)
            ):
                continue

            q.append(
                msg.position[i]
            )

            dq.append(
                msg.velocity[i]
            )


        if not q or not dq:
            raise RuntimeError(
                f"No usable samples for "
                f"{joint_name} in {name}"
            )


        q_deg = [
            math.degrees(v)
            for v in q
        ]


        joint_metrics[
            joint_name
        ] = {
            "samples":
                len(q),

            "q_mean_deg":
                mean(q_deg),

            "q_p2p_deg":
                p2p(q_deg),

            "q_max_abs_deg":
                max(
                    abs(v)
                    for v in q_deg
                ),

            "dq_max_abs_rad_s":
                max(
                    abs(v)
                    for v in dq
                ),
        }


    summary["windows"][name] = {

        "sim_start_s":
            bounds[0],

        "sim_end_s":
            bounds[1],

        "sim_duration_s":
            bounds[1] - bounds[0],


        "ground_truth": {

            "samples":
                len(truth),

            "z_mean_m":
                mean(tz),

            "z_p2p_m":
                p2p(tz),

            "z_std_m":
                statistics.pstdev(tz),

            "xy_rms_about_mean_m":
                rms(radial_about_mean),

            "xy_max_about_mean_m":
                max(
                    radial_about_mean
                ),

            "xy_max_from_window_start_m":
                max(
                    radial_from_start
                ),

            "xy_start_to_end_m":
                math.hypot(
                    tx[-1] - tx[0],
                    ty[-1] - ty[0],
                ),

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


        "px4_local_position": {

            "samples":
                len(px4),

            "z_mean_m":
                mean(pz),

            "z_p2p_m":
                p2p(pz),

            "z_std_m":
                statistics.pstdev(pz),

            "xy_rms_about_mean_m":
                rms(pradial),

            "xy_max_about_mean_m":
                max(pradial),
        },


        "joints":
            joint_metrics,
    }


# ============================================================
# Save / compact output
# ============================================================

out = (
    folder
    / "analysis_baseline_simtime_v2.json"
)

out.write_text(
    json.dumps(
        summary,
        indent=2,
    )
)


print()
print("============================================================")
print(" SIMULATION-TIME BASELINE SUMMARY")
print("============================================================")


for name in (
    "pre",
    "baseline",
    "post",
):

    w = summary["windows"][name]

    gt = w["ground_truth"]
    px = w["px4_local_position"]

    j1 = w["joints"]["arm_joint_1"]
    j2 = w["joints"]["arm_joint_2"]

    print()
    print(
        f"[{name.upper()}] "
        f"{w['sim_start_s']:.3f} -> "
        f"{w['sim_end_s']:.3f}s"
    )

    print(
        "  ground_truth:"
    )

    print(
        f"    z_p2p = "
        f"{gt['z_p2p_m']:.6f} m"
    )

    print(
        f"    z_std = "
        f"{gt['z_std_m']:.6f} m"
    )

    print(
        f"    xy_rms_about_mean = "
        f"{gt['xy_rms_about_mean_m']:.6f} m"
    )

    print(
        f"    xy_start_to_end = "
        f"{gt['xy_start_to_end_m']:.6f} m"
    )

    print(
        f"    roll_p2p = "
        f"{gt['roll_p2p_deg']:.6f} deg"
    )

    print(
        f"    pitch_p2p = "
        f"{gt['pitch_p2p_deg']:.6f} deg"
    )

    print(
        "  PX4:"
    )

    print(
        f"    z_p2p = "
        f"{px['z_p2p_m']:.6f} m"
    )

    print(
        f"    xy_rms_about_mean = "
        f"{px['xy_rms_about_mean_m']:.6f} m"
    )

    print(
        "  joints:"
    )

    print(
        f"    q1 maxabs = "
        f"{j1['q_max_abs_deg']:.9f} deg"
    )

    print(
        f"    q2 maxabs = "
        f"{j2['q_max_abs_deg']:.9f} deg"
    )


print()
print("Saved:", out)
