#!/usr/bin/env python3

import json
import math
import statistics
import sys
from pathlib import Path

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


if len(sys.argv) != 2:
    raise SystemExit(
        "Usage: python3 analyze_baseline.py <baseline_result_dir>"
    )

folder = Path(sys.argv[1]).resolve()
bag_dir = folder / "bag"
result_path = folder / "result.json"

if not bag_dir.is_dir():
    raise SystemExit(f"Missing bag: {bag_dir}")

if not result_path.is_file():
    raise SystemExit(f"Missing result.json: {result_path}")

result = json.loads(result_path.read_text())


# ------------------------------------------------------------
# Experiment windows from recorded wall-clock events
# ------------------------------------------------------------

events = result["events"]


def event_time(exact):
    matches = [
        e for e in events
        if e["event"] == exact
    ]

    if len(matches) != 1:
        raise RuntimeError(
            f"Expected exactly one event {exact!r}, got {len(matches)}"
        )

    return float(matches[0]["wall_time"])


windows = {
    "pre": (
        event_time("Pre-baseline stabilization hold"),
        event_time(
            "Pre-baseline stabilization hold finished: "
            "requested=10.0s, actual_sim=10.000s"
        ),
    ),

    "baseline": (
        event_time(
            "STATIONARY UNDERFOLD BASELINE INTERVAL BEGIN"
        ),
        event_time(
            "STATIONARY UNDERFOLD BASELINE INTERVAL END"
        ),
    ),

    "post": (
        event_time("Post-baseline stabilization hold"),
        event_time(
            "Post-baseline stabilization hold finished: "
            "requested=10.0s, actual_sim=10.000s"
        ),
    ),
}


# ------------------------------------------------------------
# Read only topics needed for baseline analysis
# ------------------------------------------------------------

wanted = {
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
        "Missing required topics: "
        + ", ".join(sorted(missing))
    )

msg_types = {
    topic: get_message(topic_types[topic])
    for topic in wanted
}

data = {
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

    data[topic].append(
        (bag_ns / 1e9, msg)
    )


# ------------------------------------------------------------
# Math helpers
# ------------------------------------------------------------

def window_samples(topic, bounds):
    lo, hi = bounds

    return [
        msg for t, msg in data[topic]
        if lo <= t <= hi
    ]


def mean(xs):
    return sum(xs) / len(xs)


def rms(xs):
    return math.sqrt(
        sum(x*x for x in xs) / len(xs)
    )


def p2p(xs):
    return max(xs) - min(xs)


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

    pitch = math.asin(sinp)

    return (
        math.degrees(roll),
        math.degrees(pitch),
    )


# ------------------------------------------------------------
# Metrics
# ------------------------------------------------------------

summary = {
    "experiment_dir": str(folder),
    "model_sha256": result.get(
        "model_sha256"
    ),
    "ulog_sha256": result.get(
        "ulog_sha256"
    ),
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

    if not truth or not joints or not px4:
        raise RuntimeError(
            f"Empty samples in window {name}"
        )

    # ----- Gazebo truth -----

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

    radial = [
        math.hypot(
            x - mx,
            y - my,
        )
        for x, y in zip(tx, ty)
    ]

    roll = []
    pitch = []

    for m in truth:
        r, p = quat_to_roll_pitch(
            m.pose.pose.orientation
        )

        roll.append(r)
        pitch.append(p)

    # ----- PX4 local position -----

    px = [m.x for m in px4]
    py = [m.y for m in px4]
    pz = [m.z for m in px4]

    pmx = mean(px)
    pmy = mean(py)

    pradial = [
        math.hypot(
            x - pmx,
            y - pmy,
        )
        for x, y in zip(px, py)
    ]

    # ----- joints -----

    joint_values = {
        "arm_joint_1": {
            "q": [],
            "dq": [],
        },
        "arm_joint_2": {
            "q": [],
            "dq": [],
        },
    }

    for m in joints:
        for joint in joint_values:
            if joint not in m.name:
                continue

            i = m.name.index(joint)

            if (
                i < len(m.position)
                and
                i < len(m.velocity)
            ):
                joint_values[joint]["q"].append(
                    m.position[i]
                )

                joint_values[joint]["dq"].append(
                    m.velocity[i]
                )

    joint_metrics = {}

    for joint, vals in joint_values.items():
        q = vals["q"]
        dq = vals["dq"]

        if not q or not dq:
            raise RuntimeError(
                f"No samples for {joint}"
            )

        q_deg = [
            math.degrees(v)
            for v in q
        ]

        joint_metrics[joint] = {
            "samples": len(q),
            "q_mean_deg": mean(q_deg),
            "q_p2p_deg": p2p(q_deg),
            "q_max_abs_deg": max(
                abs(v) for v in q_deg
            ),
            "dq_max_abs_rad_s": max(
                abs(v) for v in dq
            ),
        }

    summary["windows"][name] = {
        "wall_duration_s":
            bounds[1] - bounds[0],

        "ground_truth": {
            "samples": len(truth),
            "z_mean_m": mean(tz),
            "z_p2p_m": p2p(tz),
            "z_std_m":
                statistics.pstdev(tz),

            "xy_rms_about_mean_m":
                rms(radial),

            "xy_max_about_mean_m":
                max(radial),

            "roll_p2p_deg":
                p2p(roll),

            "pitch_p2p_deg":
                p2p(pitch),

            "max_abs_roll_deg":
                max(abs(v) for v in roll),

            "max_abs_pitch_deg":
                max(abs(v) for v in pitch),
        },

        "px4_local_position": {
            "samples": len(px4),
            "z_mean_m": mean(pz),
            "z_p2p_m": p2p(pz),
            "z_std_m":
                statistics.pstdev(pz),

            "xy_rms_about_mean_m":
                rms(pradial),

            "xy_max_about_mean_m":
                max(pradial),
        },

        "joints": joint_metrics,
    }


output = (
    folder
    / "analysis_baseline_v1.json"
)

output.write_text(
    json.dumps(
        summary,
        indent=2,
    )
)

print(
    json.dumps(
        summary,
        indent=2,
    )
)

print()
print(
    "Saved:",
    output,
)
