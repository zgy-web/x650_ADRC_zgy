"""Read-only extraction of recovery evidence from the original hold21 ULog.

Run with an existing pyulog environment: python review_ulog.py /path/to/px4.ulg.
Prints derived JSON; never modifies the ULog or writes server files.
"""
import hashlib
import json
import math
from pathlib import Path
import sys

from pyulog import ULog


def scalar(value):
    value = value.item() if hasattr(value, "item") else value
    if isinstance(value, float) and not math.isfinite(value):
        return "NaN" if math.isnan(value) else ("+Inf" if value > 0 else "-Inf")
    return value


def rows(dataset, fields):
    return [{key: scalar(dataset.data[key][i]) for key in ["timestamp", *fields]}
            for i in range(len(dataset.data["timestamp"]))]


def changes(records, fields):
    return [row for i, row in enumerate(records)
            if i == 0 or i == len(records) - 1
            or any(row[field] != records[i - 1][field] for field in fields)]


def main(path):
    raw = Path(path).read_bytes()
    ulog = ULog(str(path))
    get = lambda name: next(d for d in ulog.data_list if d.name == name and d.multi_id == 0)
    status_fields = ["arming_state", "nav_state", "failsafe"]
    status = rows(get("vehicle_status"), status_fields)
    land = rows(get("vehicle_land_detected"), ["landed", "ground_contact", "maybe_landed"])
    commands = rows(get("vehicle_command"), ["command", "param1", "param2", "from_external"])
    acknowledgments = rows(get("vehicle_command_ack"), ["command", "result", "from_external"])
    offboard = rows(get("offboard_control_mode"), ["position", "velocity", "acceleration", "attitude", "body_rate"])
    lost = rows(get("failsafe_flags"), ["offboard_control_signal_lost"])
    armed_seen = any(row["arming_state"] == 2 for row in status)
    airborne_seen = any(not row["landed"] for row in land)
    land_commands = [row for row in commands if row["command"] == 21]
    last_land_command = land_commands[-1]["timestamp"] if land_commands else None
    landing_after_command = [row for row in land if last_land_command is not None
                             and row["timestamp"] >= last_land_command and row["landed"]]
    disarming_after_command = [row for row in status if last_land_command is not None
                               and row["timestamp"] >= last_land_command and row["arming_state"] != 2]
    report = {
        "kind": "hold21_ulog_recovery_extraction_v1", "derived_analysis": True,
        "source_path": str(path), "source_bytes": len(raw),
        "source_sha256": hashlib.sha256(raw).hexdigest(),
        "timestamp_semantics": "Original PX4 ULog microseconds; no fitted conversion to host monotonic or simulation time.",
        "nonfinite_semantics": "Nonfinite optional command parameters are preserved as NaN/+Inf/-Inf strings.",
        "start_timestamp_us": int(ulog.start_timestamp), "last_timestamp_us": int(ulog.last_timestamp),
        "vehicle_status_rows": len(status), "vehicle_status_changes": changes(status, status_fields),
        "land_detector_rows": len(land), "land_detector_changes": changes(land, ["landed"]),
        "vehicle_commands": commands, "vehicle_command_acknowledgments": acknowledgments,
        "offboard_control_mode": {"count": len(offboard), "first": offboard[0], "last": offboard[-1],
                                  "changes": changes(offboard, ["position", "velocity", "acceleration", "attitude", "body_rate"])},
        "offboard_signal_lost_changes": changes(lost, ["offboard_control_signal_lost"]),
        "recovery": {"armed_seen": armed_seen, "airborne_seen": airborne_seen,
                     "land_command_present": bool(land_commands),
                     "first_landed_report_after_land_command": landing_after_command[0] if landing_after_command else None,
                     "first_not_armed_report_after_land_command": disarming_after_command[0] if disarming_after_command else None,
                     "final_status": status[-1], "final_land_detector": land[-1],
                     "landed_and_not_armed_at_log_end": bool(land[-1]["landed"] and status[-1]["arming_state"] != 2)},
        "scope": "Recovery and offboard-state evidence only; not full-window controller performance or command-by-command PVA acceptance."
    }
    assert armed_seen and airborne_seen and landing_after_command and disarming_after_command
    assert report["recovery"]["landed_and_not_armed_at_log_end"]
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main(sys.argv[1])
