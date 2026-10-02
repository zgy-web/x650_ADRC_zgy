from __future__ import annotations

import json
import time
from pathlib import Path

from pymavlink import mavutil


def probe(connection: str, baud: int = 921600, timeout_s: float = 8.0) -> dict:
    if connection.startswith("/") and not Path(connection).exists():
        return {
            "ok": False,
            "connection": connection,
            "error": "device does not exist",
        }

    master = mavutil.mavlink_connection(
        connection,
        baud=baud,
        autoreconnect=False,
        source_system=255,
    )
    try:
        hb = master.wait_heartbeat(timeout=timeout_s)
        if hb is None:
            return {
                "ok": False,
                "connection": connection,
                "error": f"no MAVLink heartbeat within {timeout_s:.1f}s",
            }
        base_mode = int(hb.base_mode)
        hil_enabled = bool(
            base_mode & mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED
        )
        autopilot_px4 = (
            int(hb.autopilot) == mavutil.mavlink.MAV_AUTOPILOT_PX4
        )
        return {
            "ok": True,
            "connection": connection,
            "system_id": int(master.target_system),
            "component_id": int(master.target_component),
            "autopilot_px4": autopilot_px4,
            "hil_enabled": hil_enabled,
            "base_mode": base_mode,
            "vehicle_type": int(hb.type),
            "autopilot_type": int(hb.autopilot),
        }
    finally:
        master.close()


def main() -> int:
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--mavlink", required=True)
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--timeout", type=float, default=8.0)
    p.add_argument("--require-hil", action="store_true")
    args = p.parse_args()

    result = probe(args.mavlink, args.baud, args.timeout)
    if args.require_hil and result.get("ok") and not result.get("hil_enabled"):
        result["ok"] = False
        result["error"] = "PX4 heartbeat is present but HIL mode is not enabled"
    print(json.dumps(result, indent=2))
    return 0 if result.get("ok") else 2


if __name__ == "__main__":
    raise SystemExit(main())
