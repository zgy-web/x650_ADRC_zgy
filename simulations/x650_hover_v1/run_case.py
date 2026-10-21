"""No-arm X650 hover protocol; no propulsion parameters are inferred here."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time
from types import SimpleNamespace

import yaml

from amsim.modeling.manager import ensure_flight_ready, resolve, validate_px4_airframe
from amsim.paths import PROJECT_ROOT, results_dir

DEFAULT_CONFIG = PROJECT_ROOT / "configs/scenarios/x650_hover_v1.yaml"


def preflight(config: dict, *, check_airframe: bool = True):
    """Resolve and verify immutable inputs before importing ROS or commanding PX4."""
    try:
        model = resolve(str(config["model"]))
    except (KeyError, FileNotFoundError) as exc:
        raise RuntimeError(
            "X650 flight model is not built. Confirm motor/propeller and battery, "
            "motor mapping/directions and documented propulsion parameters; then "
            "bind an explicit flight profile. Unmeasured torque/response terms may "
            "use explicitly documented estimates for an initial simulation. "
            "No x500 propulsion or PID values will be substituted."
        ) from exc
    ensure_flight_ready(model)
    if model.source != "urdf_flight" or model.joints:
        raise ValueError("x650_hover_v1 requires an imported flight model with zero arm joints")
    expected_sha = config.get("expected_urdf_sha256")
    if not isinstance(expected_sha, str) or not re.fullmatch(r"[0-9a-f]{64}", expected_sha):
        raise ValueError(
            "Set expected_urdf_sha256 to the reviewed flight-preparation URDF hash "
            "after rotor-frame, continuous-joint and velocity-limit corrections. "
            "The original CAD URDF hash is provenance only."
        )
    manifest = json.loads(model.manifest_path.read_text(encoding="utf-8"))
    source = json.loads((model.sdf_path.parent / "source_import_manifest.json").read_text(encoding="utf-8"))
    if source.get("source", {}).get("urdf_sha256") != expected_sha:
        raise ValueError("Flight model does not originate from the selected X650 URDF")
    if check_airframe:
        validate_px4_airframe(model)
    for section, fields in {
        "vehicle": ("takeoff_height_m",),
        "timeline": ("stable_required_s", "pre_hold_s", "experiment_hold_s", "post_hold_s"),
        "sampling": ("rate_hz",),
        "acceptance": ("position_rmse_m", "position_max_m", "tilt_max_deg"),
    }.items():
        for field in fields:
            value = config[section][field]
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{section}.{field} must be finite and positive")
    return model, {
        "model_name": model.model_name,
        "model_sha256": model.sha256,
        "total_mass_kg": model.total_mass_kg,
        "flight_profile_sha256": manifest["profile_sha256"],
        "source_urdf_sha256": source["source"]["urdf_sha256"],
        "original_urdf_sha256_reference": config.get("original_urdf_sha256"),
        "px4_sys_autostart": model.px4_sys_autostart,
        "controller": "PX4 flight controller from the selected airframe",
        "arm_control": False,
        "flight_validated_before_run": bool(manifest.get("flight_validated", False)),
    }


def euler(qx, qy, qz, qw):
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Ground-truth quaternion is invalid")
    x, y, z, w = (v / norm for v in (qx, qy, qz, qw))
    return (
        math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
        math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))),
        math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)),
    )


def sample(observation, target, target_yaw: float, phase: str) -> dict:
    truth, vehicle = observation.ground_truth, observation.vehicle
    if truth is None:
        raise RuntimeError("Ground truth is required for the hover metric")
    roll, pitch, yaw = euler(truth.qx, truth.qy, truth.qz, truth.qw)
    error = [truth.x-target[0], truth.y-target[1], truth.z-target[2]]
    tilt = math.acos(max(-1.0, min(1.0, math.cos(roll)*math.cos(pitch))))
    row = {
        "sim_time_s": observation.sim_time_s, "phase": phase,
        "world_x_m": truth.x, "world_y_m": truth.y, "world_z_m": truth.z,
        "world_qx": truth.qx, "world_qy": truth.qy, "world_qz": truth.qz, "world_qw": truth.qw,
        "target_world_x_m": target[0], "target_world_y_m": target[1], "target_world_z_m": target[2],
        "error_x_m": error[0], "error_y_m": error[1], "error_z_m": error[2],
        "position_error_m": math.sqrt(sum(v*v for v in error)),
        "roll_deg": math.degrees(roll), "pitch_deg": math.degrees(pitch),
        "yaw_deg": math.degrees(yaw),
        "yaw_error_deg": math.degrees(math.atan2(math.sin(yaw-target_yaw), math.cos(yaw-target_yaw))),
        "tilt_deg": math.degrees(tilt),
        "estimate_ned_x_m": vehicle.x, "estimate_ned_y_m": vehicle.y, "estimate_ned_z_m": vehicle.z,
    }
    if not all(math.isfinite(v) for k, v in row.items() if k != "phase"):
        raise RuntimeError("Nonfinite hover telemetry")
    return row


class FlightTelemetry:
    """Record real truth messages on the API node's existing single-thread executor.

    VehicleAPI pumps that executor throughout blocking takeoff/land commands. A
    second truth subscription therefore captures those phases without threads,
    monkey-patching VehicleAPI, or replacing the command/ACK processing loop.
    """

    def __init__(self, folder, rate_hz, model_sha256, target, target_yaw, rows):
        self.folder = Path(folder)
        self.period = 1.0 / rate_hz
        self.rate_hz = rate_hz
        self.model_sha256 = model_sha256
        self.target, self.target_yaw = target, target_yaw
        self.rows = rows
        self.phase = "ground_ready"
        self.next_sample = None
        self.last_stamp = None
        self.error = None
        self.context = None
        self.subscription = None
        self.stream = (self.folder / "flight_samples.csv").open("w", newline="", encoding="utf-8")
        self.writer = None
        self.last_flush = time.monotonic()

    def attach(self, context):
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        self.context = context
        self.subscription = context.node.create_subscription(
            Odometry, "/ground_truth/odometry", self.receive, qos_profile_sensor_data,
        )
        # Seed with the actual initial message, never an invented zero pose.
        self.receive(context.node.data["truth"])

    def receive(self, message):
        try:
            node = self.context.node
            if not all(key in node.data for key in ("position", "status", "land")):
                return
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if self.last_stamp is not None and stamp < self.last_stamp - 1e-9:
                raise RuntimeError("Ground-truth time moved backwards during capture")
            self.last_stamp = stamp
            if self.next_sample is not None and stamp < self.next_sample - 1e-9:
                return
            position, orientation = message.pose.pose.position, message.pose.pose.orientation
            truth = SimpleNamespace(
                x=position.x, y=position.y, z=position.z,
                qx=orientation.x, qy=orientation.y, qz=orientation.z, qw=orientation.w,
            )
            observation = SimpleNamespace(
                ground_truth=truth, vehicle=node.observation().vehicle, sim_time_s=stamp,
            )
            row = sample(observation, self.target, self.target_yaw, self.phase)
            status, land, estimate = node.data["status"], node.data["land"], node.data["position"]
            row.update({
                "model_sha256": self.model_sha256,
                "frame_id": message.header.frame_id,
                "child_frame_id": message.child_frame_id,
                "armed": int(observation.vehicle.armed),
                "landed": int(land.landed), "failsafe": int(status.failsafe),
                "px4_arming_state": int(status.arming_state),
                "px4_nav_state": int(status.nav_state),
                "px4_status_timestamp_us": int(status.timestamp),
                "px4_position_timestamp_us": int(estimate.timestamp),
            })
            if self.writer is None:
                self.writer = csv.DictWriter(self.stream, fieldnames=list(row))
                self.writer.writeheader()
            self.writer.writerow(row)
            self.rows.append(row)
            if self.next_sample is None:
                self.next_sample = stamp + self.period
            else:
                # Advance the schedule but retain the observed message stamp.
                # No interpolated samples are inserted for missed messages.
                skipped = max(1, math.floor((stamp-self.next_sample)/self.period + 1e-9) + 1)
                self.next_sample += skipped*self.period
            if time.monotonic() - self.last_flush >= 1.0:
                self.stream.flush()
                self.last_flush = time.monotonic()
        except Exception as exc:
            # Preserve the flight command processing loop so recovery can land.
            # The scenario checks this flag before/after commands and each hold.
            if self.error is None:
                self.error = str(exc) or exc.__class__.__name__

    def check(self):
        if self.error is not None:
            raise RuntimeError("Flight telemetry capture failed: " + self.error)

    def detach(self):
        if self.subscription is not None:
            self.context.node.destroy_subscription(self.subscription)
            self.subscription = None

    def close(self):
        self.stream.close()
        intervals = [b["sim_time_s"]-a["sim_time_s"] for a, b in zip(self.rows, self.rows[1:])]
        metadata = {
            "schema_version": 1, "file": "flight_samples.csv", "model_sha256": self.model_sha256,
            "sha256": hashlib.sha256((self.folder / "flight_samples.csv").read_bytes()).hexdigest(),
            "source_topic": "/ground_truth/odometry", "requested_rate_hz": self.rate_hz,
            "time": "Original Odometry header stamp in simulation seconds; no interpolation",
            "pose": "Gazebo world pose of the odometry child frame (base_link); original frame IDs are preserved",
            "quaternion": "world_qx/y/z/w: body-to-world active rotation, ROS xyzw ordering, no frame conversion",
            "target": "Fixed hover point: initial base_link world position plus takeoff_height_m along world +Z; not a dynamic takeoff/landing reference",
            "px4_status": "Latest received PX4 state at each truth callback; separate PX4 microsecond timestamps identify asynchronous telemetry",
            "sample_count": len(self.rows),
            "observed_interval_s": {"min": min(intervals), "max": max(intervals), "mean": sum(intervals)/len(intervals)} if intervals else None,
            "phases": {phase: sum(row["phase"] == phase for row in self.rows) for phase in dict.fromkeys(row["phase"] for row in self.rows)},
            "capture_error": self.error,
        }
        (self.folder / "flight_samples_manifest.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        return metadata


def summarize(rows: list[dict], acceptance: dict) -> dict:
    measured = [row for row in rows if row["phase"] == "measure"]
    if len(measured) < 2:
        raise RuntimeError("Insufficient measured hover samples")
    stamps = [row["sim_time_s"] for row in measured]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise RuntimeError("Hover sample timestamps are not strictly increasing")
    def rms(key):
        return math.sqrt(sum(row[key]**2 for row in measured) / len(measured))
    metrics = {
        "frame": "Gazebo world; target = pre-takeoff base pose + requested world-Z height",
        "sample_count": len(measured), "duration_s": stamps[-1] - stamps[0],
        "position_rmse_m": rms("position_error_m"),
        "position_max_m": max(row["position_error_m"] for row in measured),
        "axis_rmse_m": {axis: rms(f"error_{axis}_m") for axis in "xyz"},
        "roll_rms_deg": rms("roll_deg"), "pitch_rms_deg": rms("pitch_deg"),
        "yaw_error_rms_deg": rms("yaw_error_deg"),
        "tilt_max_deg": max(row["tilt_deg"] for row in measured),
    }
    metrics["criteria"] = {key: metrics[key] <= limit for key, limit in acceptance.items()}
    metrics["accepted"] = all(metrics["criteria"].values())
    return metrics


def execute(config, model, provenance, folder):
    # Import only after dry-run validation, keeping dry-run usable without ROS.
    from amsim import ScenarioContext
    from amsim.recording import RosbagRecorder
    from parameter_evidence import ParameterEvidence

    rows, events = [], []
    result = {"success": False, "experiment": config["name"], "provenance": provenance, "protocol": config}
    recorder = None
    telemetry = None
    parameter_evidence = None
    attempted_takeoff = False
    def event(text, ctx):
        events.append({"event": text, "sim_time_s": ctx.node.sim_time_s(), "wall_time": time.time()})
        print(text, flush=True)
    try:
        with ScenarioContext() as ctx:
            if ctx.model["model_sha256"] != model.sha256 or ctx.model["model"] != model.model_name or ctx.model["joint_names"]:
                raise RuntimeError("Active runtime does not match the selected no-arm flight model")
            try:
                ctx.vehicle.wait_ready()
                initial = ctx.observe(ground_truth=True)
                if initial.vehicle.armed or not initial.vehicle.landed:
                    raise RuntimeError("Hover protocol requires a fresh disarmed vehicle on the ground")
                parameter_evidence = ParameterEvidence(
                    folder, validate_px4_airframe(model),
                    PROJECT_ROOT / "configs/vehicles/x650_no_arm/4999_gz_x650_mn4014_v1",
                    model.px4_sys_autostart, model.sha256,
                )
                event("parameter_readback_before_begin", ctx)
                parameter_evidence.capture(ctx.vehicle, "before", enforce_contract=True)
                event("parameter_readback_before_verified", ctx)
                # Parameter reads may take several seconds. Establish the flight
                # reference from a newly received pose after ground checks.
                initial = ctx.observe(ground_truth=True)
                if initial.vehicle.armed or not initial.vehicle.landed:
                    raise RuntimeError("Vehicle left the disarmed ground state during parameter verification")
                truth = initial.ground_truth
                height = config["vehicle"]["takeoff_height_m"]
                target = (truth.x, truth.y, truth.z + height)
                target_yaw = euler(truth.qx, truth.qy, truth.qz, truth.qw)[2]
                resets = (ctx.node.data["position"].xy_reset_counter, ctx.node.data["position"].z_reset_counter)
                if config["record"]["rosbag"] and not os.environ.get("AMSIM_RUN_DIR"):
                    recorder = RosbagRecorder(folder, [], include_ground_truth=True)
                    recorder.start()
                telemetry = FlightTelemetry(folder, config["sampling"]["rate_hz"], model.sha256, target, target_yaw, rows)
                telemetry.attach(ctx)
                telemetry.check()
                ground_start = ctx.node.sim_time_s()
                ctx.vehicle.wait(lambda: ctx.node.sim_time_s()-ground_start >= 0.5, 10, "ground telemetry capture")
                telemetry.check()
                telemetry.phase = "takeoff"
                event("takeoff_begin", ctx)
                attempted_takeoff = True
                ctx.vehicle.takeoff(height, stable_s=config["timeline"]["stable_required_s"])
                telemetry.check()
                event("stable_hover", ctx)
                for phase, key in (("settle", "pre_hold_s"), ("measure", "experiment_hold_s"), ("post", "post_hold_s")):
                    start = ctx.node.sim_time_s()
                    last = [start]
                    telemetry.phase = phase
                    event(phase + "_begin", ctx)
                    def monitor():
                        telemetry.check()
                        if recorder is not None and recorder.process.poll() is not None:
                            raise RuntimeError("Rosbag recorder exited during hover")
                        now = time.monotonic()
                        if any(now-ctx.node.received.get(k, 0) > (3 if k in ("status", "land") else 2) for k in ("position", "status", "land", "truth")):
                            raise RuntimeError("Hover telemetry is stale")
                        p = ctx.node.data["position"]
                        if (p.xy_reset_counter, p.z_reset_counter) != resets or not (p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid):
                            raise RuntimeError("PX4 position estimate invalid or reference reset")
                        obs = ctx.node.observation(include_ground_truth=True)
                        if obs.vehicle.failsafe or not obs.vehicle.armed or obs.vehicle.landed:
                            raise RuntimeError("Unexpected failsafe, disarm or landing during hover")
                        if obs.sim_time_s < last[0] - 1e-9:
                            raise RuntimeError("Simulation time moved backwards")
                        last[0] = obs.sim_time_s
                        row = sample(obs, target, target_yaw, phase)
                        if row["position_error_m"] > 2.0 or row["tilt_deg"] > 45:
                            raise RuntimeError("Hover trial envelope exceeded")
                        return obs.sim_time_s - start >= config["timeline"][key]
                    ctx.vehicle.wait(monitor, config["timeline"][key]*10 + 30, phase)
                telemetry.phase = "landing"
                event("landing_begin", ctx)
                ctx.vehicle.land()
                telemetry.check()
                telemetry.phase = "landed"
                event("landed_and_disarmed", ctx)
                landed_start = ctx.node.sim_time_s()
                ctx.vehicle.wait(lambda: ctx.node.sim_time_s()-landed_start >= 1.0, 15, "landed telemetry capture")
                telemetry.check()
                result["landing_confirmed"] = True
                result["metrics"] = summarize(rows, config["acceptance"])
                result["success"] = result["metrics"]["accepted"]
                # Finish the video motion timeline before potentially lengthy
                # post-flight ground reads. MAVLink and ROS still use one pump.
                telemetry.detach()
                event("parameter_readback_after_begin", ctx)
                parameter_evidence.capture(ctx.vehicle, "after", enforce_contract=False)
                event("parameter_readback_after_complete", ctx)
            except (Exception, KeyboardInterrupt) as exc:
                result["error"] = str(exc) or "Interrupted"
                result["success"] = False
                if attempted_takeoff and not result.get("landing_confirmed"):
                    try:
                        if telemetry is not None:
                            telemetry.phase = "recovery_landing"
                        event("recovery_landing_begin", ctx)
                        ctx.vehicle.land()
                        result["recovery_landed"] = True
                        if telemetry is not None:
                            telemetry.phase = "recovered_landed"
                        event("recovery_landed_and_disarmed", ctx)
                        if telemetry is not None:
                            telemetry.detach()
                        if parameter_evidence is not None and "after" not in parameter_evidence.reports:
                            parameter_evidence.capture(ctx.vehicle, "after", enforce_contract=False)
                    except (Exception, KeyboardInterrupt) as recovery:
                        result["recovery_error"] = str(recovery) or "Interrupted"
            finally:
                if telemetry is not None:
                    telemetry.detach()
    except (Exception, KeyboardInterrupt) as exc:
        result["error"] = str(exc) or "Interrupted"
        result["success"] = False
    finally:
        if parameter_evidence is not None:
            result["parameter_evidence"] = parameter_evidence.reports
            if "after" in parameter_evidence.reports:
                result["parameters_unchanged"] = parameter_evidence.reports["after"]["parameters_unchanged"]
        if telemetry is not None:
            try:
                result["flight_telemetry"] = telemetry.close()
            except Exception as exc:
                result["telemetry_finalize_error"] = str(exc)
                result["success"] = False
        if recorder is not None:
            try:
                recorder.stop()
            except Exception as exc:
                result["recording_error"] = str(exc)
                result["success"] = False
        if rows:
            # Preserve the initial steady-phase file for existing analysis, while
            # flight_samples.csv is the authoritative full-motion video source.
            hover_rows = [row for row in rows if row["phase"] in {"settle", "measure", "post"}]
            with (folder / "hover_samples.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(hover_rows)
        result["events"] = events
        (folder / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(os.environ.get("AMSIM_SCENARIO_CONFIG", DEFAULT_CONFIG)))
    parser.add_argument("--dry-run", action="store_true", help="Check model/profile without importing ROS or starting flight")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    try:
        model, provenance = preflight(config)
    except (ValueError, RuntimeError, KeyError, FileNotFoundError) as exc:
        print(json.dumps({"ready": False, "flight_started": False, "error": str(exc)}, indent=2))
        return 2
    if args.dry_run:
        print(json.dumps({"ready": True, "flight_started": False, "provenance": provenance, "protocol": config}, indent=2))
        return 0
    if not os.environ.get("AMSIM_SCENARIO_NAME"):
        raise RuntimeError("Launch through './scripts/amsim run x650_hover_v1' so the platform owns startup and ULog archival")
    folder = Path(os.environ["AMSIM_RUN_DIR"]) if os.environ.get("AMSIM_RUN_DIR") else results_dir() / (config["name"] + "_" + time.strftime("%Y%m%d_%H%M%S") + f"_{os.getpid()}")
    folder.mkdir(parents=True, exist_ok=bool(os.environ.get("AMSIM_RUN_DIR")))
    provenance["scenario_sha256"] = hashlib.sha256(args.config.read_bytes()).hexdigest()
    result = execute(config, model, provenance, folder)
    print(f"Results: {folder}", flush=True)
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
