"""New TS20 conditional plant using the shared, explicit safe lifecycle hooks."""
from __future__ import annotations
from copy import deepcopy
import csv
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
for directory in (ROOT, ROOT / "src", ROOT / "control_packages/x650_wholebody_v1",
                  ROOT / "simulations/x650_wholebody_mpc_v1"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
from simulations.x650_ts20_hold_v1 import task
from simulations.x650_wholebody_mpc_v1 import run_case as shared
from x650_wholebody.ts20 import FrozenTs20Model
from x650_wholebody.ts20_prediction import Ts20PredictionModel
from x650_wholebody.ts20_runtime import Ts20Runtime


class Ts20Extension:
    """Every changed plant contract is explicit; default archived path is untouched."""
    def __init__(self, cfg, provenance):
        self.cfg, self.provenance = cfg, provenance
        self.runtime, self.predictor, self.loop = None, None, None
        self.disturbance = None

    def validate_runtime(self, cfg, model, provenance, out):
        _, checked = task.preflight(cfg)
        if checked != provenance or model.sha256 != provenance["model_sha256"]:
            raise RuntimeError("TS20 inputs changed between launch and actual process admission")
        self.log_dir = Path(os.environ.get("AMSIM_TS20_LOG_DIR", ""))
        if not self.log_dir.is_absolute() or not self.log_dir.is_dir():
            raise RuntimeError("TS20 plugin needs a unique absolute log directory set before Gazebo startup")
        if cfg["disturbance_enabled"]:
            # Quiet commissioning can proceed independently. A force run needs
            # a separately loaded, logged world-wrench implementation.
            raise RuntimeError("Declared external-force run requires verified disturbance integration")

    def stage_gates(self, cfg, model_sha, source_sha):
        return task.validate_stage_gates(cfg, {"model_sha256": model_sha, "source_urdf_sha256": source_sha})

    def on_loop_ready(self, ctx, loop, meta, out):
        import numpy
        import scipy
        self.loop = loop
        inputs = self.cfg["ts20_inputs"]
        frozen = FrozenTs20Model(inputs["source_dir"], task.read(inputs["source_manifest"]))
        self.predictor = task.predictor_from_bound_plant(self.cfg, frozen.metadata()["motion_model"])
        self.runtime = Ts20Runtime(self.log_dir / "ts20_state.csv", self.log_dir / "ts20_packets.csv", self.predictor,
            physical_position_tolerance_rad=self.cfg["ts20_protocol"]["internal_physical_q_tolerance_rad"])
        # Force imports/backend setup before arming, never on the first airborne
        # allocator call. No optimizer warm-up or synthetic solve is performed.
        if self.cfg["controller"] == "mpc":
            from x650_wholebody.ts20_mpc import Ts20MpcController
        meta.update(ts20_operating_condition=self.provenance["ts20_operating_condition"],
                    ts20_operating_condition_sha256=self.provenance["ts20_operating_condition_sha256"],
                    conditional_actuator_plant=True, base_response_is_prior=True,
                    upper_computation_and_ros_transport_delay_compensated=False,
                    future_upper_command_timing_assumption="new upper velocity is modeled at current control snapshot time; actual future compute/ROS consumption lag is not predicted",
                    supply_24v_qualified=False, aerial_arm_load_qualified=False,
                    ts20_actuator=self.runtime.metadata(), eligible_as_stage_gate=False,
                    ts20_acceptance_pending="new_plant_native_full_window_and_ulog_audit")
        meta["python_dependency_evidence"] = {"executable": sys.executable,
            "numpy_version": numpy.__version__, "numpy_path": numpy.__file__,
            "scipy_version": scipy.__version__, "scipy_path": scipy.__file__,
            "before_arming": True, "new_download": False}

    def after_state_read(self, loop, state, joint_stamp, sim):
        if loop.recovery:
            # The experiment is already disabled. Recovery must retain physical
            # joint feedback and NAV_LAND even after an actuator-log fault.
            return state, joint_stamp
        if self.runtime is None:
            raise RuntimeError("TS20 measured-state source was not installed before loop entry")
        return self.runtime.measured_state(state, sim)

    def on_pump(self, loop, sim):
        pass

    def before_publish(self, loop, begin, sim, command):
        if loop.recovery:
            return {"active": False, "reason": "experiment disabled for recovery"}
        elapsed = time.perf_counter() - begin
        limit = self.cfg["ts20_protocol"]["max_compute_s"]
        guard = {"active": True, "elapsed_before_first_publish_s": elapsed,
                 "limit_s": limit, "passed": elapsed <= limit,
                 "scope": "complete controller/adapter/returned-trace calculation before any joint or PVA publication"}
        if not guard["passed"]:
            error = RuntimeError("TS20 complete control computation exceeded 50 ms before publication")
            error.diagnostics = {"ts20_pre_publish_deadline": guard,
                                 "joint_or_pva_published_this_attempt": False}
            loop.execution_trace._failed({"event": "ts20_pre_publish_deadline_rejected", "sim_s": sim,
                "command_published": False, "guard": guard}, error)
            raise error
        return guard

    def feedback_kwargs(self, cfg):
        return {"require_acceleration": True, "require_prediction_timing": True}

    def adapter_kwargs(self, cfg):
        return {"acceleration_feedforward_policy": "zero"}

    def build_allocator(self, cfg, model, pid, trajectory, loop):
        from x650_wholebody.ts20_mpc import Ts20MpcController
        binding = self.provenance["base_response_prior"]
        if task.sha(binding["path"]) != binding["sha256"]:
            raise RuntimeError("Bound base response prior changed before allocator creation")
        prior = task.read(binding["path"])
        config = deepcopy(cfg["mpc_config"])
        config.pop("prediction_chain", None)
        config.update(response_identified=False,
                      response_model={"kind": "xy_second_order_z_first_order_v1", "parameters": prior["parameters"]})
        return Ts20MpcController(model, pid, trajectory=trajectory, config=config,
                                 predictor=self.predictor, context_provider=self.runtime.context)

    def activate(self, ctx, loop, cfg, meta, rotation):
        if cfg["stage"] == "tip_hold":
            start = shared.activate_prediction_chain(ctx, loop, cfg, meta, rotation)
        else:
            # Preserve the existing send scheduler through activation. Metadata
            # is saved by the ordinary completion/exception paths.
            previous = loop.rows
            ctx.vehicle.wait(lambda: loop.rows > previous, 8., "fresh TS20 probe send boundary")
            start = float(loop.last)
            if abs(loop.control_now() - start) > 1e-9:
                raise RuntimeError("TS20 probe activation lost actual send boundary")
            loop.task_start = start
            loop.recorder.phase = "measure"
            meta.update(phase="measure", task_start_sim_s=start, reference_base_enu_m=loop.anchor,
                        reference_rotation=rotation, requested_duration_s=cfg["duration_s"])
        meta["ts20_activation"] = {"task_start_sim_s": start,
            "reference_reset_to_measured_q": False, "command_source": "actual plugin-consumed upper commands",
            "packet_source": "actual fixed-grid MIT sends", "delay_applied_once": True}
        return start

    @staticmethod
    def _freeze_csv(source, destination, time_key, end):
        lines = Path(source).read_bytes().splitlines(keepends=True)
        if not lines or not lines[0].endswith(b"\n"):
            raise RuntimeError("TS20 source lacks a complete CSV header")
        header = next(csv.reader([lines[0].decode()]))
        index = header.index(time_key)
        saved, times = [lines[0]], []
        for line in lines[1:]:
            if not line.endswith(b"\n"):
                break
            fields = next(csv.reader([line.decode()]))
            if len(fields) != len(header):
                raise RuntimeError("Malformed complete TS20 source CSV row")
            at = float(fields[index])
            if at <= end + 1e-9:
                saved.append(line)
                times.append(at)
        if len(times) < 2:
            raise RuntimeError("TS20 capture has insufficient complete rows")
        Path(destination).write_bytes(b"".join(saved))
        return {"file": Path(destination).name, "sha256": task.sha(destination),
                "samples": len(times), "first_sim_s": times[0], "last_sim_s": times[-1],
                "semantics": "unmodified complete rows, no synthetic resampling"}

    def finalize(self, meta, stage, out):
        stage.update(kind="x650_ts20_conditional_run_result_v1", eligible_as_stage_gate=False,
            ts20_acceptance_pending="new_plant_native_full_window_and_ulog_audit",
            ts20_operating_condition=self.provenance["ts20_operating_condition"],
            ts20_operating_condition_sha256=self.provenance["ts20_operating_condition_sha256"],
            supply_24v_qualified=False, aerial_arm_load_qualified=False, base_response_is_prior=True)
        if self.runtime is not None:
            meta["ts20_actuator"] = self.runtime.close()
        end = meta.get("native_recording", {}).get("time_last_s")
        if end is not None:
            captures = {name: self._freeze_csv(self.log_dir / name, Path(out) / name, key, end)
                        for name, key in (("ts20_state.csv", "sim_s"), ("ts20_packets.csv", "send_s"))}
            meta["ts20_recording"] = captures
            shared.atomic_json(Path(out) / "ts20_recording_manifest.json", {
                "complete": meta["complete"], "captures": captures,
                "ts20_operating_condition_sha256": self.provenance["ts20_operating_condition_sha256"]})
            for name in captures:
                stage["artifacts"][name] = task.sha(Path(out) / name)
            stage["artifacts"]["ts20_recording_manifest.json"] = task.sha(Path(out) / "ts20_recording_manifest.json")


def scenario():
    cfg = task.selected_task_config()
    model, provenance = task.preflight(cfg)
    out = Path(os.environ["AMSIM_RUN_DIR"]).resolve()
    return shared.execute(cfg, model, provenance, out, extension=Ts20Extension(cfg, provenance))


if __name__ == "__main__":
    scenario()
