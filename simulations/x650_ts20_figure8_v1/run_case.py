"""Trajectory-only extension; the qualified numerical and lifecycle code is unchanged."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_ts20_figure8_v1 import task
from simulations.x650_ts20_hold_v1.run_case import Ts20Extension, shared


class FigureEightExtension(Ts20Extension):
    def __init__(self, cfg, provenance):
        # The inherited actuator factory uses the original, unchanged plant variant.
        super().__init__(task.baseline_config(cfg["frozen_hold"]), provenance)
        self.trajectory_cfg = cfg

    def validate_runtime(self, cfg, model, provenance, out):
        _, checked = task.preflight(cfg)
        if checked != provenance or model.sha256 != provenance["model_sha256"]:
            raise RuntimeError("Trajectory inputs changed after launch admission")
        self.log_dir = Path(os.environ.get("AMSIM_TS20_LOG_DIR", ""))
        if not self.log_dir.is_absolute() or not self.log_dir.is_dir():
            raise RuntimeError("Unique absolute actuator log directory required")

    def stage_gates(self, cfg, model_sha, source_sha):
        return task.validate_stage_gates(cfg, {"model_sha256": model_sha, "source_urdf_sha256": source_sha})

    def activate(self, ctx, loop, cfg, meta, rotation):
        start = shared.activate_prediction_chain(ctx, loop, cfg, meta, rotation)
        meta["ts20_activation"] = {"task_start_sim_s": start,
            "reference_reset_to_measured_q": False, "command_source": "actual plugin-consumed upper commands",
            "packet_source": "actual fixed-grid MIT sends", "delay_applied_once": True}
        meta.update(figure8_protocol=cfg["figure8_protocol"], frozen_hold=cfg["frozen_hold"])
        return start


def scenario():
    cfg = task.selected_task_config()
    model, provenance = task.preflight(cfg)
    return shared.execute(cfg, model, provenance, Path(os.environ["AMSIM_RUN_DIR"]).resolve(),
                          extension=FigureEightExtension(cfg, provenance))


if __name__ == "__main__":
    scenario()
