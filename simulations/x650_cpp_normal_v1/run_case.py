"""Normal reference with the accepted C++ feedback/control/execution lifecycle."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_cpp_live_v1.run_case import CppLiveExtension, shared
from simulations.x650_cpp_normal_v1 import task


class CppNormalExtension(CppLiveExtension):
    def __init__(self, cfg, provenance):
        super().__init__(task.baseline_config(cfg['frozen_cpp_hold']), provenance)
        self.normal_cfg = cfg

    def validate_trajectory_manifest(self):
        if self.cpp_manifest['trajectory'] != task.trajectory():
            raise ValueError('Worker reference differs from the frozen normal trajectory')

    def source_trajectory_parameters(self):
        return task.source_parameters()

    def validate_runtime(self, cfg, model, provenance, out):
        _, checked = task.preflight(cfg)
        if checked != provenance or model.sha256 != provenance['model_sha256']:
            raise RuntimeError('Normal evidence/configuration changed before runtime')
        self.log_dir = Path(os.environ.get('AMSIM_TS20_LOG_DIR', ''))
        if not self.log_dir.is_absolute() or not self.log_dir.is_dir():
            raise RuntimeError('Unique absolute actuator log directory required')

    def stage_gates(self, cfg, model_sha, source_sha):
        return task.validate_stage_gates(cfg, {'model_sha256': model_sha, 'source_urdf_sha256': source_sha})

    def validate_activation_stage(self, cfg):
        # Original-file validation completes on the ground, not at handoff.
        if cfg != self.normal_cfg or cfg['stage'] != 'normal':
            raise ValueError('Normal activation differs from ground admission')

    def activate(self, ctx, loop, cfg, meta, rotation):
        start = super().activate(ctx, loop, cfg, meta, rotation)
        meta.update(figure8_protocol=cfg['figure8_protocol'], frozen_cpp_hold=cfg['frozen_cpp_hold'],
                    cpp_hold_certificate=cfg['cpp_hold_certificate'])
        return start

    def handle_active_pump(self, loop, sim):
        handled = super().handle_active_pump(loop, sim)
        if handled:
            # Low-priority display/capture only. The C++ source independently
            # samples its actual input time; this cached value is not feedback.
            loop.recorder.tip_target = loop.trajectory.sample(sim-loop.task_start)[0].copy()
        return handled

    def finalize(self, meta, stage, out):
        super().finalize(meta, stage, out)
        target_recording = {
            'flight_samples_tip_target': 'Latest normal-reference sample at Python diagnostic monitor time; cached until the next pump, not exact truth-row-time reference',
            'authoritative_control_target': 'Original C++ ControlSnapshot target at its control_time_s in isolated_live/cpp_source',
            'authoritative_error_reference': 'Frozen FigureEight evaluated at each native sample time relative to original LiveStart.origin_s and target_position_enu',
            'used_by_cpp_controller': False,
        }
        for document in (meta, stage):
            document.update(upper_allocator_active=True, tip_target_recording=target_recording.copy())


def scenario():
    if (os.environ.get('X650_EXECUTION_ADAPTER') != 'cpp_full_pipeline_v1'
            or os.environ.get('AMSIM_SCENARIO_NAME') != task.SCENARIO):
        raise ValueError('Explicit dedicated C++ normal scenario required')
    cfg = task.selected_task_config()
    model, provenance = task.preflight(cfg)
    out = Path(os.environ['AMSIM_RUN_DIR']).resolve()
    extension = CppNormalExtension(cfg, provenance)
    try:
        extension.prepare(out)
        return shared.execute(cfg, model, provenance, out, extension=extension)
    finally:
        if hasattr(extension, 'procs'): extension.cleanup()


if __name__ == '__main__':
    scenario()
