"""Fast trajectory, unchanged physical feedback and asynchronous C++ execution."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_cpp_live_v1.run_case import shared
from simulations.x650_cpp_normal_v1.run_case import CppNormalExtension
from simulations.x650_cpp_fast_v1 import task


class CppFastExtension(CppNormalExtension):
    def __init__(self, cfg, provenance):
        super().__init__(cfg, provenance)
        self.fast_cfg = cfg

    def validate_trajectory_manifest(self):
        if self.cpp_manifest['trajectory'] != task.trajectory():
            raise ValueError('Worker reference differs from the frozen fast trajectory')

    def source_trajectory_parameters(self):
        return task.source_parameters()

    def validate_runtime(self, cfg, model, provenance, out):
        _, checked = task.preflight(cfg)
        if checked != provenance or model.sha256 != provenance['model_sha256']:
            raise RuntimeError('Fast evidence/configuration changed before runtime')
        self.log_dir = Path(os.environ.get('AMSIM_TS20_LOG_DIR', ''))
        if not self.log_dir.is_absolute() or not self.log_dir.is_dir():
            raise RuntimeError('Unique absolute actuator log directory required')

    def stage_gates(self, cfg, model_sha, source_sha):
        return task.validate_stage_gates(cfg, {'model_sha256': model_sha, 'source_urdf_sha256': source_sha})

    def validate_activation_stage(self, cfg):
        if cfg != self.fast_cfg or cfg['stage'] != 'fast':
            raise ValueError('Fast activation differs from ground admission')

    def activate(self, ctx, loop, cfg, meta, rotation):
        start = super().activate(ctx, loop, cfg, meta, rotation)
        meta.update(frozen_cpp_normal=cfg['frozen_cpp_normal'],
                    cpp_normal_certificate=cfg['cpp_normal_certificate'])
        return start

    def finalize(self, meta, stage, out):
        super().finalize(meta, stage, out)
        for document in (meta, stage):
            document['tip_target_recording']['flight_samples_tip_target'] = (
                'Latest fast-reference sample at Python diagnostic monitor time; cached until the next pump, '
                'not exact truth-row-time reference')


def scenario():
    if (os.environ.get('X650_EXECUTION_ADAPTER') != 'cpp_full_pipeline_v1'
            or os.environ.get('AMSIM_SCENARIO_NAME') != task.SCENARIO):
        raise ValueError('Explicit dedicated C++ fast scenario required')
    cfg = task.selected_task_config()
    model, provenance = task.preflight(cfg)
    out = Path(os.environ['AMSIM_RUN_DIR']).resolve()
    extension = CppFastExtension(cfg, provenance)
    try:
        extension.prepare(out)
        return shared.execute(cfg, model, provenance, out, extension=extension)
    finally:
        if hasattr(extension, 'procs'): extension.cleanup()


if __name__ == '__main__':
    scenario()
