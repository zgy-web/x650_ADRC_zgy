"""Actual C++ fast loop, explicit actual plant and physical wrench supervision."""
import os
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_cpp_fast_v1.run_case import CppFastExtension, shared
from simulations.x650_cpp_robustness_v1 import task
from simulations.x650_cpp_robustness_v1.disturbance import DisturbanceMixin


class CppRobustExtension(DisturbanceMixin, CppFastExtension):
    def __init__(self, cfg, provenance):
        self.robust_cfg = cfg
        super().__init__(cfg, provenance)

    def validate_runtime(self, cfg, model, provenance, out):
        _, checked = task.preflight(cfg)
        if checked != provenance or model.sha256 != provenance['model_sha256']:
            raise RuntimeError('Actual robustness condition changed before runtime')
        self.log_dir = Path(os.environ.get('AMSIM_TS20_LOG_DIR', ''))
        if not self.log_dir.is_absolute() or not self.log_dir.is_dir():
            raise RuntimeError('Unique absolute actuator log directory required')

    def stage_gates(self, cfg, model_sha, source_sha):
        return task.validate_stage_gates(cfg, {'model_sha256':model_sha,'source_urdf_sha256':source_sha})

    def validate_activation_stage(self, cfg):
        if cfg != self.robust_cfg or cfg['stage'] != 'fast':
            raise ValueError('Actual robustness activation differs from ground admission')

    def finalize(self, meta, stage, out):
        super().finalize(meta, stage, out)
        for document in (meta, stage):
            document.update(robustness=self.robust_cfg['robustness'],
                frozen_cpp_fast=self.robust_cfg['frozen_cpp_fast'],
                cpp_fast_certificate=self.robust_cfg['cpp_fast_certificate'],
                actual_plant_robustness=True, delay_injection_enabled=False,
                external_force_disturbance=self.robust_cfg['disturbance_enabled'])


def scenario():
    cfg = task.selected_task_config()
    if (os.environ.get('X650_EXECUTION_ADAPTER') != 'cpp_full_pipeline_v1'
            or os.environ.get('AMSIM_SCENARIO_NAME') != cfg['scenario']):
        raise ValueError('Dedicated explicit robustness simulation required')
    model, provenance = task.preflight(cfg); out = Path(os.environ['AMSIM_RUN_DIR']).resolve()
    extension = CppRobustExtension(cfg, provenance)
    try:
        extension.prepare(out)
        return shared.execute(cfg, model, provenance, out, extension=extension)
    finally:
        if hasattr(extension,'procs'): extension.cleanup()


if __name__ == '__main__': scenario()
