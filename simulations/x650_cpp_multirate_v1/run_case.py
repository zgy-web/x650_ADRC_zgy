"""Use the existing actual PX4/Gazebo lifecycle with an explicit new timing task."""
import os
from copy import deepcopy
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_cpp_live_v1.run_case import CppLiveExtension, ExternalCppAllocator, expected_core_config, shared, sha
from simulations.x650_cpp_robustness_v1.disturbance import DisturbanceMixin
from . import task, profile


class CppMultirateExtension(DisturbanceMixin, CppLiveExtension):
    def __init__(self, cfg, provenance):
        self.multirate_cfg = cfg
        self.robust_cfg = cfg
        # The legacy Python plant adapter selects the same original motor
        # variant. Its configuration is not the new native controller config.
        adapter_cfg = deepcopy(task.baseline_config(cfg))
        adapter_cfg['timing_profile'] = profile.NAME
        super().__init__(adapter_cfg, provenance)

    def _external_wrench(self):
        return self.multirate_cfg['multirate']['case_id'] == 'wrench_fast'

    def validate_trajectory_manifest(self):
        if self.cpp_manifest['trajectory'] != task.trajectory(self.multirate_cfg):
            raise ValueError('Multirate worker reference differs from its exact task')

    def source_trajectory_parameters(self):
        value = task.trajectory(self.multirate_cfg)
        return ({'trajectory_kind': 'hold'} if value['kind'] == 'hold' else
            {'trajectory_kind': 'figure8', **{k:v for k,v in value.items() if k not in {'kind','duration_s'}}})

    def validate_runtime(self, cfg, model, provenance, out):
        _, checked = task.preflight(cfg)
        if checked != provenance or model.sha256 != provenance['model_sha256']:
            raise RuntimeError('Multirate task/physical preparation changed after admission')
        self.log_dir = Path(os.environ.get('AMSIM_TS20_LOG_DIR', ''))
        if not self.log_dir.is_absolute() or not self.log_dir.is_dir():
            raise RuntimeError('Unique absolute actuator log directory required')

    def stage_gates(self, cfg, model_sha, source_sha):
        return task.validate_stage_gates(cfg, {'model_sha256':model_sha,'source_urdf_sha256':source_sha})

    def validate_activation_stage(self, cfg):
        if cfg != self.multirate_cfg or cfg['stage'] not in {'tip_hold','fast'}:
            raise ValueError('Multirate activation differs from ground admission')

    def _prepare_allocator(self, loop):
        binding = self.provenance['base_response_prior']
        if sha(binding['path']) != binding['sha256']:
            raise RuntimeError('Original response prior changed')
        expected = expected_core_config(self.multirate_cfg, loop.model, loop.pid,
            self.predictor, task.read(binding['path']))
        if expected != self.cpp_manifest['config']:
            raise RuntimeError('New native timing/configuration does not match actual plant/PID')
        return ExternalCppAllocator(loop.model, loop.pid, self.cpp_manifest, self.procs['worker'].pid)

    def handle_active_pump(self, loop, sim):
        handled = super().handle_active_pump(loop, sim)
        if handled and self._external_wrench():
            loop.recorder.tip_target = loop.trajectory.sample(sim-loop.task_start)[0].copy()
        return handled

    def finalize(self, meta, stage, out):
        super().finalize(meta, stage, out)
        for document in (meta, stage):
            document.update(timing_profile=profile.NAME, effective_timing_profile=profile.effective(),
                multirate_derivation=self.multirate_cfg['multirate'], delay_injection_enabled=False,
                hardware_qualified=False, physical_acceptance_inherited=False)
            if self._external_wrench():
                document.update(robustness=self.multirate_cfg['robustness'],
                    frozen_cpp_fast=self.multirate_cfg['frozen_cpp_fast'],
                    cpp_fast_certificate=self.multirate_cfg['cpp_fast_certificate'])


def scenario():
    cfg = task.selected_task_config()
    if (os.environ.get('X650_EXECUTION_ADAPTER') != 'cpp_full_pipeline_v1'
            or os.environ.get('AMSIM_SCENARIO_NAME') != cfg['scenario']):
        raise ValueError('Dedicated explicit multirate simulation required')
    model, provenance = task.preflight(cfg)
    out = Path(os.environ['AMSIM_RUN_DIR']).resolve()
    extension = CppMultirateExtension(cfg, provenance)
    try:
        extension.prepare(out)
        return shared.execute(cfg, model, provenance, out, extension=extension)
    finally:
        if hasattr(extension, 'procs'):
            extension.cleanup()
