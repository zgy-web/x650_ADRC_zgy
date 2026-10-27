"""Upper NMPC / callable MPC on the same actual-feedback execution stack."""
import os
from copy import deepcopy
from pathlib import Path

from simulations.x650_cpp_live_v1.run_case import (
    CppLiveExtension, ExternalCppAllocator, expected_core_config, shared, sha)
from simulations.x650_cpp_multirate_v1 import profile
from . import task


class UpperExtension(CppLiveExtension):
    def __init__(self, cfg, provenance):
        self.upper_cfg = cfg
        adapter = deepcopy(task.baseline_config(cfg))
        adapter['timing_profile'] = profile.NAME
        super().__init__(adapter, provenance)

    def expected_worker_implementation(self):
        return ('cpp_upper_nmpc_v1' if self.upper_cfg['upper_controller']['backend'] == 'nmpc'
                else 'cpp_callable_mpc_v1')

    def worker_command(self, params):
        path = Path(os.environ['FLYARM_UPPER_OPTIONS']).resolve(strict=True)
        expected = os.environ['FLYARM_UPPER_OPTIONS_SHA256']
        options = task.read(path)
        if (sha(path) != expected or options['task_config_sha256'] != self.task_sha
                or options['backend'] != self.upper_cfg['upper_controller']['backend']):
            raise ValueError('Upper worker backend/options/task binding differs')
        self.upper_options = options
        return self._ros('flyarm_nmpc_cpp', 'isolated_worker', dict(params,
            timing_profile=profile.NAME, nmpc_options_manifest=path,
            nmpc_options_sha256=expected))

    def validate_trajectory_manifest(self):
        if self.cpp_manifest['trajectory'] != task.trajectory(self.upper_cfg):
            raise ValueError('Worker reference differs from selected upper experiment')

    def source_trajectory_parameters(self):
        return task.source_parameters(self.upper_cfg)

    def validate_runtime(self, cfg, model, provenance, out):
        _, checked = task.preflight(cfg)
        if cfg != self.upper_cfg or checked != provenance or model.sha256 != provenance['model_sha256']:
            raise RuntimeError('Upper task or physical preparation changed')
        self.log_dir = Path(os.environ.get('AMSIM_TS20_LOG_DIR', ''))
        if not self.log_dir.is_absolute() or not self.log_dir.is_dir():
            raise RuntimeError('Unique absolute actual actuator log directory required')

    def stage_gates(self, cfg, model_sha, source_sha):
        return task.validate_stage_gates(cfg, {'model_sha256': model_sha, 'source_urdf_sha256': source_sha})

    def validate_activation_stage(self, cfg):
        if cfg != self.upper_cfg or cfg['stage'] not in {'tip_hold', 'normal', 'fast'}:
            raise ValueError('Activation differs from independently admitted task')

    def _prepare_allocator(self, loop):
        binding = self.provenance['base_response_prior']
        if sha(binding['path']) != binding['sha256']:
            raise RuntimeError('Original response prior changed')
        expected = expected_core_config(self.upper_cfg, loop.model, loop.pid,
            self.predictor, task.read(binding['path']))
        if expected != self.cpp_manifest['config']:
            raise RuntimeError('Actual plant/PID/predictor does not match upper controller configuration')
        return ExternalCppAllocator(loop.model, loop.pid, self.cpp_manifest, self.procs['worker'].pid)

    def _prepare_additional_transport(self, ctx, loop, meta):
        super()._prepare_additional_transport(ctx, loop, meta)
        meta['isolated_execution'].update(active_controller=self.expected_worker_implementation(),
            numerical_policy='Same explicit plant, weights, limits and 25/50 Hz execution profile; selected upper backend',
            upper_backend=self.upper_cfg['upper_controller']['backend'])

    def handle_active_pump(self, loop, sim):
        handled = super().handle_active_pump(loop, sim)
        if handled and self.upper_cfg['upper_controller']['case_id'] != 'hold':
            loop.recorder.tip_target = loop.trajectory.sample(sim-loop.task_start)[0].copy()
        return handled

    def finalize(self, meta, stage, out):
        super().finalize(meta, stage, out)
        for doc in (meta, stage):
            doc.update(upper_controller=deepcopy(self.upper_cfg['upper_controller']),
                active_controller=self.expected_worker_implementation(), timing_profile=profile.NAME,
                effective_timing_profile=profile.effective(), hardware_qualified=False,
                physical_acceptance_inherited=False)


def scenario():
    cfg = task.selected_task_config()
    if (os.environ.get('X650_EXECUTION_ADAPTER') != 'cpp_full_pipeline_v1'
            or os.environ.get('AMSIM_SCENARIO_NAME') != cfg['scenario']):
        raise ValueError('Explicit upper-controller scenario required')
    model, provenance = task.preflight(cfg)
    out = Path(os.environ['AMSIM_RUN_DIR']).resolve()
    extension = UpperExtension(cfg, provenance)
    try:
        extension.prepare(out)
        return shared.execute(cfg, model, provenance, out, extension=extension)
    finally:
        if hasattr(extension, 'procs'):
            extension.cleanup()
