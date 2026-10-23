"""Same actual isolated execution, admitted normal trajectory reference only."""
import os
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_isolated_live_v1.run_case import LiveExtension, shared
from simulations.x650_isolated_figure8_v1 import task


class FigureEightExtension(LiveExtension):
    def __init__(self, cfg, provenance):
        super().__init__(task.baseline_config(cfg['frozen_hold']), provenance)
        self.trajectory_cfg = cfg

    def validate_runtime(self, cfg, model, provenance, out):
        _, checked = task.preflight(cfg)
        if checked != provenance or model.sha256 != provenance['model_sha256']:
            raise RuntimeError('Normal trajectory admission changed between launch and runtime')
        self.log_dir = Path(os.environ.get('AMSIM_TS20_LOG_DIR',''))
        if not self.log_dir.is_absolute() or not self.log_dir.is_dir():
            raise RuntimeError('Unique absolute actuator log directory required')

    def stage_gates(self,cfg,model_sha,source_sha):
        return task.validate_stage_gates(cfg,{'model_sha256':model_sha,'source_urdf_sha256':source_sha})

    def validate_activation_stage(self,cfg):
        # Expensive evidence checks already ran before arm; do not read/hash
        # hundreds of MB at the airborne handoff boundary.
        if cfg != self.trajectory_cfg or cfg['stage'] != 'normal':
            raise ValueError('Normal trajectory differs from preflight admission')

    def activate(self,ctx,loop,cfg,meta,rotation):
        start = super().activate(ctx,loop,cfg,meta,rotation)
        meta.update(figure8_protocol=cfg['figure8_protocol'],frozen_hold=cfg['frozen_hold'],
                    isolated_live_hold=cfg['isolated_live_hold'])
        return start


def scenario():
    if (os.environ.get('X650_EXECUTION_ADAPTER') != 'isolated_simulation_v1'
            or os.environ.get('AMSIM_SCENARIO_NAME') != task.SCENARIO):
        raise ValueError('Explicit dedicated isolated normal scenario required')
    cfg=task.selected_task_config();model,provenance=task.preflight(cfg)
    out=Path(os.environ['AMSIM_RUN_DIR']).resolve();extension=FigureEightExtension(cfg,provenance)
    try:
        extension.prepare(out)
        return shared.execute(cfg,model,provenance,out,extension=extension)
    finally:
        if hasattr(extension,'procs'):extension.cleanup()


if __name__=='__main__':scenario()
