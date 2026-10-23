"""Trajectory-only extension around the unchanged qualified hold constructor."""
import sys
from pathlib import Path
from .factory import create_engine, _pinned_json
from .engine import ReferenceEngine


class FigureEightEngine(ReferenceEngine):
    def __init__(self, base, parameters, configuration_sha256, trajectory_class):
        super().__init__(base.adapter,base.context,task_origin=base.task_origin,
            position_gains=base.position_gains,target_position=base.target_position,
            session_id=base.session_id,epoch=base.epoch,configuration_sha256=configuration_sha256,
            state_factory=base.state_factory,target_factory=base.target_factory,
            clock_ns=base.clock_ns,metadata=base._metadata)
        self.simulation_live=True
        self.live_reference_deferred=True
        self.trajectory_parameters=dict(parameters)
        self.trajectory_class=trajectory_class

    def begin_live(self,start):
        super().begin_live(start)
        # No solve has occurred; the actual anchor/histories were reset by the
        # exact hold handoff before both target consumers receive one reference.
        trajectory=self.trajectory_class(self.target_position.copy(),**self.trajectory_parameters)
        self.trajectory=self.adapter.allocator.trajectory=trajectory


def create_figure8_engine(repo_root,task_config,task_config_sha256,initialization_manifest,
                         initialization_sha256,session_id,epoch):
    root=Path(repo_root).resolve(strict=True)
    if str(root) not in sys.path:sys.path.insert(0,str(root))
    from simulations.x650_isolated_figure8_v1 import task
    cfg,_=_pinned_json(task_config,task_config_sha256)
    cfg=task.validate_task_config(cfg)
    gate=task.validate_live_hold(cfg)
    initialization,_=_pinned_json(initialization_manifest,initialization_sha256)
    if (initialization.get('simulation_live') is not True
            or initialization.get('deferred_actual_handoff') is not True
            or initialization.get('synthetic_software_fixture_only') is not True):
        raise ValueError('Normal worker requires constructor-only template and actual handoff')
    baseline=cfg['frozen_hold']
    base=create_engine(repo_root,baseline['path'],baseline['sha256'],initialization_manifest,
                       initialization_sha256,session_id,epoch)
    from simulations.x650_wholebody_mpc_v1.run_case import FigureEight
    engine=FigureEightEngine(base,cfg['trajectory_parameters'],bytes.fromhex(task_config_sha256),FigureEight)
    engine._metadata.update(task_config={'path':str(Path(task_config).resolve()),'sha256':task_config_sha256},
        frozen_hold=baseline,isolated_live_hold=gate,figure8_protocol=cfg['figure8_protocol'],
        scope='actual PX4/Gazebo normal figure eight; frozen hold numerics and new reference',
        physical_flight_qualification=False,constructor_only_template=True,actual_handoff_required=True)
    return engine
