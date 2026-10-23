"""Explicit live boundary around the byte-frozen qualified constructor."""
from .factory import create_engine, _pinned_json


def create_live_engine(repo_root, task_config, task_config_sha256, initialization_manifest,
                       initialization_sha256, session_id, epoch):
    cfg, _ = _pinned_json(task_config, task_config_sha256)
    if cfg.get('scenario') == 'x650_isolated_figure8_v1':
        from .figure8_factory import create_figure8_engine
        return create_figure8_engine(repo_root, task_config, task_config_sha256,
            initialization_manifest, initialization_sha256, session_id, epoch)
    initialization, _ = _pinned_json(initialization_manifest, initialization_sha256)
    deferred = initialization.get('deferred_actual_handoff') is True
    if (initialization.get('simulation_live') is not True
            or initialization.get('synthetic_software_fixture_only') is not deferred):
        raise ValueError('Live simulation requires explicit actual prestream initialization')
    engine = create_engine(repo_root, task_config, task_config_sha256, initialization_manifest,
                           initialization_sha256, session_id, epoch)
    engine.simulation_live = True
    engine.live_reference_deferred = deferred
    engine._metadata.update(scope='explicit PX4/Gazebo simulation; not hardware',
                            physical_flight_qualification=False,
                            constructor_only_template=deferred, actual_handoff_required=True)
    return engine
