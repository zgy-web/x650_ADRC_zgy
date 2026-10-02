from __future__ import annotations

import json
import os

import rclpy

from .arm import ArmAPI
from .node import ApiNode
from .vehicle import VehicleAPI
from ..runtime.process import status as runtime_status


class ScenarioContext:
    def __init__(self):
        joints_env = os.environ.get("AMSIM_MODEL_JOINTS")
        if joints_env is not None:
            try:
                joints = json.loads(joints_env)
            except json.JSONDecodeError as exc:
                raise RuntimeError("Invalid AMSIM_MODEL_JOINTS") from exc
        else:
            joints = None
            state = runtime_status()
            meta_path = state.get("runtime_meta") if state.get("running") else None
            if meta_path:
                try:
                    meta = json.loads(open(meta_path).read())
                    value = meta.get("joint_names")
                    if isinstance(value, list) and value:
                        joints = [str(x) for x in value]
                except (OSError, json.JSONDecodeError):
                    pass
            if joints is None:
                joints = ["arm_joint_1", "arm_joint_2"]

        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()

        self.node = ApiNode(joints)
        self.vehicle = VehicleAPI(self.node)
        self.arm = ArmAPI(self.node)
        self.node.wait_for_data(timeout_s=45.0, require_truth=True)

    def observe(self, *, ground_truth: bool = False):
        for _ in range(3):
            self.node.spin_once()
        return self.node.observation(include_ground_truth=ground_truth)

    def wait(self, sim_seconds: float):
        if sim_seconds < 0:
            raise ValueError("sim_seconds must be >= 0")
        start = self.node.sim_time_s()
        while self.node.sim_time_s() - start < sim_seconds:
            self.node.spin_once()
            self.node.zero_all()

    def close(self):
        try:
            self.arm.stop()
        except Exception:
            pass
        try:
            self.vehicle.close()
        finally:
            self.node.close()
            if self._owns_rclpy and rclpy.ok():
                rclpy.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
