from __future__ import annotations

import rclpy

from .arm import ArmAPI
from .node import ApiNode
from .vehicle import VehicleAPI
from ..runtime.model_metadata import resolve_active_model


class ScenarioContext:
    def __init__(self):
        self.model = resolve_active_model()
        joints = self.model["joint_names"]

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
