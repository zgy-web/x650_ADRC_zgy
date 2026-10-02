from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path


BASE_TOPICS = [
    "/arm/joint_states",
    "/fmu/out/vehicle_local_position_v1",
    "/fmu/out/vehicle_attitude",
    "/fmu/out/vehicle_odometry",
    "/fmu/out/vehicle_status_v1",
    "/fmu/out/vehicle_control_mode",
    "/fmu/out/vehicle_land_detected",
    "/clock",
]


class RosbagRecorder:
    def __init__(
        self,
        folder: Path,
        joint_names: list[str],
        *,
        include_ground_truth: bool = True,
    ):
        self.folder = Path(folder)
        self.joint_names = list(joint_names)
        self.include_ground_truth = bool(include_ground_truth)
        self.process = None
        self._output = None

    @property
    def command_topics(self) -> list[str]:
        return [
            f"/arm/{joint}/cmd_vel"
            for joint in self.joint_names
        ]

    @property
    def required_topics(self) -> list[str]:
        topics = list(BASE_TOPICS)
        if self.include_ground_truth:
            topics.append("/ground_truth/odometry")
        return topics

    @property
    def topics(self) -> list[str]:
        # Command topics may have no publisher until user scenario code creates
        # ScenarioContext. rosbag discovers them later, so they are requested
        # but are not startup blockers.
        return self.required_topics + self.command_topics

    def start(self, timeout_s: float = 30.0):
        if self.process is not None:
            raise RuntimeError("rosbag recorder already started")

        self.folder.mkdir(parents=True, exist_ok=True)
        log_path = self.folder / "recorder.log"
        self._output = log_path.open("w")

        self.process = subprocess.Popen(
            [
                "ros2",
                "bag",
                "record",
                "-o",
                str(self.folder / "bag"),
                *self.topics,
            ],
            stdin=subprocess.DEVNULL,
            stdout=self._output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self._output.flush()
                text = log_path.read_text(errors="replace")
                raise RuntimeError(
                    f"rosbag recorder exited early ({self.process.returncode})\n"
                    + text[-5000:]
                )

            self._output.flush()
            text = log_path.read_text(errors="replace")
            subscribed = {
                topic
                for topic in self.required_topics
                if any(
                    "Subscribed to topic" in line and topic in line
                    for line in text.splitlines()
                )
            }
            if len(subscribed) == len(self.required_topics):
                return
            time.sleep(0.2)

        missing = sorted(set(self.required_topics) - subscribed)
        raise RuntimeError(
            "Timed out waiting for rosbag subscriptions: "
            + ", ".join(missing)
        )

    def stop(self, timeout_s: float = 20.0):
        if self.process is None:
            return

        try:
            if self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGINT)
                try:
                    self.process.wait(timeout=timeout_s)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    self.process.wait(timeout=5)
        finally:
            if self._output:
                self._output.close()
            self.process = None
            self._output = None
