from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import yaml

from .paths import PROJECT_ROOT

BUILTIN_CONFIG_DIR = PROJECT_ROOT / "configs" / "scenarios"
USER_SCENARIO_DIR = PROJECT_ROOT / "scenarios"


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    config_path: Path
    module: str
    config: dict
    source: str


def _validate_name(name: str) -> str:
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]*", name):
        raise ValueError(
            "Scenario name must start with a letter and contain only letters, "
            "numbers, '_' or '-'."
        )
    return name


def discover() -> dict[str, ScenarioSpec]:
    found: dict[str, ScenarioSpec] = {}

    if BUILTIN_CONFIG_DIR.is_dir():
        for path in sorted(BUILTIN_CONFIG_DIR.glob("*.yaml")):
            data = yaml.safe_load(path.read_text()) or {}
            name = str(data.get("name") or path.stem)
            _validate_name(name)
            module = data.get("module") or f"amsim.scenarios.{name}"

            if name in found:
                raise RuntimeError(f"Duplicate scenario name: {name}")

            found[name] = ScenarioSpec(
                name=name,
                config_path=path,
                module=str(module),
                config=data,
                source="builtin",
            )

    if USER_SCENARIO_DIR.is_dir():
        for path in sorted(USER_SCENARIO_DIR.glob("*.yaml")):
            data = yaml.safe_load(path.read_text()) or {}
            name = str(data.get("name") or path.stem)
            _validate_name(name)
            module = data.get("module") or str(Path("scenarios") / f"{name}.py")

            if name in found:
                raise RuntimeError(
                    f"User scenario '{name}' conflicts with built-in scenario."
                )

            found[name] = ScenarioSpec(
                name=name,
                config_path=path,
                module=str(module),
                config=data,
                source="user",
            )

    return found


def get(name: str) -> ScenarioSpec:
    scenarios = discover()
    if name not in scenarios:
        available = ", ".join(sorted(scenarios)) or "(none)"
        raise KeyError(f"Unknown scenario '{name}'. Available: {available}")
    return scenarios[name]


def create_template(name: str) -> tuple[Path, Path]:
    _validate_name(name)
    USER_SCENARIO_DIR.mkdir(parents=True, exist_ok=True)
    py_path = USER_SCENARIO_DIR / f"{name}.py"
    yaml_path = USER_SCENARIO_DIR / f"{name}.yaml"

    if py_path.exists() or yaml_path.exists():
        raise FileExistsError(f"Scenario already exists: {name}")

    py_path.write_text(
        """\"\"\"User experiment.

The platform owns PX4/Gazebo startup, rendering, readiness checks, logging,
shutdown, and ULog archival. Experiment logic uses the stable public API.
\"\"\"

from amsim import ScenarioContext


def main():
    with ScenarioContext() as ctx:
        ctx.vehicle.takeoff(2.5)

        # Example arm command:
        # ctx.arm.move_position(
        #     "arm_joint_1",
        #     -5.0,
        #     unit="deg",
        #     max_speed=0.5,
        #     speed_unit="deg/s",
        # )

        obs = ctx.observe()
        print(obs)

        ctx.wait(2.0)
        ctx.vehicle.land()


if __name__ == "__main__":
    main()
"""
    )

    yaml_path.write_text(
        f"""schema_version: 1
name: {name}
module: scenarios/{name}.py
backend: px4_sitl
model: x500_underfold
visualization:
  default: gui
record:
  rosbag: true
  px4_ulog: true
  ground_truth: true
"""
    )
    return py_path, yaml_path
