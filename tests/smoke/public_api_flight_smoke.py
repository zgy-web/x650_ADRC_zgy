#!/usr/bin/env python3
import json

from amsim import ScenarioContext


def main():
    with ScenarioContext() as ctx:
        ctx.vehicle.wait_ready()
        target_z = ctx.vehicle.takeoff(2.0, stable_s=2.0)
        obs_air = ctx.observe(ground_truth=False)
        ctx.wait(2.0)
        ctx.vehicle.land()
        obs_land = ctx.observe(ground_truth=False)

        result = {
            "target_z": target_z,
            "airborne_estimate": {
                "z": obs_air.vehicle.z,
                "vz": obs_air.vehicle.vz,
                "landed": obs_air.vehicle.landed,
                "armed": obs_air.vehicle.armed,
            },
            "landed_estimate": {
                "z": obs_land.vehicle.z,
                "landed": obs_land.vehicle.landed,
                "armed": obs_land.vehicle.armed,
            },
            "joints": list(obs_land.arm.keys()),
        }
        print(json.dumps(result, indent=2))

        if obs_air.vehicle.landed:
            raise RuntimeError("Vehicle API takeoff did not become airborne")
        if not obs_land.vehicle.landed or obs_land.vehicle.armed:
            raise RuntimeError("Vehicle API land did not finish landed/disarmed")


if __name__ == "__main__":
    main()
