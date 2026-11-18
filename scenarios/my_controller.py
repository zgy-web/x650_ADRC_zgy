"""User experiment.

The platform owns PX4/Gazebo startup, rendering, readiness checks, logging,
shutdown, and ULog archival. Experiment logic uses the stable public API.
"""

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
