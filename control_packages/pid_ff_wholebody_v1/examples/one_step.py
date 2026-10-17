"""Run after installing the wheel or adding this package root to PYTHONPATH."""
import numpy as np
from flyarm_pid_ff import ControlState, EndpointTarget, PidFeedforwardController, tip_world

initial = ControlState([0., 0., 2.5], np.eye(3), [0., 0., 0.],
                       [0., 0., 0.], [0., 0.], [0., 0.], 10.)
controller = PidFeedforwardController(initial)
target = EndpointTarget(tip_world(initial.position, initial.rotation, initial.q))
command = controller.step(0., .02, initial, target)
px4 = command.px4_setpoint([0., 0., -2.5], [0., 0., 0.])
print("PX4 position NED:", px4.position)
print("PX4 velocity NED:", px4.velocity)
print("PX4 acceleration NED:", px4.acceleration)
print("Joint velocity rad/s:", command.joint_velocity)
print("PID SHA256:", controller.metadata()["pid_sha256"])
