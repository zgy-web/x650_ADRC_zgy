from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from x650_wholebody.api import ControlState, ControlCommand, enu_to_ned


class ApiContractTests(unittest.TestCase):
    def state(self):
        return ControlState([1., 2., 3.], np.eye(3), [.2, -.1, .05], [0, 0, 0], [0, 0], [0, 0], 1.)

    def test_servo_reconstruction_and_frame_bridge(self):
        state = self.state()
        gains = np.array([2., 2., 1.])
        desired = np.array([.3, -.2, .1])
        command = ControlCommand.from_executed_velocity(state, desired, [.1, -.05],
                    nominal_position=[1.1, 1.9, 3.2], acceleration_feedforward=[.1, -.3, .2],
                    position_gains=gains)
        reconstructed = gains*(command.position_reference-state.position)+command.velocity_feedforward
        np.testing.assert_allclose(reconstructed, desired, atol=1e-14)
        ep, ev = np.array([4., 5., 6.]), np.array([.6, .7, .8])
        sp = command.px4_setpoint(ep, ev)
        np.testing.assert_allclose(sp.position-ep, enu_to_ned(command.position_reference-state.position))
        np.testing.assert_allclose(sp.velocity-ev, enu_to_ned(command.velocity_feedforward-state.velocity))
        np.testing.assert_allclose(sp.acceleration, [-.3, .1, -.2])
        np.testing.assert_allclose(command.joint_velocity, [.1, -.05])
        self.assertEqual(sp.type_mask, 2048)

    def test_snapshot_copies_and_rejects_invalid_frames(self):
        source = np.zeros(3)
        state = ControlState(source, np.eye(3), source, source, [0, 0], [0, 0], 0.)
        source[0] = 9
        self.assertEqual(state.position[0], 0.)
        with self.assertRaises(ValueError):
            state.position[0] = 2
        with self.assertRaises(ValueError):
            ControlState([0]*3, np.diag([-1., 1., 1.]), [0]*3, [0]*3, [0]*2, [0]*2, 0.)
        with self.assertRaises(ValueError):
            ControlState([0]*3, np.eye(3), [0]*3, [0]*3, [0]*2, [0]*2, float('nan'))

    def test_actual_position_gains_are_required(self):
        with self.assertRaises(ValueError):
            ControlCommand.from_executed_velocity(self.state(), [0]*3, [0]*2,
                nominal_position=[0]*3, acceleration_feedforward=[0]*3, position_gains=[0, 2, 1])


if __name__ == '__main__':
    unittest.main()
