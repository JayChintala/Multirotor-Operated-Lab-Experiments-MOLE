"""Synthetic fixtures validate conventions; these are not vehicle measurements."""
import tempfile
import unittest
from pathlib import Path
import numpy as np
from drone_control import Allocator, MotorTable, RatePID, State, Setpoint


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'synthetic.csv'
        self.path.write_text('command,thrust_n,torque_nm,current_a\n0,0,0,0\n0.5,5,0.05,2\n1,10,0.1,5\n')
        self.table = MotorTable(self.path)
        self.allocator = Allocator([[.2,.1,0],[-.2,.1,0],[-.2,-.1,0],[.2,-.1,0]],
                                   [1,-1,1,-1], [self.table]*4, [1,.1,.1,.01])

    def test_hover_and_current(self):
        result = self.allocator.allocate([20,0,0,0])
        np.testing.assert_allclose(result.thrust_n, 5, atol=1e-6)
        np.testing.assert_allclose(result.current_a, 2, atol=1e-6)
        self.assertTrue(result.converged)

    def test_roll_pitch_and_yaw(self):
        for wrench in ([20,.2,0,0], [20,0,.2,0], [20,0,0,.04]):
            result = self.allocator.allocate(wrench)
            np.testing.assert_allclose(result.achieved_wrench, wrench, atol=1e-5)

    def test_infeasible_thrust(self):
        result = self.allocator.allocate([100,0,0,0])
        self.assertTrue(np.all(result.command <= 1))
        self.assertLess(result.residual[0], -59)

    def test_reject_table_and_extrapolation(self):
        with self.assertRaises(ValueError):
            self.table.at_thrust(11)
        self.path.write_text('command,thrust_n,torque_nm,current_a\n0,0,0,0\n1,0,1,1\n')
        with self.assertRaises(ValueError):
            MotorTable(self.path)

    def test_measured_table_stays_in_measured_range(self):
        table = MotorTable(Path(__file__).with_name('motor_data.csv'))
        self.assertEqual(len(table.data), 41)
        self.assertGreater(table.data[0, 1], 0)
        with self.assertRaises(ValueError):
            table.at_thrust(0)
        allocator = Allocator([[.2,.1,0],[-.2,.1,0],[-.2,-.1,0],[.2,-.1,0]],
                              [1,-1,1,-1], [table]*4, [1,.1,.1,.01])
        result = allocator.allocate([0,0,0,0])
        self.assertTrue(np.all(result.thrust_n >= table.data[0, 1]))
        self.assertGreater(result.residual[0], 0)
        result = allocator.allocate([10,0,0,0])
        np.testing.assert_allclose(result.achieved_wrench, [10,0,0,0], atol=1e-5)

    def test_pid_limits_and_reset(self):
        controller = RatePID(1,1,0,.5,.1)
        for _ in range(20):
            output = controller.update(State(np.zeros(3)), Setpoint(np.ones(3),10), .01)
        np.testing.assert_allclose(output[1:], .1)
        np.testing.assert_allclose(controller.integral, 0)
        controller.reset()
        with self.assertRaises(ValueError):
            controller.update(State(np.zeros(3)), Setpoint(np.zeros(3),10),0)


if __name__ == '__main__':
    unittest.main()
