"""Exercise the editable controller's motor and water force path."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from final_control import PhysicsController
from physics_control import load_config


class FinalControlTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        config = load_config()
        config['linear_drag_n_per_m_s'] = 0
        config['angular_drag_nm_per_rad_s'] = 0
        self.path = Path(self.temp.name) / 'vehicle.json'
        self.path.write_text(json.dumps(config))

    def test_individual_motor_and_buoyancy(self):
        with patch('final_control.your_controller', return_value=np.array([.4, .2, .2, .2])):
            dry = PhysicsController(self.path)
            dry_wrench = dry.update(0, [0, 0, .15], [0, 0, 0, 1], [0, 0, 0], [0, 0, 0], .002)
            wet = PhysicsController(self.path)
            wet_wrench = wet.update(0, [2, 3, -.2], [0, 0, 0, 1], [0, 0, 0], [0, 0, 0], .002)
        self.assertGreater(dry_wrench[3], 0)  # motor 1's +Y lever arm rolls +X
        self.assertLess(dry_wrench[4], 0)     # motor 1's +X lever arm pitches -Y
        self.assertGreater(wet_wrench[2], dry_wrench[2])
        self.assertGreater(wet_wrench[2] - dry_wrench[2], 5)

    def test_each_motor_produces_expected_roll_pitch_and_yaw(self):
        base_command = np.full(4, .2)
        with patch('final_control.your_controller', return_value=base_command):
            baseline = PhysicsController(self.path)
            base_wrench = np.asarray(baseline.update(
                0, [0, 0, .15], [0, 0, 0, 1], [0, 0, 0], [0, 0, 0], .04))

        for i in range(4):
            command = base_command.copy()
            command[i] = .4
            with self.subTest(motor=i + 1), patch('final_control.your_controller', return_value=command):
                controller = PhysicsController(self.path)
                wrench = np.asarray(controller.update(
                    0, [0, 0, .15], [0, 0, 0, 1], [0, 0, 0], [0, 0, 0], .04))
            table = controller.tables[i].data
            low = baseline.actual_command[i]
            high = controller.actual_command[i]
            delta_thrust = np.interp(high, table[:, 0], table[:, 1]) - np.interp(low, table[:, 0], table[:, 1])
            delta_reaction = np.interp(high, table[:, 0], table[:, 2]) - np.interp(low, table[:, 0], table[:, 2])
            x, y, _ = controller.motor_positions[i]
            expected = [0, 0, delta_thrust,
                        y * delta_thrust, -x * delta_thrust,
                        controller.yaw_signs[i] * delta_reaction]
            np.testing.assert_allclose(wrench - base_wrench, expected, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
