"""Buoyancy checks for the finite pond and separated drone components."""
import unittest

import numpy as np

from physics_control import GRAVITY, load_config
from water_physics import buoyancy_wrench, component_samples


class WaterPhysicsTests(unittest.TestCase):
    def setUp(self):
        self.points, self.volumes = component_samples(load_config())
        self.water = {'density_kg_m3': 1000, 'gravity_m_s2': GRAVITY,
                      'center_m': [2, 3], 'radius_m': 2,
                      'surface_z_m': 0, 'bottom_z_m': -2}
        self.identity = np.eye(3)

    def test_water_is_on_by_default(self):
        water = load_config()['water']
        self.assertEqual(water['density_kg_m3'], 1000)
        self.assertEqual(water['gravity_m_s2'], GRAVITY)

    def test_dry_and_fully_submerged(self):
        dry, torque = buoyancy_wrench([2, 3, .5], self.identity,
                                      self.points, self.volumes, self.water)
        np.testing.assert_allclose(dry, 0)
        np.testing.assert_allclose(torque, 0)
        force, torque = buoyancy_wrench([2, 3, -1], self.identity,
                                        self.points, self.volumes, self.water)
        np.testing.assert_allclose(force, [0, 0, 1000 * GRAVITY * self.volumes.sum()])
        self.assertLess(np.linalg.norm(torque), .01)

    def test_component_enters_before_center(self):
        # The positive-X motor/arm is wet while the body center stays dry.
        rotation = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
        force, torque = buoyancy_wrench([2, 3, .1], rotation,
                                        self.points, self.volumes, self.water)
        self.assertGreater(abs(force[0]), 0)
        self.assertGreater(np.linalg.norm(torque), 0)
        self.assertLess(abs(force[0]), 1000 * GRAVITY * self.volumes.sum())
        outside, _ = buoyancy_wrench([5, 3, -1], self.identity,
                                     self.points, self.volumes, self.water)
        np.testing.assert_allclose(outside, 0)


if __name__ == '__main__':
    unittest.main()
