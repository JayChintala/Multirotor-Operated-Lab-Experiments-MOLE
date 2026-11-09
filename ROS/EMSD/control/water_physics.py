"""Finite pond buoyancy for the rigid drone's frame, arms, and motors."""
import math

import numpy as np


def component_samples(config):
    """Return body-frame sample positions and displaced volume per sample.

    The component dimensions are provisional, like the drone collision geometry.
    Sampling each component separately lets an immersed arm or motor exert a
    force and a moment even when the centre of mass remains above the surface.
    """
    com = np.asarray(config['com_m'], dtype=float)
    motors = np.asarray(config['motor_positions_m'], dtype=float) - com
    points = []
    volumes = []

    def box(center, size, yaw=0):
        coordinates = [(np.arange(5) + .5) / 5 - .5] * 3
        grid = np.stack(np.meshgrid(*coordinates, indexing='ij'), -1).reshape(-1, 3)
        grid *= size
        c, s = math.cos(yaw), math.sin(yaw)
        grid[:, :2] = grid[:, :2] @ np.array([[c, s], [-s, c]])
        points.extend(grid + center)
        volumes.extend([np.prod(size) / len(grid)] * len(grid))

    box(-com, np.array([.14, .12, .04]))
    for motor in motors:
        yaw = math.atan2(motor[1], motor[0])
        arm_center = motor.copy()
        arm_center[:2] *= .55
        box(arm_center, np.array([.19, .035, .016]), yaw)
        # Motor housing volume; square samples outside the cylinder are omitted.
        xy = (np.arange(7) + .5) / 7 * .028 - .014
        samples = [(x, y, z) for x in xy for y in xy
                   if x*x + y*y <= .014**2
                   for z in (np.arange(5) + .5) / 5 * .02 - .01]
        points.extend(np.asarray(samples) + motor + [0, 0, .01])
        volumes.extend([math.pi * .014**2 * .02 / len(samples)] * len(samples))
    return np.asarray(points), np.asarray(volumes)


def buoyancy_wrench(position, rotation, sample_points, sample_volumes, water):
    """Body-frame force and torque from water occupying a bounded cylinder."""
    world_points = np.asarray(position) + sample_points @ rotation.T
    center = np.asarray(water['center_m'], dtype=float)
    inside = ((world_points[:, 0] - center[0])**2 +
              (world_points[:, 1] - center[1])**2 <= water['radius_m']**2)
    inside &= (world_points[:, 2] < water['surface_z_m'])
    inside &= (world_points[:, 2] >= water['bottom_z_m'])
    if not np.any(inside):
        return np.zeros(3), np.zeros(3)
    force_per_sample = water['density_kg_m3'] * water['gravity_m_s2'] * sample_volumes[inside]
    force_world = np.array([0., 0., force_per_sample.sum()])
    torque_world = np.array([np.dot(world_points[inside, 1] - position[1], force_per_sample),
                             -np.dot(world_points[inside, 0] - position[0], force_per_sample), 0.])
    return rotation.T @ force_world, rotation.T @ torque_world
