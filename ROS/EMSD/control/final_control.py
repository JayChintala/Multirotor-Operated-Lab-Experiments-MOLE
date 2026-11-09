"""Editable four-motor controller for the physical EMSD Gazebo simulation.

Run from the project root:
    python3 control/final_control.py
    python3 control/final_control.py --headless --duration 10

Edit reference_trajectory(), your_controller(), and optionally the inertia
override below. The Gazebo runner still supplies the same drone, terrain,
pond, measured motor data, motor lag, gravity, collision, and buoyancy.
"""
import argparse
import csv
import json
from pathlib import Path
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation

from drone_control import MotorTable
from physics_control import DEFAULT_CONFIG, GRAVITY, load_config
from water_physics import buoyancy_wrench, component_samples


# COORDINATES (right-handed, SI units)
# World: +X and +Y run along the ground; +Z points up. The pond is near
# world (X=2 m, Y=3 m). Position and linear velocity below use world axes.
# Body: origin at the drone's center of mass. Treat +X as forward and +Y as
# left in this controller (verify that convention against your physical frame).
# +Z points up, along all four motor thrust axes.
# A positive roll/pitch/yaw is a right-hand rotation about body X/Y/Z.
# Quaternion order is [x, y, z, w]; R maps body vectors into world vectors.
# Motor order: 1 (+X,+Y), 2 (-X,+Y), 3 (-X,-Y), 4 (+X,-Y).
# Commands are four numbers in [0,1], in that order. Command 0 is the lowest
# measured spinning setting; it does NOT mean motor off.


# OPTIONAL TODO: replace None with your measured 3x3 inertia matrix in kg m².
# It is about the body origin (center of mass), expressed in body X/Y/Z axes.
# Keep it symmetric and physically valid. This value changes Gazebo physics,
# not just the controller's internal estimate. None uses drone.json as-is.
# Example format: np.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])
INERTIA_MATRIX_KG_M2 = None


def reference_trajectory(t):
    """TODO: return your desired motion at simulation time t (seconds).

    Targets use world coordinates. Add velocity, acceleration, yaw, or any
    other fields your controller needs. This starter target holds the initial
    location. A target moves the drone only when your_controller uses it.
    """
    return {
        'position_m': np.array([0., 0., .15]),
        'velocity_m_s': np.zeros(3),
        'acceleration_m_s2': np.zeros(3),
        'yaw_rad': 0.,
    }


def your_controller(state, reference, vehicle):
    """TODO: compute one command for EACH motor; return shape (4,).

    This function is called every vehicle['control_period_s'] seconds of
    simulation time. Use any control method you like: geometric, LQR, MPC,
    PID, open loop, or your own algorithm. Keep persistent state in vehicle
    or another object if your algorithm needs memory.

    state contains time_s, position_m and velocity_m_s (world), quaternion_xyzw,
    rotation_body_to_world, and angular_velocity_body_rad_s. reference is the
    dictionary returned above. vehicle includes mass_kg, gravity_m_s2,
    inertia_matrix_kg_m2, motor_positions_body_m, yaw_reaction_signs,
    motor_tables, control_period_s, and hover_commands.

    The starter below commands approximate level hover, without feedback.
    Replace it with your controller to follow reference_trajectory().
    """
    return vehicle['hover_commands'].copy()


def motor_wrench(positions_body_m, thrust_n, reaction_nm, yaw_signs):
    """Return total body force and each motor's moment about the center of mass.

    Motor i applies +Z thrust at its own position. Its signed propeller
    reaction acts about body Z, so unequal motors can produce roll, pitch,
    and yaw simultaneously. Gazebo integrates the resulting rigid-body motion.
    """
    forces = np.column_stack((np.zeros((4, 2)), thrust_n))
    moments = np.cross(positions_body_m, forces)
    moments[:, 2] += yaw_signs * reaction_nm
    return forces.sum(axis=0), moments


class PhysicsController:
    """Bridge four requested motor commands to the existing Gazebo wrench API."""

    def __init__(self, filename=DEFAULT_CONFIG):
        self.config = c = load_config(filename)
        self.tables = [MotorTable(path) for path in c['motor_tables']]
        self.motor_positions = np.asarray(c['motor_positions_m']) - c['com_m']
        self.yaw_signs = np.asarray(c['yaw_signs'])
        inertia = np.diag(c['inertia_kg_m2'])
        ixy, ixz, iyz = c.get('inertia_products_kg_m2', [0, 0, 0])
        inertia[0, 1] = inertia[1, 0] = ixy
        inertia[0, 2] = inertia[2, 0] = ixz
        inertia[1, 2] = inertia[2, 1] = iyz
        hover_thrust = c['mass_kg'] * GRAVITY / 4
        if any(hover_thrust < table.data[0, 1] or hover_thrust > table.data[-1, 1]
               for table in self.tables):
            raise ValueError('Hover thrust is outside a motor table measured range')
        hover_commands = np.array([
            np.interp(hover_thrust, table.data[:, 1], table.data[:, 0])
            for table in self.tables])
        self.vehicle = {
            'mass_kg': c['mass_kg'], 'gravity_m_s2': GRAVITY,
            'inertia_matrix_kg_m2': inertia,
            'motor_positions_body_m': self.motor_positions,
            'yaw_reaction_signs': self.yaw_signs,
            'motor_tables': self.tables,
            'control_period_s': c['control_period_s'],
            'hover_commands': hover_commands,
        }
        self.command = np.zeros(4)
        self.actual_command = np.zeros(4)
        self.last_control = None
        self.last_time = None
        self.water = c['water']
        if self.water is not None:
            self.water_points, self.water_volumes = component_samples(c)
        self.log = None
        if c.get('telemetry_csv'):
            self.log = open(c['telemetry_csv'], 'w', newline='', buffering=1)
            self.writer = csv.writer(self.log)
            self.writer.writerow(['time_s', 'x_m', 'y_m', 'z_m',
                                  'ref_x_m', 'ref_y_m', 'ref_z_m',
                                  'motor_1', 'motor_2', 'motor_3', 'motor_4',
                                  'thrust_1_n', 'thrust_2_n', 'thrust_3_n', 'thrust_4_n',
                                  'fx_body_n', 'fy_body_n', 'fz_body_n',
                                  'tx_body_nm', 'ty_body_nm', 'tz_body_nm'] +
                                 [f'motor_{i}_{axis}_body_nm'
                                  for i in range(1, 5) for axis in ('tx', 'ty', 'tz')])

    def update(self, t, position, quaternion, velocity, rates, dt):
        """Called by Gazebo each physics step; return body force/torque (SI)."""
        if not np.isfinite(t) or not np.isfinite(dt) or dt <= 0:
            raise ValueError('Invalid simulation time or step')
        if self.last_time is not None and t < self.last_time - 1e-9:
            raise ValueError('Simulation time reset: restart final_control.py')
        self.last_time = t
        position = np.asarray(position, float)
        velocity = np.asarray(velocity, float)
        rates = np.asarray(rates, float)
        if any(v.shape != (3,) or not np.all(np.isfinite(v))
               for v in (position, velocity, rates)):
            raise ValueError('Invalid drone state')
        rotation = Rotation.from_quat(quaternion).as_matrix()
        reference = reference_trajectory(t)
        state = {
            'time_s': t, 'position_m': position, 'velocity_m_s': velocity,
            'quaternion_xyzw': np.asarray(quaternion),
            'rotation_body_to_world': rotation,
            'angular_velocity_body_rad_s': rates,
        }
        due = (self.last_control is None or
               t - self.last_control >= self.config['control_period_s'] - 1e-9)
        if due:
            candidate = np.asarray(your_controller(state, reference, self.vehicle), float)
            if (candidate.shape != (4,) or not np.all(np.isfinite(candidate)) or
                    np.any(candidate < 0) or np.any(candidate > 1)):
                raise ValueError('your_controller must return four finite commands in [0,1]')
            self.command = candidate
            self.last_control = t

        # Same measured motor tables and first-order motor lag as physics_control.py.
        alpha = -np.expm1(-dt / np.asarray(self.config['motor_time_constant_s']))
        self.actual_command += alpha * (self.command - self.actual_command)
        thrust = np.array([np.interp(command, table.data[:, 0], table.data[:, 1])
                           for command, table in zip(self.actual_command, self.tables)])
        reaction = np.array([np.interp(command, table.data[:, 0], table.data[:, 2])
                             for command, table in zip(self.actual_command, self.tables)])
        force_body, motor_moments = motor_wrench(
            self.motor_positions, thrust, reaction, self.yaw_signs)
        torque_body = motor_moments.sum(axis=0)

        # Gazebo itself applies gravity and contact. These are the other forces
        # already present in the normal physical simulation.
        force_body -= self.config['linear_drag_n_per_m_s'] * (rotation.T @ velocity)
        torque_body -= self.config['angular_drag_nm_per_rad_s'] * rates
        if self.water is not None:
            water_force, water_torque = buoyancy_wrench(
                position, rotation, self.water_points, self.water_volumes, self.water)
            force_body += water_force
            torque_body += water_torque
        if due and self.log:
            self.writer.writerow([t, *position, *reference['position_m'],
                                  *self.actual_command, *thrust,
                                  *force_body, *torque_body,
                                  *motor_moments.flat])
        return [*force_body, *torque_body]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG,
                        help='Vehicle data JSON; default control/config/drone.json')
    parser.add_argument('--headless', action='store_true',
                        help='Run the same Gazebo DART physics without the GUI')
    parser.add_argument('--duration', type=float, default=0,
                        help='Simulation seconds; 0 means run until stopped (30 s headless)')
    parser.add_argument('--prepare-only', action='store_true',
                        help='Build and generate the Gazebo world without starting it')
    args = parser.parse_args()
    args.controller_module = 'final_control'
    args.mode = 'custom'
    args.rates = [0., 0., 0.]
    args.roll_deg = args.pitch_deg = args.yaw_deg = 0.
    args.motor_commands = [0., 0., 0., 0.]

    from physics_sim import main as run_simulation
    if INERTIA_MATRIX_KG_M2 is None:
        return run_simulation(args)

    inertia = np.asarray(INERTIA_MATRIX_KG_M2, float)
    if inertia.shape != (3, 3) or not np.all(np.isfinite(inertia)) or not np.allclose(inertia, inertia.T):
        raise ValueError('INERTIA_MATRIX_KG_M2 must be a finite symmetric 3x3 matrix')
    config = load_config(args.config)
    config['inertia_kg_m2'] = np.diag(inertia).tolist()
    config['inertia_products_kg_m2'] = [float(inertia[0, 1]), float(inertia[0, 2]),
                                       float(inertia[1, 2])]
    with tempfile.TemporaryDirectory(prefix='emsd_final_control_') as directory:
        custom_config = Path(directory) / 'drone.json'
        custom_config.write_text(json.dumps(config))
        args.config = custom_config
        return run_simulation(args)


if __name__ == '__main__':
    main()
