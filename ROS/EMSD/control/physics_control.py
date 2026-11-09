"""Closed-loop control called by Gazebo's native force plugin, in simulation time.

Axes: world Z up, body +Z thrust; quaternions x/y/z/w. Ground-truth feedback
is intentional: no sensor noise, delay, or state estimator is claimed.
"""
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from drone_control import Allocator, MotorTable, RatePID, Setpoint, State
from water_physics import buoyancy_wrench, component_samples

GRAVITY = 9.80665
DEFAULT_CONFIG = Path(__file__).with_name('config') / 'drone.json'
DEFAULT_WATER = Path(__file__).with_name('config') / 'water.json'


def load_config(filename=DEFAULT_CONFIG):
    filename = Path(filename).resolve()
    config = json.loads(filename.read_text())
    # Water is part of the simulation environment, including when the
    # controller is constructed directly instead of through physics_sim.py.
    if 'water' not in config:
        config['water'] = json.loads(DEFAULT_WATER.read_text())
    if config['water'] is not None:
        water = config['water']
        water['gravity_m_s2'] = GRAVITY
        if (not np.isfinite(water['density_kg_m3']) or water['density_kg_m3'] <= 0 or
                not np.isfinite(water['radius_m']) or water['radius_m'] <= 0 or
                not np.all(np.isfinite(water['center_m'])) or len(water['center_m']) != 2 or
                not np.isfinite(water['surface_z_m']) or
                not np.isfinite(water['bottom_z_m']) or
                water['bottom_z_m'] >= water['surface_z_m']):
            raise ValueError('Invalid water density or pond dimensions')
    shapes = {'com_m': (3,), 'inertia_kg_m2': (3,),
              'motor_positions_m': (4, 3), 'yaw_signs': (4,),
              'motor_time_constant_s': (4,)}
    for key, shape in shapes.items():
        a = np.asarray(config[key], dtype=float)
        if a.shape != shape or not np.all(np.isfinite(a)):
            raise ValueError(f'{key}: expected finite array of shape {shape}')
    for key in ('mass_kg', 'physics_step_s', 'control_period_s'):
        if not np.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    tensor = np.diag(config['inertia_kg_m2'])
    products = np.asarray(config.get('inertia_products_kg_m2', [0,0,0]), float)
    if products.shape != (3,) or not np.all(np.isfinite(products)):
        raise ValueError('Expected finite inertia products Ixy/Ixz/Iyz')
    tensor[0,1] = tensor[1,0] = products[0]
    tensor[0,2] = tensor[2,0] = products[1]
    tensor[1,2] = tensor[2,1] = products[2]
    inertia = np.linalg.eigvalsh(tensor)
    if np.any(inertia <= 0) or np.any(2 * inertia > inertia.sum()):
        raise ValueError('Principal inertias must be positive and obey triangle inequalities')
    if np.any(np.asarray(config['motor_time_constant_s']) <= 0):
        raise ValueError('Motor time constants must be positive')
    if not np.all(np.isin(config['yaw_signs'], [-1, 1])):
        raise ValueError('Yaw reaction signs must be +1 or -1')
    if config['control_period_s'] < config['physics_step_s']:
        raise ValueError('Control period must be at least one physics step')
    for key in ('linear_drag_n_per_m_s', 'angular_drag_nm_per_rad_s'):
        if not np.isfinite(config[key]) or config[key] < 0:
            raise ValueError(f'{key} must be finite and nonnegative')
    for key in ('attitude_kp', 'position_kp', 'velocity_kd'):
        a = np.asarray(config[key], float)
        if a.shape != (3,) or not np.all(np.isfinite(a)) or np.any(a < 0):
            raise ValueError(f'{key}: expected three finite nonnegative gains')
    if not np.isfinite(config['max_tilt_deg']) or not 0 < config['max_tilt_deg'] < 80:
        raise ValueError('max_tilt_deg must be between 0 and 80 degrees')
    if len(config['motor_tables']) != 4:
        raise ValueError('Four motor tables are required')
    config['motor_tables'] = [str((filename.parent / p).resolve())
                              for p in config['motor_tables']]
    ref = config['reference']
    if not np.all(np.isfinite([ref['altitude_m'], ref['takeoff_s'], ref['period_s'],
                               ref['path_start_s'], ref['yaw_deg'], *ref['amplitude_m']])):
        raise ValueError('Reference parameters must be finite')
    if ref['takeoff_s'] <= 0 or ref['period_s'] <= 0 or ref['path_start_s'] < ref['takeoff_s']:
        raise ValueError('Invalid reference timing')
    return config


def reference(t, config):
    """Position, velocity, acceleration reference; never writes a vehicle pose."""
    ref = config['reference']
    duration = ref['takeoff_s']
    s = np.clip(t / duration, 0, 1)
    height = ref['altitude_m'] - 0.15
    # Quintic blend has zero velocity and acceleration at both ends.
    b = 10*s**3 - 15*s**4 + 6*s**5
    db = (30*s**2 - 60*s**3 + 30*s**4) / duration if t < duration else 0
    ddb = (60*s - 180*s**2 + 120*s**3) / duration**2 if t < duration else 0
    p = np.array([0., 0., 0.15 + height*b])
    v = np.array([0., 0., height*db])
    a = np.array([0., 0., height*ddb])
    if t >= ref['path_start_s']:
        elapsed = t - ref['path_start_s']
        s = min(elapsed / 3, 1)
        b = 10*s**3 - 15*s**4 + 6*s**5
        db = (30*s**2 - 60*s**3 + 30*s**4) / 3 if elapsed < 3 else 0
        ddb = (60*s - 180*s**2 + 120*s**3) / 9 if elapsed < 3 else 0
        w = 2*np.pi / ref['period_s']
        for i, frequency in enumerate((w, 2*w)):
            amplitude = ref['amplitude_m'][i]
            sn, cs = np.sin(frequency*elapsed), np.cos(frequency*elapsed)
            p[i] = amplitude*b*sn
            v[i] = amplitude*(db*sn + b*frequency*cs)
            a[i] = amplitude*(ddb*sn + 2*db*frequency*cs - b*frequency**2*sn)
    return p, v, a


class PhysicsController:
    def __init__(self, filename=DEFAULT_CONFIG):
        self.config = c = load_config(filename)
        self.water = c.get('water')
        if self.water:
            self.water_points, self.water_volumes = component_samples(c)
        self.mode = c.get('mode', 'trajectory')
        if self.mode not in ('trajectory', 'attitude', 'rates', 'motors', 'freefall'):
            raise ValueError('Unknown simulation mode')
        self.tables = [MotorTable(path) for path in c['motor_tables']]
        self.positions = np.asarray(c['motor_positions_m']) - c['com_m']
        self.signs = np.asarray(c['yaw_signs'])
        self.allocator = Allocator(self.positions, self.signs, self.tables, c['wrench_scales'])
        self.pid = RatePID(c['rate_kp'], c['rate_ki'], c['rate_kd'],
                           c['integral_limit'], c['torque_limit_nm'])
        self.command = np.zeros(4)
        self.actual_command = np.zeros(4)
        self.thrust = np.zeros(4)
        self.reaction = np.zeros(4)
        self.current = np.zeros(4)
        self.requested = np.zeros(4)
        self.residual = np.zeros(4)
        self.last_control = None
        self.previous_time = None
        self.log = None
        if c.get('telemetry_csv'):
            self.log = open(c['telemetry_csv'], 'w', newline='', buffering=1)
            self.writer = csv.writer(self.log)
            self.writer.writerow(['time_s', 'x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad',
                                  'p_rad_s', 'q_rad_s', 'r_rad_s',
                                  'ref_x_m', 'ref_y_m', 'ref_z_m'] +
                                 [f'command_{i}' for i in range(1, 5)] +
                                 [f'thrust_{i}_n' for i in range(1, 5)] +
                                 [f'reaction_{i}_nm' for i in range(1, 5)] +
                                 ['fz_n', 'tx_nm', 'ty_nm', 'tz_nm', 'current_a',
                                  'residual_fz_n', 'residual_tx_nm', 'residual_ty_nm', 'residual_tz_nm'])

    def update(self, t, position, quaternion, velocity, rates, dt):
        c = self.config
        if dt <= 0 or not np.all(np.isfinite([t, dt])):
            raise ValueError('Invalid simulation time')
        if self.previous_time is not None and t < self.previous_time - 1e-9:
            raise ValueError('Simulation time reset: restart the runner')
        self.previous_time = t
        position, velocity, rates = (np.asarray(x, float) for x in (position, velocity, rates))
        if any(x.shape != (3,) or not np.all(np.isfinite(x)) for x in (position, velocity, rates)):
            raise ValueError('Invalid physics state')
        rotation = Rotation.from_quat(quaternion)
        R = rotation.as_matrix()
        ref_p, ref_v, ref_a = reference(t, c)
        control_due = self.last_control is None or t - self.last_control >= c['control_period_s'] - 1e-9
        if control_due:
            control_dt = c['control_period_s'] if self.last_control is None else t-self.last_control
            self.last_control = t
            if self.mode in ('freefall', 'motors'):
                self.command = np.asarray(c.get('motor_commands', [0.25]*4), float)
                if self.command.shape != (4,) or not np.all(np.isfinite(self.command)) or np.any((self.command < 0) | (self.command > 1)):
                    raise ValueError('Four motor commands must be in [0,1]')
            else:
                acceleration = ref_a + np.asarray(c['position_kp'])*(ref_p-position) + np.asarray(c['velocity_kd'])*(ref_v-velocity)
                acceleration[2] = np.clip(acceleration[2], -0.7*GRAVITY, GRAVITY)
                force = c['mass_kg']*(acceleration + [0, 0, GRAVITY])
                max_horizontal = force[2]*np.tan(np.radians(c['max_tilt_deg']))
                horizontal = np.linalg.norm(force[:2])
                if horizontal > max_horizontal:
                    force[:2] *= max_horizontal/horizontal
                yaw = np.radians(c['reference']['yaw_deg'])
                z_axis = force / np.linalg.norm(force)
                y_axis = np.cross(z_axis, [np.cos(yaw), np.sin(yaw), 0])
                y_axis /= np.linalg.norm(y_axis)
                desired_R = np.column_stack((np.cross(y_axis, z_axis), y_axis, z_axis))
                thrust = max(0., force @ R[:, 2])
                if self.mode in ('attitude', 'rates'):
                    desired_R = Rotation.from_euler('xyz', c.get('attitude_deg', [0,0,0]), degrees=True).as_matrix()
                    thrust = force[2]/max(0.3, R[2,2])
                error_matrix = (desired_R.T@R - R.T@desired_R)/2
                error = np.array([error_matrix[2,1], error_matrix[0,2], error_matrix[1,0]])
                desired_rates = -np.asarray(c['attitude_kp'])*error
                if self.mode == 'rates':
                    desired_rates = np.asarray(c.get('rates_rad_s', [0,0,0]))
                desired_rates = np.clip(desired_rates, -2., 2.)
                self.requested = self.pid.update(State(rates), Setpoint(desired_rates, thrust), control_dt)
                allocation = self.allocator.allocate(self.requested)
                self.command = allocation.command
                self.residual = allocation.residual
                if not allocation.converged:
                    raise RuntimeError('Motor allocation failed to converge')
        # Assumed first-order command dynamics, distinct from measured static maps.
        alpha = -np.expm1(-dt / np.asarray(c['motor_time_constant_s']))
        self.actual_command += alpha*(self.command-self.actual_command)
        for i, table in enumerate(self.tables):
            self.thrust[i], self.reaction[i], self.current[i] = [
                np.interp(self.actual_command[i], table.data[:,0], table.data[:,j]) for j in (1,2,3)]
        if self.mode == 'freefall':
            # Explicitly disconnected propulsion, not a fabricated lookup row.
            self.thrust[:] = self.reaction[:] = self.current[:] = 0
        motor_forces = np.column_stack((np.zeros((4,2)), self.thrust))
        torque = np.cross(self.positions, motor_forces).sum(axis=0)
        torque[2] += self.signs @ self.reaction
        force_body = motor_forces.sum(axis=0)
        if control_due and self.log:
            self.writer.writerow([t, *position, *rotation.as_euler('xyz'), *rates, *ref_p,
                                  *self.actual_command, *self.thrust, *(self.signs*self.reaction),
                                  force_body[2], *torque, self.current.sum(), *self.residual])
        # Linear drag is defined in world coordinates; isotropic, converted to body.
        force_body -= c['linear_drag_n_per_m_s']*(R.T@velocity)
        torque -= c['angular_drag_nm_per_rad_s']*rates
        if self.water:
            water_force, water_torque = buoyancy_wrench(
                position, R, self.water_points, self.water_volumes, self.water)
            force_body += water_force
            torque += water_torque
        return [*force_body, *torque]
