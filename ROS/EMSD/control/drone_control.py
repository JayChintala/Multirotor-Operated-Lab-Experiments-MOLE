"""Offline controller building blocks. SI units; body axes right-handed, thrust +Z."""
from dataclasses import dataclass
from typing import Protocol
import csv
import numpy as np
from scipy.optimize import least_squares


@dataclass
class State:
    rates_rad_s: np.ndarray


@dataclass
class Setpoint:
    rates_rad_s: np.ndarray
    thrust_n: float


class Controller(Protocol):
    def update(self, state: State, target: Setpoint, dt: float) -> np.ndarray:
        """Return [total thrust N, body torque X/Y/Z Nm]."""
        ...


class RatePID:
    """Body-rate PID. Outer attitude/position loops can produce its setpoint."""
    def __init__(self, kp, ki, kd, integral_limit, torque_limit):
        self.kp, self.ki, self.kd = [np.broadcast_to(np.asarray(v, float), (3,)).copy()
                                    for v in (kp, ki, kd)]
        self.integral_limit = np.broadcast_to(np.asarray(integral_limit, float), (3,))
        self.torque_limit = np.broadcast_to(np.asarray(torque_limit, float), (3,))
        if (not all(np.all(np.isfinite(v)) for v in
                    (self.kp, self.ki, self.kd, self.integral_limit, self.torque_limit))
                or np.any(self.integral_limit < 0) or np.any(self.torque_limit <= 0)):
            raise ValueError('Gains and limits must be finite; limits must be positive')
        self.reset()

    def reset(self):
        self.integral = np.zeros(3)
        self.previous_rates = None

    def update(self, state, target, dt):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        rates = np.asarray(state.rates_rad_s, float)
        desired = np.asarray(target.rates_rad_s, float)
        if rates.shape != (3,) or desired.shape != (3,) or not np.all(np.isfinite([rates, desired])):
            raise ValueError('Rates must be finite three-vectors')
        if not np.isfinite(target.thrust_n) or target.thrust_n < 0:
            raise ValueError('Thrust must be finite and nonnegative')
        error = desired - rates
        derivative = np.zeros(3) if self.previous_rates is None else -(rates - self.previous_rates) / dt
        candidate = np.clip(self.integral + error * dt, -self.integral_limit, self.integral_limit)
        raw = self.kp * error + self.ki * candidate + self.kd * derivative
        # Conditional integration prevents winding up against controller torque limits.
        accept = (abs(raw) <= self.torque_limit) | (error * raw < 0)
        self.integral = np.where(accept, candidate, self.integral)
        torque = np.clip(self.kp * error + self.ki * self.integral + self.kd * derivative,
                         -self.torque_limit, self.torque_limit)
        self.previous_rates = rates.copy()
        return np.r_[target.thrust_n, torque]


class MotorTable:
    """Measured steady-state data for ONE motor/propeller at a fixed supply voltage."""
    def __init__(self, filename):
        with open(filename, newline='') as stream:
            rows = list(csv.DictReader(stream))
        self.data = np.array([[float(row[k]) for k in
                               ('command', 'thrust_n', 'torque_nm', 'current_a')]
                              for row in rows])
        if self.data.ndim != 2 or len(self.data) < 2 or not np.all(np.isfinite(self.data)):
            raise ValueError('Need at least two finite measurement rows')
        if np.any(self.data < 0) or np.any(self.data[:, 0] > 1):
            raise ValueError('Commands must be in [0,1]; measurements must be nonnegative magnitudes')
        if np.any(np.diff(self.data[:, 0]) <= 0) or np.any(np.diff(self.data[:, 1]) <= 0):
            raise ValueError('Rows must have strictly increasing command and thrust')

    def at_thrust(self, thrust):
        if np.any(np.asarray(thrust) < self.data[0, 1]) or np.any(np.asarray(thrust) > self.data[-1, 1]):
            raise ValueError('Requested thrust is outside measured range')
        return np.array([np.interp(thrust, self.data[:, 1], self.data[:, i]) for i in (0, 2, 3)])


@dataclass
class Allocation:
    thrust_n: np.ndarray
    command: np.ndarray
    current_a: np.ndarray
    achieved_wrench: np.ndarray
    residual: np.ndarray  # achieved - requested, in N / Nm
    converged: bool


class Allocator:
    """Bounded nonlinear mixing using measured reaction torque versus thrust.

    positions are relative to the actual centre of mass in controller body axes.
    yaw_signs specify BODY reaction torque signs, not propeller spin signs.
    """
    def __init__(self, positions, yaw_signs, tables, wrench_scales):
        self.positions = np.asarray(positions, float)
        self.signs = np.asarray(yaw_signs, float)
        self.tables = list(tables)
        self.scales = np.asarray(wrench_scales, float)
        if (self.positions.shape != (4, 3) or self.signs.shape != (4,)
                or len(self.tables) != 4 or self.scales.shape != (4,)
                or not np.all(np.isfinite(self.positions))
                or not np.all(np.isin(self.signs, [-1, 1]))
                or not np.all(np.isfinite(self.scales)) or np.any(self.scales <= 0)):
            raise ValueError('Need four positions, four reaction signs, four tables and four positive scales')

    def wrench(self, thrust):
        torque = np.array([t.at_thrust(f)[1] for t, f in zip(self.tables, thrust)])
        return np.array([sum(thrust), self.positions[:, 1] @ thrust,
                         -self.positions[:, 0] @ thrust, self.signs @ torque])

    def allocate(self, requested):
        requested = np.asarray(requested, float)
        if requested.shape != (4,) or not np.all(np.isfinite(requested)):
            raise ValueError('Requested wrench must be a finite four-vector')
        minimum = np.array([t.data[0, 1] for t in self.tables])
        maximum = np.array([t.data[-1, 1] for t in self.tables])
        result = least_squares(lambda f: (self.wrench(f) - requested) / self.scales,
                               np.clip(np.full(4, requested[0] / 4), minimum + (maximum-minimum) * 1e-6, maximum - (maximum-minimum) * 1e-6),
                               bounds=(minimum, maximum), max_nfev=200)
        measured = np.array([t.at_thrust(f) for t, f in zip(self.tables, result.x)])
        achieved = self.wrench(result.x)
        return Allocation(result.x, measured[:, 0], measured[:, 2], achieved,
                          achieved - requested, bool(result.success))
