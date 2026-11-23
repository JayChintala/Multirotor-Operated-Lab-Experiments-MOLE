"""ROS-free helpers for the mocap nodes, so they can be unit-tested without a ROS install.

Quaternions are (x, y, z, w) tuples, the order geometry_msgs uses.
"""
import collections
import math


def quat_mul(a, b):
    """Hamilton product a * b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_from_rotvec(v):
    """Quaternion for a rotation of |v| radians about the axis v."""
    angle = math.sqrt(sum(c * c for c in v))
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2) / angle
    return (v[0] * s, v[1] * s, v[2] * s, math.cos(angle / 2))


def perturb(position, quat, pos_sigma, ang_sigma, rng):
    """Add Gaussian noise: pos_sigma metres per axis, ang_sigma radians per axis (world frame)."""
    pos = tuple(p + rng.gauss(0.0, pos_sigma) for p in position)
    noise = quat_from_rotvec([rng.gauss(0.0, ang_sigma) for _ in range(3)])
    return pos, quat_mul(noise, quat)


def pose_ok(position, quat):
    """Reject poses the FC must never see: NaN/inf, or a quaternion that isn't a rotation
    (an all-zero quaternion is what a tracker sends for an object it has lost)."""
    values = list(position) + list(quat)
    if not all(math.isfinite(v) for v in values):
        return False
    norm = math.sqrt(sum(q * q for q in quat))
    return 0.9 < norm < 1.1


class DelayLine:
    """Holds timestamped samples and releases the newest one that is at least `latency` old."""

    def __init__(self, max_len=1000):
        self._q = collections.deque(maxlen=max_len)

    def push(self, t, item):
        self._q.append((t, item))

    def pop_ready(self, now, latency):
        ready = None
        while self._q and self._q[0][0] <= now - latency:
            ready = self._q.popleft()[1]
        return ready


class RateLimiter:
    """Passes at most one sample per 1/rate_hz seconds (by sample time)."""

    def __init__(self, rate_hz):
        # 5 % slack so a 100 Hz source isn't thinned to 33 Hz by jitter when limiting to 50 Hz.
        self.period = 0.95 / rate_hz if rate_hz > 0 else 0.0
        self.last = None

    def accept(self, t):
        if self.last is not None and 0.0 <= t - self.last < self.period:
            return False
        self.last = t     # also resyncs if time jumps backwards (sim reset)
        return True


# ArduPilot's LOCATION_SCALING_FACTOR (metres per 1e-7 degree of latitude), so a point we place
# here lands where ArduPilot's own local<->lat/lon conversion puts it.
_M_PER_DEG = 0.011131884502145034 * 1e7


def offset_latlon(lat, lon, north_m, east_m):
    """lat/lon (deg) of a point north_m/east_m metres from (lat, lon), flat-earth like ArduPilot."""
    return (lat + north_m / _M_PER_DEG,
            lon + east_m / (_M_PER_DEG * max(math.cos(math.radians(lat)), 0.01)))


def in_volume(position, half_x, half_y, z_max, z_min=-1.0):
    """True if position (x, y, z in the Vicon frame, origin at the floor centre) is in the box."""
    x, y, z = position
    return abs(x) <= half_x and abs(y) <= half_y and z_min <= z <= z_max


def yaw_deg(quat):
    """Heading in degrees (rotation about z, counter-clockwise positive) of an (x, y, z, w) quaternion."""
    x, y, z, w = quat
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def wrap_deg(angle):
    """Wrap to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0
