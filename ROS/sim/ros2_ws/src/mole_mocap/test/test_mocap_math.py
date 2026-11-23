import math
import random

from mole_mocap.mocap_math import DelayLine, RateLimiter, perturb, pose_ok, quat_from_rotvec, quat_mul


def test_quat_mul_composes_yaw():
    q = quat_mul(quat_from_rotvec((0, 0, math.pi / 4)), quat_from_rotvec((0, 0, math.pi / 4)))
    expected = quat_from_rotvec((0, 0, math.pi / 2))
    assert all(abs(a - b) < 1e-12 for a, b in zip(q, expected))


def test_perturb_noise_size_and_unit_quaternion():
    rng = random.Random(1)
    dx, angles = [], []
    for _ in range(5000):
        p, q = perturb((1.0, 2.0, 3.0), (0, 0, 0, 1), 0.001, 0.002, rng)
        dx.append(p[0] - 1.0)
        assert abs(math.sqrt(sum(c * c for c in q)) - 1) < 1e-9
        angles.append(2 * math.acos(min(1.0, abs(q[3]))))
    sigma = math.sqrt(sum(d * d for d in dx) / len(dx))
    assert 0.0009 < sigma < 0.0011
    # rotation angle of a 3-axis Gaussian rotvec has RMS sigma*sqrt(3)
    rms = math.sqrt(sum(a * a for a in angles) / len(angles))
    assert abs(rms - 0.002 * math.sqrt(3)) < 0.0002


def test_pose_ok_rejects_lost_object():
    assert pose_ok((0, 0, 1), (0, 0, 0, 1))
    assert not pose_ok((0, 0, 1), (0, 0, 0, 0))
    assert not pose_ok((float('nan'), 0, 1), (0, 0, 0, 1))
    assert not pose_ok((0, 0, 1), (0, 0, 0, float('inf')))


def test_delay_line_releases_newest_old_enough_sample():
    d = DelayLine()
    for i in range(10):
        d.push(i * 0.01, i)            # samples at 0..90 ms
    assert d.pop_ready(0.05, 0.02) == 3     # newest sample <= 30 ms
    assert d.pop_ready(0.05, 0.02) is None  # nothing new became ready
    assert d.pop_ready(0.20, 0.02) == 9


def test_rate_limiter_thins_100hz_to_50hz():
    r = RateLimiter(50.0)
    passed = sum(r.accept(i * 0.01 + random.Random(i).uniform(-0.0005, 0.0005)) for i in range(1000))
    assert 490 <= passed <= 510


def test_rate_limiter_resyncs_after_time_goes_backwards():
    r = RateLimiter(50.0)
    assert r.accept(100.0)
    assert r.accept(1.0)      # sim restarted
    assert not r.accept(1.005)
