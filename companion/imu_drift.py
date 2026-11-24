#!/home/molepi/mav/bin/python3
"""Read-only inertial drift test: hold the drone still and watch the estimates wander.

Prints, every second, how far each estimate has moved since the start. The drone
is not moving, so all of it is error:

  att   EKF roll / pitch / yaw change (deg). With no compass, yaw comes only from
        integrating the gyro, so yaw drift is expected to grow.
  ekf   EKF position change north / east / down (m) and speed (m/s), when the EKF
        reports a local position (without GPS it may not).
  baro  altitude change from the raw barometer (m), independent of the EKF.
  gyro  yaw from integrating the raw gyro z axis (deg): the uncorrected gyro drift.
  dr    pure inertial dead reckoning: the raw accelerometer rotated by the EKF
        attitude, gravity removed, integrated twice (m). No corrections at all,
        so this grows roughly with time squared. It shows why inertial-only
        position is unusable, not a fault.

A summary with drift rates is printed at the end (Ctrl+C stops early).
Only requests telemetry; never arms or sends any actuator or mode command.

    python imu_drift.py                               # on the Pi, 120 s
    python imu_drift.py --url tcp:mole.local:5760     # from the laptop
    python imu_drift.py --url tcp:127.0.0.1:5763      # against the Gazebo sim
    python imu_drift.py --duration 300 --csv drift.csv

Holding the drone by hand adds real motion (a few degrees, centimetres). For
the cleanest numbers, rest it on a table; the "still" column flags movement.
"""
import argparse
import csv
import math
import sys
import time

from pymavlink import mavutil

mav = mavutil.mavlink
G = 9.80665
MY_SYSID = 254
STREAMS = {  # message id: rate in Hz
    mav.MAVLINK_MSG_ID_ATTITUDE: 50,
    mav.MAVLINK_MSG_ID_RAW_IMU: 50,
    mav.MAVLINK_MSG_ID_SCALED_PRESSURE: 10,
    mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 10,
    mav.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 2,
}
MOVE_GYRO_DPS = 3.0     # any axis above this counts as "moved" for that second
MOVE_ACC_MSS = 0.5      # |accel| further than this from 1 g counts as "moved"


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def baro_alt(p_hpa, p0_hpa):
    return 44330.0 * (1.0 - (p_hpa / p0_hpa) ** 0.190284)


def body_to_ned(roll, pitch, yaw, v):
    """Rotate a body-frame (forward-right-down) vector into north-east-down."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x, y, z = v
    return (cy * cp * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z,
            sy * cp * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z,
            -sp * x + cp * sr * y + cp * cr * z)


def slope(ts, ys):
    """Least-squares slope of ys over ts (per second)."""
    n = len(ts)
    if n < 2:
        return float("nan")
    mt, my = sum(ts) / n, sum(ys) / n
    den = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / den if den else float("nan")


def connect(url):
    m = mavutil.mavlink_connection(url, source_system=MY_SYSID,
                                   source_component=mav.MAV_COMP_ID_ONBOARD_COMPUTER, retries=2)
    deadline = time.time() + 10
    while time.time() < deadline:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if hb and hb.autopilot != mav.MAV_AUTOPILOT_INVALID and hb.type != mav.MAV_TYPE_GCS:
            m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            return m
    raise SystemExit("no autopilot heartbeat on %s (run check_link.py)" % url)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    ap.add_argument("--duration", type=float, default=120.0, help="seconds (default 120)")
    ap.add_argument("--csv", help="also write the per-second rows to this file")
    a = ap.parse_args()

    m = connect(a.url)
    m.mav.heartbeat_send(mav.MAV_TYPE_ONBOARD_CONTROLLER, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    time.sleep(0.5)

    def request_streams():
        # Re-sent every 2 s: a GCS such as QGroundControl resets stream rates when it
        # connects or reloads, which would otherwise drop RAW_IMU to ~2 Hz mid-test.
        for msg_id, hz in STREAMS.items():
            m.mav.command_long_send(m.target_system, m.target_component, mav.MAV_CMD_SET_MESSAGE_INTERVAL,
                                    0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    request_streams()
    print("connected to system %d on %s; hold still. Sampling for %.0f s (Ctrl+C to stop)\n"
          % (m.target_system, a.url, a.duration))

    att = att0 = None          # (roll, pitch, yaw) rad
    pos0 = pos = None          # LOCAL_POSITION_NED (x, y, z, vx, vy, vz)
    p0 = alt = None            # baro
    imu_t = None
    gyro_yaw = 0.0
    vel = [0.0, 0.0, 0.0]
    dr = [0.0, 0.0, 0.0]
    gsum = [0.0, 0.0, 0.0]
    nimu = 0
    gaps = 0                   # IMU intervals too long to integrate across
    nimu_sec = 0               # IMU samples in the current second
    ekf_flags = None
    moved = False
    rows = []
    t_start = time.time()
    next_print = t_start + 1.0
    next_request = t_start + 2.0

    hdr = ("   t  still | att droll dpitch   dyaw | ekf    dN     dE     dD  speed | baro  dalt"
           " | gyro  dyaw |  dr  horiz   vert | imu Hz")
    print(hdr)
    print("-" * len(hdr))
    writer = None
    fh = open(a.csv, "w", newline="") if a.csv else None
    if fh:
        writer = csv.writer(fh)
        writer.writerow(["t_s", "still", "droll_deg", "dpitch_deg", "dyaw_deg", "ekf_dn_m", "ekf_de_m",
                         "ekf_dd_m", "ekf_speed_mps", "baro_dalt_m", "gyro_dyaw_deg", "dr_horiz_m", "dr_vert_m",
                         "imu_hz"])

    try:
        while time.time() - t_start < a.duration:
            msg = m.recv_match(type=["ATTITUDE", "RAW_IMU", "SCALED_PRESSURE", "LOCAL_POSITION_NED",
                                     "EKF_STATUS_REPORT"], blocking=True, timeout=0.2)
            if msg is not None and msg.get_srcSystem() == m.target_system:
                k = msg.get_type()
                if k == "ATTITUDE":
                    att = (msg.roll, msg.pitch, msg.yaw)
                    att0 = att0 or att
                elif k == "LOCAL_POSITION_NED":
                    pos = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
                    pos0 = pos0 or pos
                elif k == "SCALED_PRESSURE":
                    p0 = p0 or msg.press_abs
                    alt = baro_alt(msg.press_abs, p0)
                elif k == "EKF_STATUS_REPORT":
                    ekf_flags = msg.flags
                elif k == "RAW_IMU" and att is not None:
                    t = msg.time_usec / 1e6
                    gyro = (msg.xgyro / 1000.0, msg.ygyro / 1000.0, msg.zgyro / 1000.0)      # rad/s
                    acc = (msg.xacc * G / 1000.0, msg.yacc * G / 1000.0, msg.zacc * G / 1000.0)  # m/s^2
                    if max(abs(w) for w in gyro) > math.radians(MOVE_GYRO_DPS) or \
                            abs(math.sqrt(sum(x * x for x in acc)) - G) > MOVE_ACC_MSS:
                        moved = True
                    if imu_t is not None and not 0 < t - imu_t < 0.2:
                        gaps += 1
                    elif imu_t is not None:
                        dt = t - imu_t
                        gyro_yaw += gyro[2] * dt   # body z ~ vertical when level
                        a_ned = body_to_ned(*att, acc)
                        a_ned = (a_ned[0], a_ned[1], a_ned[2] + G)
                        for i in range(3):
                            dr[i] += vel[i] * dt + 0.5 * a_ned[i] * dt * dt
                            vel[i] += a_ned[i] * dt
                    imu_t = t
                    for i in range(3):
                        gsum[i] += gyro[i]
                    nimu += 1
                    nimu_sec += 1

            if time.time() >= next_request:
                next_request += 2.0
                request_streams()

            if time.time() >= next_print:
                next_print += 1.0
                ts = time.time() - t_start
                imu_hz, nimu_sec = nimu_sec, 0
                if att is None:
                    print("%4.0f  waiting for ATTITUDE..." % ts)
                    continue
                d_att = [math.degrees(att[0] - att0[0]), math.degrees(att[1] - att0[1]),
                         wrap180(math.degrees(att[2] - att0[2]))]
                if pos is not None:
                    d_pos = [pos[i] - pos0[i] for i in range(3)]
                    spd = math.sqrt(sum(v * v for v in pos[3:]))
                    ekf_s = "%6.2f %6.2f %6.2f %6.2f" % (*d_pos, spd)
                else:
                    d_pos, spd = [float("nan")] * 3, float("nan")
                    ekf_s = "   no local position     "
                alt_s = "%6.2f" % alt if alt is not None else "   n/a"
                dr_h, dr_v = math.hypot(dr[0], dr[1]), -dr[2]
                still = "moved" if moved else "  yes"
                print("%4.0f  %s | %9.2f %6.2f %6.2f | %s | %s | %10.2f | %10.1f %6.1f | %3d%s"
                      % (ts, still, *d_att, ekf_s, alt_s, math.degrees(gyro_yaw), dr_h, dr_v,
                         imu_hz, "  <- IMU rate low: gyro/dr columns unreliable" if imu_hz < 10 else ""))
                rows.append((ts, not moved, *d_att, *d_pos, spd,
                             alt if alt is not None else float("nan"), math.degrees(gyro_yaw), dr_h, dr_v,
                             imu_hz))
                if writer:
                    writer.writerow(["%.4f" % x if isinstance(x, float) else int(x) for x in rows[-1]])
                moved = False
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        if fh:
            fh.close()

    if len(rows) < 3:
        print("not enough data for a summary")
        return 1

    t = [r[0] for r in rows]
    mins = t[-1] / 60.0
    print("\nSummary over %.0f s (%d of %d seconds still)" % (t[-1], sum(r[1] for r in rows), len(rows)))

    def line(label, idx, unit, scale=60.0):
        ys = [r[idx] for r in rows]
        if any(math.isnan(y) for y in ys):
            print("  %-26s n/a" % label)
            return
        print("  %-26s end %+8.2f %-4s  rate %+8.3f %s/min  max |%.2f|"
              % (label, ys[-1], unit, slope(t, ys) * scale, unit, max(abs(y) for y in ys)))

    line("EKF roll", 2, "deg")
    line("EKF pitch", 3, "deg")
    line("EKF yaw", 4, "deg")
    line("raw gyro yaw (integrated)", 10, "deg")
    line("EKF north", 5, "m")
    line("EKF east", 6, "m")
    line("EKF down", 7, "m")
    line("baro altitude", 9, "m")
    print("  %-26s horizontal %.1f m, vertical %.1f m after %.1f min (grows ~ t^2)"
          % ("dead reckoning (accel)", rows[-1][11], rows[-1][12], mins))
    rate = nimu / t[-1]
    print("  %-26s %.1f Hz average, %d gaps skipped%s"
          % ("IMU sample rate", rate, gaps,
             "  <- too low: gyro and dead-reckoning results are not valid" if rate < 10 else ""))
    if nimu:
        print("  %-26s x %+.3f  y %+.3f  z %+.3f deg/s (%d samples)"
              % ("mean gyro (bias if still)", *[math.degrees(s / nimu) for s in gsum], nimu))
    if ekf_flags is not None:
        names = [e.name.replace("EKF_", "") for b, e in mav.enums["EKF_STATUS_FLAGS"].items()
                 if b and b & ekf_flags and b != max(mav.enums["EKF_STATUS_FLAGS"])]
        print("  %-26s 0x%04x %s" % ("EKF flags at end", ekf_flags, " ".join(names) or "none"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
