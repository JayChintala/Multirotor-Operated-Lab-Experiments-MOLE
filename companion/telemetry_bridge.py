#!/home/molepi/mav/bin/python3
"""Read-only telemetry bridge: FC (via mavlink-router) -> laptop control center.

Reads attitude, IMU, position, battery, mode, arming state, motor outputs and
ESC RPM from
tcp:127.0.0.1:5760 and sends JSON TELEMETRY datagrams to the laptop backend
(UDP 5001). Only requests telemetry -- never arms or sends any actuator/motor
command.

    python telemetry_bridge.py --laptop-ip 192.168.1.161
"""
import argparse
import json
import socket
import sys
import time

from pymavlink import mavutil

CONN = "tcp:127.0.0.1:5760"
HEARTBEAT_TIMEOUT = 10  # s
MY_SYSID = 253          # avoid clashing with a laptop GCS (255) and check_link.py (254)
G = 9.80665

mav = mavutil.mavlink

# Message -> interval in microseconds
STREAMS = {
    mav.MAVLINK_MSG_ID_ATTITUDE: 50000,             # 20 Hz
    mav.MAVLINK_MSG_ID_SCALED_IMU: 50000,           # 20 Hz
    mav.MAVLINK_MSG_ID_SYS_STATUS: 500000,          # 2 Hz
    mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 200000,  # 5 Hz
    mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 200000,   # 5 Hz
    mav.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: 100000,     # 10 Hz
    11030: 100000,                                   # ESC_TELEMETRY_1_TO_4 (ArduPilot dialect), 10 Hz
}
MOTORS = 4
PWM_MIN, PWM_MAX = 1000, 2000  # MOT_PWM_MIN / MOT_PWM_MAX


def wait_autopilot(m):
    deadline = time.time() + HEARTBEAT_TIMEOUT
    while time.time() < deadline:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg and msg.autopilot != mav.MAV_AUTOPILOT_INVALID and msg.type != mav.MAV_TYPE_GCS:
            return msg
    return None


def request_streams(m):
    for msg_id, interval in STREAMS.items():
        m.mav.command_long_send(m.target_system, m.target_component,
                                mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, interval, 0, 0, 0, 0, 0)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--laptop-ip", required=True, help="laptop running control_center_backend.py")
    p.add_argument("--laptop-port", type=int, default=5001)
    p.add_argument("--rate", type=float, default=10, help="TELEMETRY packets per second")
    args = p.parse_args()

    m = mavutil.mavlink_connection(CONN, source_system=MY_SYSID,
                                   source_component=mav.MAV_COMP_ID_ONBOARD_COMPUTER,
                                   retries=2)
    hb = wait_autopilot(m)
    if hb is None:
        print("No heartbeat from the flight controller; run check_link.py.")
        return 1
    sysid = hb.get_srcSystem()
    m.target_system, m.target_component = sysid, hb.get_srcComponent()

    # The FC ignores our first request unless it has seen a heartbeat from us.
    m.mav.heartbeat_send(mav.MAV_TYPE_ONBOARD_CONTROLLER, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    time.sleep(1)
    request_streams(m)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.laptop_ip, args.laptop_port)
    print("Sending TELEMETRY to %s:%d at %g Hz (Ctrl-C to stop)" % (dest + (args.rate,)))

    fc = {"flight_mode": mavutil.mode_string_v10(hb),
          "armed": bool(hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED),
          "motors": [{} for _ in range(MOTORS)]}
    period = 1.0 / args.rate
    next_send = next_heartbeat = time.time()
    sent = 0
    while True:
        msg = m.recv_match(blocking=True, timeout=period)
        if msg and msg.get_srcSystem() == sysid:
            t = msg.get_type()
            if t == "HEARTBEAT" and msg.type != mav.MAV_TYPE_GCS:
                fc["flight_mode"] = mavutil.mode_string_v10(msg)
                fc["armed"] = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "ATTITUDE":
                fc["attitude"] = {"roll": round(msg.roll * 57.29578, 2),
                                  "pitch": round(msg.pitch * 57.29578, 2),
                                  "yaw": round(msg.yaw * 57.29578, 2)}
            elif t == "SCALED_IMU":  # accel in mG, gyro in mrad/s
                fc["acceleration"] = {k: round(getattr(msg, k + "acc") * G / 1000, 3) for k in "xyz"}
                fc["angular_velocity"] = {k: round(getattr(msg, k + "gyro") / 1000, 4) for k in "xyz"}
            elif t == "SYS_STATUS" and msg.voltage_battery != 65535:
                fc["battery"] = {"voltage": msg.voltage_battery / 1000.0}
                if msg.battery_remaining >= 0:
                    fc["battery"]["remaining"] = msg.battery_remaining
            elif t == "GLOBAL_POSITION_INT" and (msg.lat or msg.lon):
                fc.setdefault("position", {}).update(
                    lat=msg.lat / 1e7, lon=msg.lon / 1e7, alt=msg.alt / 1000.0)
            elif t == "SERVO_OUTPUT_RAW" and msg.port == 0:
                for i, motor in enumerate(fc["motors"]):
                    pwm = getattr(msg, "servo%d_raw" % (i + 1))
                    motor["output"] = round(min(max((pwm - PWM_MIN) / (PWM_MAX - PWM_MIN), 0), 1) * 100, 1)
            elif t == "ESC_TELEMETRY_1_TO_4":
                for motor, rpm, temp in zip(fc["motors"], msg.rpm, msg.temperature):
                    motor["rpm"] = rpm
                    motor["temperature"] = temp
            elif t == "LOCAL_POSITION_NED":
                fc.setdefault("position", {}).update(x=msg.x, y=msg.y, z=-msg.z)

        now = time.time()
        if now >= next_heartbeat:
            m.mav.heartbeat_send(mav.MAV_TYPE_ONBOARD_CONTROLLER, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            next_heartbeat = now + 1
        if now >= next_send:
            packet = {"version": 1, "type": "TELEMETRY", "time_ms": int(now * 1000), "fc": fc}
            sock.sendto(json.dumps(packet, separators=(",", ":")).encode(), dest)
            sent += 1
            if sent == 1 or sent % 100 == 0:
                print("sent %d packets" % sent)
            next_send = now + period


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
