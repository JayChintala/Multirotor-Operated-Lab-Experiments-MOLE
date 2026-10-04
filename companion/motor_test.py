#!/home/molepi/mav/bin/python3
"""Bench motor test for the MOLE flight controller via mavlink-router (tcp:127.0.0.1:5760).

Spins motors one at a time using MAV_CMD_DO_MOTOR_TEST, the same command as
Mission Planner's Motor Test page. The FC stops each motor by itself after
--duration seconds. This script NEVER arms the vehicle.

    python motor_test.py                  # motors 1-4, 7% throttle, 2 s each
    python motor_test.py -m 3 -t 10       # only motor C (3) at 10%

Motor numbers are ArduPilot's test order: 1=A, 2=B, ... going clockwise from
the front-right motor on a quad X. REMOVE THE PROPELLERS FIRST.
"""
import argparse
import sys
import time

from pymavlink import mavutil

CONN = "tcp:127.0.0.1:5760"
MY_SYSID = 254           # avoid clashing with a laptop GCS (usually 255)
MAX_THROTTLE = 15        # % hard cap for bench testing
MAX_DURATION = 5         # s hard cap per motor
HEARTBEAT_TIMEOUT = 10   # s

mav = mavutil.mavlink


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-m", "--motor", type=int, action="append",
                   help="motor number to test (repeatable); default: all, in order")
    p.add_argument("-n", "--count", type=int, default=4,
                   help="number of motors on the frame (default 4)")
    p.add_argument("-t", "--throttle", type=float, default=7,
                   help="throttle percent (default 7, max %d)" % MAX_THROTTLE)
    p.add_argument("-d", "--duration", type=float, default=2,
                   help="seconds per motor (default 2, max %d)" % MAX_DURATION)
    a = p.parse_args()
    if not 0 < a.throttle <= MAX_THROTTLE:
        p.error("throttle must be between 0 and %d %%" % MAX_THROTTLE)
    if not 0 < a.duration <= MAX_DURATION:
        p.error("duration must be between 0 and %d s" % MAX_DURATION)
    a.motor = a.motor or list(range(1, a.count + 1))
    if any(not 1 <= n <= a.count for n in a.motor):
        p.error("motor numbers must be between 1 and %d" % a.count)
    return a


def wait_autopilot(m):
    """Wait for a heartbeat from the flight controller (not a GCS)."""
    deadline = time.time() + HEARTBEAT_TIMEOUT
    while time.time() < deadline:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if hb and hb.autopilot != mav.MAV_AUTOPILOT_INVALID and hb.type != mav.MAV_TYPE_GCS:
            return hb
    return None


def announce_and_verify(m):
    """Send our heartbeat, then check the FC answers a harmless request.

    On this setup the first command after connecting is often ignored unless
    the FC has already seen a heartbeat from us. Verifying with a read-only
    request means a motor command is only ever sent over a working link.
    """
    m.mav.heartbeat_send(mav.MAV_TYPE_ONBOARD_CONTROLLER, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    time.sleep(1)
    for _ in range(3):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mav.MAV_CMD_REQUEST_MESSAGE, 0,
                                mav.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
        deadline = time.time() + 2
        while time.time() < deadline:
            ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if ack and ack.command == mav.MAV_CMD_REQUEST_MESSAGE:
                return True
    return False


def print_statustext(m):
    """Show any messages the FC sent (e.g. why a motor test was refused)."""
    while True:
        msg = m.recv_match(type="STATUSTEXT", blocking=False)
        if msg is None:
            return
        print("  FC: %s" % msg.text)


def motor_test(m, motor, throttle, duration):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mav.MAV_CMD_DO_MOTOR_TEST, 0,
                            motor,                            # motor (test order)
                            mav.MOTOR_TEST_THROTTLE_PERCENT,  # throttle type
                            throttle,                         # throttle %
                            duration,                         # timeout s
                            0, 0, 0)                          # one motor, default order
    deadline = time.time() + 3
    while time.time() < deadline:
        ack = m.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=0.5)
        if ack is None:
            continue
        if ack.get_type() == "STATUSTEXT":
            print("  FC: %s" % ack.text)
        elif ack.command == mav.MAV_CMD_DO_MOTOR_TEST:
            return ack.result
    return None


def stop_motor(m, motor):
    # 0 % throttle replaces any running test on that motor
    m.mav.command_long_send(m.target_system, m.target_component,
                            mav.MAV_CMD_DO_MOTOR_TEST, 0,
                            motor, mav.MOTOR_TEST_THROTTLE_PERCENT, 0, 0, 0, 0, 0)


def main():
    a = parse_args()
    print("Motor test: motors %s at %g %% for %g s each." % (a.motor, a.throttle, a.duration))
    print("PROPELLERS OFF, FC on battery power, frame held down.")
    if input("Type YES to continue: ").strip() != "YES":
        print("Aborted.")
        return 1

    m = mavutil.mavlink_connection(CONN, source_system=MY_SYSID,
                                   source_component=mav.MAV_COMP_ID_ONBOARD_COMPUTER)
    hb = wait_autopilot(m)
    if hb is None:
        print("No heartbeat from the flight controller. Run check_link.py for a checklist.")
        return 1
    m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
    if hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
        print("Vehicle is ARMED. Disarm it first; motor test only runs disarmed.")
        return 1
    if not announce_and_verify(m):
        print("FC is sending telemetry but not answering commands. Check Pi pin 8 (TX)"
              " -> FC RX7 and SERIAL7_PROTOCOL=2. No motor command was sent.")
        return 1
    print_statustext(m)

    current = None
    try:
        for motor in a.motor:
            current = motor
            letter = chr(ord("A") + motor - 1)
            print("Motor %d (%s): spinning..." % (motor, letter))
            result = motor_test(m, motor, a.throttle, a.duration)
            if result != mav.MAV_RESULT_ACCEPTED:
                name = mav.enums["MAV_RESULT"][result].name if result is not None else "no reply"
                print("  Refused: %s. Common causes: RC not calibrated, safety switch on,"
                      " vehicle not landed, ESC/motor outputs not configured." % name)
                return 1
            time.sleep(a.duration + 1)  # FC stops the motor after the timeout
            current = None
    except KeyboardInterrupt:
        if current is not None:
            stop_motor(m, current)
            print("\nStopped motor %d." % current)
        return 1
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
