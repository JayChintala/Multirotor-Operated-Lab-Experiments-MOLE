#!/home/molepi/mav/bin/python3
"""Read-only MAVLink link check against mavlink-router (tcp:127.0.0.1:5760).

Waits for an autopilot heartbeat, then reports vehicle/firmware/mode/arming,
GPS, battery and EKF status. Only requests telemetry -- never arms or sends
any actuator/motor command.
"""
import sys
import time

from pymavlink import mavutil

CONN = "tcp:127.0.0.1:5760"
HEARTBEAT_TIMEOUT = 10  # s
COLLECT_TIME = 4        # s spent gathering telemetry after the heartbeat
MY_SYSID = 254          # avoid clashing with a laptop GCS (usually 255)

CHECKLIST = """
No heartbeat from the flight controller. Check:
  [ ] TX/RX swapped: Pi pin 8 (TXD) -> FC RX7, Pi pin 10 (RXD) -> FC TX7
  [ ] Missing common ground: Pi GND (e.g. pin 6) wired to FC GND
  [ ] FC params: SERIAL7_PROTOCOL=2 (MAVLink2) and SERIAL7_BAUD=921, then reboot FC
  [ ] /dev/serial0 not mapped to ttyAMA0:  ls -l /dev/serial0
      (needs dtoverlay=disable-bt + enable_uart=1 in /boot/firmware/config.txt, reboot)
  [ ] Serial console still on the UART:  systemctl status serial-getty@ttyAMA0
  [ ] mavlink-router not running:  systemctl status mavlink-router
      journalctl -u mavlink-router -n 50
"""

mav = mavutil.mavlink


def fw_version(v):
    types = {0: "dev", 64: "alpha", 128: "beta", 192: "rc", 255: "official"}
    return "%d.%d.%d (%s)" % ((v >> 24) & 0xFF, (v >> 16) & 0xFF,
                              (v >> 8) & 0xFF, types.get(v & 0xFF, v & 0xFF))


def enum_name(enum, value):
    e = mav.enums.get(enum, {})
    return e[value].name if value in e else str(value)


def flag_names(enum, value):
    return [entry.name.replace("EKF_", "") for bit, entry in mav.enums[enum].items()
            if bit and bit & value and bit != max(mav.enums[enum])] or ["none"]


def main():
    try:
        m = mavutil.mavlink_connection(CONN, source_system=MY_SYSID,
                                       source_component=mav.MAV_COMP_ID_ONBOARD_COMPUTER,
                                       retries=2)
    except Exception as e:  # connection refused etc.
        print("Cannot connect to %s: %s" % (CONN, e))
        print(CHECKLIST)
        return 2

    # Wait for a heartbeat from an actual autopilot (ignore GCS / other heartbeats).
    hb = None
    deadline = time.time() + HEARTBEAT_TIMEOUT
    while time.time() < deadline:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg and msg.autopilot != mav.MAV_AUTOPILOT_INVALID and msg.type != mav.MAV_TYPE_GCS:
            hb = msg
            break
    if hb is None:
        print(CHECKLIST)
        return 1

    sysid, compid = hb.get_srcSystem(), hb.get_srcComponent()
    m.target_system, m.target_component = sysid, compid
    print("Heartbeat from system %d component %d" % (sysid, compid))

    # Ask for telemetry (read-only requests).
    m.mav.command_long_send(sysid, compid, mav.MAV_CMD_REQUEST_MESSAGE, 0,
                            mav.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
    for msg_id in (mav.MAVLINK_MSG_ID_SYS_STATUS, mav.MAVLINK_MSG_ID_GPS_RAW_INT,
                   mav.MAVLINK_MSG_ID_EKF_STATUS_REPORT):
        m.mav.command_long_send(sysid, compid, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, 500000, 0, 0, 0, 0, 0)  # 2 Hz

    seen = {"HEARTBEAT": hb}
    want = ["AUTOPILOT_VERSION", "SYS_STATUS", "GPS_RAW_INT", "EKF_STATUS_REPORT"]
    deadline = time.time() + COLLECT_TIME
    while time.time() < deadline and not all(k in seen for k in want):
        msg = m.recv_match(type=want + ["HEARTBEAT"], blocking=True, timeout=0.5)
        if msg and msg.get_srcSystem() == sysid:
            seen[msg.get_type()] = msg
    hb = seen["HEARTBEAT"]

    print("Vehicle type : %s" % enum_name("MAV_TYPE", hb.type))
    print("Autopilot    : %s" % enum_name("MAV_AUTOPILOT", hb.autopilot))
    av = seen.get("AUTOPILOT_VERSION")
    print("Firmware     : %s" % (fw_version(av.flight_sw_version) if av else "n/a"))
    print("Flight mode  : %s" % mavutil.mode_string_v10(hb))
    print("Armed        : %s" % bool(hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED))
    gps = seen.get("GPS_RAW_INT")
    print("GPS          : %s" % ("%s, %d sats" % (enum_name("GPS_FIX_TYPE", gps.fix_type),
                                                  gps.satellites_visible) if gps else "n/a"))
    ss = seen.get("SYS_STATUS")
    print("Battery      : %s" % ("%.2f V" % (ss.voltage_battery / 1000.0)
                                 if ss and ss.voltage_battery != 65535 else "n/a"))
    ekf = seen.get("EKF_STATUS_REPORT")
    if ekf:
        print("EKF flags    : 0x%04x %s" % (ekf.flags,
                                            " ".join(flag_names("EKF_STATUS_FLAGS", ekf.flags))))
    else:
        print("EKF flags    : n/a")
    return 0


if __name__ == "__main__":
    sys.exit(main())
