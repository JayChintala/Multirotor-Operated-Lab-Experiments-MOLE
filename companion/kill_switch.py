#!/home/molepi/mav/bin/python3
"""Kill switch: stop the props on a laptop KILL command or when the laptop link drops.

Listens on UDP 5002 for JSON packets from the laptop backend
(control_center_backend.py):

    {"version": 1, "type": "KILL_HEARTBEAT", "time_ms": ...}    10 Hz while the control center is open
    {"version": 1, "type": "KILL", "id": ..., "time_ms": ...}   KILL button

and answers each one with a KILL_STATUS packet.

The FC is force-disarmed (MAV_CMD_COMPONENT_ARM_DISARM with 21196, which works
in flight, so the drone drops) when
  - a KILL packet arrives, or
  - the FC is armed and no KILL_HEARTBEAT has arrived for --timeout seconds,
    including when the laptop was never connected.

Runs on the Pi, so it still works when the Wi-Fi link is gone. Installed as the
kill-switch systemd service by setup.sh.

    python kill_switch.py --timeout 1.0
"""
import argparse
import json
import select
import socket
import sys
import time

from pymavlink import mavutil

CONN = "tcp:127.0.0.1:5760"
MY_SYSID = 252          # telemetry_bridge.py is 253, check_link.py 254, a laptop GCS 255
FORCE_DISARM = 21196    # param2 magic number: disarm even in flight
DISARM_RETRY = 0.1      # s between disarm commands until the FC reports disarmed
FC_LINK_TIMEOUT = 3.0   # s without an FC heartbeat before reporting the FC link as down

mav = mavutil.mavlink


def log(text):
    print(time.strftime("%H:%M:%S ") + text, flush=True)


def connect_fc():
    while True:
        m = mavutil.mavlink_connection(CONN, source_system=MY_SYSID,
                                       source_component=mav.MAV_COMP_ID_ONBOARD_COMPUTER,
                                       autoreconnect=True, retries=5)
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
        while msg and (msg.autopilot == mav.MAV_AUTOPILOT_INVALID or msg.type == mav.MAV_TYPE_GCS):
            msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
        if msg:
            m.target_system, m.target_component = msg.get_srcSystem(), msg.get_srcComponent()
            return m, msg
        log("No heartbeat from the flight controller; retrying (is mavlink-router running?)")
        m.close()


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--port", type=int, default=5002, help="UDP port for laptop packets")
    p.add_argument("--timeout", type=float, default=1.0,
                   help="seconds without a laptop heartbeat before disarming an armed FC")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    sock.setblocking(False)

    m, hb = connect_fc()
    armed = bool(hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
    # The FC ignores our first command unless it has seen a heartbeat from us.
    m.mav.heartbeat_send(mav.MAV_TYPE_ONBOARD_CONTROLLER, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    log("FC sysid %d, %s. Listening on UDP %d, laptop timeout %.1f s"
        % (m.target_system, "ARMED" if armed else "disarmed", args.port, args.timeout))

    now = time.monotonic()
    last_fc_hb = now
    last_laptop_hb = 0.0     # 0 = no heartbeat yet
    laptop_linked = False
    kill_reason = None       # set while we are trying to disarm
    last_kill_id = None
    last_disarm_sent = 0.0
    next_heartbeat = now + 1

    def status():
        return {"version": 1, "type": "KILL_STATUS", "time_ms": int(time.time() * 1000),
                "armed": armed, "fc_linked": time.monotonic() - last_fc_hb < FC_LINK_TIMEOUT,
                "killing": kill_reason, "timeout_s": args.timeout}

    while True:
        select.select([m.port, sock], [], [], 0.05)
        now = time.monotonic()

        while True:
            msg = m.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if (t == "HEARTBEAT" and msg.get_srcSystem() == m.target_system
                    and msg.get_srcComponent() == m.target_component):
                last_fc_hb = now
                was_armed, armed = armed, bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                if armed != was_armed:
                    log("FC " + ("ARMED" if armed else "disarmed"))
            elif (t == "COMMAND_ACK" and msg.command == mav.MAV_CMD_COMPONENT_ARM_DISARM
                    and msg.get_srcSystem() == m.target_system and kill_reason):
                if msg.result != mav.MAV_RESULT_ACCEPTED:
                    log("FC refused the disarm: result %d" % msg.result)

        while True:
            try:
                data, addr = sock.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                packet = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(packet, dict) or packet.get("version") != 1:
                continue
            if packet.get("type") == "KILL_HEARTBEAT":
                last_laptop_hb = now
                if not laptop_linked:
                    laptop_linked = True
                    log("Laptop link up: %s" % addr[0])
            elif packet.get("type") == "KILL":
                last_laptop_hb = now
                # The backend sends each KILL several times; act on it once.
                if packet.get("id") != last_kill_id:
                    last_kill_id = packet.get("id")
                    kill_reason = "manual kill"
                    log("KILL from %s" % addr[0])
            else:
                continue
            sock.sendto(json.dumps(status(), separators=(",", ":")).encode(), addr)

        if laptop_linked and now - last_laptop_hb > args.timeout:
            laptop_linked = False
            log("Laptop link lost")
        if armed and not laptop_linked and kill_reason is None:
            kill_reason = "laptop link lost" if last_laptop_hb else "no laptop link"

        if kill_reason and not armed:
            log("Props stopped (%s)" % kill_reason)
            kill_reason = None
        if kill_reason and now - last_disarm_sent >= DISARM_RETRY:
            if not last_disarm_sent or now - last_disarm_sent > 1:
                log("Force-disarming: %s" % kill_reason)
            m.mav.command_long_send(m.target_system, m.target_component,
                                    mav.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                    0, FORCE_DISARM, 0, 0, 0, 0, 0)
            last_disarm_sent = now

        if now >= next_heartbeat:
            m.mav.heartbeat_send(mav.MAV_TYPE_ONBOARD_CONTROLLER, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            next_heartbeat = now + 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
