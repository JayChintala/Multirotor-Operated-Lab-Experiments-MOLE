"""
Laptop-side control-center bridge.

Browser (React)
      |
      | WebSocket + JSON
      v
This process
      |
      | UDP + JSON
      v
Raspberry Pi gateway

The important design choice is that the browser never needs to know the
Raspberry Pi's UDP port. Only this file knows about the Pi network endpoint.
"""

import asyncio
import json
import socket
import threading
import time
from typing import Optional, Set

from websockets.asyncio.server import ServerConnection, serve


# ============================================================
# 1. CHANGE THESE NETWORK SETTINGS LATER
# ============================================================

# Raspberry Pi address on your local/Wi-Fi network.
PI_IP = "192.168.1.50"

# Must match the UDP CONTROL_PORT in rpi_gateway.py.
PI_CONTROL_PORT = 5000

# Pi sends telemetry to this laptop port.
LAPTOP_TELEMETRY_PORT = 5001

# Browser connects to this WebSocket server.
WEBSOCKET_HOST = "127.0.0.1"
WEBSOCKET_PORT = 8765


# ============================================================
# 2. CONTROL/TIMING SETTINGS
# ============================================================

# Used only for status information.
EXPECTED_CONTROL_HZ = 20

# How old a Pi telemetry packet can be before we mark it stale.
TELEMETRY_STALE_AFTER_MS = 1500


# ============================================================
# 3. UDP SOCKETS
# ============================================================

# Separate sockets keep the data flow obvious:
#   control_socket       -> sends laptop -> Pi
#   telemetry_socket     -> receives Pi -> laptop
#
# This also fixes an easy-to-miss problem in the original example:
# binding a receive socket to CONTROL_PORT would not receive telemetry
# that the Pi sends to LAPTOP_TELEMETRY_PORT.
control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
telemetry_socket.bind(("0.0.0.0", LAPTOP_TELEMETRY_PORT))
telemetry_socket.settimeout(0.2)


# ============================================================
# 4. SHARED STATUS
# ============================================================

state_lock = threading.Lock()
last_telemetry: Optional[dict] = None
last_telemetry_received_monotonic = 0.0
last_control_sent_monotonic = 0.0
last_control_seq = None


# The WebSocket event loop lives in a dedicated thread.
ws_loop: Optional[asyncio.AbstractEventLoop] = None
ws_clients: Set[ServerConnection] = set()


# ============================================================
# 5. STATUS SNAPSHOT
# ============================================================

def get_status_packet():
    now = time.monotonic()

    with state_lock:
        telemetry_age = None
        control_age = None

        if last_telemetry_received_monotonic:
            telemetry_age = int(
                (now - last_telemetry_received_monotonic) * 1000
            )

        if last_control_sent_monotonic:
            control_age = int(
                (now - last_control_sent_monotonic) * 1000
            )

        return {
            "version": 1,
            "type": "BRIDGE_STATUS",
            "pi_ip": PI_IP,
            "pi_control_port": PI_CONTROL_PORT,
            "laptop_telemetry_port": LAPTOP_TELEMETRY_PORT,
            "expected_control_hz": EXPECTED_CONTROL_HZ,
            "pi_telemetry_age_ms": telemetry_age,
            "last_control_age_ms": control_age,
            "last_control_seq": last_control_seq,
        }


# ============================================================
# 6. BROWSER -> PI
# ============================================================

def forward_control_to_pi(packet: dict):
    """
    Forward the exact application-level CONTROL packet to the Pi.

    This is intentionally JSON over UDP to match the earlier Pi gateway.
    """

    global last_control_sent_monotonic
    global last_control_seq

    data = json.dumps(packet, separators=(",", ":")).encode("utf-8")

    control_socket.sendto(
        data,
        (PI_IP, PI_CONTROL_PORT),
    )

    with state_lock:
        last_control_sent_monotonic = time.monotonic()
        last_control_seq = packet.get("seq")


# ============================================================
# 7. BROADCAST FROM LAPTOP BACKEND -> ALL BROWSERS
# ============================================================

async def broadcast(message: dict):
    if not ws_clients:
        return

    serialized = json.dumps(message, separators=(",", ":"))

    # Iterate over a snapshot so a client can disconnect safely.
    for client in list(ws_clients):
        try:
            await client.send(serialized)
        except Exception:
            ws_clients.discard(client)


# ============================================================
# 8. WEB SOCKET HANDLER
# ============================================================

async def browser_handler(websocket: ServerConnection):
    ws_clients.add(websocket)

    try:
        await websocket.send(
            json.dumps(get_status_packet())
        )

        async for raw_message in websocket:
            try:
                message = json.loads(raw_message)
            except json.JSONDecodeError:
                await websocket.send(
                    json.dumps({
                        "version": 1,
                        "type": "ERROR",
                        "message": "Invalid JSON"
                    })
                )
                continue

            message_type = message.get("type")

            # ------------------------------------------------
            # CONTROL PACKET
            # ------------------------------------------------
            if message_type == "CONTROL":

                # These checks are intentionally simple for now.
                # Add stricter validation before using this on a
                # real aircraft.
                if message.get("version") != 1:
                    continue

                forward_control_to_pi(message)


            # ------------------------------------------------
            # CONFIG PACKET
            # ------------------------------------------------
            elif message_type == "CONFIG":
                # The UI currently uses this only to announce its
                # desired control rate. Keep configuration in this
                # process rather than allowing arbitrary network
                # parameters from the browser.
                await websocket.send(
                    json.dumps(get_status_packet())
                )

            else:
                await websocket.send(
                    json.dumps({
                        "version": 1,
                        "type": "ERROR",
                        "message": f"Unknown message type: {message_type}"
                    })
                )

    except Exception as error:
        print(f"WebSocket client ended: {error}")

    finally:
        ws_clients.discard(websocket)


# ============================================================
# 9. WEBSOCKET SERVER THREAD
# ============================================================

def run_websocket_server():
    global ws_loop

    ws_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(ws_loop)

    async def start():
        # For a local-only control center, binding to 127.0.0.1
        # means browsers on other computers cannot connect directly.
        #
        # If you later intentionally place the UI on another machine,
        # change WEBSOCKET_HOST and configure network/authentication
        # before exposing this endpoint.
        async with serve(
            browser_handler,
            WEBSOCKET_HOST,
            WEBSOCKET_PORT,
            max_size=64 * 1024,
            ping_interval=20,
            ping_timeout=20,
        ):
            print(
                f"WebSocket server: ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}"
            )
            await asyncio.Future()

    ws_loop.run_until_complete(start())


# ============================================================
# 10. PI TELEMETRY RECEIVER
# ============================================================

def telemetry_receiver_loop():
    global last_telemetry
    global last_telemetry_received_monotonic

    while True:
        try:
            data, address = telemetry_socket.recvfrom(65535)
            message = json.loads(data.decode("utf-8"))

            if message.get("type") != "TELEMETRY":
                continue

            received_time_ms = int(time.time() * 1000)
            message["bridge_received_time_ms"] = received_time_ms

            with state_lock:
                last_telemetry = message
                last_telemetry_received_monotonic = time.monotonic()

            # Forward telemetry to all browser clients.
            if ws_loop is not None:
                future = asyncio.run_coroutine_threadsafe(
                    broadcast(message),
                    ws_loop,
                )
                # We don't block the telemetry receiver waiting for
                # the browser sockets to finish sending.
                future.add_done_callback(lambda _: None)

        except socket.timeout:
            continue

        except json.JSONDecodeError:
            print("Received invalid telemetry JSON.")

        except OSError as error:
            print(f"Telemetry socket error: {error}")
            break


# ============================================================
# 11. STATUS BROADCAST LOOP
# ============================================================

def status_loop():
    while True:
        time.sleep(1.0)

        if ws_loop is None:
            continue

        status = get_status_packet()

        future = asyncio.run_coroutine_threadsafe(
            broadcast(status),
            ws_loop,
        )
        future.add_done_callback(lambda _: None)


# ============================================================
# 12. MAIN
# ============================================================

def main():
    print("============================================")
    print("Drone Control Center laptop backend")
    print("============================================")
    print(f"Pi target: {PI_IP}:{PI_CONTROL_PORT}")
    print(f"Telemetry listen port: {LAPTOP_TELEMETRY_PORT}")
    print(f"Browser WebSocket: ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
    print()

    ws_thread = threading.Thread(
        target=run_websocket_server,
        daemon=True,
    )
    ws_thread.start()

    telemetry_thread = threading.Thread(
        target=telemetry_receiver_loop,
        daemon=True,
    )
    telemetry_thread.start()

    status_thread = threading.Thread(
        target=status_loop,
        daemon=True,
    )
    status_thread.start()

    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping laptop backend.")

    finally:
        control_socket.close()
        telemetry_socket.close()


if __name__ == "__main__":
    main()
