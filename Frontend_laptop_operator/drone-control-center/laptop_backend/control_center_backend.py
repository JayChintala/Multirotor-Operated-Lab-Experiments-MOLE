"""WebSocket control center bridge for a Raspberry Pi 3B.

The browser connects to port 5173. Control and trigger messages are forwarded
as JSON over UDP to the Pi. Telemetry is received from UDP port 5001 and
broadcast back to browser clients. Video frames are received from a camera
producer over WebSocket and relayed to the browser clients.
"""

import argparse
import asyncio
import json
import math
import os
import socket
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Set

from websockets.asyncio.client import connect
from websockets.asyncio.server import ServerConnection, serve


MAX_WEBSOCKET_MESSAGE_SIZE = 2 * 1024 * 1024


class Bridge:
    def __init__(
        self,
        pi_ip: str,
        pi_control_port: int,
        telemetry_port: int,
        host: str,
        websocket_port: int,
        video_url: str,
    ) -> None:
        self.pi_ip = pi_ip
        self.pi_control_port = pi_control_port
        self.telemetry_port = telemetry_port
        self.host = host
        self.websocket_port = websocket_port
        self.video_url = video_url
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.telemetry_socket.bind(("0.0.0.0", telemetry_port))
        self.telemetry_socket.settimeout(0.2)
        self.state_lock = threading.Lock()
        self.clients: Set[ServerConnection] = set()
        self.ws_loop: Optional[asyncio.AbstractEventLoop] = None
        self.last_telemetry_monotonic = 0.0
        self.last_control_monotonic = 0.0
        self.last_control_seq = 0
        self.sequence = 0
        self.video_connected = False
        self.stopping = threading.Event()
        self.log_lock = threading.Lock()
        logs_dir = Path(__file__).resolve().parent.parent / "logs"
        logs_dir.mkdir(parents=True, exist_ok=True)
        log_name = f"control_center_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')}_{os.getpid()}.jsonl"
        self.log_path = logs_dir / log_name
        self.log_file = self.log_path.open("a", encoding="utf-8")
        self.log_json("system", "startup", self.status())
        print(f"JSON log: {self.log_path}")

    def send_udp(self, message: dict) -> None:
        payload = json.dumps(message, separators=(",", ":")).encode("utf-8")
        self.control_socket.sendto(payload, (self.pi_ip, self.pi_control_port))
        self.log_json("tx", "udp", message)

    def log_json(self, direction: str, channel: str, message: dict) -> None:
        record = {
            "logged_time_ms": int(time.time() * 1000),
            "direction": direction,
            "channel": channel,
            "message": message,
        }
        try:
            with self.log_lock:
                self.log_file.write(json.dumps(record, separators=(",", ":")) + "\n")
                self.log_file.flush()
        except OSError as error:
            print(f"JSON logging failed: {error}")

    def status(self) -> dict:
        now = time.monotonic()
        with self.state_lock:
            telemetry_age = int((now - self.last_telemetry_monotonic) * 1000) if self.last_telemetry_monotonic else None
            control_age = int((now - self.last_control_monotonic) * 1000) if self.last_control_monotonic else None
            return {
                "version": 1,
                "type": "BRIDGE_STATUS",
                "pi_ip": self.pi_ip,
                "pi_control_port": self.pi_control_port,
                "laptop_telemetry_port": self.telemetry_port,
                "pi_telemetry_age_ms": telemetry_age,
                "last_control_age_ms": control_age,
                "last_control_seq": self.last_control_seq,
                "video_url": self.video_url,
                "video_connected": self.video_connected,
            }

    async def broadcast(self, message: dict) -> None:
        serialized = json.dumps(message, separators=(",", ":"))
        for client in list(self.clients):
            try:
                await client.send(serialized)
            except Exception as error:
                print(f"WebSocket broadcast failed: {error}")
                self.clients.discard(client)

    @staticmethod
    def video_frame(message: object) -> Optional[dict]:
        if not isinstance(message, dict):
            return None
        if message.get("type") != "VIDEO_FRAME":
            return None
        if message.get("content_type") not in {"image/jpeg", "image/png", "image/webp"}:
            return None
        if not isinstance(message.get("data"), str) or not message["data"]:
            return None
        return {
            "version": 1,
            "type": "VIDEO_FRAME",
            "seq": message.get("seq"),
            "time_ms": message.get("time_ms"),
            "content_type": message["content_type"],
            "data": message["data"],
            "water_contact": message.get("water_contact") is True,
            "sensors": {
                "water_detected": (
                    isinstance(message.get("sensors"), dict)
                    and message["sensors"].get("water_detected") is True
                ),
            },
        }

    async def video_loop(self) -> None:
        while True:
            try:
                async with connect(
                    self.video_url,
                    max_size=MAX_WEBSOCKET_MESSAGE_SIZE,
                    ping_interval=20,
                    ping_timeout=20,
                ) as camera:
                    with self.state_lock:
                        self.video_connected = True
                    print(f"Video producer connected: {self.video_url}")
                    async for raw_message in camera:
                        try:
                            message = json.loads(raw_message)
                        except (TypeError, json.JSONDecodeError):
                            print("Received invalid video JSON.")
                            continue
                        frame = self.video_frame(message)
                        if frame is None:
                            print("Received invalid VIDEO_FRAME message.")
                            continue
                        await self.broadcast(frame)
            except asyncio.CancelledError:
                raise
            except Exception as error:
                print(f"Video producer unavailable: {error}")
            finally:
                with self.state_lock:
                    self.video_connected = False
            await asyncio.sleep(2.0)

    @staticmethod
    def valid_axis_values(values: object, required: tuple[str, ...], minimum: float, maximum: float) -> bool:
        return (
            isinstance(values, dict)
            and all(
                key in values
                and isinstance(values[key], (int, float))
                and not isinstance(values[key], bool)
                and math.isfinite(float(values[key]))
                and minimum <= float(values[key]) <= maximum
                for key in required
            )
        )

    def control_packet(self, message: dict) -> Optional[dict]:
        flight = message.get("flight")
        if not self.valid_axis_values(flight, ("roll", "pitch", "yaw", "throttle"), -1.0, 1.0):
            return None
        if float(flight["throttle"]) < 0.0:
            return None
        pico = message.get("pico", {
            "output0": 0,
            "output1": 0,
            "output2": 0,
            "output3": 0,
        })
        if not self.valid_axis_values(pico, ("output0", "output1", "output2", "output3"), -1000, 1000):
            return None
        packet = {
            "version": 1,
            "type": "CONTROL",
            "seq": self.sequence % 256,
            "time_ms": int(time.time() * 1000),
            "flight": {key: float(flight[key]) for key in ("roll", "pitch", "yaw", "throttle")},
            "pico": {key: int(pico[key]) for key in ("output0", "output1", "output2", "output3")},
        }
        self.sequence += 1
        return packet

    async def browser_handler(self, websocket: ServerConnection) -> None:
        self.clients.add(websocket)
        try:
            await websocket.send(json.dumps(self.status()))
            async for raw_message in websocket:
                try:
                    message = json.loads(raw_message)
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({"version": 1, "type": "ERROR", "message": "Invalid JSON"}))
                    continue
                self.log_json("rx", "websocket", message)

                message_type = message.get("type")
                if message_type == "CONTROL":
                    if message.get("version") != 1:
                        await websocket.send(json.dumps({"version": 1, "type": "ERROR", "message": "Invalid CONTROL packet"}))
                        continue
                    packet = self.control_packet(message)
                    if packet is None:
                        await websocket.send(json.dumps({"version": 1, "type": "ERROR", "message": "Invalid CONTROL values"}))
                        continue
                    self.send_udp(packet)
                    with self.state_lock:
                        self.last_control_monotonic = time.monotonic()
                        self.last_control_seq = packet["seq"]
                elif message_type == "TRIGGER":
                    name = message.get("name")
                    if name not in {"valve_1_on", "valve_2_on", "valve_1_off", "valve_2_off"}:
                        await websocket.send(json.dumps({"version": 1, "type": "ERROR", "message": "Unknown trigger"}))
                        continue
                    self.send_udp({
                        "version": 1,
                        "type": "TRIGGER",
                        "name": name,
                        "time_ms": int(time.time() * 1000),
                    })
                elif message_type == "CONFIG":
                    await websocket.send(json.dumps(self.status()))
                else:
                    await websocket.send(json.dumps({"version": 1, "type": "ERROR", "message": f"Unknown message type: {message_type}"}))
        except Exception as error:
            print(f"WebSocket client ended: {error}")
        finally:
            self.clients.discard(websocket)

    def telemetry_loop(self) -> None:
        while not self.stopping.is_set():
            try:
                data, address = self.telemetry_socket.recvfrom(65535)
                message = json.loads(data.decode("utf-8"))
                if message.get("type") != "TELEMETRY":
                    continue
                self.log_json("rx", "udp", message)
                message["bridge_received_time_ms"] = int(time.time() * 1000)
                with self.state_lock:
                    self.last_telemetry_monotonic = time.monotonic()
                if self.ws_loop is not None:
                    asyncio.run_coroutine_threadsafe(self.broadcast(message), self.ws_loop)
            except socket.timeout:
                continue
            except json.JSONDecodeError:
                print("Received invalid telemetry JSON.")
            except OSError as error:
                if not self.stopping.is_set():
                    print(f"Telemetry socket error from {address}: {error}")
                return

    async def status_loop(self) -> None:
        while True:
            await asyncio.sleep(1.0)
            await self.broadcast(self.status())

    async def run(self) -> None:
        self.ws_loop = asyncio.get_running_loop()
        telemetry_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        telemetry_thread.start()
        async with serve(self.browser_handler, self.host, self.websocket_port, max_size=MAX_WEBSOCKET_MESSAGE_SIZE, ping_interval=20, ping_timeout=20):
            print(f"WebSocket: ws://{self.host}:{self.websocket_port}")
            print(f"Pi UDP target: {self.pi_ip}:{self.pi_control_port}")
            print(f"Telemetry UDP listen: 0.0.0.0:{self.telemetry_port}")
            await asyncio.gather(self.status_loop(), self.video_loop())

    def close(self) -> None:
        self.stopping.set()
        self.control_socket.close()
        self.telemetry_socket.close()
        with self.log_lock:
            self.log_file.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi-ip", default="192.168.1.50")
    parser.add_argument("--pi-control-port", type=int, default=5000)
    parser.add_argument("--telemetry-port", type=int, default=5001)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5173)
    parser.add_argument("--video-url", default="ws://127.0.0.1:8765")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bridge = Bridge(
        args.pi_ip,
        args.pi_control_port,
        args.telemetry_port,
        args.host,
        args.port,
        args.video_url,
    )
    try:
        asyncio.run(bridge.run())
    except KeyboardInterrupt:
        print("\nStopping laptop backend.")
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
