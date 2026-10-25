"""Bench-test the control-center WebSocket and UDP interfaces.

This script does not connect to a flight controller. It sends deterministic
synthetic telemetry to the laptop backend and records every packet exchanged
with the backend in JSON Lines format.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import socket
import time
from pathlib import Path
from typing import Any

from websockets.asyncio.client import ClientConnection, connect
from websockets.exceptions import WebSocketException


DEFAULT_OUTPUT = Path("control_center_io.jsonl")


class EventLog:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.file = path.open("w", encoding="utf-8")

    def write(self, direction: str, channel: str, message: dict[str, Any]) -> None:
        record = {
            "logged_time_ms": int(time.time() * 1000),
            "direction": direction,
            "channel": channel,
            "message": message,
        }
        self.file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.file.flush()

    def close(self) -> None:
        self.file.close()


def telemetry_packet(sequence: int, elapsed: float) -> dict[str, Any]:
    """Create one packet using the field names consumed by both frontends."""
    phase = elapsed * 0.8
    return {
        "version": 1,
        "type": "TELEMETRY",
        "seq": sequence % 256,
        "time_ms": int(time.time() * 1000),
        "fc": {
            "flight_mode": "STABILIZE",
            "armed": False,
            "attitude": {
                "roll": round(5.0 * math.sin(phase), 3),
                "pitch": round(3.0 * math.cos(phase), 3),
                "yaw": round((elapsed * 10.0) % 360.0, 3),
            },
            "acceleration": {
                "x": round(0.1 * math.sin(phase), 3),
                "y": round(0.1 * math.cos(phase), 3),
                "z": 9.81,
            },
            "angular_velocity": {
                "x": round(0.02 * math.sin(phase), 4),
                "y": round(0.02 * math.cos(phase), 4),
                "z": 0.01,
            },
            "position": {
                "x": round(8.0 * math.sin(phase), 3),
                "y": round(8.0 * math.cos(phase), 3),
                "z": round(12.0 + 0.2 * math.sin(phase), 3),
                "lat": 40.4432,
                "lon": -79.9436,
                "alt": round(12.0 + 0.2 * math.sin(phase), 3),
                "relative_alt_m": round(12.0 + 0.2 * math.sin(phase), 3),
            },
            "battery": {
                "voltage": 15.8,
                "voltage_v": 15.8,
                "remaining": 87,
                "remaining_percent": 87,
            },
        },
        # The current UI does not render this object yet, but the backend
        # forwards it unchanged and the log preserves all four readings.
        "ultrasonic": {
            "front_cm": round(120.0 + 5.0 * math.sin(phase), 2),
            "rear_cm": round(135.0 + 5.0 * math.cos(phase), 2),
            "left_cm": round(95.0 + 3.0 * math.sin(phase), 2),
            "right_cm": round(100.0 + 3.0 * math.cos(phase), 2),
        },
        "water_contact": False,
        "sensors": {
            "water_detected": False,
            "temperature_c": 22.5,
            "pressure_hpa": 1013.25,
        },
    }


def control_packet(sequence: int) -> dict[str, Any]:
    return {
        "version": 1,
        "type": "CONTROL",
        "seq": sequence % 256,
        "time_ms": int(time.time() * 1000),
        "flight": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0},
        "pico": {"output0": 0, "output1": 0, "output2": 0, "output3": 0},
        "source": "test_control_center_io",
    }


async def send_telemetry(
    host: str,
    port: int,
    rate_hz: float,
    duration: float,
    log: EventLog,
) -> None:
    address = (host, port)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
        started = time.monotonic()
        sequence = 0
        interval = 1.0 / rate_hz
        next_send = started
        while time.monotonic() - started < duration:
            elapsed = time.monotonic() - started
            message = telemetry_packet(sequence, elapsed)
            payload = json.dumps(message, separators=(",", ":")).encode("utf-8")
            udp_socket.sendto(payload, address)
            log.write("tx", "udp-telemetry", message)
            sequence += 1
            next_send += interval
            await asyncio.sleep(max(0.0, next_send - time.monotonic()))


async def exercise_websocket(
    ws_url: str,
    rate_hz: float,
    duration: float,
    log: EventLog,
) -> None:
    try:
        async with connect(ws_url, max_size=2 * 1024 * 1024) as websocket:
            await websocket.send(json.dumps({
                "version": 1,
                "type": "CONFIG",
                "desired_control_hz": rate_hz,
                "source": "test_control_center_io",
            }))
            log.write("tx", "websocket", {
                "version": 1,
                "type": "CONFIG",
                "desired_control_hz": rate_hz,
                "source": "test_control_center_io",
            })

            started = time.monotonic()
            next_control = started
            sequence = 0
            while time.monotonic() - started < duration:
                while True:
                    try:
                        raw_message = await asyncio.wait_for(websocket.recv(), timeout=0.01)
                    except asyncio.TimeoutError:
                        break
                    if isinstance(raw_message, bytes):
                        log.write("rx", "websocket", {
                            "type": "BINARY_MESSAGE",
                            "size": len(raw_message),
                        })
                        continue
                    message = json.loads(raw_message)
                    log.write("rx", "websocket", message)

                now = time.monotonic()
                if now >= next_control:
                    message = control_packet(sequence)
                    await websocket.send(json.dumps(message, separators=(",", ":")))
                    log.write("tx", "websocket", message)
                    sequence += 1
                    next_control += 1.0 / rate_hz
                await asyncio.sleep(0.001)

            # Drain short-lived final status/telemetry messages before closing.
            end = time.monotonic() + 0.25
            while time.monotonic() < end:
                try:
                    raw_message = await asyncio.wait_for(websocket.recv(), timeout=0.05)
                except asyncio.TimeoutError:
                    continue
                if isinstance(raw_message, str):
                    log.write("rx", "websocket", json.loads(raw_message))
    except (OSError, WebSocketException) as error:
        raise RuntimeError(
            f"Could not connect to {ws_url}. Start control_center_backend.py first."
        ) from error


async def run(args: argparse.Namespace) -> None:
    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    log = EventLog(output_path)
    try:
        await asyncio.gather(
            exercise_websocket(args.ws_url, args.rate_hz, args.duration, log),
            send_telemetry(
                args.telemetry_host,
                args.telemetry_port,
                args.rate_hz,
                args.duration,
                log,
            ),
        )
    finally:
        log.close()
    print(f"Saved control-center I/O log to {output_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ws-url", default="ws://127.0.0.1:5173")
    parser.add_argument("--telemetry-host", default="127.0.0.1")
    parser.add_argument("--telemetry-port", type=int, default=5001)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be greater than zero")
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be greater than zero")
    return args


def main() -> None:
    try:
        asyncio.run(run(parse_args()))
    except RuntimeError as error:
        raise SystemExit(str(error)) from error


if __name__ == "__main__":
    main()
