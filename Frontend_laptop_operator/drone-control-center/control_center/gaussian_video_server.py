"""Stream synthetic Gaussian-noise video to the new control center.

The laptop backend connects to this process over WebSocket on port 8765. Each
frame is a JPEG encoded as base64 inside a JSON VIDEO_FRAME message, along with
a synthetic water-detected signal for frontend testing.
"""

import argparse
import asyncio
import base64
import io
import json
import random
import time
from typing import Optional

from PIL import Image
from websockets.asyncio.server import ServerConnection, serve


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 360
DEFAULT_FPS = 12


def create_noise_frame(width: int, height: int, rng: random.Random) -> str:
    """Return a JPEG data payload containing grayscale Gaussian noise."""
    pixels = bytearray(
        max(0, min(255, int(rng.gauss(128, 42))))
        for _ in range(width * height)
    )
    image = Image.frombytes("L", (width, height), bytes(pixels))
    output = io.BytesIO()
    image.save(output, format="JPEG", quality=80)
    return base64.b64encode(output.getvalue()).decode("ascii")


async def stream_video(
    websocket: ServerConnection,
    width: int,
    height: int,
    fps: float,
    water_detected: bool,
) -> None:
    rng = random.Random()
    frame_interval = 1.0 / fps
    sequence = 0

    while True:
        started = time.perf_counter()
        frame = await asyncio.to_thread(create_noise_frame, width, height, rng)
        await websocket.send(
            json.dumps(
                {
                    "type": "VIDEO_FRAME",
                    "seq": sequence,
                    "time_ms": int(time.time() * 1000),
                    "content_type": "image/jpeg",
                    "data": frame,
                    "water_contact": water_detected,
                    "sensors": {"water_detected": water_detected},
                },
                separators=(",", ":"),
            )
        )
        sequence += 1
        await asyncio.sleep(max(0, frame_interval - (time.perf_counter() - started)))


async def handle_client(
    websocket: ServerConnection,
    width: int,
    height: int,
    fps: float,
    water_detected: bool,
) -> None:
    print(f"Client connected: {websocket.remote_address}")
    try:
        await stream_video(websocket, width, height, fps, water_detected)
    except Exception as error:
        print(f"Client disconnected: {error}")


async def run_server(
    host: str,
    port: int,
    width: int,
    height: int,
    fps: float,
    water_detected: bool,
) -> None:
    async with serve(
        lambda websocket: handle_client(websocket, width, height, fps, water_detected),
        host,
        port,
    ):
        print(f"Gaussian video WebSocket: ws://{host}:{port}")
        print(f"Streaming {width}x{height} JPEG frames at {fps:g} FPS; water_detected={water_detected}")
        await asyncio.Future()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    parser.add_argument(
        "--water-detected",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Include water_detected=true in each video frame (default: enabled)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise SystemExit("width, height, and fps must be greater than zero")

    try:
        asyncio.run(
            run_server(
                args.host,
                args.port,
                args.width,
                args.height,
                args.fps,
                args.water_detected,
            )
        )
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
