# Drone Control Center

This repository contains the active Drone Control Center UI, a Python
WebSocket/UDP bridge, and a standalone synthetic video producer. The current
runtime architecture is:

```text
control_center/ (React + Vite)
   |                         ^
   | WebSocket (JSON)        | VIDEO_FRAME (JSON, base64 JPEG)
   v                         |
laptop_backend/              |  <-- camera frames are relayed here
control_center_backend.py    |
   |                         |
   | UDP JSON                 | standalone test only:
   v                         | control_center/gaussian_video_server.py
Raspberry Pi gateway         | ws://127.0.0.1:8765
   |            |             |
   | MAVLink    | SPI         |
   v            v             |
Flight Ctrl    Pico           |
```

The active control-center browser connects only to
`ws://127.0.0.1:5173`. Control, trigger, telemetry, bridge status, and video
frames therefore need to use that WebSocket unless the frontend is explicitly
changed to open a second video connection.

## Why there is a laptop backend

A normal browser cannot directly open an arbitrary UDP socket to your Raspberry Pi. The browser therefore connects to a Python WebSocket server on the laptop. That server translates browser JSON into the UDP JSON packets expected by `rpi_gateway.py`.

This means the existing Pi-side application remains conceptually unchanged.

## Current application packet

Browser -> laptop backend -> Pi:

```json
{
  "version": 1,
  "type": "CONTROL",
  "seq": 152,
  "time_ms": 1727483000123,
  "flight": {
    "roll": 0.2,
    "pitch": -0.1,
    "yaw": 0.05,
    "throttle": 0.3
  },
  "pico": {
    "output0": 100,
    "output1": 0,
    "output2": -200,
    "output3": 50
  }
}
```

## Interfaces

This project contains two browser interfaces:

1. **Control Center** — the active interface in `control_center/`.
2. **Current Frontend** — the older standalone interface in `current_frontend/`.

The `current_frontend/` interface is retained as a previous iteration. It is not
the implementation of the camera panel described below.

## Run the active Drone Control Center

Requirements:

- Node.js 20.19+ or 22.12+ for the current Vite major.
- npm.

From `control_center/`:

```bash
npm install
npm run dev
```

The active UI is configured for `http://127.0.0.1:6969` in
`control_center/vite.config.ts`. Keep the backend on port 5173; these are
different services.

## Run the older standalone frontend

From `current_frontend/`:

```powershell
npm install
npm run dev
```

The older UI is configured for `http://127.0.0.1:6970` in
`current_frontend/vite.config.js`.

Both interfaces can use the control-center WebSocket at
`ws://127.0.0.1:5173`, but only `control_center/` implements the current
camera panel.

## Run the control center backend

Both browser interfaces use the laptop backend for WebSocket control and
telemetry. Install its dependencies and start
it from `laptop_backend/`:

```powershell
python -m pip install -r requirements.txt
python control_center_backend.py --pi-ip 192.168.1.50
```

The backend listens for browser WebSocket traffic on port 5173, forwards
control and trigger JSON packets to the Raspberry Pi 3B over UDP port 5000,
receives telemetry on UDP port 5001, and relays validated video frames from the
camera producer at `ws://127.0.0.1:8765`. If the producer is not running, the
backend stays available and retries the video connection every two seconds.
Use `Ctrl+C` to stop the backend.

The frontend maps `W/A/S/D` to pitch/roll and the Up/Down arrows to
throttle. Hold a key to send the command; releasing it returns that axis to
neutral.

## Run the synthetic video producer (standalone test only)

The repository includes `control_center/gaussian_video_server.py`. It generates
JPEG frames and serves them as base64 JSON messages on a separate WebSocket.
By default, each frame also contains `water_contact: true` and
`sensors.water_detected: true`, allowing the green water-contact indicator to
be tested through the video path:

```powershell
python control_center\gaussian_video_server.py
```

It listens on `ws://127.0.0.1:8765` by default. The laptop backend connects to
this producer and forwards its frames and water signal to the browser WebSocket
on port 5173. Use `--no-water-detected` to test the inactive state.

## Run the control-center I/O test harness

With the backend running, use the harness from `laptop_backend/` to send
synthetic browser control packets and UDP telemetry:

```powershell
python test_control_center_io.py --duration 10 --output .\logs\control-center.jsonl
```

The harness connects to `ws://127.0.0.1:5173`, sends neutral `CONTROL`
packets, and sends `TELEMETRY` packets to UDP port 5001. Each telemetry packet
contains the IMU fields used by the UI, flight/GPS/battery values, four
ultrasonic readings (`front_cm`, `rear_cm`, `left_cm`, and `right_cm`), and
water status. Every transmitted and received message is saved as one JSON
object per line with its direction and channel. The current frontend displays
the IMU and water fields; ultrasonic values are preserved in the telemetry
packet and log but are not currently rendered.

The `current_frontend/` UI includes flight controls, Pico outputs, telemetry,
link statistics, protocol preview, and the event log. The active
`control_center/` UI contains the camera and IMU panels, live 3D trajectory,
telemetry, control log, water contact, and a 2x2 valve trigger grid. The grid
contains `VALVE 1 ON`, `VALVE 2 ON`, `VALVE 1 OFF`, and `VALVE 2 OFF`. On a
16:10 display,
it uses a two-column grid with relative row heights `4:4:2`.

## Why the frontend cannot accept the streamed video

The frontend originally could not render video because the producer and browser
were on separate WebSockets. The backend now bridges that gap:

1. `control_center/src/main.tsx` opens one WebSocket at
   `ws://127.0.0.1:5173`.
2. It looks for a JSON message with `type: "VIDEO_FRAME"`, then constructs
   `data:image/jpeg;base64,...` for an `<img>`.
3. `laptop_backend/control_center_backend.py` connects to the producer at
   port `8765`, validates JPEG/PNG/WebP `VIDEO_FRAME` messages, and broadcasts
   them to browser clients on port `5173`.
4. `control_center/gaussian_video_server.py` publishes the test frames that
   the backend relays.

Start both processes to test video:

```powershell
python control_center\gaussian_video_server.py
python laptop_backend\control_center_backend.py --pi-ip 192.168.1.50
```

For a real camera producer, pass its endpoint to the backend:

```powershell
python laptop_backend\control_center_backend.py `
  --pi-ip 192.168.1.50 `
  --video-url ws://192.168.1.20:8765
```

For a real camera, the producer must also encode frames as a browser-supported
format (the current test producer uses JPEG), preserve the
`content_type`/`data` fields, and stay within the backend WebSocket
`max_size` (currently 2 MiB) if frames are relayed through port 5173.

## Run the laptop backend

From `laptop_backend/`:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python control_center_backend.py
```

Windows PowerShell activation:

```powershell
.venv\Scripts\Activate.ps1
```

## Change the Pi IP

Edit `laptop_backend/control_center_backend.py`:

```python
PI_IP = "192.168.1.50"
```

This must match the actual IP of the Raspberry Pi.

## Ports

| Link | Port | Location |
|---|---:|---|
| Browser -> laptop | 5173/TCP | WebSocket |
| Active control-center UI | 6969/TCP | Vite HTTP |
| Older standalone UI | 6970/TCP | Vite HTTP |
| Camera producer -> laptop | 8765/TCP | WebSocket video source; relayed by backend |
| Laptop -> Pi | 5000/UDP | Control |
| Pi -> laptop | 5001/UDP | Telemetry |

These values must agree with the corresponding Python files.

## Bench-test order

1. Run the laptop backend.
2. Run the React UI.
3. Confirm Browser ↔ Backend says `CONNECTED`.
4. Keep the vehicle disarmed and verify the UI is sending neutral values.
5. Start the Pi UDP gateway with flight controller and Pico disabled.
6. Confirm the Pi receives increasing sequence numbers.
7. Enable Pico only and confirm the Pico prints the expected output values.
8. Add the flight controller only after the network and Pico links are working.

Do not test with propellers installed while validating the communications stack.

# Expected hiccups and why they happen

## 1. The Pi IP changes

Your router's DHCP may give the Raspberry Pi a different address. The frontend will still work, but the laptop backend will send UDP to the old address.

Later solution: give the Pi a DHCP reservation/static address, or discover it dynamically.

## 2. Windows firewall blocks ports

The browser only talks to localhost, but the laptop still has to receive UDP telemetry on port 5001. A firewall can block that receive path.

Symptom: commands reach the Pi, but telemetry never appears in the UI.

Later solution: add a firewall rule for the telemetry port or use a single TCP/WebSocket connection for the laptop-side interface.

## 3. Wi-Fi drops packets

UDP does not guarantee delivery, ordering, or retransmission. A control packet can disappear without an error.

The packet has a sequence number so you can detect loss later.

For control, latest-command semantics are usually more useful than retrying old commands. Do not build a queue that allows stale commands to execute after newer commands.

## 4. Browser timers are not real-time

The frontend currently emits CONTROL packets at 20 Hz using `setInterval()`.

Browsers can throttle timers, especially in background tabs or under CPU load. The interface is therefore suitable for development and human-in-the-loop control, but it should not be treated as a hard-real-time flight-control loop.

Later solution: move the time-critical loop to the Raspberry Pi or flight controller. The browser should communicate desired setpoints, modes, or high-level commands.

## 5. WebSocket disconnects while the Pi keeps running

The UI automatically attempts to reconnect to the laptop backend.

The Pi gateway has its own control-link timeout. Keep that timeout as the aircraft-side safety mechanism; do not rely only on the browser.

## 6. JSON becomes too large or too slow

JSON is intentionally used because you can inspect packets easily.

If the control/telemetry rate grows substantially, switch the laptop↔Pi protocol to a compact binary message format. Keep the field definitions documented and version the protocol.

## 7. Sequence number wraps

The current sequence number is an 8-bit value, so it wraps from 255 back to 0.

That is acceptable for a prototype. Later, make it uint32 if you want longer sequence history and easier debugging.

## 8. Coordinate/sign conventions differ

Roll and pitch direction can be opposite to what the UI visually suggests depending on your joystick, vehicle frame, and flight-stack convention.

Do not assume `+roll` or `+pitch` means the same physical direction everywhere.

Later solution: define a single explicit convention and document it, e.g. body-frame FLU/FRD, positive directions, units, and normalized ranges.

## 9. MAVLink manual-control requirements differ

The example uses `MANUAL_CONTROL`. Exact behavior depends on the flight controller and flight-stack configuration.

You may eventually need a different MAVLink command/message set, for example attitude targets, velocity targets, position targets, or custom commands.

Do not change only the UI. Change the documented command mapping and the Pi gateway together.

## 10. SPI slave behavior can surprise you

The Pico cannot independently start an SPI transfer. The Raspberry Pi must provide the SPI clock.

Later, when the Pico needs to send telemetry back, use a request/response design or add a READY GPIO so the Pi knows when the Pico has data ready.

## 11. Multiple browser tabs connect at once

The backend currently accepts multiple WebSocket clients and broadcasts telemetry to them.

However, all connected browsers can send commands to the same Pi. For actual operation, you should later enforce a single controlling client and make additional clients read-only.

## 12. Control authority and authentication

The current WebSocket endpoint is intentionally a development endpoint on `127.0.0.1` and has no authentication.

If you move the UI to another computer, bind the WebSocket server to a network interface only after adding authentication and an explicit authorization model.

## 13. Frontend and backend schema drift

This is one of the most likely maintenance problems.

If you change:

```text
flight.roll
```

to:

```text
controls.roll
```

then both the React frontend and Python gateway must be updated.

Use a versioned protocol document and keep `version` in every packet.

# What you will probably change later

| Current prototype | Likely later version |
|---|---|
| JSON | Binary/CBOR/MessagePack/custom binary |
| UDP laptop -> Pi | UDP remains possible; add stronger link supervision |
| Browser WebSocket | WebSocket remains, or native desktop app |
| 20 Hz browser timer | Hardware/software loop on Pi |
| Manual sliders | Joystick/gamepad |
| Four generic Pico outputs | Named actuators/sensors |
| Simple checksum on SPI | CRC-16/CRC-32 |
| 8-bit sequence | uint32 sequence number |
| No authentication | Authenticated control client |
| Multiple writers | One controller + read-only observers |
| Fixed Pi IP | DHCP reservation/discovery |
| `MANUAL_CONTROL` | Flight-stack-specific setpoint/control API |
| Basic timeout | Explicit watchdog + mode/failsafe state machine |
| No command ACK | ACK/NACK + command IDs |
| No timestamps checked | Timestamp/age validation on Pi |

# One important architectural recommendation

Keep the flight controller responsible for the fast stabilization/control loop.

A useful separation is:

```text
Browser
  high-level commands / desired setpoints

        ↓

Laptop bridge
  networking + UI integration

        ↓

Raspberry Pi
  onboard autonomy + command translation

        ↓

Flight controller
  fast attitude/rate stabilization + failsafes

        ↕

Pico
  deterministic peripheral/actuator/sensor I/O
```

That architecture means a temporary browser freeze should not directly become a flight-control-loop failure.
