# Drone Control Center

Local React/Vite frontend + Python WebSocket/UDP bridge for the architecture:

```text
Browser UI
   |
   | WebSocket (JSON)
   v
Laptop Python backend
   |
   | UDP (JSON)
   v
Raspberry Pi gateway
   |            |
   | MAVLink    | SPI
   v            v
Flight Ctrl    Pico
```

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

## Run the frontend

Requirements:

- Node.js 20.19+ or 22.12+ for the current Vite major.
- npm.

From `frontend/`:

```bash
npm install
npm run dev
```

Open the URL printed by Vite, normally `http://localhost:5173`.

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
| Browser -> laptop | 8765/TCP | WebSocket |
| Laptop -> Pi | 5000/UDP | Control |
| Pi -> laptop | 5001/UDP | Telemetry |

These values must agree with the corresponding Python files.

## Bench-test order

1. Run the laptop backend.
2. Run the React UI.
3. Confirm Browser ↔ Backend says `CONNECTED`.
4. Set `controlEnabled` off initially. The UI sends neutral values.
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
