# Drone Control Center: System Understanding Guide

This document explains the current project as it exists in this repository.
There are two browser applications:

- `control_center/`: the active Drone Control Center. This is the application
  with the camera panel, water-contact panel, trajectory view, telemetry,
  control log, and valve triggers.
- `current_frontend/`: the older standalone control interface. It contains
  flight sliders, Pico output controls, telemetry, connection controls, and a
  protocol preview, but it is not the active camera/water/valve interface.

The Python laptop backend is shared by both browser applications.

## 1. System architecture

### 1.1 High-level diagram

```text
                          Synthetic/test camera
                    control_center/gaussian_video_server.py
                                   |
                                   | WebSocket JSON
                                   | ws://127.0.0.1:8765
                                   v
Browser UI                    Laptop backend
control_center/         <----> laptop_backend/
React + Vite             WS     control_center_backend.py
                         5173       |
                           |         | UDP JSON controls/triggers
                           |         v
                           |    Raspberry Pi gateway
                           |         |
                           |         | MAVLink
                           |         v
                           |    Flight controller
                           |
                           |         SPI
                           |         v
                           |       Pico
                           ^
                           |
              UDP JSON telemetry from Pi, port 5001
```

The browser does not communicate directly with the Raspberry Pi. The browser
opens a WebSocket to the Python backend on the laptop. The backend translates
browser control and trigger messages into UDP JSON packets for the Pi, and
translates incoming Pi telemetry back into browser WebSocket messages.

The video path is separate from the Pi control path:

1. A camera producer publishes `VIDEO_FRAME` messages to the backend's camera
   input WebSocket, normally `ws://127.0.0.1:8765`.
2. The backend validates the frame and relays it to browser clients on
   `ws://127.0.0.1:5173`.
3. The active `control_center` browser decodes the base64 image into a data URL
   and displays it in an image element.

The synthetic video producer also includes a water signal in each frame so the
active UI can be tested without hardware:

```json
{
  "type": "VIDEO_FRAME",
  "seq": 12,
  "time_ms": 1727483000123,
  "content_type": "image/jpeg",
  "data": "<base64 JPEG>",
  "water_contact": true,
  "sensors": {
    "water_detected": true
  }
}
```

The backend forwards the frame and the normalized water fields to the browser.
The active UI turns the water-contact box green when either
`water_contact === true` or `sensors.water_detected === true`.

### 1.2 Control path

The browser sends `CONTROL` messages to the laptop backend. A typical browser
message is:

```json
{
  "version": 1,
  "type": "CONTROL",
  "seq": 15,
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

The backend validates the flight values in the range `-1.0` to `1.0`. Throttle
must be non-negative. Pico values are validated in the range `-1000` to `1000`.
The backend creates its own outgoing sequence number and sends the normalized
packet to the Pi over UDP.

The active control-center keyboard mapping is:

| Input | Control |
|---|---|
| `W` | Positive pitch |
| `S` | Negative pitch |
| `A` | Negative roll |
| `D` | Positive roll |
| Up arrow | Positive throttle |
| Down arrow | Negative throttle, clamped by backend validation |

The browser sends control packets at 20 Hz. This is suitable for a
development/human-in-the-loop interface, not for a hard-real-time
stabilization loop.

### 1.3 Trigger path

The active control center displays four valve buttons:

```text
Valve 1 on     Valve 2 on
Valve 1 off    Valve 2 off
```

They send `TRIGGER` messages such as:

```json
{
  "version": 1,
  "type": "TRIGGER",
  "name": "valve_1_on",
  "time_ms": 1727483000123
}
```

The backend currently accepts:

- `valve_1_on`
- `valve_2_on`
- `valve_1_off`
- `valve_2_off`

It forwards accepted triggers as UDP JSON to the Pi. The Pi gateway must map
these names to the actual GPIO, actuator, or Pico command required by the
hardware.

### 1.4 Telemetry path

The Pi sends `TELEMETRY` JSON datagrams to the laptop's UDP port 5001. The
backend adds `bridge_received_time_ms`, tracks telemetry age, and broadcasts
the message to all connected browser clients.

The telemetry structure used by the current test harness includes:

- Flight mode and armed state
- Attitude: roll, pitch, yaw
- Acceleration
- Angular velocity
- Position, including local or GPS fields
- Battery voltage and remaining percentage
- Ultrasonic readings
- Water-contact fields
- Additional sensor fields

The active UI uses the flight-controller fields for the IMU, trajectory, and
status panels. The telemetry packet remains extensible; fields not rendered by
the UI are still forwarded.

## 2. What the files do

### 2.1 `laptop_backend/`

#### `laptop_backend/control_center_backend.py`

This is the main laptop bridge.

Responsibilities:

- Starts a WebSocket server for browser clients.
- Listens for browser `CONTROL`, `TRIGGER`, and `CONFIG` messages.
- Validates control-axis and Pico values.
- Sends control and trigger packets to the Pi over UDP.
- Listens for Pi telemetry on UDP port 5001.
- Adds bridge timing metadata to telemetry.
- Broadcasts telemetry and bridge status to browser clients.
- Connects to a video producer over WebSocket.
- Reconnects to the video producer every two seconds if it is unavailable.
- Validates JPEG, PNG, and WebP `VIDEO_FRAME` messages.
- Relays validated video frames and water fields to browser clients.

Important command-line options:

```text
--pi-ip          Raspberry Pi address; default 192.168.1.50
--pi-control-port UDP destination for Pi controls; default 5000
--telemetry-port  UDP port where laptop receives telemetry; default 5001
--host            Laptop WebSocket bind host; default 127.0.0.1
--port            Browser WebSocket port; default 5173
--video-url       Camera producer WebSocket; default ws://127.0.0.1:8765
```

The backend binds the telemetry UDP socket to `0.0.0.0`, so it can receive
telemetry arriving on any laptop network interface. The browser WebSocket
server defaults to localhost for development safety.

#### `laptop_backend/test_control_center_io.py`

This is the bench-test harness. It does not connect to a real flight
controller. It simultaneously:

- Connects to the browser WebSocket endpoint.
- Sends a `CONFIG` message.
- Sends neutral `CONTROL` packets at a configurable rate.
- Sends synthetic telemetry UDP datagrams to the laptop.
- Receives backend status and telemetry messages.
- Writes transmitted and received messages to JSON Lines format.

The output log is useful for checking packet schemas, sequence numbers,
telemetry forwarding, and timing without operating the aircraft.

#### `laptop_backend/requirements.txt`

Declares the Python WebSocket dependency:

```text
websockets>=17,<18
```

#### `laptop_backend/.env.example`

Contains example network settings/reference values. The backend currently uses
command-line arguments for its primary runtime configuration, so verify the
actual Python script before assuming an environment variable is active.

#### `laptop_backend/control_center_io.jsonl`

This is generated test output, not application source. Each line records one
transmitted or received event with its direction, channel, timestamp, and
message.

### 2.2 `current_frontend/`

This is the older standalone React/Vite application.

#### `current_frontend/src/main.jsx`

Bootstraps React and mounts the `App` component into the HTML root element.

#### `current_frontend/src/App.jsx`

Contains the older control UI:

- WebSocket URL input
- Connect and disconnect controls
- Automatic reconnect behavior
- Flight control sliders
- Control enable/disable state
- Pico output inputs
- Telemetry display
- Link statistics
- Protocol preview
- Event log

It connects to the configurable WebSocket URL stored in browser local storage
under `wsUrl`. Its default is `ws://127.0.0.1:5173`.

This application is not the active camera/water/valve UI. Use
`control_center/` for those features.

#### `current_frontend/src/styles.css`

Defines the older application's layout, cards, controls, telemetry styles,
status indicators, and responsive behavior.

#### `current_frontend/index.html`

Vite HTML entry point. It provides the page shell and loads
`src/main.jsx`.

#### `current_frontend/vite.config.js`

Configures the older frontend to bind to `127.0.0.1:6970`.

#### `current_frontend/package.json`

Defines the npm scripts and dependencies for the older frontend. Common
commands are:

```powershell
npm install
npm run dev
npm run build
```

### 2.3 `control_center/`

This is the active Drone Control Center application and synthetic video
producer.

#### `control_center/src/main.tsx`

Contains the active React UI and its WebSocket client.

Responsibilities:

- Connects to `ws://127.0.0.1:5173`.
- Reconnects after a WebSocket close.
- Sends a configuration message after connection.
- Displays relayed JPEG/PNG/WebP video frames.
- Reads water signals from video frames and telemetry.
- Displays the green water-contact state.
- Displays IMU values from telemetry.
- Builds a trajectory from local position or GPS values.
- Sends flight controls at 20 Hz.
- Maps keyboard input to roll, pitch, and throttle.
- Sends the four valve trigger commands.
- Displays telemetry, logs, and trigger status.

#### `control_center/src/style.css`

Defines the active UI's full-screen two-column layout, camera panel, IMU
panel, telemetry/trajectory panels, control log, water-contact box, and
2x2 valve trigger grid.

#### `control_center/gaussian_video_server.py`

Provides a synthetic video source for development. It:

- Generates grayscale Gaussian-noise JPEG frames.
- Encodes each JPEG as base64.
- Publishes `VIDEO_FRAME` JSON messages over WebSocket port 8765.
- Includes `water_contact` and `sensors.water_detected` in each frame.
- Defaults to water detected so the green UI state is easy to test.

Use `--no-water-detected` to publish false water signals.

It is a test producer, not a real camera driver.

#### `control_center/index.html`

Vite HTML entry point for the active application. It loads
`src/main.tsx`.

#### `control_center/vite.config.ts`

Configures the active frontend to bind to `127.0.0.1:6969`.

#### `control_center/package.json`

Defines the active frontend's npm scripts and React/Vite dependencies.

#### `control_center/requirements.txt`

Contains Python dependencies needed by the synthetic video server:

- Pillow for JPEG generation
- websockets for the video WebSocket server

## 3. Ports and what they mean

| Port | Transport | Endpoints | Purpose |
|---:|---|---|---|
| `6969` | TCP/HTTP | Vite -> browser | Serves the active `control_center/` web application |
| `6970` | TCP/HTTP | Vite -> browser | Serves the older `current_frontend/` application |
| `5173` | TCP/WebSocket | Browser <-> laptop backend | Carries controls, triggers, status, telemetry, and relayed video frames |
| `8765` | TCP/WebSocket | Video producer -> laptop backend | Carries synthetic or real camera `VIDEO_FRAME` messages |
| `5000` | UDP | Laptop -> Raspberry Pi | Carries validated `CONTROL` and `TRIGGER` JSON packets |
| `5001` | UDP | Raspberry Pi -> laptop | Carries `TELEMETRY` JSON packets |

### 3.1 Port usage details

#### Port 6969: active web UI

This is an HTTP development server supplied by Vite. It serves JavaScript,
CSS, and the HTML shell. It is not the control protocol port.

#### Port 6970: older web UI

This is the Vite development server for `current_frontend/`. It is optional
and can be stopped if only the active control center is needed.

#### Port 5173: browser/backend WebSocket

The active browser connects here. The backend sends:

- `BRIDGE_STATUS`
- `TELEMETRY`
- `VIDEO_FRAME`
- `ERROR`

The browser sends:

- `CONFIG`
- `CONTROL`
- `TRIGGER`

#### Port 8765: camera producer WebSocket

The backend is the client on this connection. A camera process is the server.
For local testing, `gaussian_video_server.py` listens here. For a real camera,
the camera producer can run on another address and the backend can be started
with a different `--video-url`.

#### UDP port 5000: Pi command destination

The laptop sends JSON datagrams here. The Pi-side gateway must bind or receive
on this port and parse `CONTROL` and `TRIGGER` packets.

#### UDP port 5001: laptop telemetry receiver

The laptop backend binds this port and waits for datagrams from the Pi. The
Pi should send `TELEMETRY` JSON packets to the laptop's IP address and port
5001.

UDP is connectionless and does not guarantee delivery, ordering, or
retransmission. The protocol includes sequence numbers and timestamps so the
Pi and laptop can detect stale or missing data.

## 4. Test scripts and their functions

### 4.1 `test_control_center_io.py`

Run it after starting the laptop backend:

```powershell
cd C:\Users\ASUS\Desktop\CMU\Sem1\EMSD\Frontend\drone-control-center\laptop_backend
..\.venv\Scripts\python.exe test_control_center_io.py `
  --duration 10 `
  --rate-hz 10 `
  --output .\control_center_io.jsonl
```

Important functions:

#### `EventLog`

Opens the JSONL output file, writes one event per line, and closes the file.

#### `telemetry_packet(sequence, elapsed)`

Creates deterministic synthetic telemetry. It uses sine and cosine functions
to make attitude, acceleration, position, and ultrasonic values change over
time. This lets the UI trajectory and telemetry displays visibly update.

#### `control_packet(sequence)`

Creates a neutral control command with zero flight and Pico values. It is safe
for bench testing because it does not intentionally command movement.

#### `send_telemetry(...)`

Sends generated telemetry over UDP to the backend telemetry port at the
requested rate for the requested duration.

#### `exercise_websocket(...)`

Connects to the backend WebSocket, sends `CONFIG`, sends neutral controls,
receives backend messages, and records all activity.

#### `run(args)`

Creates the output log and runs the WebSocket exercise and UDP telemetry
sender concurrently.

### 4.2 `gaussian_video_server.py`

Run it to test the camera and water indicator:

```powershell
cd C:\Users\ASUS\Desktop\CMU\Sem1\EMSD\Frontend\drone-control-center\control_center
..\.venv\Scripts\python.exe gaussian_video_server.py
```

To test no water contact:

```powershell
..\.venv\Scripts\python.exe gaussian_video_server.py --no-water-detected
```

Important functions:

- `create_noise_frame(...)`: creates and base64-encodes one JPEG.
- `stream_video(...)`: sends frames at the configured FPS.
- `handle_client(...)`: handles a connected backend client.
- `run_server(...)`: starts the WebSocket server.
- `parse_args()`: parses host, port, image size, FPS, and water-state options.

## 5. Complete run order

### 5.1 First-time setup

Create or use the repository Python environment:

```powershell
cd C:\Users\ASUS\Desktop\CMU\Sem1\EMSD\Frontend\drone-control-center
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r laptop_backend\requirements.txt
python -m pip install -r control_center\requirements.txt
```

Install frontend dependencies:

```powershell
cd control_center
npm install

cd ..\current_frontend
npm install
```

### 5.2 Start the synthetic video source

Open PowerShell window 1:

```powershell
cd C:\Users\ASUS\Desktop\CMU\Sem1\EMSD\Frontend\drone-control-center\control_center
..\.venv\Scripts\python.exe gaussian_video_server.py
```

Leave it running. It should report that it is serving
`ws://127.0.0.1:8765`.

### 5.3 Start the laptop backend

Open PowerShell window 2:

```powershell
cd C:\Users\ASUS\Desktop\CMU\Sem1\EMSD\Frontend\drone-control-center\laptop_backend
..\.venv\Scripts\python.exe control_center_backend.py --pi-ip 127.0.0.1
```

For a real Pi, replace `127.0.0.1` with the Pi's address:

```powershell
..\.venv\Scripts\python.exe control_center_backend.py `
  --pi-ip 192.168.1.50 `
  --video-url ws://127.0.0.1:8765
```

The backend should report the browser WebSocket on port 5173 and eventually
report that the video producer is connected.

### 5.4 Start the active control center

Open PowerShell window 3:

```powershell
cd C:\Users\ASUS\Desktop\CMU\Sem1\EMSD\Frontend\drone-control-center\control_center
npm run dev
```

Open `http://127.0.0.1:6969`.

Expected results:

- Browser/backend status is connected.
- Camera panel displays synthetic frames.
- Water-contact panel is green because the test producer defaults to detected.
- Use `--no-water-detected` and restart the producer to test the inactive state.
- Valve trigger buttons send their commands when the backend is connected.

### 5.5 Start the older frontend, if needed

Open another PowerShell window:

```powershell
cd C:\Users\ASUS\Desktop\CMU\Sem1\EMSD\Frontend\drone-control-center\current_frontend
npm run dev
```

Open `http://127.0.0.1:6970`. Ensure its WebSocket URL field is
`ws://127.0.0.1:5173`.

### 5.6 Run the I/O test harness

With the backend running, open another PowerShell window:

```powershell
cd C:\Users\ASUS\Desktop\CMU\Sem1\EMSD\Frontend\drone-control-center\laptop_backend
..\.venv\Scripts\python.exe test_control_center_io.py `
  --duration 10 `
  --rate-hz 10 `
  --output .\control_center_io.jsonl
```

This provides synthetic telemetry for the UI and records the browser/backend
traffic. It does not require a Pi.

### 5.7 Safe bench-test order

When hardware is connected:

1. Keep propellers removed and the vehicle disarmed.
2. Start the Pi gateway in a safe mode with actuators disabled.
3. Start the laptop backend with the correct `--pi-ip`.
4. Start the frontend and verify the browser/backend connection.
5. Run the I/O harness or send controlled test telemetry.
6. Verify that the Pi receives increasing control sequence numbers.
7. Test Pico communication with safe outputs.
8. Test valve commands with the valves physically isolated or otherwise safe.
9. Only then enable additional flight-controller functionality.

## 6. Interfacing with a Raspberry Pi

### 6.1 What must run on the Pi

The Pi needs a gateway program that:

1. Listens for UDP JSON commands on port 5000.
2. Parses `CONTROL` packets.
3. Parses `TRIGGER` packets.
4. Maps flight values to the flight controller using MAVLink or the chosen
   flight-control API.
5. Maps Pico values to the Pico over SPI.
6. Maps valve trigger names to GPIO, a Pico command, or another actuator
   interface.
7. Reads flight-controller, sensor, GPS, battery, ultrasonic, and water
   values.
8. Sends `TELEMETRY` JSON datagrams to the laptop's IP on UDP port 5001.
9. Implements a control timeout/failsafe so loss of the laptop or Wi-Fi does
   not leave the vehicle running indefinitely.

The Pi should own the safety-critical behavior. The browser and laptop are
operator and networking layers, not the flight stabilization loop.

### 6.2 Network setup

Give the Pi a reachable address on the same network as the laptop. Use a DHCP
reservation or static address, then start the backend with that address:

```powershell
..\.venv\Scripts\python.exe control_center_backend.py `
  --pi-ip 192.168.1.50 `
  --pi-control-port 5000 `
  --telemetry-port 5001
```

The laptop must be able to send UDP packets to the Pi. The laptop firewall
must allow inbound UDP port 5001. If the video producer is on another machine,
also ensure that the backend can reach its configured WebSocket endpoint.

### 6.3 Implement the Pi command receiver

The Pi gateway should decode each UDP datagram as UTF-8 JSON and dispatch by
`type`:

```python
if message["type"] == "CONTROL":
    # Validate version, sequence, timestamp, ranges, and freshness.
    # Send flight values to the flight controller.
    # Send Pico values to the Pico.
elif message["type"] == "TRIGGER":
    # Dispatch valve_1_on, valve_2_on, valve_1_off, valve_2_off.
```

Recommended checks on the Pi:

- Reject unknown protocol versions.
- Reject non-finite or out-of-range values.
- Reject stale timestamps.
- Track sequence numbers and detect gaps.
- Apply a watchdog timeout.
- Return actuators to a safe state after timeout.
- Keep emergency/safety behavior local to the Pi rather than relying on the
  browser.

### 6.4 Implement telemetry transmission

The Pi should periodically send packets like:

```json
{
  "version": 1,
  "type": "TELEMETRY",
  "seq": 42,
  "time_ms": 1727483000123,
  "fc": {
    "flight_mode": "STABILIZE",
    "armed": false,
    "attitude": {
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": 0.0
    },
    "position": {
      "lat": 40.4432,
      "lon": -79.9436,
      "alt": 12.0
    },
    "battery": {
      "voltage_v": 15.8,
      "remaining_percent": 87
    }
  },
  "water_contact": false,
  "sensors": {
    "water_detected": false
  }
}
```

Send it to:

```text
<laptop-ip>:5001/UDP
```

The laptop backend will forward the packet to all connected browser clients.

### 6.5 Implementing real video

The current Gaussian-noise server is only a test source. A real camera
producer should:

1. Capture frames from the Pi camera or another camera system.
2. Encode each frame as JPEG, PNG, or WebP.
3. Base64-encode the image.
4. Publish a `VIDEO_FRAME` JSON message over WebSocket.
5. Include a monotonically increasing frame sequence number.
6. Include water fields if the camera process is also the water sensor source,
   or leave those fields false and send water in telemetry instead.

Point the laptop backend at the real producer:

```powershell
..\.venv\Scripts\python.exe control_center_backend.py `
  --pi-ip 192.168.1.50 `
  --video-url ws://192.168.1.50:8765
```

Keep individual serialized WebSocket messages below the backend's 2 MiB
maximum. If full-resolution frames exceed that limit, reduce image quality or
resolution, use a more suitable video transport, or redesign the protocol
around a streaming format rather than base64 JSON.

### 6.6 Integration sequence

Integrate in this order:

1. Verify laptop backend and browser connection using localhost only.
2. Verify the I/O harness and synthetic telemetry.
3. Implement the Pi UDP receiver and send a harmless telemetry packet.
4. Confirm telemetry appears in the browser.
5. Send neutral control packets and verify Pi sequence numbers.
6. Connect the Pico and test safe output values.
7. Implement and test valve trigger mapping.
8. Connect the flight controller while disarmed.
9. Add real camera streaming.
10. Add hardware watchdogs and validate all timeout/failsafe behavior.

Do not treat a successful browser connection as proof that the Pi, flight
controller, Pico, valves, or camera are connected. Each link must be tested
independently.
