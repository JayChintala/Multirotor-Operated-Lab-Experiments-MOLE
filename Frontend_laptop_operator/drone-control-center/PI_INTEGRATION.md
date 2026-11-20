# Raspberry Pi Integration

How the laptop control center talks to the Raspberry Pi on the drone, and what
the Pi side must implement. Packet formats below match
`laptop_backend/control_center_backend.py`.

> **Status: the Pi command gateway is NOT YET IMPLEMENTED.** Nothing in
> `companion/` listens on UDP 5000, so `CONTROL` and `TRIGGER` packets from the
> backend currently go nowhere. What does exist on the Pi:
>
> | Script | Port | Does |
> |---|---|---|
> | `companion/telemetry_bridge.py` | sends to laptop UDP 5001 | Read-only FC → laptop `TELEMETRY` (attitude, IMU, position, battery, mode, armed, motor outputs, ESC RPM). Never commands the FC. |
> | `companion/kill_switch.py` | listens UDP 5002 | Watchdog kill: disarms on `KILL`, or if the FC is armed and no `KILL_HEARTBEAT` arrives for `--timeout` s (default 1.0). |
> | `companion/manual_kill_switch.py` | listens UDP 5002 | Bench-only manual kill: disarms on `KILL` only, no link-loss shutdown. |
>
> Only one of the two kill receivers can bind UDP 5002 at a time. See
> `companion/README.md` for Pi setup.

## Data flow

```text
Browser
  | HTTP :6969 (control_center, active) or :6970 (current_frontend, older)
  v
Frontend application
  | WebSocket ws://127.0.0.1:5173
  v
Laptop backend (control_center_backend.py)
  | UDP <pi-ip>:5000   CONTROL, TRIGGER          (gateway: not implemented)
  | UDP <pi-ip>:5002   KILL, KILL_HEARTBEAT      (kill_switch / manual_kill_switch)
  v
Raspberry Pi

Raspberry Pi
  | UDP <laptop-ip>:5001   TELEMETRY, SIGNAL     (telemetry_bridge.py)
  | UDP reply to sender    KILL_STATUS           (kill receivers)
  v
Laptop backend
  | WebSocket broadcast :5173
  v
Browser

Video producer
  | WebSocket :8765   VIDEO_FRAME   (backend is the client)
  v
Laptop backend
```

The `127.0.0.1` addresses are on the laptop. The backend binds telemetry on
`0.0.0.0:5001`, so it accepts datagrams on any laptop interface.

## Network setup

**Field (Pi hotspot).** Run `sudo mole-wifi hotspot` on the Pi, join Wi-Fi
`MOLE` / `MOLEMOLE` from the laptop. Pi is `10.42.0.1`; the laptop gets a
`10.42.0.x` address (`sudo mole-wifi status` on the Pi shows it).

**Direct Ethernet.** Static addresses on one subnet, no gateway:

| Device | Address | Mask |
|---|---|---|
| Laptop Ethernet adapter | `192.168.1.10` | `255.255.255.0` |
| Raspberry Pi | `192.168.1.50` | `255.255.255.0` |

Check with `ping 192.168.1.50` from the laptop.

**Rules for either setup**

- Start the backend with the Pi's address (`192.168.1.50` is the default):

  ```powershell
  ..\.venv\Scripts\python.exe control_center_backend.py `
    --pi-ip 192.168.1.50 `
    --pi-control-port 5000 `
    --pi-kill-port 5002 `
    --telemetry-port 5001
  ```

- The Pi sends telemetry to the **laptop's** address on that link (for example
  `192.168.1.10:5001` or `10.42.0.x:5001`), never to `127.0.0.1`, which is the
  Pi itself. `telemetry_bridge.py --laptop-ip <laptop-ip>`.
- The laptop firewall must allow inbound UDP 5001 (and the `KILL_STATUS`
  replies to the backend's kill socket).
- Give the Pi a fixed address (static or DHCP reservation) so `--pi-ip` doesn't
  go stale.

## Packet formats

All packets are UTF-8 JSON, one per datagram, with `"version": 1`.

### UDP 5000, laptop → Pi: `CONTROL`

```json
{
  "version": 1,
  "type": "CONTROL",
  "seq": 17,
  "time_ms": 1727483000123,
  "flight": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0},
  "pico": {"output0": 0, "output1": 0, "output2": 0, "output3": 0}
}
```

The backend builds this packet itself from the browser's message:

| Field | Rule |
|---|---|
| `seq` | Backend counter, `% 256` (wraps 255 → 0). The browser's `seq`, if any, is ignored. |
| `time_ms` | Backend wall-clock time in ms. |
| `flight.roll/pitch/yaw` | Finite numbers in −1…1. |
| `flight.throttle` | Finite number in 0…1. A negative throttle makes the backend **reject** the whole message with an `ERROR` (it does not clamp). |
| `pico.output0–3` | Integers in −1000…1000. If the browser omits `pico` (the active `control_center` UI does), the backend fills zeros. Only `current_frontend` sends Pico values. |

`CONTROL` is not forwarded at all once a `KILL` has been latched (until the
backend restarts).

Known issue: in `control_center` the Down arrow sets throttle to −1, so holding
it makes the backend reject every `CONTROL` message instead of sending
throttle 0.

### UDP 5000, laptop → Pi: `TRIGGER`

```json
{"version": 1, "type": "TRIGGER", "name": "valve_1_on", "time_ms": 1727483000123}
```

`name` is one of `valve_1_on`, `valve_1_off`, `valve_2_on`, `valve_2_off`;
anything else is rejected by the backend. Sent once per button press, with no
retry or acknowledgement.

### UDP 5002, laptop → Pi: kill switch

| Packet | When |
|---|---|
| `{"version":1,"type":"KILL_HEARTBEAT","time_ms":...}` | 10 Hz while at least one browser is connected and no kill is latched |
| `{"version":1,"type":"KILL","id":...,"time_ms":...}` | KILL button: sent 5 times, then repeated at 10 Hz until the backend restarts, even if browsers disconnect |

The Pi answers each packet with `KILL_STATUS` to the sending socket:

```json
{"version": 1, "type": "KILL_STATUS", "time_ms": 0, "armed": false,
 "fc_linked": true, "killing": null, "timeout_s": 1.0}
```

`manual_kill_switch.py` adds `"kill_latched"` and `"mode": "manual_only"`. If
the Pi reports `kill_latched: true`, the backend latches too.

### UDP 5001, Pi → laptop: `TELEMETRY`

The backend forwards any `TELEMETRY` packet unchanged to every browser (adding
`bridge_received_time_ms`); it does not validate fields. Shape the UIs read:

```json
{
  "version": 1,
  "type": "TELEMETRY",
  "time_ms": 1727483000123,
  "fc": {
    "flight_mode": "STABILIZE",
    "armed": false,
    "attitude": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "position": {"lat": 40.4432, "lon": -79.9436, "alt": 12.0},
    "battery": {"voltage": 15.8, "remaining": 87}
  },
  "water_contact": false,
  "sensors": {"water_detected": false}
}
```

- `control_center` also accepts local `position.x/y/z` (preferred over
  lat/lon/alt when present).
- **Battery field names don't match across components:**

  | Component | Battery fields | Altitude |
  |---|---|---|
  | `companion/telemetry_bridge.py` (sender) | `voltage`, `remaining` | `position` |
  | `control_center` (active UI) | reads `remaining` only | `position.alt` |
  | `current_frontend` (older UI) | reads `voltage_v`, `remaining_percent` | `position.relative_alt_m` |
  | `laptop_backend/test_control_center_io.py` | sends all four | sends both |

  So with the real Pi, the older UI shows no battery data. Pick one set of names
  before relying on either UI.

`telemetry_bridge.py` does not send `seq`. The test harness also sends
`ultrasonic` and extra `sensors` fields, which the backend forwards but no UI
shows yet.

### UDP 5001, Pi → laptop: `SIGNAL`

Optional trace events for the Signal Monitor (`companion/signal_trace.py`).
Accepted only with `version` 1, a `payload` object, a string `name`, a numeric
`timestamp_ms`, and `direction` of `laptop_to_pi`, `pi_to_fc` or `fc_to_pi`.

## Command receiver to implement (sketch)

```python
import json, math, socket, time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5000))
sock.settimeout(0.1)
last_control = time.monotonic()
TIMEOUT_S = 0.5

while True:
    try:
        data, addr = sock.recvfrom(65535)
        msg = json.loads(data.decode("utf-8"))
    except socket.timeout:
        msg = None
    except (json.JSONDecodeError, UnicodeDecodeError):
        continue

    if msg and msg.get("version") == 1:
        if msg.get("type") == "CONTROL":
            # Validate ranges and finiteness again here; check time_ms freshness
            # and track seq (mod 256) for gaps.
            # flight -> flight controller (MAVLink), pico -> Pico over SPI
            last_control = time.monotonic()
        elif msg.get("type") == "TRIGGER":
            # valve_1_on / valve_1_off / valve_2_on / valve_2_off
            # -> GPIO, Pico command, or other actuator
            pass

    if time.monotonic() - last_control > TIMEOUT_S:
        pass  # neutral flight command, safe Pico outputs, valves off
```

Required Pi-side behaviour:

- Reject unknown versions and non-finite, out-of-range or stale values. Don't
  rely on the backend's checks.
- Track `seq` and detect gaps, allowing for the wrap at 256.
- Run a control watchdog and put every actuator in a safe state on timeout.
- Keep safety-critical behaviour (failsafe, emergency stop) on the Pi and FC.
  The browser and laptop are operator and networking layers, not the
  stabilisation loop.
- The laptop must not treat a successful `sendto()` as delivery. UDP gives no
  delivery, ordering, duplicate protection or retransmission.

**One-shot commands (not implemented).** Valve triggers are single datagrams
and can be lost. A reliable version would: give each trigger a unique command
ID; have the Pi reply with an `ACK` carrying that ID; have the laptop resend
until acknowledged or timed out; have the Pi ignore IDs it has already
executed. Continuous `CONTROL` packets don't need this: the next packet
replaces a lost one.

## Real camera

`control_center/gaussian_video_server.py` is a synthetic test source. A real
producer on the Pi should:

1. Capture frames from the camera.
2. Encode each as JPEG, PNG or WebP (the backend accepts only `image/jpeg`,
   `image/png`, `image/webp`).
3. Base64-encode the image.
4. Serve a WebSocket on port 8765 and publish messages like
   `{"type": "VIDEO_FRAME", "seq": n, "time_ms": ..., "content_type": "image/jpeg", "data": "<base64>", "water_contact": false, "sensors": {"water_detected": false}}`.
5. Increase `seq` monotonically.
6. Include the water fields if the camera process is the water sensor;
   otherwise leave them false and send water state in telemetry.

Point the backend at it:

```powershell
..\.venv\Scripts\python.exe control_center_backend.py `
  --pi-ip 192.168.1.50 `
  --video-url ws://192.168.1.50:8765
```

The backend reconnects every 2 s if the producer is down. Each WebSocket
message must stay under 2 MiB; base64 adds about a third, so reduce resolution
or JPEG quality, or switch to a proper video stream if frames don't fit.

## Bench test and integration order

Props off throughout. Test each link on its own: a connected browser proves
nothing about the Pi, FC, Pico, valves or camera.

1. Run the backend and the active UI on the laptop only (localhost).
2. Run `laptop_backend/test_control_center_io.py` and confirm synthetic
   telemetry appears in the UI.
3. Bring up the network (hotspot or Ethernet) and start the backend with the
   right `--pi-ip`.
4. Start `telemetry_bridge.py` on the Pi and confirm real telemetry appears.
5. Start a kill receiver on the Pi and confirm `KILL_STATUS` shows in the UI
   (see `companion/MANUAL_KILL_BENCH.md`).
6. Implement the Pi UDP 5000 receiver. Send neutral `CONTROL` packets and
   confirm the Pi sees consecutive `seq` values.
7. Connect the Pico and test safe output values.
8. Implement and test the valve trigger mapping.
9. Connect the flight controller while disarmed and check the mapping of each
   axis and its sign.
10. Add real camera streaming.
11. Validate the watchdog and every failsafe: unplug Ethernet or leave Wi-Fi,
    close the browser, stop the backend, and confirm the Pi reaches a safe
    state each time.
