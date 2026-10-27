# Drone Control Center Setup

This package contains:

- `control_center/`: the active React/Vite control interface.
- `current_frontend/`: an older standalone React/Vite interface.
- `laptop_backend/`: the Python WebSocket-to-UDP bridge.
- `network_protocol_for_drone.tex`: network architecture and protocol notes.

Generated dependencies, build output, local virtual environments, runtime logs,
and caches are intentionally excluded from the distribution archive. Install
them locally by following this guide.

## Requirements

Install the following on the computer that will run the laptop software:

- Node.js 20.19+ or 22.12+.
- npm.
- Python 3.10 or newer.
- A Raspberry Pi reachable over Ethernet, its `MOLE` Wi-Fi hotspot, or another IP network.

The Python backend requires the packages listed in
`laptop_backend/requirements.txt`. The synthetic video producer additionally
uses the packages listed in `control_center/requirements.txt`.

## Install the project

Extract the archive or clone the repository, then open a terminal in the
project root.

### Install JavaScript dependencies

Install dependencies for the active frontend:

```powershell
cd control_center
npm install
cd ..
```

Install dependencies for the older frontend only if you plan to use it:

```powershell
cd current_frontend
npm install
cd ..
```

### Create the Python environment

Windows PowerShell:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r laptop_backend\requirements.txt
python -m pip install -r control_center\requirements.txt
```

Linux or macOS:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r laptop_backend/requirements.txt
python -m pip install -r control_center/requirements.txt
```

## Network configuration

### In the field: the Pi's MOLE hotspot

With no router available, the Pi can broadcast its own Wi-Fi network. On the
Pi, run `sudo mole-wifi hotspot` while still on the home network (see
`companion/README.md`), then join it from the laptop:

| | |
|---|---|
| Wi-Fi network | `MOLE` |
| Password | `MOLEMOLE` |
| Raspberry Pi | `10.42.0.1` |
| Laptop | `10.42.0.x`, assigned by the Pi; `sudo mole-wifi status` on the Pi lists it |

Use `--pi-ip 10.42.0.1` in the backend commands below, and run the Pi's
`telemetry_bridge.py` with `--laptop-ip` set to the laptop's `10.42.0.x`
address. QGroundControl connects over TCP to `10.42.0.1:5760`.

The hotspot provides no internet. For internet at the same time, connect a
phone by USB cable with its hotspot turned on (on an iPhone: Settings →
Personal Hotspot → Allow Others to Join). The laptop's Wi-Fi is in use by
`MOLE`, so a phone hotspot over Wi-Fi won't work.

### Direct Ethernet

For a direct Ethernet connection, assign both devices addresses on the same
subnet. One example is:

| Device | IP address | Subnet mask |
|---|---|---|
| Laptop Ethernet adapter | `192.168.1.10` | `255.255.255.0` |
| Raspberry Pi Ethernet | `192.168.1.50` | `255.255.255.0` |

No gateway is required for a direct cable. Confirm that the laptop can reach
the Pi:

```powershell
ping 192.168.1.50
```

If the Pi uses a different address, replace `192.168.1.50` in the backend
command below.

The laptop firewall must allow inbound UDP port `5001` for Pi telemetry.

## Ports

| Port | Protocol | Direction | Purpose |
|---:|---|---|---|
| `6969` | HTTP | Browser to laptop | Active `control_center/` Vite server |
| `6970` | HTTP | Browser to laptop | Older `current_frontend/` Vite server |
| `5173` | WebSocket/TCP | Browser to laptop | Backend control, status, telemetry, and relayed video |
| `8765` | WebSocket/TCP | Video producer to laptop | Synthetic or real camera frames |
| `5000` | UDP | Laptop to Pi | `CONTROL` and `TRIGGER` packets |
| `5001` | UDP | Pi to laptop | `TELEMETRY` packets |

`127.0.0.1` refers to the same computer. The Pi must use the laptop's
Ethernet IP, such as `192.168.1.10`, when sending telemetry.

## Run the system

Open separate terminals for the following processes. Activate the virtual
environment in each PowerShell terminal if it is not already active.

### 1. Start the backend

From the project root:

```powershell
.\.venv\Scripts\python.exe laptop_backend\control_center_backend.py `
  --pi-ip 192.168.1.50
```

The backend listens for the browser at `ws://127.0.0.1:5173`, sends control
commands to `192.168.1.50:5000/UDP`, and listens for Pi telemetry on
`0.0.0.0:5001/UDP`.

Every backend run creates a separate JSONL log file in the root `logs/`
directory. The log records startup, WebSocket JSON, transmitted UDP packets,
and received telemetry.

### 2. Optional: start the synthetic video producer

For local video testing:

```powershell
.\.venv\Scripts\python.exe control_center\gaussian_video_server.py
```

This starts a WebSocket producer at `ws://127.0.0.1:8765`. The backend
reconnects automatically if the producer is not running.

If a real producer runs on the Raspberry Pi, start the backend with:

```powershell
.\.venv\Scripts\python.exe laptop_backend\control_center_backend.py `
  --pi-ip 192.168.1.50 `
  --video-url ws://192.168.1.50:8765
```

The Pi-side producer must listen on an Ethernet-reachable interface and port
`8765`.

### 3. Start the active frontend

From the project root:

```powershell
cd control_center
npm run dev
```

Open:

```text
http://127.0.0.1:6969
```

### 4. Optional: start the older frontend

From another terminal:

```powershell
cd current_frontend
npm run dev
```

Open:

```text
http://127.0.0.1:6970
```

Only one process should use a given frontend port. Both frontends can connect
to the backend at port `5173`.

## Raspberry Pi requirements

The Raspberry Pi-side gateway must:

1. Listen for JSON `CONTROL` and `TRIGGER` datagrams on UDP port `5000`.
2. Send JSON `TELEMETRY` datagrams to the laptop Ethernet IP on UDP port `5001`.
3. Validate ranges, timestamps, and sequence numbers.
4. Apply a control timeout failsafe that neutralizes outputs if commands stop.

UDP does not guarantee delivery, ordering, or retransmission. One-time trigger
commands should use an application-level acknowledgement and retry mechanism
if they are safety-critical.

## Test the backend interfaces

With the backend running, execute the included I/O harness:

```powershell
.\.venv\Scripts\python.exe laptop_backend\test_control_center_io.py `
  --duration 10 `
  --output logs\control-center-test.jsonl
```

The harness sends synthetic WebSocket control messages and UDP telemetry. It
does not control real motors or valves.

## Troubleshooting

### `WinError 1225` or video producer unavailable

No producer is listening at the configured video URL. Start
`gaussian_video_server.py`, or pass the actual producer URL with
`--video-url`.

### The frontend cannot connect

Confirm that the backend is running and that it printed:

```text
WebSocket: ws://127.0.0.1:5173
```

Also confirm that the frontend is using the same WebSocket port.

### The backend cannot control the Pi

Check the following:

- `ping 192.168.1.50` succeeds from the laptop.
- The Pi gateway is listening on UDP `5000`.
- `--pi-ip` matches the Pi's actual Ethernet address.
- The Pi and laptop are on the same subnet.
- Local firewall rules permit the required traffic.

### Telemetry does not appear

The Pi should send telemetry to the laptop's Ethernet address, not
`127.0.0.1`. For the example configuration, send to
`192.168.1.10:5001/UDP` and allow inbound UDP `5001` through the laptop
firewall.

