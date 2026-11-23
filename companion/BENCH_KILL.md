# Bench KILL test and Signal Monitor

A props-off bench test of the manual KILL path, watched live with the control center's **Signal Monitor**. This is a bench procedure, not a validated flight emergency stop. For flying, use the watchdog kill switch (see [README](README.md#kill-switch)).

```text
KILL MOTORS button -> WebSocket :5173 -> laptop backend
    -> repeated KILL packets to Pi UDP :5002
    -> companion/manual_kill_switch.py
    -> mavlink-router TCP localhost:5760 -> FC
```

## What KILL does

- **Laptop backend:** latches KILL, retransmits it at 10 Hz and stops forwarding CONTROL until the backend is restarted. The frontend resends a pending KILL when it reconnects.
- **`manual_kill_switch.py`:** latches the stop request and, at 20 Hz (`--rate`), sends a pair of commands:
  1. `MAV_CMD_DO_MOTOR_TEST` with motor 1, throttle type percentage, throttle 0, timeout 0, count 1. This replaces ArduCopter's single active motor-test state with a zero-length test, which ends a running test (count 1 also stops a sequential test from continuing; it does not mean "stop only output 1").
  2. `MAV_CMD_COMPONENT_ARM_DISARM` with param1 0, param2 21196 (forced disarm), for the normal armed-output path.

  No heartbeat or armed-state check gates these commands, and missing laptop heartbeats do **not** trigger a stop. It keeps sending even if the browser disconnects. If the FC link is lost it keeps the request and retries the connection.
- **Marker file:** KILL also creates `/tmp/mole-bench-kill-<uid>`. `motor_test.py` checks it before each start, while waiting for an ACK and between motors, and exits instead of starting the next motor. Run both scripts as the same user. Other tools (Mission Planner etc.) ignore the marker, so don't start competing motor tests.

Repeated zero-length test commands can briefly enter and leave motor-test mode when no test is running.

## Files

- **Pi**, from `companion/`, deployed together: `signal_trace.py` (required by the others), `manual_kill_switch.py`, `motor_test.py`, `telemetry_bridge.py`.
- **Laptop**, under `Frontend_laptop_operator/drone-control-center/`: `laptop_backend/control_center_backend.py` and the active frontend `control_center/` (not `current_frontend/`, which has neither the KILL button nor the Signal Monitor).

## Run

Remove all propellers. mavlink-router must be running on the Pi. Only one kill receiver can bind UDP 5002, so stop the watchdog first:

```bash
sudo systemctl stop kill-switch
```

On the Pi, from the repo root, in separate terminals:

```bash
~/mav/bin/python companion/manual_kill_switch.py --target-system 1 --target-component 1 --rate 20
```

```bash
~/mav/bin/python companion/telemetry_bridge.py --laptop-ip <LAPTOP_IP>
```

Use the FC's real system/component IDs if different (`check_link.py` reports them). The default connection, `tcp:127.0.0.1:5760`, matches `mavlink-router/main.conf`.

On the laptop, from `Frontend_laptop_operator/drone-control-center`, with the Python environment active:

```bash
python laptop_backend/control_center_backend.py --pi-ip <PI_IP>
```

```bash
cd control_center && npm install && npm run dev
```

On the MOLE hotspot the Pi is `10.42.0.1`. Allow UDP 5002 to the Pi and UDP 5001 to the laptop; the Signal Monitor needs no extra port.

Open `http://127.0.0.1:6969`. The KILL button is in the TRIGGERS panel and fires immediately, with no confirmation. Then on the Pi:

```bash
~/mav/bin/python companion/motor_test.py -m 1 -t 7 -d 5 --signal-ip <LAPTOP_IP>
```

Press KILL while the motor spins. To check that a sequence can't continue after KILL, run the default multi-motor test as a separate test (after a reset). Without `--signal-ip`, the motor test still works but its commands won't appear in the monitor.

## What to observe

- The receiver logs `KILL latched` and the ACK results for the motor-test stop and the disarm.
- The frontend shows `Pi KILL latched`, a rising stop-pair count and the motor-test ACK result (`0` = accepted). `KILL_STATUS` carries `motor_test_ack_result`, `disarm_ack_result`, `disarm_commands_sent`, `fc_linked` and `error`.
- Every motor's measured RPM should decay to about zero. Record the click time and spin-down time; no stop-time figure has been established on this hardware.
- `motor_test.py` exits instead of starting the next motor.

RPM comes from `ESC_TELEMETRY_1_TO_4` (requested at 10 Hz). The UI never shows zero just because KILL was clicked; stale telemetry shows as `—`. An accepted command or a disarmed heartbeat is not proof of zero RPM.

## Signal Monitor

In the control center, click **SIGNAL MONITOR** (bottom left). It shows three links:

| Link | What is recorded |
|---|---|
| Laptop → Pi | Every backend UDP write: CONTROL, TRIGGER, KILL, status heartbeat. Failed writes and CONTROL blocked by a latched KILL are shown separately. The kill receiver also reports when it actually receives KILL/status packets |
| Pi → FC | MAVLink COMMAND_LONG and HEARTBEAT writes from `manual_kill_switch.py`, `motor_test.py` and `telemetry_bridge.py`, with target/source IDs and all parameters |
| FC → Pi | COMMAND_ACK and STATUSTEXT seen by those scripts, with FC source IDs and result codes |

The manual receiver learns the laptop's IP from incoming packets; the telemetry bridge uses `--laptop-ip`; `motor_test.py` needs `--signal-ip`. All send to UDP 5001 by default (`--signal-port`); change it if the backend uses a different `--telemetry-port`. The backend also writes these records to its JSONL log (`direction=monitor`, `channel=signal`).

**Reading it:**
- These are application-level records, not UART, PWM, DShot or ESC measurements. `sent` means the write completed, not that it arrived or a motor responded. An ACK names a command number, not one specific repeat, and several scripts can log the same FC reply. Commands from other programs or an RC receiver aren't shown.
- Rows are in laptop-receipt order. Sender and laptop clocks may differ, so subtracting timestamps isn't a reliable latency measure. Card counts cover the retained window only, and no new records doesn't by itself prove a broken link.
- The browser keeps up to 1,000 records. Observer queues are bounded and best-effort UDP; the drop count covers known local losses only. Observation runs on its own thread and never delays motor commands.

**Controls:** click a link card to filter (**All links** resets); search names, sources, outcomes or values; heartbeats are hidden unless you tick the box; click a timestamp for raw JSON; **Freeze display** holds a snapshot without stopping capture; **Export JSON** saves the retained records; **KILL MOTORS** uses the real KILL path in live mode. Closing the monitor doesn't reset a KILL.

**Without hardware:** **Show example** displays an illustrative motor-start → KILL → cancel → disarm → ACK sequence, clearly bannered. It sends nothing and its KILL button is disabled.

## Reset for another test

1. Stop `motor_test.py` and anything else commanding motors.
2. Stop the manual receiver and the laptop backend.
3. On the Pi: `~/mav/bin/python companion/manual_kill_switch.py --reset-latch`.
4. Close or reload the frontend to clear its pending KILL.
5. Restart the receiver, backend and frontend.

Don't clear the marker while the receiver is running; there is no remote unlatch. When finished, restart the watchdog: `sudo systemctl start kill-switch`.

## Limits

Not yet tested on hardware. Local tests cover the backend KILL handler with mocked transports, the signal tracing module and the frontend build. Physical stop behaviour and latency still need a bench test.
