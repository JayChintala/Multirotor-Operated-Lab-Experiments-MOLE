# Live signal monitor

Open the active control center at `http://127.0.0.1:6969` and click **SIGNAL MONITOR** (bottom left).

The monitor displays three paths:

| Path | What is recorded |
| --- | --- |
| Laptop → Pi | Each backend UDP write: CONTROL, TRIGGER, KILL, and status heartbeat. Failed writes and CONTROL blocked by a latched KILL are separate outcomes. The kill receiver also reports actual receipt of KILL/status packets. |
| Pi → FC | MAVLink COMMAND_LONG and HEARTBEAT writes made by the instrumented kill receiver, bench motor test, and telemetry bridge. Includes target/source IDs and all command parameters. |
| FC → Pi | COMMAND_ACK and STATUSTEXT observed by those companion scripts, with FC source IDs and result codes. |

These are **application-level records**, not a measurement of UART voltage, PWM, DShot, or ESC output. `sent` means the application write completed; it does not prove physical delivery or motor response. An ACK identifies a command number, not an individual repeated request. Multiple scripts can observe copies of the same FC reply. The monitor does not observe commands from other programs, Mission Planner, or the RC receiver unless separately instrumented.

## View without hardware

Click **Show example** inside the monitor to see an illustrative motor-start → KILL → cancellation → disarm → ACK sequence. The banner clearly marks the records as an illustration. Example mode sends no commands, and its KILL button is disabled. Existing background control is unaffected. Return to live capture to see real records.

## Deploy and run

Copy these updated companion files together to the Pi:

- `signal_trace.py` (required shared capture module)
- `manual_kill_switch.py`
- `motor_test.py`
- `telemetry_bridge.py`

Use the updated laptop backend and active frontend under `Frontend_laptop_operator/drone-control-center/`. The older `current_frontend` does not contain this monitor. The EMSD `ros2_ws/control_center_backend.py` copy is also updated.

On the laptop, from `Frontend_laptop_operator/drone-control-center`:

```bash
python laptop_backend/control_center_backend.py --pi-ip <PI_IP>
```

In another laptop terminal:

```bash
cd control_center
npm run dev
```

On the Pi, from the branch root, start the manual receiver and telemetry bridge in separate terminals:

```bash
~/mav/bin/python companion/manual_kill_switch.py
~/mav/bin/python companion/telemetry_bridge.py --laptop-ip <LAPTOP_IP>
```

The manual receiver learns the laptop's IP from incoming KILL/status packets and sends observer records back to UDP 5001. The telemetry bridge uses its existing `--laptop-ip`/`--laptop-port`. Neither needs a separate observer service. Follow [the bench guide](MANUAL_KILL_BENCH.md) for propeller removal, stopping the old receiver, target IDs, and resetting a previous KILL latch.

Run the bench producer with the additional observer destination:

```bash
~/mav/bin/python companion/motor_test.py -m 1 -t 7 -d 5 --signal-ip <LAPTOP_IP>
```

Without `--signal-ip`, the motor test still works, but its start/stop writes will not appear in the monitor. `--signal-port` defaults to 5001 for the motor test and kill receiver; change it if the laptop backend uses a different `--telemetry-port`.

No new firewall port is required: observer records share the existing inbound laptop UDP 5001 with telemetry. Normal commands still use Pi UDP 5000/5002; MAVLink Router remains TCP 5760.

## Controls and interpretation

- Select a path card to filter that link; **All links** restores the full list.
- Search command names, sources, outcomes, or interpreted values.
- Heartbeats are hidden by default to keep motor commands readable. Enable the checkbox to show them.
- Click a timestamp to inspect raw JSON, source/target IDs, and command parameters.
- **Freeze display** holds a snapshot. It does not stop capture or vehicle commands.
- **Export JSON** saves the current retained live/frozen/example records. The file identifies example mode explicitly.
- **KILL MOTORS** uses the existing real KILL path when connected and in live mode. Closing the monitor does not reset a KILL.

Rows are ordered by laptop receipt. Both sender and laptop timestamps are shown; clocks may differ, so subtracting them is not a reliable network-latency measurement. Path cards show counts in the retained window, not lifetime counts or guaranteed packet rates. No new records does not itself prove a broken link.

The browser retains up to 1,000 records. Observer queues are bounded, may drop records under load, and use best-effort UDP. The displayed drop count covers known local queue/send losses, not undetected network loss. A separate thread sends companion observer packets, so missing/slow observation does not gate motor commands. Pi records received by the backend are also written to its JSONL log under `direction=monitor`, `channel=signal`.

Actual motor RPM remains in the original telemetry display. The signal monitor never substitutes a zero command for measured zero RPM.

## Validation

Validated locally with backend capture tests, companion tracing tests (including send failure, queue saturation, and MAVLink-version negotiation), frontend signal-summary checks, a React server-render check with representative records, and the Vite production build. Existing manual-KILL regression tests also pass.

Live socket/browser-to-hardware validation remains pending: the execution environment rejects local socket creation, and no browser automation tool is available. Use the monitor during the propeller-free bench test to validate the real path.
