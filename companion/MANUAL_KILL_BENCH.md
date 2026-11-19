# Manual KILL: propeller-free bench test

A live [signal monitor](SIGNAL_MONITOR.md) now shows backend UDP writes, companion MAVLink commands, and FC acknowledgments.

## What was wrong

`motor_test.py` uses `MAV_CMD_DO_MOTOR_TEST` (209), at a throttle **percentage**, not an RPM setpoint. It deliberately starts only when the FC reports disarmed. The earlier kill script could see that disarmed heartbeat and discard KILL without transmitting anything. Force-disarm alone also does not explicitly cancel the separate motor-test state.

The supplied branch's laptop backend had no KILL handler, and its active `control_center/src/main.tsx` had no KILL button. Those are now connected.

## Implemented path

```text
KILL MOTORS button -> WebSocket :5173 -> laptop backend
    -> repeated KILL packets to Pi UDP :5002
    -> companion/manual_kill_switch.py
    -> MAVLink Router TCP localhost:5760 -> FC
```

On KILL, the receiver immediately latches a stop request. At 20 Hz it sends:

1. `MAV_CMD_DO_MOTOR_TEST`: motor=1, throttle type=percentage, throttle=0, timeout=0, motor count=1.
2. `MAV_CMD_COMPONENT_ARM_DISARM`: param1=0, param2=21196.

The motor-test cancellation uses the same command path as `motor_test.py`'s existing stop function. In [ArduCopter 4.7.2](https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-4.7.2/ArduCopter/motor_test.cpp), there is one active motor-test state. Replacing it with a zero-duration single-motor test makes the next motor-test output cycle stop the test. Count=1 prevents continuation of a sequential test; this is not a request to stop only output 1. Force-disarm is additionally sent for the normal armed-output path.

This is a bench-specific implementation: repeated zero-duration test commands can briefly enter/exit motor-test mode when no test is active. It is not a validated flight emergency-stop implementation.

No heartbeat or armed-state check gates these commands. Missing laptop heartbeats do **not** initiate shutdown in this receiver. The existing backend heartbeat packets serve only as status polls; MAVLink heartbeats announce the receiver for routing.

KILL also creates `/tmp/mole-bench-kill-<uid>` on the Pi. The updated `motor_test.py` checks this marker before each start, while waiting for an ACK, and during its inter-motor delay. It exits instead of starting the next motor. Run both scripts as the same user. The marker survives process restarts, but `/tmp` may be cleared on reboot. Other external tools such as Mission Planner do not honor this marker; do not start competing motor tests during this test.

The laptop backend latches/retransmits KILL at 10 Hz and stops forwarding CONTROL packets until restarted. The frontend reconnects with its pending KILL request. The Pi continues sending stop pairs even if the browser disconnects after KILL.

## Files to deploy

On the Pi, deploy these together from `companion/`:

- `signal_trace.py` — shared signal-capture module (required by the scripts below).
- `manual_kill_switch.py` — new manual-only receiver.
- `motor_test.py` — stops the bench sequence when KILL is latched.
- `telemetry_bridge.py` — clears RPM fields to unknown after one second without an ESC telemetry message.

On the laptop use the updated files under:

- `Frontend_laptop_operator/drone-control-center/laptop_backend/`
- `Frontend_laptop_operator/drone-control-center/control_center/` (active frontend, not `current_frontend/`).

The EMSD workspace's `ros2_ws/control_center_backend.py` is kept in sync with the branch backend. `ros2_ws/manual_kill_switch.py` is a compatibility entry point that loads the companion implementation; deploy the actual companion script to the Pi, not that wrapper alone. The original heartbeat-driven `ros2_ws/kill_switch.py` remains unchanged.

## Run

Keep all propellers removed. Only one kill receiver may bind UDP 5002. Stop any previously installed kill service first:

```bash
sudo systemctl stop kill-switch
```

If that service is not installed, no service change is necessary. This change does not install or enable a new systemd service.

On the Pi, from the branch root, in separate terminals:

```bash
~/mav/bin/python companion/manual_kill_switch.py --target-system 1 --target-component 1 --rate 20
```

Use the actual FC system/component IDs if different; `check_link.py` reports them. The default connection matches `companion/mavlink-router/main.conf`: TCP 5760, forwarding to the FC on `/dev/serial0` at 921600 baud. MAVLink Router must already be running.

```bash
~/mav/bin/python companion/telemetry_bridge.py --laptop-ip <LAPTOP_IP>
```

On the laptop, from `Frontend_laptop_operator/drone-control-center`, with the existing Python environment activated:

```bash
python laptop_backend/control_center_backend.py --pi-ip <PI_IP>
```

For the documented MOLE hotspot, the Pi address is `10.42.0.1`; use the actual address for your network. Allow UDP 5002 to the Pi and UDP 5001 to the laptop.

Start the active frontend:

```bash
cd control_center
npm install
npm run dev
```

Open `http://127.0.0.1:6969`. The KILL button appears in the TRIGGERS panel. It sends the stop request immediately, without a confirmation dialog.

Now run the existing bench test on the Pi:

```bash
~/mav/bin/python companion/motor_test.py -m 1 -t 7 -d 5
```

Press KILL while the motor is spinning. To verify that the sequence cannot continue after KILL, use the default multi-motor test in a separate reset test.

## What to observe

- Receiver logs `KILL latched` and motor-test-stop/disarm ACK results.
- Frontend status shows `Pi KILL latched`, an increasing stop-pair count, and the motor-test ACK result (`0` means accepted).
- Every motor's measured RPM should decay to approximately zero. Record click time and spin-down time. No numerical stop-time guarantee has been established on this hardware.
- The bench producer exits rather than starting the next motor.

The UI never sets displayed RPM to zero because KILL was clicked. RPM comes from `ESC_TELEMETRY_1_TO_4`, requested at 10 Hz. Missing/stale telemetry is displayed as unknown (`—`). Fresh packet delivery still depends on the ESC/FC reporting meaningful measurements. An accepted command or disarmed heartbeat is not proof of zero RPM.

`KILL_STATUS` includes `motor_test_ack_result`, `disarm_ack_result`, `disarm_commands_sent` (completed stop-pair count), `fc_linked`, and `error`. A lost FC connection retains the request and retries connection; it cannot stop the FC while that communication path is unavailable.

## Reset for another bench test

1. Stop `motor_test.py` and any other motor-command producer.
2. Stop the manual receiver and laptop backend.
3. On the Pi, run `~/mav/bin/python companion/manual_kill_switch.py --reset-latch`.
4. Close/reload the frontend so its local KILL request is cleared.
5. Restart receiver, backend, and frontend connection before starting another test.

Do not clear the marker while the receiver is running. There is no remote unlatch command.

## Validation and limits

Local checks cover the actual backend KILL handler with mocked transports, duplicate requests, cached disarmed state, reconnecting with a pending stop, persistent inhibition of the next motor, and encoded MAVLink cancellation/disarm parameters. Python compilation passes.

Real socket tests were blocked by the execution environment during the initial checks. Frontend dependency installation and the Vite production build subsequently passed (`npm install` and `npm run build`). This verifies bundling, not browser-to-hardware behavior. No hardware test has been performed. Bench verification is still required to establish physical stop behavior and latency.
