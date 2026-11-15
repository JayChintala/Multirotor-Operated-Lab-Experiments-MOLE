# Session notes: 2026-11-08

Where things stand at the end of the session, and what to do next.

## Done

- **EMSD ROS package ported to ROS 2 Jazzy / Gazebo Harmonic** (`ROS/EMSD`). Builds and passes its tests on this machine; still supports Humble/Fortress (not built there).
- **ArduPilot SITL + Gazebo Harmonic simulation of the MOLE quad** (`sim/`). Same firmware as the FC (ArduCopter 4.7.2, `fb773567`), 337 real FC parameters from flight log 00016.
- Both pushed to GitHub on `stability_controller_jay`: commits `90982fc` (EMSD port) and `d805b15` (simulation).
- First simulated flight flown with MAVProxy (ALT_HOLD, then LOITER and RTL to recover).

## Vehicle data (confirmed by the team)

| Item | Value |
|---|---|
| Frame | QAV-RXL2, ~460 mm motor to motor (280 mm front–back × 368 mm side–side) |
| Front | CAD −y axis (the frame is mounted rotated 90° from the CAD) |
| Frame + arms + motors + Pi + FC + ESC + wiring | 0.700 kg |
| Battery | 1.500 kg (re-measure: 1.58 kg was also quoted), 195 × 77 × 50 mm, on top, long side front–back |
| Props | 8 in, 13 g each |
| Total | 2.252 kg, CoG 37 mm above frame centre |
| Motor | ~2 kgf at ~35 A per motor (bench file `ROS/EMSD/motor_data.xlsx`, DShot 1500); can draw ~60 A at full throttle |
| Spin | front-right and rear-left CCW, front-left and rear-right CW |
| FC outputs (log 00016) | SERVO1..4 = motors 4, 1, 2, 3 (`companion/README.md` is out of date on this) |

## Findings

- **Hover throttle:** simulation 0.353 vs real `MOT_THST_HOVER` 0.35. The model agrees with the real drone there.
- **Gains:** with the real FC's roll/pitch rate gains (ArduPilot defaults) the simulated drone oscillates. With them halved it hovers steadily.
- **Crash (log 00016):** with the motor modelled uncapped (up to ~36 N / ~60 A, as the real hardware is), the simulation reproduces both the oscillation and the climb at zero throttle seen in the crash.
- **Thrust-to-weight:** ~6.5 uncapped; ~3.6 if limited to 2 kgf per motor.
- **Other issues on the real drone:** high vibration in the crash log (72 m/s² peaks, 114 clipping events), harmonic notch off, tune parameters still generic defaults, no RC link, FC mounted upside down (`AHRS_ORIENTATION` = 8, handled).
- **The simulation cannot guarantee the real drone will fly.** Motor response time (0.02 s) and inertia are estimates; vibration, body drag and wind are not modelled.

## Needs fixing (not yet done)

1. **Motor cap in the model is wrong for the real drone.** `config/vehicle.json` has `max_torque_nm: 0.289` (caps each motor at 2 kgf). The real motors can reach ~60 A, so remove the cap and enforce the chosen limit with `MOT_SPIN_MAX` in the parameters instead.
2. **`sim/README.md` says the capped model doesn't reproduce the climb.** Update it after item 1.
3. **No body air drag in the model,** so in ALT_HOLD the drone drifts forever instead of slowing down.

## Decision pending

**Current limit per motor (`MOT_SPIN_MAX`).** Pick from the motor's and ESC's continuous current ratings (check the datasheets):

| `MOT_SPIN_MAX` | Max per motor | Current |
|---|---|---|
| 0.95 (now) | ~3.7 kgf (extrapolated) | ~60 A |
| 0.73 | 2.0 kgf | 34.5 A |
| 0.66 | 1.76 kgf | 27.4 A |
| 0.60 | 1.51 kgf | 20.8 A |

Set the limit **before** tuning: it also changes how aggressive the controller is.

## Next steps

1. Decide `MOT_SPIN_MAX`.
2. Update the model (remove the torque cap), add `MOT_SPIN_MAX` to the parameters, add body drag, rerun the smoke test, fix the README.
3. Robust gain sweep in simulation: find where oscillation starts across motor lag 0.01–0.04 s and inertia ±20 %, and pick gains with about 2× margin. Save them as a parameter file.
4. Optional: work out the initial tune parameters for 8 in / 6S (Mission Planner's formulas) and test them in simulation.
5. Real drone: set the current limit, load the new gains, enable the harmonic notch (ESC RPM), fly a short low hover (tethered or in the cage) with high-rate logging. Analyse the log for oscillation and vibration.
6. Calibrate the simulator against that real log, then do the final tune with AutoTune. Get an RC transmitter first.

## Checklist: from now to the next real flight

Shortest safe path: 1, then 3–6 (simulator work), then 7–15.

### Decide
1. **Choose the current limit per motor (`MOT_SPIN_MAX`)** from the motor and ESC datasheets (continuous ratings). At 0.73 each motor is capped at 2 kgf / 34.5 A.
2. **Get an RC transmitter and receiver.** Optional for a tethered hover, strongly recommended before free flight, essential before AutoTune. Wi-Fi to the ground station is currently the only control link.

### Simulator
3. **Update the model:** remove the 2 kgf cap, add `MOT_SPIN_MAX`, add body drag, rerun the smoke test, fix the README.
4. **Initial tune parameters** for 8 in props on 6S: gyro filter, rate-loop filters, acceleration limits, thrust curve.
5. **Robust gain sweep:** find where oscillation starts across motor lag 0.01–0.04 s and inertia ±20 %; pick gains with about 2× margin.
6. **One parameter file for the real drone:** current limit, initial tune values, chosen gains, harmonic notch (ESC RPM), high-rate logging. Rehearse loading and flying it in the simulator.

### Real drone: hardware
7. **Reduce vibration:** soft-mount the FC, balance props, tighten everything (crash log: 114 clipping events).
8. **Props and motors:** correct CW/CCW prop on each motor, no damage from the ceiling hit, bells spin freely.
9. **Battery:** fully charged; check the voltage reading against a multimeter (`BATT_VOLT_MULT`).

### Real drone: FC setup
10. **Back up the current FC parameters** (Mission Planner: Config → Full Parameter List → Save).
11. **Load the new parameter file** (step 6) and reboot.
12. **Recalibrate accelerometer and level.** Check `AHRS_ORIENTATION` is still 8 (FC mounted upside down).
13. **Bench checks, props off:** `motor_test.py` for order and spin; tilt by hand and check the horizon in the ground station moves the right way; confirm the GCS failsafe triggers when Wi-Fi drops.

### First flight
14. **Short, low hover:** tethered or in the cage, AltHold, 30–60 s at about 1 m. Be ready to switch to LAND or disarm; stop at the first wobble.
15. **Analyse the log:** oscillation, vibration, hover throttle; compare with the simulator and calibrate the model. That sets up the next gain changes and, eventually, AutoTune.

## How to run the simulation

Terminal 1 (a normal Ubuntu terminal):
```bash
cd ~/MOLE/sim/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch mole_gazebo sitl.launch.py              # gps:=false to match the real drone
```
Wait ~30 s for `EKF3 IMU0 origin set`. In Gazebo, right-click `mole_quad` → **Follow**.

Terminal 2:
```bash
cd ~/mole_sim
~/mole_sim/venv-ardupilot/bin/mavproxy.py --master=tcp:127.0.0.1:5762 --console --map
```
Wait for `Received 1370 parameters`, then:
```
param load ~/mole_sim/half_rate_gains.parm
mode LOITER          # or ALT_HOLD (with underscore) to fly like the real drone
arm throttle
rc 3 1700            # climb; rc 3 1500 to hold
mode RTL             # fly home and land
```
Sticks: `rc 1` roll, `rc 2` pitch (1400 = forward), `rc 3` throttle, `rc 4` yaw; 1500 = centre.

Automated check:
```bash
~/mole_sim/venv-ardupilot/bin/python ~/MOLE/sim/ros2_ws/src/mole_gazebo/scripts/smoke_test.py --rate-gain-scale 0.5
```

## Gotchas

- **Every launch resets parameters** to the real FC's values (`wipe:=true`). Re-run `param load` after each launch, or launch with `wipe:=false`.
- **MAVProxy mode names** need underscores: `mode ALT_HOLD`. Type `mode` alone to list them.
- **Change parameters only after** MAVProxy prints `Received 1370 parameters`. Enter commands one at a time.
- **Don't close MAVProxy while flying.** The GCS failsafe lands the drone after 5 s, as on the real drone.
- **Gazebo/RViz windows crash from VS Code's terminal** (snap). The launch file works around it; use a normal terminal for anything else.
- MAVProxy writes `mav.tlog` / `mav.parm` into the folder it's started from. Stray copies are in `~` from an earlier run and can be deleted.

## Where things are

| What | Where |
|---|---|
| Simulation package | `~/MOLE/sim/ros2_ws/src/mole_gazebo` (`config/vehicle.json` → `scripts/gen_model.py` → model) |
| Simulation documentation | `~/MOLE/sim/README.md` |
| EMSD port | `~/MOLE/ROS/EMSD` |
| ArduCopter 4.7.2 SITL build | `~/mole_sim/ardupilot-4.7.2` (git worktree of `~/mole_sim/ardupilot`) |
| ArduPilot Gazebo plugin | `~/mole_sim/ardupilot_gazebo/build` |
| MAVProxy / pymavlink venv | `~/mole_sim/venv-ardupilot` |
| Simulation run logs (`.BIN`) | `~/mole_sim/runs/<date_time>/logs/` |
| Halved gains | `~/mole_sim/half_rate_gains.parm` |
| Real crash log | `flight_logs/00016-2026-10-26_22-20-44.bin` on branch `flight-testing` |

Leftovers you can clean up: `ROS/EMSD_reproducible_source.zip` (duplicate of `ROS/EMSD`); local branch `ros-jazzy-port` (`git branch -d ros-jazzy-port`).

---

# Update: 2026-11-15, water module

## Done
- Pulled `stability_controller_jay` (2 new commits: kill switch in `companion/`, report folder deleted).
- **Water module** from branch `manan` (`ROS/water_module.urdf`, commit `33f060d`) attached to the model: copied to `config/payloads/water_module.urdf`, read by `gen_model.py`. 482 g, centred under the frame, long side across. Its meshes are not on GitHub, so it shows as a blue box.
- **Motors back to real hardware** (no model cap, up to ~60 A); **`MOT_SPIN_MAX` 0.73** added in `config/planned_changes.parm` (team decision).
- Smoke test now compares hover throttle with the model's prediction (`models/mole_quad/predicted.json`) and fails if the crash detector fires on landing.
- Spawn height raised to 0.12 m (the module hangs 106 mm below the frame).

## Results (watched in the Gazebo GUI)
| Configuration | Real gains ×0.6 | ×1.0 | ×1.1 | ×1.25 | ×1.5 |
|---|---|---|---|---|---|
| Module + `MOT_SPIN_MAX` 0.73 | steady, 11/11 | steady | steady | oscillates (71 °/s) | oscillates |
| Module + `MOT_SPIN_MAX` 0.95 (FC today) | | oscillates (70 °/s) | | | |

- The current limit is what stabilises the real gains; the module alone doesn't.
- Real gains are close to the edge; **suggested ×0.6** (`config/suggested_gains_water_module.parm`: P 0.081, I 0.081, D 0.00216 for roll and pitch).
- Hover output with module 0.394 (model predicts 0.395).
- **Tip-over risk:** landing on the module's narrow base (96 mm, CoG 126 mm above it) tripped the crash detector once.

## Open questions for the team
1. **Landing gear** with the module fitted? If yes: leg length and spacing. If no: the real drone will tip on landing too.
2. **Water module CAD/meshes** (`emsd_drone_description/meshes/water_module/*.stl`, `config/water_module.json`) are not on GitHub; needed for its real shape and to check the assumed top-face height (42.5 mm above its origin).
3. Does the module carry water in flight? If so, how much (adds mass)?

## Not committed yet
Everything from today (water module, `planned_changes.parm`, suggested gains, smoke test, README, these notes).
