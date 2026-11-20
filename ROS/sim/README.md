# MOLE quad simulation: ArduPilot SITL + Gazebo Harmonic

Simulates the MOLE quadcopter with the **same firmware as the real flight controller**
(ArduCopter 4.7.2, `fb773567`) and the **real FC's flight parameters**, in Gazebo Harmonic
on ROS 2 Jazzy (Ubuntu 24.04).

```
Gazebo (mole_quad model, 1 kHz physics) ──JSON :9002──> ArduCopter SITL 4.7.2
                                                          ├─ tcp:5760  MAVROS (ROS 2 topics under /mavros)
                                                          └─ tcp:5762  QGroundControl, MAVProxy or scripts
```

## One-time setup (outside the repo)

SITL and the ArduPilot Gazebo plugin are built in `~/mole_sim` (the launch file's default paths):

```bash
mkdir -p ~/mole_sim && cd ~/mole_sim
# ArduCopter SITL at the exact release the real FC runs
git clone --recurse-submodules -b Copter-4.7.2 https://github.com/ArduPilot/ardupilot.git ardupilot-4.7.2
cd ardupilot-4.7.2 && Tools/environment_install/install-prereqs-ubuntu.sh -y   # once per machine
./waf configure --board sitl && ./waf copter
# ArduPilot plugin for Gazebo Harmonic
cd ~/mole_sim && git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo && mkdir -p build && cd build
source /opt/ros/jazzy/setup.bash && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j8
```

Also needed: `sudo apt install ros-jazzy-ros-gz ros-jazzy-mavros ros-jazzy-mavros-extras` and
MAVROS's GeographicLib datasets. (On the first machine these steps were done by hand; the
commands above are the equivalent from scratch and have not been re-run on a clean install.)

## Build and run

```bash
cd ROS/sim/ros2_ws                                         # from the repo root
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch mole_gazebo sitl.launch.py                    # Gazebo window, simulated GPS, MAVROS
ros2 launch mole_gazebo sitl.launch.py gps:=false         # like the real drone today: no GPS/compass
ros2 launch mole_gazebo sitl.launch.py headless:=true     # no window (CI, SSH)
```

| Argument | Default | Meaning |
|---|---|---|
| `gps` | `true` | `true`: simulated GPS + compass (Loiter/Guided/Auto/RTL work). `false`: matches the real drone, only Stabilize/AltHold/Acro |
| `mavros` | `true` | start MAVROS on `tcp:5760` |
| `headless` | `false` | Gazebo server only |
| `wipe` | `true` | start SITL from the parameter files every time (no stale eeprom) |
| `ardupilot_dir` | `~/mole_sim/ardupilot-4.7.2` | ArduPilot build with `build/sitl/bin/arducopter` |
| `ardupilot_gazebo_dir` | `~/mole_sim/ardupilot_gazebo` | plugin build with `build/libArduPilotPlugin.so` |

SITL logs (`.BIN`, same format as the real FC's) go to `~/mole_sim/runs/<date_time>/logs/`.
Close the Gazebo window or press Ctrl+C to stop everything.

Launching from **VS Code's terminal** works: the launch file removes the snap environment
variables that otherwise crash Gazebo's window.

Connect **QGroundControl** with a TCP link to `127.0.0.1:5762`, or MAVProxy with
`mavproxy.py --master=tcp:127.0.0.1:5762`.

## Check it works

```bash
PY=~/mole_sim/venv-ardupilot/bin/python      # any Python with pymavlink + Gazebo bindings
$PY src/mole_gazebo/scripts/smoke_test.py --rate-gain-scale 0.6
```

1. Motor test A–D: each spins the correct propeller in the correct direction.
2. AltHold take-off, roll/pitch steady in hover, altitude hold at mid-stick, hover throttle vs
   the model's prediction (`models/mole_quad/predicted.json`; 0.354 without payload, which
   matches the real FC's `MOT_THST_HOVER` 0.35).
3. Yaw right turns right. 4. Land and auto-disarm; fails if ArduPilot's crash detector fires.

## Fly it by hand (MAVProxy)

With the launch running, wait ~30 s for `EKF3 IMU0 origin set` (in Gazebo, right-click
`mole_quad` → **Follow**). In a second terminal:

```bash
cd ~/mole_sim        # MAVProxy writes mav.tlog / mav.parm into the folder it starts in
~/mole_sim/venv-ardupilot/bin/mavproxy.py --master=tcp:127.0.0.1:5762 --console --map
```

Wait for `Received 1370 parameters`, then enter one at a time:

```
param load /path/to/repo/ROS/sim/ros2_ws/src/mole_gazebo/config/suggested_gains_water_module.parm
mode LOITER          # needs gps:=true; ALT_HOLD flies like the real drone
arm throttle
rc 3 1700            # climb; rc 3 1500 to hold
mode RTL             # fly home and land (gps:=true); otherwise mode LAND
```

Sticks: `rc 1` roll, `rc 2` pitch (1400 = forward), `rc 3` throttle, `rc 4` yaw; 1500 = centre.

Gotchas:

- **Every launch resets parameters** to the real FC's values (`wipe:=true`). Re-run `param load`
  after each launch, or launch with `wipe:=false`.
- **MAVProxy mode names need underscores** (`mode ALT_HOLD`); type `mode` alone to list them.
- **Change parameters only after** `Received 1370 parameters`.
- **Don't close MAVProxy while flying**: the GCS failsafe lands the drone after 5 s, as on the
  real drone.
- Gazebo/RViz windows crash from VS Code's terminal (snap). The launch file works around it; use
  a normal terminal for anything else.

## Current findings

All results are roll/pitch rate RMS in hover with the real FC's rate gains (`ATC_RAT_RLL/PIT_P`
0.135, `_I` 0.135, `_D` 0.0036: the ArduPilot defaults) times a multiplier.

**Current model (2026-11-15):** water module fitted, motors modelled as the real hardware (no
current limit in the model, up to ~3.8 kgf / ~60 A extrapolated), and the planned firmware limit
`MOT_SPIN_MAX` 0.73 (2.0 kgf / 34.5 A per motor, `config/planned_changes.parm`), which SITL
enforces like the real FC would.

| Configuration | ×0.6 | ×1.0 | ×1.1 | ×1.25 | ×1.5 |
|---|---|---|---|---|---|
| Module + `MOT_SPIN_MAX` 0.73 (planned) | **0.2 °/s, 11/11** | 0.2 °/s | 0.5 °/s | 71 °/s oscillates | 70 °/s oscillates |
| Module + `MOT_SPIN_MAX` 0.95 (FC today) | | 70 °/s oscillates | | | |

- The **current limit is what stabilises the real gains**: it compresses the motor output range
  to ~0.73×, which lowers the effective loop gain. The module's extra inertia alone does not.
- With the planned setup the real gains sit close to the oscillation boundary (×1.1 steady,
  ×1.25 not). **Suggested gains: ×0.6** (`config/suggested_gains_water_module.parm`: P 0.081,
  I 0.081, D 0.00216 for roll and pitch), about 2× margin, because the model is not yet
  calibrated against a real flight.
- Hover output with the module is 0.394 (model predicts 0.395).
- **Landing on the module tips the drone over easily**: the model has no landing gear, so it
  rests on the module's 96 mm-wide base with the CoG ~126 mm above it; one simulated landing
  tripped ArduPilot's crash detector. If the real drone has no legs with the module fitted, it
  will have the same problem. Add the real landing gear to `vehicle.json` once known.

**Earlier model (2026-11-08, no payload), kept for the crash analysis.** These runs compared
motor models; the 2.0 kgf cap was later removed from the model in favour of `MOT_SPIN_MAX`.

| Motor model (no payload, `MOT_SPIN_MAX` 0.95) | Real gains | Gains × 0.5 |
|---|---|---|
| Capped in the model at measured 2.0 kgf, lag 0.02 s | roll 62 °/s RMS; holds altitude | steady (0.3 °/s) |
| Uncapped (extrapolated 3.8 kgf), lag 0.01 s | oscillates, climbs 18 m in 8 s | holds altitude; fast roll wobble (±3°, ≥10 Hz) |
| Uncapped, lag 0.02 s | oscillates, climbs 9.5 m | steady |
| Uncapped, lag 0.04 s | oscillates, climbs 15 m | steady |

- The roll/pitch oscillation with the real gains appears in every variant.
- Flight log 00016 (the cage-ceiling crash, on branch `flight-testing`) shows motor outputs
  swinging between minimum (1150) and near maximum (1949) while `CTUN.ThO` was 0 and the drone
  climbed ~9 m. The **uncapped** models reproduce both the oscillation and that climb; the capped
  model oscillates more mildly and does not climb. Thrust above 73 % output is extrapolated, so
  the uncapped climb rates may be overstated.
- Thrust/weight without payload: ~6.5 uncapped, ~3.6 with motors limited to 2 kgf. With the
  module: ~5.6 uncapped, 2.9 at `MOT_SPIN_MAX` 0.73.
- The fast wobble with 0.01 s lag is probably an artifact of the simulated rotor speed loop
  (natural frequency ~10 Hz at that setting); settling it needs real motor step-response data.

**Simulation results are not a substitute for careful real-world tuning**: use them to choose a
safer starting point, then tune on the real drone (start low, tethered or at low height).

Other issues seen in the crash log but not modelled: high vibration (72 m/s² peaks, 114 clipping
events), harmonic notch off, tune parameters still generic defaults, no RC link.

## How the model was built

All numbers are in [`config/vehicle.json`](ros2_ws/src/mole_gazebo/config/vehicle.json);
[`scripts/gen_model.py`](ros2_ws/src/mole_gazebo/scripts/gen_model.py) generates
`models/mole_quad/model.sdf` from it. Edit the JSON and re-run the script; don't edit the SDF.

| Quantity | Value | Source |
|---|---|---|
| Frame + arms + motors + Pi/FC/ESC/wiring | 0.700 kg | EMSD CAD mass properties |
| Battery (6S 12000 mAh), on top, 195 × 77 × 50 mm | 1.500 kg | team; re-measure (1.58 kg also quoted) |
| Props, 8 in | 4 × 13 g | team |
| **Total** | **2.252 kg**, CoG 37 mm above frame centre | |
| Inertia Ixx / Iyy / Izz | 0.0165 / 0.0150 / 0.0275 kg·m² | CAD + battery box + props |
| Motor layout | 280 mm front–back × 368 mm side–side | EMSD CAD; front = CAD −y (team) |
| Thrust, torque | T = 6.983e-6 ω², Q = 1.019e-7 ω² | `ROS/EMSD/motor_data.xlsx`, R² = 0.999 |
| Motor speed vs output | ω = 2214 (cmd + 0.048) rad/s | DC-motor fit of the same data, ±3 % thrust from hover to full |
| Maximum per motor | ~3.8 kgf, ~22,000 rpm, ~60 A at full output (extrapolated); 2.0 kgf / 34.5 A measured at DShot 1500 | bench data + DC-motor model; team: motors can draw ~60 A. Limited in firmware with `MOT_SPIN_MAX` 0.73 (planned), not in the model (`max_torque_nm: null`) |
| Water module | 482 g, under the frame, 96 × 186 × 95 mm equivalent box | `config/payloads/water_module.urdf` (branch `manan`); mount position from the team; top-face height **assumed** |
| Total with module | 2.734 kg, CoG 20 mm above frame centre, hover output 0.394 | |
| Thrust/weight with module | 2.9 at `MOT_SPIN_MAX` 0.73; ~5.6 unlimited | |
| Hover | ~8,500 rpm, output 0.354 | matches real `MOT_THST_HOVER` 0.35 |
| Motor lag | 0.02 s | **assumed** |
| Spin, output channels | FR/RL CCW, FL/RR CW; SERVO1..4 = motors 4,1,2,3 | `companion/README.md`; FC log 00016 |

Propellers are spinning joints with Gazebo `LiftDrag` blades (as in ArduPilot's Iris model),
sized so thrust and reaction torque follow the measured curves.

## Parameters

| File | Contents |
|---|---|
| `config/mole_quad.parm` | 337 flight parameters from the real FC (tuning, filters, mixing, failsafes, EKF). **Generated**: re-run after retuning |
| `config/planned_changes.parm` | changes planned for the real FC but not yet on it: `MOT_SPIN_MAX` 0.73 |
| `config/sitl_overrides.parm` | sim-only: level IMU (real FC is mounted roll-180), no output when disarmed, file logging, 6S battery |
| `config/sitl_gps.parm`, `sitl_nogps.parm` | the `gps:=` variants |
| `config/suggested_gains_water_module.parm` | suggested roll/pitch rate gains with the module (not loaded automatically) |

Load order: ArduPilot's stock `copter.parm` → `mole_quad.parm` → `planned_changes.parm` →
`sitl_overrides.parm` → GPS variant.

After a real flight with new parameters, refresh the sim from its log:

```bash
$PY src/mole_gazebo/scripts/extract_params.py path/to/new_log.bin src/mole_gazebo/config/mole_quad.parm
```

## Known limitations

- Motor lag is assumed (0.02 s); battery sag, ESC behaviour, prop wash, ground effect and
  vibration are not modelled. Body aerodynamic drag is not modelled.
- Motor speed is linear in output (ArduPilotPlugin limitation); thrust error ≤ 3 % over 0.15–0.95,
  larger near zero.
- Thrust above ~73 % output (DShot 1500) is extrapolated from the bench data.
- The water module is a box with the right mass properties (its meshes are not on GitHub); its
  top-face height is assumed. No landing gear is modelled.
- Inertia comes from CAD with assumed component placement.
- The world is an open field; an indoor cage world needs the cage's dimensions.

## Open questions for the team

1. **Landing gear** with the module fitted? If yes: leg length and spacing. If no: the real drone
   will tip on landing too.
2. **Water module CAD/meshes** (`emsd_drone_description/meshes/water_module/*.stl`,
   `config/water_module.json`, branch `manan`) are not on this branch; they are needed for its
   real shape and to check the assumed top-face height (42.5 mm above its origin).
3. Does the module carry water in flight? If so, how much (adds mass)?
4. Body aerodynamic drag: without it the drone drifts in ALT_HOLD instead of slowing down.
   Needs a drag estimate or a real-flight log to fit.
