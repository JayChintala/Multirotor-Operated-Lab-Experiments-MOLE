# Controller experiments

The Python control library supports offline allocation and a force-driven Gazebo
Fortress runtime. `python3 control/example.py --gazebo` now runs physical flight:
the native plugin reads ground-truth state, calls the Python controller, and
applies the calculated forces and torques to a dynamic rigid body. There is no
ROS controller node or hardware ESC output. Four cylinder visuals represent
motors; their 28 mm diameter, 20 mm height and mount positions (±184, ±140,
−2.5 mm in original CAD axes) remain provisional. Rotor spin is not visualized.

The default measured lookup is `control/motor_data.csv`, imported from
`motor_data.xlsx` (sheet `MQ1053B_6s_StepsTest_thrust_1_r`). Regenerate it with
`python3 tools/import_motor_data.py`. `control/motor_data.raw.csv` preserves all
82 individual measurements and signed thrust/torque values. The lookup averages
the two measurements at each of 41 ESC settings. Thrust is `abs(kgf) * 9.80665`;
reaction torque uses the mean of measured magnitudes, with body reaction signs
specified separately by the allocator. No sensor-offset correction is applied.

The workbook identifies DShot300 and contains ESC signals from 200 to 1500.
`command = (esc_signal - 200) / 1300` is a normalized **tested-range coordinate**,
not a DShot packet or calibrated full-range throttle. Convert back using
`esc_signal = 200 + 1300 * command`. The spreadsheet includes fractional signals;
actual hardware command encoding/rounding must be defined separately.

There is no motor-off sample: command 0 corresponds to the lowest measured,
still-spinning operating point. Interpolation and allocation are bounded by the
measured minimum and maximum thrust; no zero row or below-range extrapolation
is added. Infeasible requests return an allocation residual. Voltage varies
approximately 23.94–25.12 V during the sweep; this is one empirical lookup,
not a voltage-compensated model. `motor_data.template.csv` is an empty format
reference only; its measurements are not used by the example.

```bash
cd ~/Desktop/EMSD
python3 control/example.py --thrust 10
python3 control/example.py --gazebo --headless --duration 1
```

The controller entry point needs `--thrust N` or `--gazebo`; without either it
shows a usage error. `drone_control.py` and `physics_control.py` are imported
modules, so running them directly has no output. System `python3` currently has
NumPy and SciPy. The project `.venv` has CadQuery but lacks SciPy; install
`control/requirements.txt` there before using it to run control commands.

`Controller.update(state, target, dt)` returns `[Fz, Tx, Ty, Tz]`. Implement that method in a new class to substitute LQR, MPC, or another control law. Extend `State` and the setpoint types as needed. `RatePID` is an inner angular-rate loop; position/altitude and attitude loops must supply thrust and rate targets. Demonstration gains are not tuned. Call `reset()` when restarting a controller. Its conditional integration handles its own torque limits; allocation saturation requires additional feedback/anti-windup in a closed-loop implementation.

`Allocator` accepts four motor positions relative to the actual centre of mass, four body yaw-reaction signs, four individual measurement tables, and four positive wrench scales. It assumes all thrust vectors point along body +Z. Roll/pitch moments are `y*F` and `-x*F`; yaw comes from measured reaction torque. Rotation direction viewed from above is opposite the body reaction sign. Motor order follows the four URDF mount joints, counterclockwise in CAD XY starting at (+X,+Y). Verify the CAD-to-body rotation, CoM offset, mount coordinates and spin directions before using these conventions.

The allocation minimizes scaled wrench error under per-motor thrust bounds. Smaller `wrench_scales` give that axis more priority. `converged` reports numerical optimizer success, not command feasibility; always inspect `residual`. Current is a steady-state estimate summed over motors, excluding avionics and battery/transient effects. Battery current limits and motor dynamics are not modeled.

The physical runtime and its assumptions are documented in
[simulation/README.md](../simulation/README.md). Sensor realism, identified
actuator dynamics, validated parameters and hardware integration remain future work.

## Physical Gazebo simulation

For your own four-motor controller and reference trajectory, edit and run
`control/final_control.py`:

```bash
python3 control/final_control.py
python3 control/final_control.py --headless --duration 10
```

Edit `reference_trajectory(t)` for desired world position, velocity,
acceleration and yaw; edit `your_controller(state, reference, vehicle)` to
return four normalized motor commands in motor order 1–4. Set
`INERTIA_MATRIX_KG_M2` in that file if you want to override the vehicle's
3×3 body-frame inertia in Gazebo. The starter controller only commands
approximate open-loop hover: it does not yet track your reference. The file
documents world/body axes and all controller inputs. Gazebo still applies the
measured motor thrust and reaction torque, motor lag, gravity, contacts,
drag, and pond buoyancy. Its generated run directory contains telemetry.
For motor `i`, the body moment about the center of mass is
`[y_i * thrust_i, -x_i * thrust_i, yaw_sign_i * reaction_i]`.
The `motor_i_tx_body_nm`, `motor_i_ty_body_nm`, and `motor_i_tz_body_nm`
telemetry columns show these four contributions before drag or water moments.
The reported total body torque also includes drag and water. Motor positions
come from the provisional CAD mounts; the yaw signs in `drone.json` assume
alternating propeller directions and need confirmation against the installed
motors.

Run `python3 control/example.py --gazebo` from the repository root. It builds
the native adapter, opens Gazebo, and controls takeoff to 1.5 m and a repeating
30-second figure-eight using applied motor forces. Roll and pitch result from
the physics and feedback controller. Stop with Ctrl+C or close Gazebo.

Vehicle mass/inertia, four motor tables, assumed motor lag, control gains and
reference trajectory parameters are in `control/config/drone.json`.
`physics_control.reference()` generates position/velocity/acceleration targets.
CSV telemetry and the effective world/config are saved to `simulation/runs/`.

```bash
python3 control/example.py --gazebo --headless --duration 38
python3 control/example.py --gazebo --mode attitude --roll-deg 10
python3 control/example.py --gazebo --mode attitude --pitch-deg 10
python3 control/example.py --gazebo --mode motors --motor-commands 0.30 0.25 0.27 0.21
```

Headless mode runs Gazebo's DART backend directly without transport or GUI.
Direct motor mode is open loop: a persistent nonzero torque can tumble the drone.
The original offline `--thrust` example is also available; `--rates P Q R`
sets its body-rate target (default 1.0, 0.5, 0.4 rad/s).

## Legacy scripted animation

`python3 control/example.py --gazebo --scripted` explicitly selects the older
pose animation in `gazebo_demo.py`. Only this legacy mode sets vehicle poses
directly. Its `trajectory(t)` returns x/y/z/roll/pitch/yaw and has illustrative
±12° roll and ±8° pitch, unrelated to motor forces.
