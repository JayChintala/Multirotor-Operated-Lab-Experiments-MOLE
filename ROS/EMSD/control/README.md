# Controller and motor conventions

Install and run commands are in [../README.md](../README.md); the physical model
is in [../simulation/README.md](../simulation/README.md). The Python library here
does offline allocation and drives the force-based Gazebo runtime (Harmonic or
Fortress). There is no ROS controller node or hardware ESC output.

Four cylinder visuals represent the motors; their 28 mm diameter, 20 mm height
and mount positions (±184, ±140, −2.5 mm in original CAD axes) are provisional.
Rotor spin is not visualized.

## Motor data

The default lookup is `control/motor_data.csv`, imported from `motor_data.xlsx`
(sheet `MQ1053B_6s_StepsTest_thrust_1_r`) by `python3 tools/import_motor_data.py`.
`control/motor_data.raw.csv` keeps all 82 individual measurements with signed
thrust/torque. The lookup averages the two measurements at each of 41 ESC
settings. Thrust is `abs(kgf) * 9.80665`; reaction torque is the mean of
measured magnitudes, with body reaction signs set separately by the allocator.
No sensor-offset correction is applied. `motor_data.template.csv` is an empty
format reference and is not used.

## Command normalisation

The workbook identifies DShot300 and contains ESC signals from 200 to 1500.
`command = (esc_signal - 200) / 1300` is a normalized **tested-range
coordinate**, not a DShot packet or calibrated full-range throttle. Convert back
with `esc_signal = 200 + 1300 * command`. The spreadsheet includes fractional
signals; hardware encoding/rounding must be defined separately.

There is no motor-off sample: command 0 is the lowest measured, still-spinning
point. Interpolation and allocation are bounded by the measured minimum and
maximum thrust; no zero row or below-range extrapolation is added. Infeasible
requests return an allocation residual. Voltage varies about 23.94–25.12 V
during the sweep; this is one empirical lookup, not a voltage-compensated model.

## Controller and Allocator API

`Controller.update(state, target, dt)` returns `[Fz, Tx, Ty, Tz]`. Implement that
method in a new class to substitute LQR, MPC or another law; extend `State` and
the setpoint types as needed. `RatePID` is an inner angular-rate loop;
position/altitude and attitude loops must supply thrust and rate targets.
Demonstration gains are not tuned. Call `reset()` when restarting. Its
conditional integration handles its own torque limits; allocation saturation
needs additional feedback/anti-windup in a closed loop.

`Allocator` takes four motor positions relative to the actual centre of mass,
four body yaw-reaction signs, four measurement tables and four positive wrench
scales. It assumes all thrust vectors point along body +Z. Roll/pitch moments are
`y*F` and `-x*F`; yaw comes from measured reaction torque. Rotation direction
viewed from above is opposite the body reaction sign. Motor order follows the
four URDF mount joints, counterclockwise in CAD XY starting at (+X,+Y). Verify
the CAD-to-body rotation, CoM offset, mount coordinates and spin directions
before relying on these conventions.

Allocation minimizes scaled wrench error under per-motor thrust bounds; smaller
`wrench_scales` give that axis more priority. `converged` reports optimizer
success, not feasibility, so always check `residual`. Current is a steady-state
estimate summed over motors, excluding avionics and battery/transient effects.

## Writing your own controller: `final_control.py`

Edit `reference_trajectory(t)` for desired world position, velocity,
acceleration and yaw, and `your_controller(state, reference, vehicle)` to return
four normalized motor commands in motor order 1–4. Set `INERTIA_MATRIX_KG_M2` to
override the 3×3 body-frame inertia. The starter controller only commands
approximate open-loop hover and does not yet track the reference. The file
documents world/body axes and all controller inputs.

Gazebo still applies measured thrust and reaction torque, motor lag, gravity,
contacts, drag and pond buoyancy. For motor `i`, the body moment about the CoM is
`[y_i * thrust_i, -x_i * thrust_i, yaw_sign_i * reaction_i]`. Telemetry columns
`motor_i_tx_body_nm`, `motor_i_ty_body_nm` and `motor_i_tz_body_nm` show these
contributions before drag or water moments; the total body torque includes drag
and water. The yaw signs in `drone.json` assume alternating propeller directions
and need confirming against the installed motors.

The default demo (`example.py --gazebo`) uses `physics_control.reference()` for
takeoff to 1.5 m and a repeating 30 s figure-eight; its parameters, mass/inertia,
motor tables, lag and gains are in `control/config/drone.json`.

## Legacy scripted animation

`example.py --gazebo --scripted` selects the old pose animation in
`gazebo_demo.py`, the only mode that sets vehicle poses directly. Its
`trajectory(t)` returns x/y/z/roll/pitch/yaw with illustrative ±12° roll and ±8°
pitch, unrelated to motor forces.
