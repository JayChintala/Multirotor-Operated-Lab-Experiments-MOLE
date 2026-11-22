# EMSD drone simulation

A four-motor model of the drone frame, flown in Gazebo by a Python controller.
The native plugin reads ground-truth state, calls the controller, and applies the
measured motor thrust and reaction torque to a dynamic rigid body. Gravity, full
inertia, ground contact and pond buoyancy decide the motion. The vehicle pose is
never set directly. RViz shows the geometry only.

This is a simulation of an approximate model, not a ROS hardware controller. For
ArduPilot firmware in the loop, see [`../sim`](../sim/README.md).

More detail:
- [simulation/README.md](simulation/README.md): physics model, mass/inertia, buoyancy and assumptions.
- [control/README.md](control/README.md): motor data, command normalisation, controller/allocator API, `final_control.py`.
- [analysis/motor_identification/README.md](analysis/motor_identification/README.md): current-to-thrust/torque fits.

Keep the directory layout intact: the ROS package's `CMakeLists.txt` reaches
`simulation/` and `control/` through relative paths.

## 1. Platform and prerequisites

| Ubuntu | ROS 2 | Gazebo | `<distro>` below |
| --- | --- | --- | --- |
| 24.04 (Noble) | [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) | Harmonic (gz-sim8) | `jazzy` |
| 22.04 (Jammy) | [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) | Fortress (ignition-gazebo6) | `humble` |

The build and the runner use Harmonic if `gz sim` works and fall back to
Fortress otherwise. To force one, build with `-DEMSD_GZ=fortress` and run with
`EMSD_GZ=fortress` (or `harmonic`). The
[Gazebo ROS installation guide](https://gazebosim.org/docs/latest/ros_installation/)
documents both pairings.

Jazzy / Harmonic (Gazebo headers come with `ros-jazzy-ros-gz`):

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz \
  python3-colcon-common-extensions python3-numpy python3-scipy \
  python3-matplotlib python3-dev cmake g++
```

Humble / Fortress:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-gz \
  python3-colcon-common-extensions python3-numpy python3-scipy \
  python3-matplotlib python3-dev cmake g++ \
  libignition-gazebo6-dev libignition-physics5-dev libignition-plugin1-dev
```

`ros-<distro>-desktop` supplies RViz and `robot_state_publisher`;
`ros-<distro>-ros-gz` the Gazebo integration. The dev packages and C++ toolchain
build the motor physics plugin and the direct DART runner. NumPy and SciPy are
used by the controller; Matplotlib only for analysis plots.

Use system `python3` for the commands below. No virtual environment is shipped.
If you make your own, install `control/requirements.txt` into it and give it
access to Gazebo's system Python bindings. CAD regeneration needs CadQuery,
which is not needed to build or run with the supplied meshes; keep it in a
separate environment.

## 2. Build

From the repository root:

```bash
source /opt/ros/<distro>/setup.bash
cd ROS/EMSD/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix emsd_drone_description
```

The last command should print a path under `ros2_ws/install`. In each new
terminal, source `/opt/ros/<distro>/setup.bash` and then
`ROS/EMSD/ros2_ws/install/setup.bash`.

## 3. Run by goal

Run these from `ROS/EMSD` after sourcing the workspace. Gazebo GUI and RViz need
a graphical desktop.

| Goal | Command | What it does |
| --- | --- | --- |
| View the frame in RViz | `ros2 launch emsd_drone_description view.launch.py` | Loads the URDF and frame mesh; no flight. |
| Fly the default path in Gazebo | `ros2 launch emsd_drone_description gazebo.launch.py` | Takeoff to 1.5 m, then a repeating 30 s figure-eight. Stop with Ctrl+C. |
| Run without a GUI | `ros2 launch emsd_drone_description gazebo.launch.py headless:=true duration:=38` | Same DART dynamics, writes telemetry. |
| Run directly from source | `python3 control/example.py --gazebo --headless --duration 38` | Builds the native adapter if needed; no ROS launch. |
| One-step motor allocation | `python3 control/example.py --thrust 10` | Prints a controller/allocation calculation; no flight. `--rates P Q R` sets the body-rate target (default 1.0 0.5 0.4 rad/s). |
| Write your own controller | `python3 control/final_control.py --headless --duration 10` | Runs the editable controller in `final_control.py`. |
| Attitude target | `python3 control/example.py --gazebo --mode attitude --roll-deg 10` | Holds a roll target (`--pitch-deg`, `--yaw-deg` too); horizontal drift is expected. |
| Body-rate target | `python3 control/example.py --gazebo --mode rates --rates 0.2 0.1 0.1` | Persistent rates keep rotating the vehicle. |
| Direct motor commands | `python3 control/example.py --gazebo --mode motors --motor-commands .30 .25 .27 .21` | Open loop; nonzero torque can tumble the drone. |
| Gravity/contact check | `python3 control/example.py --gazebo --mode freefall --headless --duration 2` | Propulsion disconnected. |
| Legacy pose animation | `python3 control/example.py --gazebo --scripted` | Old non-physical animation in `gazebo_demo.py`. |

`example.py` needs `--thrust N` or `--gazebo`; without either it prints usage.
`drone_control.py` and `physics_control.py` are imported modules and do nothing
when run directly.

`gazebo.launch.py` also accepts `mode:=trajectory|attitude|rates|motors|freefall`,
`config:=/absolute/path/to/drone.json`, `headless:=true|false` and
`duration:=SECONDS`. See `python3 control/example.py --help` for the source runner.
Headless runs default to 30 simulated seconds; GUI runs with duration 0 run until
stopped. Each run writes `config.json`, `physics.sdf` and `telemetry.csv` under
`simulation/runs/` relative to the working directory (gitignored).

In the Gazebo GUI, find `emsd_drone_frame` in the entity tree and double-click it
to focus the camera.

## 4. File map

| Path | Purpose |
| --- | --- |
| `ros2_ws/src/emsd_drone_description/package.xml`, `CMakeLists.txt` | ROS package metadata and build/install rules. |
| `ros2_ws/src/emsd_drone_description/launch/` | `view.launch.py` (RViz), `gazebo.launch.py` (physics). |
| `ros2_ws/src/emsd_drone_description/urdf/frame.urdf` | Frame and motor links for the visual description. |
| `ros2_ws/src/emsd_drone_description/worlds/frame.sdf` | Scene template; the runner turns the drone into a dynamic body. |
| `ros2_ws/src/emsd_drone_description/meshes/*.stl` | Frame, ground and pond meshes, so no CAD software is needed to build. |
| `simulation/MotorPhysics.cc`, `headless.cc`, `gz_compat.hh`, `CMakeLists.txt` | C++ force adapter, direct DART runner, Harmonic/Fortress compatibility, native build. |
| `control/physics_sim.py`, `physics_control.py`, `gazebo_backend.py` | Generates the dynamic SDF, launches the sim, computes feedback and motor forces; picks Harmonic or Fortress. |
| `control/drone_control.py` | Motor lookup, allocation and body-rate PID. |
| `control/water_physics.py`, `config/water.json` | Pond buoyancy. |
| `control/example.py`, `simulation_args.py` | Main demonstrations and command-line options. |
| `control/final_control.py` | Starting point for your own trajectory and controller. |
| `control/config/drone.json` | Mass, inertia, motor positions/signs, lookup tables, lag, drag, gains, reference. |
| `control/motor_data.csv`, `motor_data.raw.csv`, `motor_data.template.csv` | Measured motor lookup, individual rows, and format reference. |
| `motor_data.xlsx`, `tools/import_motor_data.py` | Original measurements and the converter that regenerates both motor CSVs. |
| `cad/QAV-RXL2_8inch.step`, `tools/convert_step.py`, `tools/derive_mass_properties.py` | Original CAD and optional mesh/mass regeneration. |
| `tools/create_environment.py` | Regenerates the ground and pond meshes. |
| `tools/fit_motor_models.py`, `compare_higher_order_models.py`, `plot_motor_benchmarks.py`, `fit_two_pole_one_zero.py`, `build_notion_motor_tables.py` | Motor-fit exploration; outputs go to `analysis/motor_identification/`. |
| `tools/validate_physics.py`, `tools/plot_physics.py` | Physics regression checks and plots, written to `analysis/physics_validation/`. |
| `control/test_*.py`, `simulation/test_motor_physics.cc` | Controller and native adapter tests. |

## 5. Regenerate inputs

The meshes, motor table and `drone.json` are ready to use. To rebuild the motor
lookup from the workbook:

```bash
python3 tools/import_motor_data.py
```

After changing the STEP file, run these with a Python that has CadQuery, then
rebuild the workspace:

```bash
python3 tools/convert_step.py
python3 tools/derive_mass_properties.py
```

`convert_step.py` writes `meshes/frame.stl` and `cad/conversion_report.json`
(bounds and solid count; currently 19 solids, about 406 × 320 × 42 mm, original
CAD axes and origin preserved). `derive_mass_properties.py` writes
`cad/mass_properties.json` and updates `control/config/drone.json`; read
[simulation/README.md](simulation/README.md) before changing the physical model.
`python3 tools/create_environment.py` regenerates the ground and pond meshes.

For motor-model analysis, run `fit_motor_models.py`,
`compare_higher_order_models.py`, then `plot_motor_benchmarks.py`. These fits are
exploratory and are not fed into the controller.

## 6. Tests

```bash
python3 -m unittest discover -s control -v
ctest --test-dir ros2_ws/build/emsd_drone_description/physics --output-on-failure
python3 tools/validate_physics.py
```

The CTest directory is the native adapter built by `colcon build`. If you build
from source instead (`python3 control/example.py --gazebo --prepare-only`), the
same tests are in `simulation/build`. Physics validation takes longer and writes
outputs to `analysis/physics_validation/` (gitignored). The headless route suits
servers and CI.

Last recorded validation: freefall, both signs of roll/pitch/yaw from unequal
motor commands, and a 38 s takeoff/figure-eight with 5.74 cm path RMS position
error passed. Gazebo GUI/server transport was not validated in that environment.

## Limits and remaining work

Measured steady-state motor data, but assumed motor lag, drag, water geometry
and approximate mass distribution. Feedback uses exact simulated state. Needed
for realism: verified motor mounts and body axes, better component masses and
locations, motor/propeller dimensions, identified motor lag and drag, sensor and
estimator models, battery/voltage effects, allocation-saturation feedback, and
flight-controller integration.
