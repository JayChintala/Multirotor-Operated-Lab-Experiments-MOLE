# EMSD setup and reproduction guide

This repository runs a four-motor drone model in ROS 2 Humble, Gazebo Fortress,
and RViz. The ZIP beside this guide contains the source and input data needed to
rebuild it on another computer. Keep the directory layout intact: the ROS
package's `CMakeLists.txt` reaches `simulation/` and `control/` through relative
paths.

## 1. Platform and prerequisites

Use **Ubuntu 22.04 (Jammy), ROS 2 Humble, and Gazebo Fortress**. This project
uses Fortress's `ignition-gazebo6` and DART C++ APIs, so another ROS/Gazebo
pairing requires code and build changes. For ROS installation and repository
setup, follow the [official ROS 2 Humble Ubuntu instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
The [Gazebo ROS installation guide](https://gazebosim.org/docs/latest/ros_installation/)
documents the Humble/Fortress pairing.

After ROS's apt repository is configured, install the project dependencies:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-gz \
  python3-colcon-common-extensions python3-numpy python3-scipy \
  python3-matplotlib python3-dev cmake g++ \
  libignition-gazebo6-dev libignition-physics5-dev libignition-plugin1-dev
```

`ros-humble-desktop` supplies RViz and `robot_state_publisher`. `ros-humble-ros-gz`
supplies the Gazebo integration. The development packages and C++ toolchain build
the custom motor physics plugin and direct DART runner. NumPy and SciPy are used
by the controller. Matplotlib is only needed for analysis plots.

For a Git checkout, clone the repository. For the ZIP, extract it; it contains
one top-level `EMSD/` directory. From then on, use its project root:

```bash
cd /path/to/EMSD
source /opt/ros/humble/setup.bash
```

Use system `python3` for the commands below. A copied Python virtual environment
is deliberately absent from the ZIP. If you create your own environment for
controller scripts, install `control/requirements.txt` into it and ensure it can
access Gazebo's system Python bindings. CAD regeneration separately needs
CadQuery, which is not needed to build or run the supplied meshes.

## 2. Build the ROS workspace

```bash
cd /path/to/EMSD
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix emsd_drone_description
```

The last command should print a path under `ros2_ws/install`. In each new
terminal, source `/opt/ros/humble/setup.bash` and then
`/path/to/EMSD/ros2_ws/install/setup.bash`. The `build/`, `install/`, and `log/`
directories are generated locally by colcon and are intentionally excluded
from the bundle.

## 3. Run by goal

Run these from the project root after sourcing the workspace. A graphical
desktop is needed for Gazebo GUI and RViz.

| Goal | Command | What it does |
| --- | --- | --- |
| View the frame in RViz | `ros2 launch emsd_drone_description view.launch.py` | Loads the URDF and frame mesh; does not simulate flight. |
| Fly the default path in Gazebo | `ros2 launch emsd_drone_description gazebo.launch.py` | Opens the physical simulation with takeoff and a figure-eight reference. Stop with Ctrl+C. |
| Run without a GUI | `ros2 launch emsd_drone_description gazebo.launch.py headless:=true duration:=38` | Runs the same DART dynamics and writes telemetry. |
| Run directly from source | `python3 control/example.py --gazebo --headless --duration 38` | Builds the native adapter if needed and runs the default path without ROS launch. |
| Try a one-step motor allocation | `python3 control/example.py --thrust 10` | Prints a controller/allocation calculation; no flight. |
| Write your own controller | `python3 control/final_control.py --headless --duration 10` | Runs the editable controller in `final_control.py`. |
| Test an attitude target | `python3 control/example.py --gazebo --mode attitude --roll-deg 10` | Holds a roll target; horizontal drift is possible. |
| Test direct motor commands | `python3 control/example.py --gazebo --mode motors --motor-commands .30 .25 .27 .21` | Applies open-loop measured-range commands. |
| Check gravity/contact | `python3 control/example.py --gazebo --mode freefall --headless --duration 2` | Runs with propulsion disconnected. |

`gazebo.launch.py` also accepts `mode:=trajectory|attitude|rates|motors|freefall`,
`config:=/absolute/path/to/drone.json`, `headless:=true|false`, and
`duration:=SECONDS`. For more source-runner options, run
`python3 control/example.py --help`. Outputs are written under
`simulation/runs/` relative to the working directory (including `config.json`,
`physics.sdf`, and, for completed runs, `telemetry.csv`). These are generated
artifacts, not inputs to commit.

## 4. File map

| Path | Purpose and when to edit/use it |
| --- | --- |
| `ros2_ws/src/emsd_drone_description/package.xml`, `CMakeLists.txt` | ROS package metadata and build/install rules. Keep the workspace structure. |
| `ros2_ws/src/emsd_drone_description/launch/` | `view.launch.py` for RViz; `gazebo.launch.py` for physical simulation. |
| `ros2_ws/src/emsd_drone_description/urdf/frame.urdf` | Frame and motor links for the visual robot description. |
| `ros2_ws/src/emsd_drone_description/worlds/frame.sdf` | Gazebo scene template; the runner turns its drone into a dynamic body. |
| `ros2_ws/src/emsd_drone_description/meshes/*.stl` | Ready-to-use frame, ground, and pond visuals. These are included so CAD software is unnecessary for a normal build. |
| `simulation/MotorPhysics.cc`, `headless.cc`, `CMakeLists.txt` | C++ force adapter, direct DART runner, and native build. |
| `control/physics_sim.py`, `physics_control.py` | Generates the dynamic SDF, launches simulation, computes feedback and motor forces. |
| `control/drone_control.py` | Motor lookup, allocation, and body-rate PID used by the example. |
| `control/example.py`, `simulation_args.py` | Main demonstrations and command-line options. |
| `control/final_control.py` | Starting point for your own trajectory and four-motor controller. Edit `reference_trajectory()` and `your_controller()`. |
| `control/config/drone.json` | Mass, inertia, motor locations/signs, lookup tables, lag, drag, gains, and reference settings. |
| `control/config/water.json` | Pond/water settings for buoyancy. |
| `control/motor_data.csv` | Measured motor lookup consumed by the controller. `motor_data.raw.csv` keeps individual rows; `motor_data.template.csv` documents the format. |
| `motor_data.xlsx`, `tools/import_motor_data.py` | Original measurements and converter to regenerate both motor CSV files. |
| `cad/QAV-RXL2_8inch.step`, `tools/convert_step.py`, `tools/derive_mass_properties.py` | Original CAD and optional mesh/mass regeneration. |
| `tools/create_environment.py` | Regenerates ground and pond STL meshes. |
| `tools/fit_motor_models.py`, `tools/compare_higher_order_models.py`, `tools/plot_motor_benchmarks.py` | Explore motor fits using the workbook; write results under `analysis/motor_identification/`. |
| `tools/validate_physics.py`, `tools/plot_physics.py` | Physical regression checks and plots under `analysis/physics_validation/`. |
| `control/test_*.py`, `simulation/test_motor_physics.cc` | Controller and native adapter tests. |
| `README.md`, `control/README.md`, `simulation/README.md` | Project overview, motor/controller conventions, and physics assumptions. |

## 5. Reproduce or modify inputs

The included meshes, motor table, and `drone.json` are ready to use. To rebuild
the motor lookup from its raw workbook:

```bash
python3 tools/import_motor_data.py
```

To regenerate the frame mesh and mass/inertia after changing the STEP file,
install a compatible CadQuery into a separate Python environment, then run:

```bash
python3 tools/convert_step.py
python3 tools/derive_mass_properties.py
cd ros2_ws
colcon build --symlink-install
```

`convert_step.py` writes `meshes/frame.stl` and `cad/conversion_report.json`.
`derive_mass_properties.py` writes `cad/mass_properties.json` and updates
`control/config/drone.json`. The CAD-derived parameters use stated assumptions
about component masses and provisional motor positions; see
`simulation/README.md` before changing the physical model. To regenerate the
ground and pond meshes, run `python3 tools/create_environment.py` and rebuild.

To reproduce motor-model analysis, install Matplotlib and run the scripts in
this order: `fit_motor_models.py`, `compare_higher_order_models.py`, then
`plot_motor_benchmarks.py`. `analysis/motor_identification/README.md` explains
the resulting fits and their limits. These fits are exploratory; they are not
automatically substituted into the flight controller.

## 6. Check the build

```bash
python3 -m unittest discover -s control -v
ctest --test-dir ros2_ws/build/emsd_drone_description/physics --output-on-failure
python3 tools/validate_physics.py
```

The CTest command uses the native adapter built by `colcon build`. The
physics validation takes longer and writes generated outputs. GUI launch needs
a working display and graphics stack; the headless route is suitable for
servers and CI.

## Publishing to Git

You can commit `EMSD_reproducible_source.zip` and `SetupRos.md` to an existing
repository, or extract the ZIP and commit its `EMSD/` contents as normal source
files. Committing the extracted source makes code review and later changes
easier. The included `.gitignore` keeps local build products out of a source
checkout. Do not commit a local `.venv`, `ros2_ws/build`, `ros2_ws/install`,
`ros2_ws/log`, or `simulation/runs` directory.

## Reproducibility boundary

The archive contains source files, the original STEP and motor workbook, and
the prebuilt mesh assets needed for a fresh build. It excludes local Python
environments, colcon/CMake outputs, run logs, generated plots, and an older ZIP.
The current model uses measured steady-state motor data but assumed motor lag,
drag, water geometry, and approximate mass distribution. It controls Gazebo
ground-truth state; it is not a ROS hardware controller or a validated real
aircraft model. See `simulation/README.md` and `control/README.md` for those
assumptions before using results beyond simulation.
