# EMSD drone project

## Current state

The corrected converter exports the complete assembly: 19 solids, approximately 406 x 320 x 42 mm. The mesh has already been regenerated. Original CAD axes and origin are preserved. The earlier converter exported only the first solid; that bug is fixed.

A force-driven Gazebo Fortress simulation now connects position/attitude/rate
control and measured-table motor allocation to a dynamic drone. Gravity, full
rigid-body inertia, motor forces/reaction torques and ground contact determine
motion. The 700 g mass uses supplied component weights; inertia is derived from
CAD using a shared frame/arm density, electronics at CoM and point-mass motors.
Motor dimensions and mount coordinates remain provisional.

See [simulation/README.md](simulation/README.md) for physical assumptions,
references, tuning and validation. [control/README.md](control/README.md) documents
the offline body-rate PID/allocator, measured CSV format and current estimates.

From this repository root:

```bash
python3 control/example.py --thrust 10
python3 control/example.py --gazebo
# Same DART physics without GUI/transport:
python3 control/example.py --gazebo --headless --duration 38
```

`example.py` needs `--thrust N` for a one-step offline calculation or `--gazebo`
for a simulation. Running it without either option prints usage and exits.
`control/drone_control.py` and `control/physics_control.py` define controller
classes used by the example and native Gazebo plugin; running those files
directly produces no flight. Use system `python3` for the controller commands
above. The project's `.venv` contains CadQuery for CAD tools but currently
lacks SciPy, so `.venv/bin/python control/example.py --thrust 10` fails at import.
To use that environment for control, install `control/requirements.txt` into it.

The old pose animation is available only with `--gazebo --scripted`.

## Install Gazebo and launch

```bash
sudo apt update
sudo apt install ros-humble-ros-gz libignition-gazebo6-dev libignition-physics5-dev libignition-plugin1-dev python3-dev cmake g++ python3-numpy python3-scipy
source /opt/ros/humble/setup.bash
cd ~/Desktop/EMSD/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch emsd_drone_description gazebo.launch.py
```

In Gazebo, find `emsd_drone_frame` in the entity tree and double-click it to focus the camera if needed.

Gazebo Fortress is the standard Humble pairing: https://gazebosim.org/docs/garden/ros_installation/

## RViz

After building and sourcing the workspace:

```bash
ros2 launch emsd_drone_description view.launch.py
```

## Reconvert after changing CAD

From EMSD:

```bash
.venv/bin/python tools/convert_step.py
.venv/bin/python tools/derive_mass_properties.py
```

Rebuild afterwards. See `cad/conversion_report.json` for bounds and solid count.

## Validation and remaining work

Native plugin/standalone DART builds, a Gazebo ECM-to-Python force-adapter test,
six controller tests, and physical integration checks passed. The latter verify
freefall, positive/negative roll/pitch/yaw from unequal motor commands, and a
38-second takeoff/figure-eight with 5.74 cm path RMS position error. Package
configure/build/install and execution from the installed package also passed.
GUI/server transport is not validated here because tool-environment sockets are
restricted; the direct headless DART backend runs successfully.

Needed for realism: verified motor mounts/body axes, more accurate component
masses and locations, motor/propeller dimensions, identified motor lag and drag,
sensor/estimator models, battery/voltage effects, allocation-saturation feedback
and flight-controller integration. Current feedback uses exact simulated state.
