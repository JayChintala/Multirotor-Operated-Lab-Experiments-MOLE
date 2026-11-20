# ROS simulations

Two separate simulations of the MOLE quadcopter. Pick the one that matches what you are testing.

| | [`EMSD/`](EMSD/README.md) | [`sim/`](sim/README.md) |
|---|---|---|
| What flies the drone | Our own Python controller (`EMSD/control/`) | The real flight-controller firmware (ArduCopter 4.7.2 in SITL) with the real FC's parameters |
| Physics | Gazebo with a custom motor/drag/buoyancy plugin, from CAD mass properties and bench motor data | Gazebo Harmonic with ArduPilot's plugin, from `config/vehicle.json` |
| Use it for | The course project: controller design, motor modelling, water-landing physics | Tuning and checking the real drone's flight parameters before flying |
| Platform | ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04) preferred; Humble + Fortress (22.04) also supported | ROS 2 Jazzy + Gazebo Harmonic + MAVROS (Ubuntu 24.04) only |

Each folder's README has its own install, build and run steps.
