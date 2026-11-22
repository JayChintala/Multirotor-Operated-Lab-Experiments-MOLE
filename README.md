# Multirotor Operated Lab Experiments (MOLE)

A modular drone with hot-swappable payloads, built for CMU's Electromechanical
Systems class. This repository holds the project's software, simulations and
schematics.

## Repository layout

| Folder | What it is |
|---|---|
| [`companion/`](companion/README.md) | Raspberry Pi companion computer: MAVLink routing to the flight controller, link check, bench motor test, kill switch, telemetry bridge |
| [`Frontend_laptop_operator/`](Frontend_laptop_operator/drone-control-center/README.md) | Laptop ground station: web control center, Python backend that bridges the browser to the Pi over UDP |
| [`ROS/`](ROS/README.md) | Simulations (two separate ones, see below) |
| `Valve_sense/` | Arduino Uno sketch: water sensing and valve control |
| `Valve_sense_pico/` | The same valve and water-sensing sketch ported to the Raspberry Pi Pico 2 W |
| `water_test/` | Arduino sketch: water sensor test |
| `Electrical/` | Electrical circuit simulation (`Sim1.png`) |

## Which simulation do I want?

- **[`ROS/EMSD`](ROS/EMSD/README.md)**: course physics sim. A custom Python
  controller drives a Gazebo model of the frame, built from CAD and measured
  motor data. Use it to study the dynamics and design controllers.
- **[`ROS/sim`](ROS/sim/README.md)**: the real ArduCopter firmware running in
  SITL against Gazebo Harmonic, controlled through MAVROS. Use it to tune and
  check flight-controller parameters before flying. Also holds the Vicon
  motion-capture pipeline (`mole_mocap`), simulated and for the lab.

## Platform

The main platform is Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. `ROS/EMSD`
also supports Ubuntu 22.04 with ROS 2 Humble and Gazebo Fortress; `ROS/sim` is
Jazzy only.

## License

MIT, see [`LICENSE`](LICENSE).
