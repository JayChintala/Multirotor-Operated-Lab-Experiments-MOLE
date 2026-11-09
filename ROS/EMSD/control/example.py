"""Run --gazebo for force-driven flight, or CSV --thrust N for offline allocation."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('csv', type=Path, nargs='?',
                    default=Path(__file__).resolve().parent / 'motor_data.csv')
parser.add_argument('--gazebo', action='store_true', help='Open the physics-based Gazebo simulation')
parser.add_argument('--scripted', action='store_true', help='Use the old pose animation with --gazebo')
parser.add_argument('--thrust', type=float, help='Requested total thrust in newtons')
parser.add_argument('--rates', type=float, nargs=3, default=[1.0, 0.5, 0.4],
                    metavar=('P', 'Q', 'R'),
                    help='Desired body rates in rad/s (default: 1.0 0.5 0.4)')
from simulation_args import add_simulation_arguments
add_simulation_arguments(parser)
args = parser.parse_args()
if args.gazebo:
    if args.scripted:
        from gazebo_demo import main
        main()
    else:
        from physics_sim import main
        main(args)
    raise SystemExit(0)
if args.csv is None or args.thrust is None:
    parser.error('provide CSV and --thrust, or use --gazebo')
import numpy as np
from drone_control import Allocator, MotorTable, RatePID, Setpoint, State
urdf = Path(__file__).resolve().parents[1] / 'ros2_ws/src/emsd_drone_description/urdf/frame.urdf'
robot = ET.parse(urdf).getroot()
positions = [list(map(float, robot.find(f"joint[@name='motor_{i}_mount']/origin").attrib['xyz'].split()))
             for i in range(1, 5)]
table = MotorTable(args.csv)
# EXAMPLE ONLY: CAD origin assumed CoM, CAD axes assumed controller body axes.
# Reaction signs alternate around the perimeter; verify your actual propeller setup.
yaw_signs = np.array([1, -1, 1, -1])
allocator = Allocator(positions, yaw_signs, [table] * 4,
                      wrench_scales=[1, 0.1, 0.1, 0.01])
# Demonstration gains/limits only, not tuned for this vehicle.
controller = RatePID(kp=[0.1, 0.1, 0.05], ki=0, kd=0,
                     integral_limit=0.1, torque_limit=0.2)
state = State(rates_rad_s=np.zeros(3))
# Nonzero rate errors request a net body torque and unequal motor outputs.
target = Setpoint(rates_rad_s=np.array(args.rates), thrust_n=args.thrust)
requested = controller.update(state, target, dt=0.01)
result = allocator.allocate(requested)
reaction_torque = yaw_signs * np.array([table.at_thrust(f)[1] for f in result.thrust_n])
positions = np.asarray(positions)
motor_torque = np.column_stack((positions[:, 1] * result.thrust_n,
                               -positions[:, 0] * result.thrust_n,
                               reaction_torque))
print('Commands [0..1]:', result.command)
print('Motor thrust [N]:', result.thrust_n)
print('Motor signed yaw-reaction torque [Nm]:', reaction_torque)
print('Motor body torque [Tx, Ty, Tz] [Nm] (one row per motor):')
print(motor_torque)
print('Net body torque [Tx, Ty, Tz] [Nm]:', motor_torque.sum(axis=0))
print('Estimated total motor current [A]:', result.current_a.sum())
print('Requested wrench [N, Nm, Nm, Nm]:', requested)
print('Achieved wrench [N, Nm, Nm, Nm]:', result.achieved_wrench)
print('Residual (achieved - requested):', result.residual)
print('Optimizer converged:', result.converged)
