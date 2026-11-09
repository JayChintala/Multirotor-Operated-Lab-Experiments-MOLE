"""Shared CLI options for the physical Gazebo runner."""


def add_simulation_arguments(parser):
    parser.add_argument('--config', help='Vehicle JSON; default control/config/drone.json')
    parser.add_argument('--mode', choices=['trajectory', 'attitude', 'rates', 'motors', 'freefall'],
                        default='trajectory', help='Physics reference/actuation mode')
    parser.add_argument('--roll-deg', type=float, default=0.)
    parser.add_argument('--pitch-deg', type=float, default=0.)
    parser.add_argument('--yaw-deg', type=float, default=0.)
    parser.add_argument('--motor-commands', type=float, nargs=4, default=[0.25]*4,
                        help='Four normalized measured-range commands in motors mode')
    parser.add_argument('--duration', type=float, default=0., help='Simulation seconds; 0 runs until stopped')
    parser.add_argument('--headless', action='store_true',
                        help='Run Gazebo DART directly without GUI/transport (default duration: 30 s)')
    parser.add_argument('--prepare-only', action='store_true', help='Build plugin and generate world without launching')
