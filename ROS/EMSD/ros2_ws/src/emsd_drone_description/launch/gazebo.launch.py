"""Launch physical flight using the installed Python controller and plugin."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('emsd_drone_description'))

    def start(context):
        command = ['python3', str(share / 'control/physics_sim.py'),
                   '--config', LaunchConfiguration('config').perform(context),
                   '--mode', LaunchConfiguration('mode').perform(context),
                   '--duration', LaunchConfiguration('duration').perform(context)]
        if LaunchConfiguration('headless').perform(context).lower() == 'true':
            command.append('--headless')
        return [ExecuteProcess(cmd=command, output='screen')]

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'control/config/drone.json')),
        DeclareLaunchArgument('mode', default_value='trajectory'),
        DeclareLaunchArgument('duration', default_value='0'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=start),
    ])
