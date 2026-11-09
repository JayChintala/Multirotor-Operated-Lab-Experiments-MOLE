from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('emsd_drone_description'))
    if not (share / 'meshes/frame.stl').exists():
        raise RuntimeError('CAD mesh is missing. Run EMSD/tools/convert_step.py, then rebuild the workspace. See EMSD/README.md.')
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (share / 'urdf/frame.urdf').read_text()}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'rviz/frame.rviz')]),
    ])
