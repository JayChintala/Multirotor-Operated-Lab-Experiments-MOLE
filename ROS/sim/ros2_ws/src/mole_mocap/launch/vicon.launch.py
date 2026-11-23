"""Ground-station laptop in the Vicon lab: Vicon -> ROS 2 -> MAVROS -> Wi-Fi -> Pi -> FC.

    ros2 launch mole_mocap vicon.launch.py vicon_host:=<vicon-pc-ip> pi:=mole.local

    Vicon Tracker PC --DataStream--> vicon_receiver ---> /vicon/mole/mole
                                                          |
                                     mocap_bridge <-------+
                                          |  /mavros/vision_pose/pose, EKF origin
                                       MAVROS --tcp:<pi>:5760--> mavlink-router (Pi) --UART--> FC

Needs ros2-vicon-receiver built in the same workspace (see ROS/sim/README.md) and the FC
loaded with config/mocap.parm and config/lab_limits.parm (indoor speeds + fence). lab_fence
uploads the square fence; with lab_limits.parm the FC will not arm without it. The Vicon object name in Tracker must match `subject`.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    subject = arg('subject')
    mavros_share = get_package_share_directory('mavros')
    receiver = get_package_share_directory('vicon_receiver') + '/launch/client.launch.py'
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(receiver),
            launch_arguments={'hostname': arg('vicon_host'), 'topic_namespace': 'vicon',
                              'buffer_size': '200', 'world_frame': 'map', 'vicon_frame': 'vicon',
                              'map_xyz': '[0.0, 0.0, 0.0]',
                              'map_rpy': '[0.0, 0.0, 0.0]', 'map_rpy_in_degrees': 'false'}.items()),

        Node(package='mole_mocap', executable='mocap_bridge', name='mocap_bridge', output='screen',
             parameters=[{'pose_topic': f'/vicon/{subject}/{subject}',
                          'rate_hz': float(arg('rate_hz'))}]),

        Node(package='mole_mocap', executable='lab_fence', name='lab_fence', output='screen',
             parameters=[{'half_x_m': float(arg('fence_x')) / 2, 'half_y_m': float(arg('fence_y')) / 2}]),

        # Pi's mavlink-router TCP server; it accepts several clients (QGC, kill switch, this).
        Node(package='mavros', executable='mavros_node', name='mavros', output='screen',
             parameters=[f'{mavros_share}/launch/apm_pluginlists.yaml',
                         f'{mavros_share}/launch/apm_config.yaml',
                         {'fcu_url': f'tcp://{arg("pi")}:5760', 'gcs_url': '',
                          'tgt_system': 1, 'tgt_component': 1, 'fcu_protocol': 'v2.0'}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vicon_host', default_value='192.168.10.1',
                              description='Vicon Tracker PC (DataStream server) IP or hostname'),
        DeclareLaunchArgument('subject', default_value='mole',
                              description='object name in Vicon Tracker (topic /vicon/<subject>/<subject>)'),
        DeclareLaunchArgument('pi', default_value='mole.local',
                              description="Pi's address (10.42.0.1 on the MOLE hotspot)"),
        DeclareLaunchArgument('fence_x', default_value='2.5', description='fence size along Vicon x (m)'),
        DeclareLaunchArgument('fence_y', default_value='2.5', description='fence size along Vicon y (m)'),
        DeclareLaunchArgument('rate_hz', default_value='50.0', description='max pose rate sent to the FC'),
        OpaqueFunction(function=nodes),
    ])
