"""MOLE quad: Gazebo Harmonic + ArduCopter SITL (+ optional MAVROS).

    ros2 launch mole_gazebo sitl.launch.py                 # GUI, simulated GPS
    ros2 launch mole_gazebo sitl.launch.py gps:=false      # like the real drone today
    ros2 launch mole_gazebo sitl.launch.py headless:=true mavros:=false
    ros2 launch mole_gazebo sitl.launch.py mocap:=true     # no GPS/compass, simulated Vicon lab

MAVLink endpoints (same port layout as the Pi's mavlink-router):
    tcp:127.0.0.1:5760   MAVROS when mavros:=true, otherwise free for a GCS/script
    tcp:127.0.0.1:5762   second link, e.g. QGroundControl or MAVProxy
"""
import os
import re
import tempfile
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent,
                            ExecuteProcess, OpaqueFunction, RegisterEventHandler,
                            UnsetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = '40.4433,-79.9436,280,0'   # must match <spherical_coordinates> in the world

# VS Code (snap) leaks these into its terminal; they make Qt/Gazebo GUIs crash.
SNAP_VARS = ['GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GIO_MODULE_DIR', 'LOCPATH',
             'GDK_PIXBUF_MODULE_FILE', 'GDK_PIXBUF_MODULEDIR', 'GSETTINGS_SCHEMA_DIR']


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def lab_sdf(vx, vy, vz, fx, fy):
    """Visual-only model of the Vicon lab: tracked volume (translucent box) and fence (red square)."""
    edge = lambda n, x, y, sx, sy: (  # noqa: E731
        f'<visual name="{n}"><pose>{x} {y} 0.006 0 0 0</pose><geometry><box><size>{sx} {sy} 0.01</size></box>'
        '</geometry><material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material></visual>')
    return (
        '<sdf version="1.9"><model name="vicon_lab"><static>true</static><link name="link">'
        f'<visual name="tracked_volume"><pose>0 0 {vz / 2} 0 0 0</pose>'
        f'<geometry><box><size>{vx} {vy} {vz}</size></box></geometry><cast_shadows>false</cast_shadows>'
        '<transparency>0.85</transparency><material><ambient>0.3 0.6 1 0.15</ambient>'
        '<diffuse>0.3 0.6 1 0.15</diffuse></material></visual>'
        + edge('fence_n', 0, fy, 2 * fx, 0.03) + edge('fence_s', 0, -fy, 2 * fx, 0.03)
        + edge('fence_e', fx, 0, 0.03, 2 * fy) + edge('fence_w', -fx, 0, 0.03, 2 * fy)
        + '</link></model></sdf>')


def mocap_nodes(context):
    """Simulated Vicon: Gazebo truth -> fake_vicon -> /vicon/mole/mole -> mocap_bridge -> MAVROS,
    plus the lab's fence (lab_fence) and a picture of the tracked volume in Gazebo."""
    if not is_true(context, 'mocap'):
        return []
    if not is_true(context, 'mavros'):
        raise RuntimeError('mocap:=true sends the pose through MAVROS; it needs mavros:=true')
    sim_time = {'use_sim_time': True}
    vx, vy, vz = (float(v) for v in LaunchConfiguration('mocap_volume').perform(context).split(','))
    fx, fy = (float(v) / 2 for v in LaunchConfiguration('mocap_fence').perform(context).split(','))
    return [
        Node(package='ros_gz_sim', executable='create', name='spawn_vicon_lab', output='screen',
             arguments=['-world', 'mole_field', '-name', 'vicon_lab', '-string', lab_sdf(vx, vy, vz, fx, fy)]),
        Node(package='mole_mocap', executable='lab_fence', name='lab_fence', output='screen',
             parameters=[sim_time, {'half_x_m': fx, 'half_y_m': fy}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='truth_bridge', output='screen',
             arguments=['/model/mole_quad/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry']),
        Node(package='mole_mocap', executable='fake_vicon', name='fake_vicon', output='screen',
             parameters=[sim_time, {'latency_ms': float(LaunchConfiguration('mocap_latency_ms').perform(context)),
                                    'dropout_probability': float(LaunchConfiguration('mocap_dropout').perform(context)),
                                    'volume_x_m': vx, 'volume_y_m': vy, 'volume_z_m': vz}]),
        Node(package='mole_mocap', executable='mocap_bridge', name='mocap_bridge', output='screen',
             parameters=[sim_time]),
    ]


def sitl(context):
    share = Path(get_package_share_directory('mole_gazebo'))
    ardupilot = Path(LaunchConfiguration('ardupilot_dir').perform(context)).expanduser()
    binary = ardupilot / 'build/sitl/bin/arducopter'
    if not binary.exists():
        raise RuntimeError(f'ArduCopter SITL not found at {binary}; build it with ./waf copter')
    mocap = is_true(context, 'mocap')
    gps = is_true(context, 'gps') and not mocap   # the lab setup has no GPS: mocap replaces it
    defaults = [ardupilot / 'Tools/autotest/default_params/copter.parm',
                share / 'config/mole_quad.parm',
                share / 'config/planned_changes.parm',
                share / 'config/sitl_overrides.parm',
                share / f'config/sitl_{"gps" if gps else "nogps"}.parm']
    if mocap:
        # Same files the real FC gets in the lab, so the sim flies the lab's parameters:
        # Vicon as the EKF source, then the indoor speed limits and fence.
        mocap_cfg = Path(get_package_share_directory('mole_mocap')) / 'config'
        defaults += [mocap_cfg / 'mocap.parm', mocap_cfg / 'lab_limits.parm']
    run = Path(LaunchConfiguration('run_dir').perform(context)).expanduser() / time.strftime('%Y%m%d_%H%M%S')
    run.mkdir(parents=True, exist_ok=True)
    print(f'[mole_gazebo] SITL run folder (logs, eeprom): {run}')
    cmd = [str(binary), '--model', 'JSON', '--speedup', '1', '--slave', '0',
           '--sim-address=127.0.0.1', '-I0', '--home', HOME,
           '--serial0', 'tcp:0',   # don't block boot waiting for a client on 5760
           '--defaults', ','.join(map(str, defaults))]
    if LaunchConfiguration('wipe').perform(context).lower() == 'true':
        cmd.append('-w')   # start from the parameter files, not a stale eeprom
    return [ExecuteProcess(cmd=cmd, cwd=str(run), output='screen', name='arducopter')]


def world_file(context):
    """The world, with the drone moved when spawn_x/spawn_y/spawn_yaw_deg differ from the file."""
    world = LaunchConfiguration('world').perform(context)
    x, y, yaw = (LaunchConfiguration(n).perform(context) for n in ('spawn_x', 'spawn_y', 'spawn_yaw_deg'))
    if (float(x), float(y), float(yaw)) == (0.0, 0.0, 90.0):
        return world
    sdf = Path(world).read_text()
    sdf, n = re.subn(r'(<name>mole_quad</name>.*?<pose degrees="true">)[^<]*(</pose>)',
                     rf'\g<1>{x} {y} 0.15 0 0 {yaw}\g<2>', sdf, count=1, flags=re.S)
    if n != 1:
        raise RuntimeError(f'spawn_*: no <pose degrees="true"> after mole_quad in {world}')
    out = Path(tempfile.gettempdir()) / f'mole_world_{os.getpid()}.sdf'
    out.write_text(sdf)
    print(f'[mole_gazebo] drone spawned at x={x} y={y} yaw={yaw} deg ({out})')
    return str(out)


def gazebo_procs(context):
    world = world_file(context)
    if is_true(context, 'headless'):
        cmd = ['gz', 'sim', '-s', '-r', '-v', '2', world]
    else:
        cmd = ['gz', 'sim', '-r', '-v', '2', world]
    gazebo = ExecuteProcess(cmd=cmd, output='screen', name='gazebo')
    # Closing the Gazebo window ends the whole simulation.
    return [gazebo, RegisterEventHandler(OnProcessExit(target_action=gazebo, on_exit=[EmitEvent(event=Shutdown())]))]


def generate_launch_description():
    share = get_package_share_directory('mole_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds/mole_field.sdf')),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='drone start, m east of the world origin'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='drone start, m north of the world origin'),
        DeclareLaunchArgument('spawn_yaw_deg', default_value='90.0',
                              description='drone heading, deg from east (90 = nose north, the default)'),
        DeclareLaunchArgument('headless', default_value='false', description='Gazebo server only, no window'),
        DeclareLaunchArgument('gps', default_value='true',
                              description='true: simulated GPS + compass; false: like the real drone (none)'),
        DeclareLaunchArgument('mavros', default_value='true', description='start MAVROS on tcp:5760'),
        DeclareLaunchArgument('mocap', default_value='false',
                              description='true: no GPS/compass, simulated Vicon via MAVROS (implies gps:=false)'),
        DeclareLaunchArgument('mocap_latency_ms', default_value='20.0', description='simulated Vicon latency'),
        DeclareLaunchArgument('mocap_dropout', default_value='0.0',
                              description='probability that a simulated Vicon frame is lost'),
        DeclareLaunchArgument('mocap_volume', default_value='3.0,3.0,4.0',
                              description='Vicon tracked volume x,y,z in m (floor centred on the origin)'),
        DeclareLaunchArgument('mocap_fence', default_value='2.5,2.5',
                              description='square fence x,y in m uploaded by lab_fence (inside the volume)'),
        DeclareLaunchArgument('wipe', default_value='true', description='reset SITL eeprom to the parameter files'),
        DeclareLaunchArgument('ardupilot_dir', default_value='~/mole_sim/ardupilot-4.7.2',
                              description='ArduPilot checkout with build/sitl/bin/arducopter'),
        DeclareLaunchArgument('ardupilot_gazebo_dir', default_value='~/mole_sim/ardupilot_gazebo',
                              description='ardupilot_gazebo checkout with build/libArduPilotPlugin.so'),
        DeclareLaunchArgument('run_dir', default_value='~/mole_sim/runs'),

        *[UnsetEnvironmentVariable(v) for v in SNAP_VARS],
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'worlds')),
        OpaqueFunction(function=lambda c: [AppendEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            str(Path(LaunchConfiguration('ardupilot_gazebo_dir').perform(c)).expanduser() / 'build'))]),

        OpaqueFunction(function=gazebo_procs),
        OpaqueFunction(function=sitl),
        OpaqueFunction(function=mocap_nodes),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
        Node(package='mavros', executable='mavros_node', name='mavros',
             condition=IfCondition(LaunchConfiguration('mavros')), output='screen',
             parameters=[os.path.join(get_package_share_directory('mavros'), 'launch/apm_pluginlists.yaml'),
                         os.path.join(get_package_share_directory('mavros'), 'launch/apm_config.yaml'),
                         {'fcu_url': 'tcp://127.0.0.1:5760', 'gcs_url': '', 'tgt_system': 1,
                          'tgt_component': 1, 'fcu_protocol': 'v2.0', 'use_sim_time': True}]),

    ])
