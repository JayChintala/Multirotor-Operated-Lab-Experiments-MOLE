"""Build and launch the force-driven Gazebo (Harmonic or Fortress) simulation."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import gazebo_backend
from physics_control import DEFAULT_CONFIG, GRAVITY, load_config

ROOT = Path(__file__).resolve().parents[1]
INSTALLED = (ROOT / 'worlds/frame.sdf').exists()
PACKAGE = ROOT if INSTALLED else ROOT / 'ros2_ws/src/emsd_drone_description'


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def build_world(config, plugin_path, runtime_config):
    """Reuse scene visuals; collapse the drone into one dynamic rigid body."""
    sdf = ET.parse(PACKAGE / 'worlds/frame.sdf').getroot()
    world = sdf.find('world')
    element(world, 'gravity', f'0 0 {-GRAVITY}')
    world.find('physics/max_step_size').text = str(config['physics_step_s'])
    # DART's direct SDF constructor cannot build mesh collisions. Approximate
    # the terrain opening and basin with primitives in both launch paths.
    ground_link = world.find("model[@name='ground']/link")
    ground_link.remove(ground_link.find('collision'))

    def box_collision(link, name, x, y, z, sx, sy, sz, yaw=0):
        collision = element(link, 'collision', name=name)
        element(collision, 'pose', f'{x} {y} {z} 0 0 {yaw}')
        element(element(element(collision, 'geometry'), 'box'),
                'size', f'{sx} {sy} {sz}')

    for name, x, y, sx, sy in (
        ('west', -2.5, 0, 5, 10), ('east', 4.5, 0, 1, 10),
        ('south', 2, -2, 4, 6)):
        box_collision(ground_link, name, x, y, -.05, sx, sy, .1)
    # Square cells cover the corners of the circular opening without blocking
    # access to its water surface.
    cell = .25
    for ix in range(16):
        for iy in range(16):
            x, y = (ix + .5)*cell, 1 + (iy + .5)*cell
            if (x-2)**2 + (y-3)**2 >= (2 + cell / math.sqrt(2))**2:
                box_collision(ground_link, f'shore_{ix}_{iy}', x, y, -.05,
                              cell, cell, .1)
    basin = world.find("model[@name='pond']/link")
    basin.remove(basin.find('collision'))
    floor = element(basin, 'collision', name='floor')
    element(floor, 'pose', '0 0 -2.05 0 0 0')
    element(element(floor, 'geometry'), 'cylinder')
    element(floor.find('geometry/cylinder'), 'radius', 2)
    element(floor.find('geometry/cylinder'), 'length', .1)
    for i in range(48):
        angle = 2*math.pi*i/48
        box_collision(basin, f'wall_{i}', 2.02*math.cos(angle),
                      2.02*math.sin(angle), -1, 2*math.pi*2.02/48,
                      .06, 2, angle + math.pi/2)
    model = world.find("model[@name='emsd_drone_frame']")
    model.find('static').text = 'false'
    model.find('pose').text = '0 0 0.15 0 0 0'
    base = model.find("link[@name='base_link']")
    com = config['com_m']
    frame_visual = base.find('visual')
    element(frame_visual, 'pose', ' '.join(str(-v) for v in com) + ' 0 0 0')
    for i in range(4):
        motor = model.find(f"link[@name='motor_{i+1}']")
        visual = copy.deepcopy(motor.find('visual'))
        xyz = [config['motor_positions_m'][i][j]-com[j] for j in range(3)]
        xyz[2] += 0.01
        element(visual, 'pose', ' '.join(map(str, xyz)) + ' 0 0 0')
        base.append(visual)
        model.remove(motor)
    inertial = element(base, 'inertial')
    element(inertial, 'mass', config['mass_kg'])
    inertia = element(inertial, 'inertia')
    for name, value in zip(('ixx', 'iyy', 'izz'), config['inertia_kg_m2']):
        element(inertia, name, value)
    for name, value in zip(('ixy', 'ixz', 'iyz'), config.get('inertia_products_kg_m2', [0,0,0])):
        element(inertia, name, value)
    # Provisional frame-sized box avoids expensive concave dynamic mesh collision.
    collision = element(base, 'collision', name='body_collision')
    geometry = element(collision, 'geometry')
    element(element(geometry, 'box'), 'size', '0.406 0.320 0.060')
    plugin = element(model, 'plugin', filename=str(plugin_path), name='emsd::MotorPhysics')
    element(plugin, 'python_path', Path(__file__).resolve().parent)
    element(plugin, 'controller_module', config.get('controller_module', 'physics_control'))
    element(plugin, 'config', runtime_config)
    for uri in sdf.iter('uri'):
        for token, filename in [('FRAME', 'frame'), ('GROUND', 'ground_pond'), ('POND', 'pond_bowl')]:
            if uri.text == f'__{token}_MESH_URI__':
                uri.text = (PACKAGE / f'meshes/{filename}.stl').as_uri()
    text = ET.tostring(sdf, encoding='unicode')
    try:
        return gazebo_backend.detect().convert_world(text)
    except RuntimeError:
        return text  # No Gazebo CLI; the headless DART runner ignores world systems.


def main(args):
    config = load_config(args.config or DEFAULT_CONFIG)
    config['controller_module'] = getattr(args, 'controller_module', 'physics_control')
    config['mode'] = args.mode
    config['rates_rad_s'] = args.rates
    config['attitude_deg'] = [args.roll_deg, args.pitch_deg, args.yaw_deg]
    config['motor_commands'] = args.motor_commands
    if not math.isfinite(args.duration) or args.duration < 0:
        raise ValueError('Duration must be finite and nonnegative')
    if INSTALLED:
        lib = ROOT.parent.parent / 'lib'
        plugin = lib / 'libemsd_motor_physics.so'
        headless = lib / 'emsd_drone_description/emsd_headless'
    else:
        build = ROOT / 'simulation/build'
        configured = subprocess.run(['cmake', '-S', str(ROOT / 'simulation'), '-B', str(build),
                                     '-DCMAKE_BUILD_TYPE=Release'], capture_output=True, text=True)
        if configured.returncode:
            raise RuntimeError(configured.stdout + configured.stderr)
        subprocess.run(['cmake', '--build', str(build), '-j2'], check=True)
        plugin = build / 'libemsd_motor_physics.so'
        headless = build / 'emsd_headless'
    run_root = Path.cwd() if INSTALLED else ROOT
    run = run_root / 'simulation/runs' / f'{time.strftime("%Y%m%d_%H%M%S")}_{os.getpid()}'
    run.mkdir(parents=True)
    config['telemetry_csv'] = str(run / 'telemetry.csv')
    runtime_config = run / 'config.json'
    runtime_config.write_text(json.dumps(config, indent=2) + '\n')
    world_file = run / 'physics.sdf'
    world_file.write_text(build_world(config, plugin, runtime_config))
    print(f'Physics mode: {args.mode}; configured mass {config["mass_kg"]:.6g} kg; inertia diagonal {config["inertia_kg_m2"]} kg m²', flush=True)
    print(f'Generated world, effective config and telemetry: {run}', flush=True)
    if config['controller_module'] == 'final_control':
        print('Custom four-motor commands → measured motor lag/thrust/torque + buoyancy → Gazebo physics.', flush=True)
    else:
        print('Ground-truth feedback → position/attitude/rate control → allocation → motor lag → forces/torques → Gazebo physics.', flush=True)
    if args.prepare_only:
        return
    env = os.environ.copy()
    if args.headless:
        command = [str(headless), str(world_file), str(args.duration or 30)]
    else:
        backend = gazebo_backend.detect()
        env = backend.environment(f'emsd_physics_{os.getpid()}', run / 'gazebo_logs')
        command = [*backend.sim, '-r', '-v', '3', str(world_file)]
        gui = backend.gui_config()
        if gui:
            text = gui.read_text().replace('<camera_pose>-6 0 6 0 0.5 0</camera_pose>',
                                          '<camera_pose>-7 -9 8 0 0.55 0.9</camera_pose>')
            gui_file = run / 'gui.config'
            gui_file.write_text(text)
            command += ['--gui-config', str(gui_file)]
    if args.duration and not args.headless:
        command += ['--iterations', str(math.ceil(args.duration/config['physics_step_s']))]
    process = subprocess.Popen(command, env=env, start_new_session=True)
    try:
        code = process.wait()
        if code:
            raise RuntimeError(f'Gazebo exited with code {code}; inspect terminal errors')
        telemetry = run / 'telemetry.csv'
        if not telemetry.exists() or telemetry.stat().st_size < 500:
            raise RuntimeError('Gazebo did not produce physics telemetry; plugin did not run')
    except KeyboardInterrupt:
        pass
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


if __name__ == '__main__':
    import argparse
    from simulation_args import add_simulation_arguments
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rates', nargs=3, type=float, default=[0, 0, 0])
    add_simulation_arguments(parser)
    main(parser.parse_args())
