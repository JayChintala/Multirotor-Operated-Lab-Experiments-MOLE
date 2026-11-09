"""Scripted visual flight demo; pose commands, not a flight dynamics model."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import gazebo_backend

ROOT = Path(__file__).resolve().parents[1]


def trajectory(t):
    """Return x/y/z/roll/pitch/yaw in metres/radians for scripted visual motion."""
    if t < 5:
        s = max(0, t / 5)
        return 0, 0, 0.15 + 1.35 * s*s*(3-2*s), 0, 0, 0
    phase = (t - 5) * 2 * math.pi / 30
    x = 2 * math.sin(phase)
    y = 1.5 * math.sin(2 * phase)
    yaw = math.atan2(3 * math.cos(2 * phase), 2 * math.cos(phase))
    # Illustrative tilt, independent of PID, thrust and flight dynamics.
    # Blend in over two seconds so takeoff ends with zero roll/pitch.
    blend = min(1, (t - 5) / 2)
    blend = blend * blend * (3 - 2 * blend)
    roll = blend * math.radians(12) * math.sin(phase)
    pitch = blend * math.radians(8) * math.sin(2 * phase)
    return x, y, 1.5, roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """Return x/y/z/w for Rz(yaw) Ry(pitch) Rx(roll)."""
    sr, cr = math.sin(roll / 2), math.cos(roll / 2)
    sp, cp = math.sin(pitch / 2), math.cos(pitch / 2)
    sy, cy = math.sin(yaw / 2), math.cos(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def main():
    backend = gazebo_backend.detect()
    env = backend.environment(f'emsd_demo_{os.getpid()}', '/tmp/emsd_demo_logs')
    with tempfile.TemporaryDirectory(prefix='emsd_demo_') as directory:
        directory = Path(directory)
        package = ROOT / 'ros2_ws/src/emsd_drone_description'
        world = (package / 'worlds/frame.sdf').read_text()
        for token, filename in [('FRAME', 'frame'), ('GROUND', 'ground_pond'), ('POND', 'pond_bowl')]:
            world = world.replace(f'__{token}_MESH_URI__',
                                  (package / f'meshes/{filename}.stl').as_uri())
        world_file = directory / 'demo.sdf'
        world_file.write_text(backend.convert_world(world))
        command = [*backend.sim, '-r', str(world_file)]
        gui = backend.gui_config()
        if gui:
            config = gui.read_text().replace('<camera_pose>-6 0 6 0 0.5 0</camera_pose>',
                                             '<camera_pose>-7 -9 8 0 0.55 0.9</camera_pose>')
            config_file = directory / 'gui.config'
            config_file.write_text(config)
            command += ['--gui-config', str(config_file)]
        gazebo = subprocess.Popen(command, env=env, start_new_session=True)

        def set_pose(t):
            x, y, z, roll, pitch, yaw = trajectory(t)
            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
            request = (f'name: "emsd_drone_frame" position: {{x: {x} y: {y} z: {z}}} '
                       f'orientation: {{x: {qx} y: {qy} z: {qz} w: {qw}}}')
            result = subprocess.run([*backend.service, '-s', '/world/emsd/set_pose',
                                     '--reqtype', f'{backend.msgs}.Pose',
                                     '--reptype', f'{backend.msgs}.Boolean',
                                     '--timeout', '1000', '--req', request],
                                    env=env, capture_output=True, text=True, timeout=4)
            return result.returncode == 0 and 'data: true' in result.stdout

        try:
            deadline = time.monotonic() + 45
            while not set_pose(0):
                if gazebo.poll() is not None:
                    raise RuntimeError('Gazebo exited before the pose service became available')
                if time.monotonic() > deadline:
                    raise RuntimeError('Gazebo pose service did not become available within 45 seconds')
                time.sleep(0.5)
            print('Gazebo ready: scripted takeoff to 1.5 m, then figure eight with roll/pitch/yaw.', flush=True)
            print('This demo sets poses directly; PID and flight physics are not active. Ctrl+C stops.', flush=True)
            # Give the GUI time to finish loading before takeoff.
            time.sleep(5)
            started = time.monotonic()
            last_report = -1
            failures = 0
            while gazebo.poll() is None:
                tick = time.monotonic()
                elapsed = tick - started
                failures = 0 if set_pose(elapsed) else failures + 1
                if failures >= 5:
                    raise RuntimeError('Gazebo stopped accepting pose commands')
                report = int(elapsed // 10)
                if report != last_report:
                    print(f't={elapsed:.1f}s: drone position {trajectory(elapsed)[:3]}', flush=True)
                    last_report = report
                time.sleep(max(0, 0.05 - (time.monotonic() - tick)))
        except KeyboardInterrupt:
            pass
        finally:
            if gazebo.poll() is None:
                os.killpg(gazebo.pid, signal.SIGINT)
                try:
                    gazebo.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(gazebo.pid, signal.SIGTERM)
                    gazebo.wait(timeout=5)


if __name__ == '__main__':
    main()
