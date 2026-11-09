"""Differences between Gazebo Harmonic (ROS 2 Jazzy) and Fortress (ROS 2 Humble).

Harmonic is preferred when its `gz sim` command works. Set EMSD_GZ=fortress
or EMSD_GZ=harmonic to override the automatic choice.
"""
import os
from pathlib import Path
import shutil
import subprocess


class Backend:
    def __init__(self, name):
        self.name = name
        self.harmonic = name == 'harmonic'
        self.sim = ['gz', 'sim'] if self.harmonic else ['ign', 'gazebo']
        self.service = ['gz', 'service'] if self.harmonic else ['ign', 'service']
        self.msgs = 'gz.msgs' if self.harmonic else 'ignition.msgs'

    def environment(self, partition, log_path):
        """Return a copy of os.environ isolating this run's Gazebo transport."""
        env = os.environ.copy()
        for prefix in ('GZ', 'IGN'):
            env[f'{prefix}_PARTITION'] = partition
            env[f'{prefix}_LOG_PATH'] = str(log_path)
        return env

    def gui_config(self):
        """Return the stock GUI config path, or None when it cannot be found."""
        if not self.harmonic:
            path = Path('/usr/share/ignition/ignition-gazebo6/gui/gui.config')
            return path if path.is_file() else None
        # ROS 2 Jazzy ships Gazebo inside <ros prefix>/opt/gz_sim_vendor.
        ament = [p for p in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if p]
        prefixes = [Path(p) / 'opt/gz_sim_vendor' for p in ament] + [Path('/usr')]
        for prefix in prefixes:
            path = prefix / 'share/gz/gz-sim8/gui/gui.config'
            if path.is_file():
                return path
        return None

    def convert_world(self, text):
        """Rename built-in system plugins in SDF text for this Gazebo version."""
        pairs = [('gz-sim-', 'ignition-gazebo-'), ('gz::sim::systems::', 'ignition::gazebo::systems::')]
        for modern, legacy in pairs:
            text = text.replace(legacy, modern) if self.harmonic else text.replace(modern, legacy)
        return text


def _works(command):
    if not shutil.which(command[0]):
        return False
    try:
        return subprocess.run(command, capture_output=True, timeout=30).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def detect():
    choice = os.environ.get('EMSD_GZ', 'auto').lower()
    if choice in ('harmonic', 'fortress'):
        return Backend(choice)
    if choice != 'auto':
        raise ValueError('EMSD_GZ must be auto, harmonic or fortress')
    if _works(['gz', 'sim', '--versions']):
        return Backend('harmonic')
    if shutil.which('ign'):
        return Backend('fortress')
    raise RuntimeError('Neither Gazebo Harmonic (gz sim) nor Fortress (ign gazebo) was found; '
                       'source your ROS setup.bash first')
