from glob import glob

from setuptools import setup

setup(
    name='mole_mocap',
    version='0.1.0',
    packages=['mole_mocap'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mole_mocap']),
        ('share/mole_mocap', ['package.xml']),
        ('share/mole_mocap/launch', glob('launch/*.launch.py')),
        ('share/mole_mocap/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MOLE team',
    maintainer_email='noreply@github.com',
    description='Vicon motion capture for the MOLE quad (real drone and SITL).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'mocap_bridge = mole_mocap.mocap_bridge:main',
        'fake_vicon = mole_mocap.fake_vicon:main',
    ]},
)
