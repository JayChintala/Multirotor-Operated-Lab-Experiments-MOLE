"""Integration checks against the actual Gazebo DART backend and native adapter.

Build first: python3 control/example.py --gazebo --prepare-only
Run: python3 tools/validate_physics.py
"""
import copy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'control'))
from physics_control import DEFAULT_CONFIG, GRAVITY, load_config
from physics_sim import build_world

BUILD = ROOT/'simulation/build'
OUT = ROOT/'analysis/physics_validation'


def simulate(name, config, duration):
    run = OUT/name
    run.mkdir(parents=True, exist_ok=True)
    config = copy.deepcopy(config)
    config['telemetry_csv'] = str(run/'telemetry.csv')
    config_file = run/'config.json'
    config_file.write_text(json.dumps(config, indent=2)+'\n')
    world = run/'physics.sdf'
    world.write_text(build_world(config, BUILD/'libemsd_motor_physics.so', config_file))
    result = subprocess.run([str(BUILD/'emsd_headless'), str(world), str(duration)],
                            capture_output=True, text=True, timeout=120)
    (run/'engine.log').write_text(result.stdout+result.stderr)
    if result.returncode:
        raise RuntimeError(result.stdout+result.stderr)
    data = np.genfromtxt(run/'telemetry.csv', delimiter=',', names=True)
    assert np.all(np.isfinite(np.column_stack([data[k] for k in data.dtype.names])))
    return data


def main():
    config = load_config(DEFAULT_CONFIG)
    checks = {}
    freefall = copy.deepcopy(config)
    freefall.update(mode='freefall', linear_drag_n_per_m_s=0.)
    data = simulate('freefall', freefall, .1)
    expected = .15 - .5*GRAVITY*data['time_s']**2
    error = float(np.max(np.abs(data['z_m']-expected)))
    assert error < .0011, error
    assert np.all(data['fz_n'] == 0)
    checks['freefall_max_position_error_m'] = error
    cases = {'roll': ([.3,.3,.15,.15], 'roll_rad', 'p_rad_s'),
             'pitch': ([.15,.3,.3,.15], 'pitch_rad', 'q_rad_s'),
             'yaw': ([.3,.15,.3,.15], 'yaw_rad', 'r_rad_s')}
    for axis, (commands, angle, rate) in cases.items():
        for sign in (1, -1):
            c = copy.deepcopy(config)
            c['mode'] = 'motors'
            c['motor_commands'] = commands if sign == 1 else [.45-x for x in commands]
            data = simulate(f'{axis}_{sign:+}', c, .12)
            assert sign*data[angle][-1] > .001, (axis, sign, data[angle][-1])
            assert sign*data[rate][-1] > .01, (axis, sign, data[rate][-1])
            checks[f'{axis}_{sign:+}_final_angle_deg'] = float(np.degrees(data[angle][-1]))
            checks[f'{axis}_{sign:+}_final_rate_rad_s'] = float(data[rate][-1])
    c = copy.deepcopy(config)
    c['mode'] = 'trajectory'
    data = simulate('trajectory', c, 38)
    p = np.column_stack([data[k] for k in ('x_m', 'y_m', 'z_m')])
    ref = np.column_stack([data[k] for k in ('ref_x_m', 'ref_y_m', 'ref_z_m')])
    path = data['time_s'] >= 8
    errors = np.linalg.norm(p-ref, axis=1)
    rms = float(np.sqrt(np.mean(errors[path]**2)))
    assert rms < .15, rms
    assert np.max(np.abs(data['z_m'][path]-1.5)) < .08
    assert np.max(np.abs(data['roll_rad'][path])) > .01
    assert np.max(np.abs(data['pitch_rad'][path])) > .001
    assert abs(data['z_m'][-1]-1.5) < .03
    checks['trajectory_path_rms_error_m'] = rms
    checks['trajectory_path_max_error_m'] = float(errors[path].max())
    checks['trajectory_final_altitude_m'] = float(data['z_m'][-1])
    checks['trajectory_peak_roll_deg'] = float(np.degrees(np.max(np.abs(data['roll_rad']))))
    checks['trajectory_peak_pitch_deg'] = float(np.degrees(np.max(np.abs(data['pitch_rad']))))
    report = {'backend': 'Gazebo DART (gz-physics7 or ignition-physics5)', 'mass_kg': config['mass_kg'],
              'inertia_kg_m2': config['inertia_kg_m2'], 'physics_step_s': config['physics_step_s'],
              'all_checks_passed': True, 'checks': checks,
              'limits': 'Ground-truth feedback, assumed actuator lag/drag, provisional motor mount positions; GUI/server transport not validated by this test.'}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT/'results.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
