#!/usr/bin/env python3
"""Build the SITL parameter file from a real MOLE flight log.

Keeps the parameters that decide how the drone flies (tuning, filters, motor
mixing, flight modes, failsafes, estimator settings) and drops ones tied to the
real hardware (sensor IDs and calibration, board orientation, serial ports, OSD,
battery sensing pins, DShot). Sim-only settings come from sitl_overrides.parm,
which is applied after this file.

Usage (needs pymavlink, e.g. from ~/mole_sim/venv-ardupilot):
    python3 extract_params.py path/to/log.bin ../config/mole_quad.parm
"""
import argparse
import datetime
import re
import sys

from pymavlink import mavutil

KEEP = [
    r'ATC_.*', r'PSC_.*', r'ACRO_.*', r'PILOT_.*', r'ANGLE_MAX', r'THR_.*',
    r'MOT_.*', r'FRAME_(CLASS|TYPE)', r'SERVO[1-4]_FUNCTION',
    r'INS_(GYRO|ACCEL)_FILTER', r'INS_HNTC[H2]_.*', r'SCHED_LOOP_RATE',
    r'EK3_(?!.*_ID$).*', r'AHRS_EKF_TYPE', r'AHRS_GPS_.*',
    r'FLTMODE.*', r'FS_.*', r'BATT_(ARM|LOW|CRT)_.*', r'BATT_FS_.*', r'BATT_CAPACITY',
    r'ARMING_(SKIPCHK|ACCTHRESH|RUDDER|OPTIONS|NEED_LOC)', r'DISARM_DELAY',
    r'LAND_.*', r'RTL_.*', r'LOIT_.*', r'WP_.*', r'WPNAV_.*', r'TKOFF_.*',
    r'FENCE_.*', r'AVOID_.*', r'AUTOTUNE_.*', r'RCMAP_.*', r'GPS1_TYPE', r'COMPASS_ENABLE',
]
DROP = [
    r'MOT_PWM_TYPE',          # DShot: the simulator talks PWM over JSON
    r'MOT_SAFE_DISARM',       # set in sitl_overrides.parm
    r'.*_DEV_ID.*', r'.*_ID$', r'.*OFS.*', r'.*_SCALE.*', r'.*_CALTEMP.*', r'TCAL_.*',
]


def wanted(name):
    return (any(re.fullmatch(p, name) for p in KEEP)
            and not any(re.fullmatch(p, name) for p in DROP))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log', help='ArduPilot dataflash log (.bin) from the real drone')
    parser.add_argument('output', help='parameter file to write')
    args = parser.parse_args()

    log = mavutil.mavlink_connection(args.log)
    params, version = {}, None
    while True:
        msg = log.recv_match(type=['PARM', 'MSG'])
        if msg is None:
            break
        if msg.get_type() == 'PARM':
            params[msg.Name] = msg.Value
        elif version is None and msg.Message.startswith('ArduCopter'):
            version = msg.Message
    if not params:
        sys.exit(f'No parameters found in {args.log}')

    kept = {k: v for k, v in sorted(params.items()) if wanted(k)}
    with open(args.output, 'w') as out:
        out.write(f'# MOLE quad: flight parameters from the real flight controller.\n'
                  f'# Source log: {args.log}\n'
                  f'# Firmware:   {version or "unknown"}\n'
                  f'# Generated:  {datetime.date.today()} by extract_params.py '
                  f'({len(kept)} of {len(params)} parameters kept)\n'
                  f'# Do not edit by hand: re-run extract_params.py after retuning the real drone.\n')
        for name, value in kept.items():
            out.write(f'{name:<20} {value:.7g}\n')
    print(f'Wrote {len(kept)} parameters ({version}) to {args.output}')


if __name__ == '__main__':
    main()
