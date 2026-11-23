#!/usr/bin/env python3
"""Check the MOLE SITL model end to end. Run while sitl.launch.py is up:

    PY=~/mole_sim/venv-ardupilot/bin/python
    $PY smoke_test.py                          # all checks with the real FC's tune
    $PY smoke_test.py --rate-gain-scale 0.5    # same, roll/pitch rate gains halved for this run
    $PY smoke_test.py --motors                 # motor order/spin only
    $PY smoke_test.py --mocap --rate-gain-scale 0.6   # with sitl.launch.py mocap:=true

1. Motor test (disarmed): ArduPilot's motor test A-D must spin the right
   propeller in the right direction (ArduPilot Quad X; companion/README.md).
2. AltHold flight via RC override (how the real drone is flown, no GPS needed):
   climb, roll/pitch steady, hold altitude at mid-stick, hover throttle vs the real 0.35.
3. Yaw right: heading must increase. 4. Land and auto-disarm.
--mocap replaces 2-4 with a Loiter flight on the simulated Vicon: EKF position and yaw vs
Gazebo ground truth (catches frame mistakes), position hold, stop after a stick input, then
(with lab_limits.parm's fence) full stick at the wall and the ceiling: the fence must stop it.
The whole flight must stay inside the lab's tracked volume (--volume, default 3 x 3 x 4 m).
--vicon-loss replaces the fence and Land steps: it stops the simulated Vicon in a hover and
checks that ArduPilot lands by itself (needs the ROS 2 CLI in this shell).
--param NAME=VALUE changes FC parameters for this run only, e.g. --param VISO_DELAY_MS=80.

EKF height: when the origin is set, ArduPilot resets its height datum at the drone's resting
height (the "Field Elevation Set" message), so the EKF reads about that much (~13 cm) above
Vicon from then on, on the real drone too. Height checks use the on-ground offset.
Exit code 0 when every check passes.

With the real FC's roll/pitch gains (log 00016) the steadiness check FAILS:
the roll/pitch rate loops oscillate (about 60 deg/s RMS).
See sim/README.md. --rate-gain-scale changes the gains only for this run.
"""
import argparse
import math
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

# Motor test order A-D is clockwise from front-right; expected rotor and spin (+ = CCW from above).
MOTOR_TEST = [('A', 'rotor_front_right', +1), ('B', 'rotor_rear_right', -1),
              ('C', 'rotor_rear_left', +1), ('D', 'rotor_front_left', -1)]
import json
from pathlib import Path
# Hover motor output predicted by gen_model.py for the current model (payloads included).
# Without payloads it is 0.354, matching the real FC's MOT_THST_HOVER of 0.35.
_PREDICTED = json.loads((Path(__file__).resolve().parents[1] / 'models/mole_quad/predicted.json').read_text())
EXPECTED_HOVER = _PREDICTED['hover_cmd']
WORLD, MODEL = 'mole_field', 'mole_quad'

results = []


def check(name, ok, detail=''):
    results.append(ok)
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f': {detail}' if detail else ''), flush=True)


class Joints:
    """Latest rotor joint velocities from Gazebo's JointStatePublisher."""

    def __init__(self):
        from gz.msgs10.model_pb2 import Model
        from gz.transport13 import Node
        self.vel, self.node = {}, Node()
        topic = f'/world/{WORLD}/model/{MODEL}/joint_state'
        if not self.node.subscribe(Model, topic, self._cb):
            raise RuntimeError(f'cannot subscribe to {topic}')

    def _cb(self, msg):
        for j in msg.joint:
            self.vel[j.name.replace('_joint', '')] = j.axis1.velocity


class Truth:
    """Ground-truth pose from Gazebo's OdometryPublisher (world ENU, body FLU)."""

    def __init__(self):
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.transport13 import Node
        self.pose, self.node = None, Node()
        self.lo, self.hi = [math.inf] * 3, [-math.inf] * 3   # extent of the flight (world ENU)
        topic = f'/model/{MODEL}/odometry'
        if not self.node.subscribe(Odometry, topic, self._cb):
            raise RuntimeError(f'cannot subscribe to {topic}')
        end = time.time() + 10               # discovery takes a moment; don't read before the first message
        while self.pose is None and time.time() < end:
            time.sleep(0.05)
        if self.pose is None:
            raise RuntimeError(f'no messages on {topic} (is the OdometryPublisher in model.sdf?)')

    def _cb(self, msg):
        self.pose = msg.pose
        p = msg.pose.position
        for i, c in enumerate((p.x, p.y, p.z)):
            self.lo[i], self.hi[i] = min(self.lo[i], c), max(self.hi[i], c)

    def enu(self):
        p = self.pose.position
        return p.x, p.y, p.z

    def yaw_enu(self):
        q = self.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Vehicle:
    def __init__(self, url):
        self.m = mavutil.mavlink_connection(url, source_system=255)
        # Lock onto the autopilot itself: MAVROS's heartbeat is forwarded on this link too.
        end = time.time() + 60
        while time.time() < end:
            hb = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if hb and hb.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                self.m.target_system, self.m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
                break
        else:
            raise RuntimeError(f'no autopilot heartbeat on {url}')
        self.rc = [0] * 8
        self.state = {}
        self.texts = []                   # STATUSTEXT messages from the FC
        # This link gets no telemetry until asked: request what the checks read, at 10 Hz.
        for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                       mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                       mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                       mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT):
            self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 100_000)
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        """Heartbeat (FS_GCS_ENABLE is on), RC override at 10 Hz, and message capture."""
        last_hb = 0
        while True:
            now = time.time()
            if now - last_hb > 0.5:
                self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb = now
            if any(self.rc):
                self.m.mav.rc_channels_override_send(self.m.target_system, self.m.target_component, *self.rc)
            while (msg := self.m.recv_match(blocking=False)) is not None:
                t = msg.get_type()
                if t == 'HEARTBEAT' and (msg.get_srcSystem(), msg.get_srcComponent()) != (
                        self.m.target_system, self.m.target_component):
                    continue              # e.g. MAVROS's own heartbeat, forwarded by the FC
                self.state[t] = msg
                if t == 'STATUSTEXT':
                    self.texts.append(msg.text)
                    print(f'      FC: {msg.text}', flush=True)
            time.sleep(0.1)

    def get(self, t, timeout=5):
        end = time.time() + timeout
        while time.time() < end:
            if t in self.state:
                return self.state[t]
            time.sleep(0.05)
        return None

    def command(self, cmd, *params):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0,
                                     *(list(params) + [0] * (7 - len(params))))

    def wait(self, predicate, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if predicate():
                return True
            time.sleep(0.1)
        return False

    def armed(self):
        hb = self.state.get('HEARTBEAT')
        return bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def motor_test(v, joints):
    print('1. Motor test (disarmed, 12 % for 2 s each)', flush=True)
    for i, (letter, rotor, spin) in enumerate(MOTOR_TEST, start=1):
        v.command(mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, i, 0, 12, 2, 1, 0)
        time.sleep(1.2)
        vel = dict(joints.vel)
        fastest = max(vel, key=lambda k: abs(vel[k])) if vel else None
        ok = fastest == rotor and vel[fastest] * spin > 50
        spun = f'{fastest} at {vel[fastest]:+.0f} rad/s' if fastest else 'nothing'
        check(f'motor {letter} -> {rotor} {"CCW" if spin > 0 else "CW"}', ok, spun)
        time.sleep(1.5)


RATE_GAINS = [f'ATC_RAT_{axis}_{k}' for axis in ('RLL', 'PIT') for k in ('P', 'I', 'D')]


def param(v, name, value=None):
    """Read a parameter, or set it when value is given. Returns the FC's value. Retries: a request
    sent while the FC is busy streaming is sometimes lost."""
    for _ in range(3):
        if value is None:
            v.m.mav.param_request_read_send(v.m.target_system, v.m.target_component, name.encode(), -1)
        else:
            v.m.mav.param_set_send(v.m.target_system, v.m.target_component, name.encode(), float(value),
                                   mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        end = time.time() + 3
        while time.time() < end:
            p = v.state.get('PARAM_VALUE')
            if p and p.param_id == name and (value is None or abs(p.param_value - value) < 1e-4):
                return p.param_value
            time.sleep(0.05)
    raise RuntimeError(f'no reply for parameter {name}')


def flight(v):
    print('2. AltHold flight via RC override', flush=True)
    v.rc = [1500, 1500, 1000, 1500, 0, 0, 0, 0]
    v.m.set_mode('ALT_HOLD')
    if not v.wait(lambda: v.state.get('HEARTBEAT') and mavutil.mode_string_v10(v.state['HEARTBEAT']) == 'ALT_HOLD', 15):
        check('switch to AltHold', False)
        return
    for _ in range(30):
        v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        if v.wait(v.armed, 2):
            break
    check('arm', v.armed())
    if not v.armed():
        return
    def height():
        g = v.get('GLOBAL_POSITION_INT')
        return g.relative_alt / 1000 if g else 0.0

    v.rc[2] = 1750                       # climb
    climbed = v.wait(lambda: height() > 3.0, 20)
    v.rc[2] = 1500                       # mid stick: AltHold holds altitude
    time.sleep(4)
    samples, alts, rates = [], [], []
    end = time.time() + 6
    while time.time() < end:
        s, a = v.get('SERVO_OUTPUT_RAW'), v.get('ATTITUDE')
        samples.append(sum((getattr(s, f'servo{k}_raw') - 1000) / 1000 for k in range(1, 5)) / 4)
        alts.append(height())
        rates.append(math.degrees(max(abs(a.rollspeed), abs(a.pitchspeed))))
        time.sleep(0.1)
    wobble = math.sqrt(sum(r * r for r in rates) / len(rates))
    check('roll/pitch steady in hover', wobble < 10, f'RMS of the larger of roll/pitch rate {wobble:.1f} deg/s')
    hover = sum(samples) / len(samples)
    check('climb above 3 m', climbed, f'height {height():.2f} m')
    held = max(alts) - min(alts) < 0.5
    check('hold altitude for 6 s', held, f'height {min(alts):.2f}-{max(alts):.2f} m')
    if held and wobble < 10:
        check('hover throttle matches the model', abs(hover - EXPECTED_HOVER) < 0.04,
              f'mean motor output {hover:.3f}, predicted {EXPECTED_HOVER:.3f} '
              f'({_PREDICTED["total_mass_kg"]} kg, payloads: {", ".join(_PREDICTED["payloads"]) or "none"})')
    else:
        check('hover throttle matches the model', False, 'not measured: the drone was not in a steady hover')

    print('3. Yaw right for 2 s', flush=True)
    h0 = v.get('VFR_HUD').heading
    v.rc[3] = 1700
    time.sleep(2)
    v.rc[3] = 1500
    time.sleep(1)
    d = (v.get('VFR_HUD').heading - h0 + 540) % 360 - 180
    check('yaw right increases heading', d > 20, f'heading change {d:+.0f} deg')

    print('4. Land', flush=True)
    v.rc[2] = 1300                       # descend
    v.wait(lambda: height() < 0.3, 40)
    time.sleep(2)
    n_texts = len(v.texts)
    v.rc[2] = 1000                       # stick fully down so ArduPilot auto-disarms
    disarmed = v.wait(lambda: not v.armed(), 30)
    time.sleep(1)
    crashed = [t for t in v.texts[n_texts:] if 'Crash' in t]
    check('land and auto-disarm', disarmed and not crashed,
          f'crash detector fired: {crashed[0]}' if crashed else f'height {height():.2f} m')


def set_mode_and_arm(v, mode):
    v.m.set_mode(mode)
    if not v.wait(lambda: v.state.get('HEARTBEAT') and mavutil.mode_string_v10(v.state['HEARTBEAT']) == mode, 15):
        check(f'switch to {mode}', False)
        return False
    for _ in range(30):
        v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        if v.wait(v.armed, 2):
            break
    check(f'arm in {mode}', v.armed())
    return v.armed()


def set_vicon_dropout(probability):
    """Change the simulated Vicon's frame-loss probability live (1.0 = Vicon lost)."""
    subprocess.run(['ros2', 'param', 'set', '/fake_vicon', 'dropout_probability', str(float(probability))],
                   check=True, capture_output=True, timeout=30)


def mocap_flight(v, truth, volume, vicon_loss=False):
    """Loiter on the simulated Vicon (sitl.launch.py mocap:=true): no GPS, no compass."""
    print('2. Loiter on simulated Vicon', flush=True)
    pos_flags = mavutil.mavlink.EKF_POS_HORIZ_ABS | mavutil.mavlink.EKF_POS_VERT_ABS
    ok = v.wait(lambda: (e := v.state.get('EKF_STATUS_REPORT')) and e.flags & pos_flags == pos_flags, 90)
    e = v.state.get('EKF_STATUS_REPORT')
    check('EKF has an absolute position from Vicon (origin set)', ok, f'flags 0x{e.flags:04x}' if e else 'no EKF report')
    if not ok:
        return

    def raw_errors():
        """EKF (NED) minus truth (ENU -> NED: N=y, E=x, D=-z): horizontal, down; yaw error in degrees."""
        lp, att = v.get('LOCAL_POSITION_NED'), v.get('ATTITUDE')
        x, y, z = truth.enu()
        yaw = math.degrees((att.yaw - (math.pi / 2 - truth.yaw_enu()) + math.pi) % (2 * math.pi) - math.pi)
        return math.hypot(lp.x - y, lp.y - x), lp.z + z, yaw

    horiz, dz0, yaw_err = raw_errors()

    def errors():
        """Position error with the on-ground height-datum offset removed, and yaw error."""
        h, dz, yaw = raw_errors()
        return math.hypot(h, dz - dz0), yaw

    check('EKF horizontal position matches Gazebo truth (on ground)', horiz < 0.1, f'{horiz * 100:.1f} cm')
    check('EKF height offset is the datum reset (< 0.3 m)', abs(dz0) < 0.3,
          f'EKF {-dz0 * 100:+.1f} cm vs Vicon')
    check('EKF yaw matches Gazebo truth (on ground)', abs(yaw_err) < 5, f'{yaw_err:+.1f} deg')

    v.rc = [1500, 1500, 1000, 1500, 0, 0, 0, 0]
    if not set_mode_and_arm(v, 'LOITER'):
        return
    z0 = truth.enu()[2]
    v.rc[2] = 1750
    climbed = v.wait(lambda: truth.enu()[2] - z0 > 2.0, 20)
    v.rc[2] = 1500
    check('climb above 2 m', climbed, f'height {truth.enu()[2] - z0:.2f} m')
    time.sleep(4)

    def hold(seconds):
        """Largest horizontal distance from the start point, and the worst EKF error, over `seconds`."""
        x0, y0, _ = truth.enu()
        drift = worst_pos = worst_yaw = 0.0
        end = time.time() + seconds
        while time.time() < end:
            x, y, _ = truth.enu()
            drift = max(drift, math.hypot(x - x0, y - y0))
            pe, ye = errors()
            worst_pos, worst_yaw = max(worst_pos, pe), max(worst_yaw, abs(ye))
            time.sleep(0.1)
        return drift, worst_pos, worst_yaw

    drift, pe, ye = hold(8)
    check('Loiter holds position for 8 s', drift < 0.3, f'drift {drift * 100:.0f} cm')
    check('EKF tracks truth in flight', pe < 0.15 and ye < 5, f'worst {pe * 100:.1f} cm, {ye:.1f} deg')

    print('3. Pitch forward 2 s, release: must stop and hold', flush=True)
    v.rc[1] = 1300
    time.sleep(2)
    v.rc[1] = 1500
    time.sleep(4)
    drift, _, _ = hold(5)
    check('stops and holds after stick input', drift < 0.3, f'drift {drift * 100:.0f} cm')

    if vicon_loss:
        print('4. Vicon lost in a hover: ArduPilot must land by itself', flush=True)
        n_texts = len(v.texts)
        x0, y0, _ = truth.enu()
        t0 = time.time()
        set_vicon_dropout(1.0)
        try:
            landing = v.wait(lambda: mavutil.mode_string_v10(v.state['HEARTBEAT']) == 'LAND', 10)
            t_land = time.time() - t0
            check('switches to LAND within 3 s of losing Vicon', landing and t_land < 3,
                  f'{t_land:.1f} s' if landing else 'still ' + mavutil.mode_string_v10(v.state['HEARTBEAT']))
            disarmed = v.wait(lambda: not v.armed(), 90)
            x, y, _ = truth.enu()
            drift = math.hypot(x - x0, y - y0)
            crashed = [t for t in v.texts[n_texts:] if 'Crash' in t]
            check('lands and disarms without Vicon', disarmed and not crashed,
                  crashed[0] if crashed else f'{time.time() - t0:.0f} s after the loss')
            check('blind landing drifts < 1 m', drift < 1.0, f'{drift * 100:.0f} cm, ended at ({x:+.2f}, {y:+.2f}) m')
        finally:
            set_vicon_dropout(0.0)
        return

    if param(v, 'FENCE_ENABLE'):
        alt_max, margin = param(v, 'FENCE_ALT_MAX'), param(v, 'FENCE_MARGIN')
        print('4. Lab fence: full forward stick, then full throttle; the fence must stop it', flush=True)
        n_texts = len(v.texts)
        v.rc[1] = 1000
        time.sleep(8)
        v.rc[1] = 1500
        time.sleep(3)
        x, y, _ = truth.enu()
        check('fence stops it before the wall', max(abs(x), abs(y)) < volume[0] / 2,
              f'at ({x:+.2f}, {y:+.2f}) m, tracked to +-{volume[0] / 2:g} m')
        v.rc[2] = 2000
        time.sleep(8)
        v.rc[2] = 1500
        time.sleep(2)
        z = truth.enu()[2] - z0
        check('fence stops the climb below FENCE_ALT_MAX', z < alt_max,
              f'{z:.2f} m (FENCE_ALT_MAX {alt_max:g}, margin {margin:g})')
        breaches = [t for t in v.texts[n_texts:] if 'ence' in t and 'reach' in t]
        mode = mavutil.mode_string_v10(v.state['HEARTBEAT'])
        check('no fence breach (avoidance stopped it first), still in LOITER',
              not breaches and mode == 'LOITER', breaches[0] if breaches else mode)

    print('5. Land', flush=True)
    n_texts = len(v.texts)
    v.m.set_mode('LAND')
    disarmed = v.wait(lambda: not v.armed(), 60)
    crashed = [t for t in v.texts[n_texts:] if 'Crash' in t]
    check('land and auto-disarm', disarmed and not crashed,
          f'crash detector fired: {crashed[0]}' if crashed else f'height {truth.enu()[2] - z0:.2f} m')
    hx, hy, hz = volume[0] / 2, volume[1] / 2, volume[2]
    inside = max(abs(truth.lo[0]), abs(truth.hi[0])) <= hx and max(abs(truth.lo[1]), abs(truth.hi[1])) <= hy \
        and truth.hi[2] <= hz
    check('whole flight inside the tracked volume', inside,
          f'x {truth.lo[0]:+.2f}..{truth.hi[0]:+.2f}, y {truth.lo[1]:+.2f}..{truth.hi[1]:+.2f}, '
          f'z up to {truth.hi[2]:.2f} m (volume +-{hx:g} x +-{hy:g} x {hz:g} m)')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--url', default='tcp:127.0.0.1:5762')
    parser.add_argument('--motors', action='store_true', help='only run the motor test')
    parser.add_argument('--mocap', action='store_true',
                        help='Loiter flight on the simulated Vicon (launch with mocap:=true)')
    parser.add_argument('--volume', default='3.0,3.0,4.0',
                        help='Vicon tracked volume x,y,z in m, floor centred on the origin (as mocap_volume)')
    parser.add_argument('--vicon-loss', action='store_true',
                        help='with --mocap: lose the simulated Vicon in a hover instead of the fence/Land steps')
    parser.add_argument('--param', action='append', default=[], metavar='NAME=VALUE',
                        help='set an FC parameter for this run only (repeatable), e.g. VISO_DELAY_MS=80')
    parser.add_argument('--rate-gain-scale', type=float, default=1.0,
                        help='multiply ATC_RAT_RLL/PIT P, I, D for this run (restored afterwards)')
    args = parser.parse_args()
    v = Vehicle(args.url)
    print(f'Connected to system {v.m.target_system}', flush=True)
    joints = Joints()
    time.sleep(2)
    motor_test(v, joints)
    if not args.motors:
        original = {}
        if args.rate_gain_scale != 1.0:
            original = {n: param(v, n) for n in RATE_GAINS}
            for n, val in original.items():
                param(v, n, val * args.rate_gain_scale)
            print(f'   roll/pitch rate gains x{args.rate_gain_scale} for this run', flush=True)
        for item in args.param:
            name, value = item.split('=')
            if name not in original:
                original[name] = param(v, name)
            param(v, name, float(value))
            print(f'   {name} = {value} for this run (was {original[name]:g})', flush=True)
        try:
            if args.mocap:
                mocap_flight(v, Truth(), [float(c) for c in args.volume.split(',')], args.vicon_loss)
            else:
                flight(v)
        finally:
            for n, val in original.items():
                param(v, n, val)
    passed = sum(results)
    print(f'\n{passed}/{len(results)} checks passed', flush=True)
    sys.exit(0 if results and all(results) else 1)


if __name__ == '__main__':
    main()
