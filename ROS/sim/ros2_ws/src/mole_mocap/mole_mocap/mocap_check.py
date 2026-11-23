"""Hand-carry check, props off: does ArduPilot see the drone move the way Vicon does?

    ros2 run mole_mocap mocap_check                  # guided steps, pass/fail each
    ros2 run mole_mocap mocap_check --ros-args -p watch:=true    # live side-by-side numbers only
    ros2 run mole_mocap mocap_check --ros-args -p steps:=nose,+x,yaw  # a subset of the steps

Run with vicon.launch.py up and the FC disarmed. Each step asks you to move the drone and
detects when you have put it down again (still for 1.5 s), then compares the change Vicon
measured (/vicon/mole/mole) with the change the EKF reports (/mavros/local_position/pose).
Both are ENU: x = Vicon +x (ArduPilot east), y = Vicon +y (ArduPilot north), z up.

Steps: nose along Vicon +x (checks the Vicon object's x axis is the nose), 1 m along +x,
1 m along +y, lift 0.5 m, turn 90 deg left (counter-clockwise from above). A wrong sign or a
swapped axis here would make the drone fly away in Loiter.

Comparing changes, not absolute values, ignores the EKF's constant height offset (ArduPilot
resets its height datum at the resting height when the origin is set).
"""
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import MessageInterval
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mole_mocap.mocap_math import wrap_deg, yaw_deg

STEPS = [
    # name, instruction, expected (dx, dy, dz, dyaw) or None, check
    ('nose', 'Turn the drone so its NOSE points along Vicon +x, then put it down.', None, 'nose'),
    ('+x', 'Move the drone about 1 m along Vicon +x (keep the heading), then put it down.', (1, 0, 0, 0), 'move'),
    ('+y', 'Move the drone about 1 m along Vicon +y (keep the heading), then put it down.', (0, 1, 0, 0), 'move'),
    ('up', 'Lift the drone about 0.5 m straight up and hold it still.', (0, 0, 1, 0), 'move'),
    ('yaw', 'Turn the drone about 90 deg LEFT (counter-clockwise seen from above), hold it still.',
     (0, 0, 0, 1), 'yaw'),
]


class Track:
    """Latest pose and a short history, for one topic."""

    def __init__(self):
        self.lock = threading.Lock()
        self.hist = []          # (t, x, y, z, yaw_deg)

    def add(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        with self.lock:
            self.hist.append((time.time(), p.x, p.y, p.z, yaw_deg((q.x, q.y, q.z, q.w))))
            if len(self.hist) > 2000:
                del self.hist[:1000]

    def last(self):
        with self.lock:
            return self.hist[-1] if self.hist else None

    def window(self, seconds):
        now = time.time()
        with self.lock:
            return [h for h in self.hist if now - h[0] <= seconds]

    def age(self):
        h = self.last()
        return time.time() - h[0] if h else math.inf


class MocapCheck(Node):
    def __init__(self):
        super().__init__('mocap_check')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.watch = p('watch', False)
        self.tol_m = p('tolerance_m', 0.05)        # plus tolerance_frac of the distance moved
        self.tol_frac = p('tolerance_frac', 0.10)
        self.tol_deg = p('tolerance_deg', 5.0)
        self.steps = [s.strip() for s in p('steps', 'nose,+x,+y,up,yaw').split(',')]
        self.vicon, self.ekf = Track(), Track()
        self.create_subscription(PoseStamped, p('vicon_topic', '/vicon/mole/mole'), self.vicon.add,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.ekf.add, qos_profile_sensor_data)
        self.interval = self.create_client(MessageInterval, '/mavros/set_message_interval')

    def request_streams(self):
        """Ask the FC for LOCAL_POSITION_NED (32) and ATTITUDE_QUATERNION (31) at 20 Hz: whether it
        sends them by default depends on the link's SRx_ stream rates."""
        if not self.interval.wait_for_service(timeout_sec=5.0):
            print('  (no /mavros/set_message_interval service; relying on the FC stream rates)', flush=True)
            return
        for msg_id in (32, 31):
            self.interval.call_async(MessageInterval.Request(message_id=msg_id, message_rate=20.0))


def still(track, seconds=1.5, pos_m=0.01, yaw_deg_=1.0):
    w = track.window(seconds)
    if len(w) < 5 or w[-1][0] - w[0][0] < seconds * 0.8:
        return False
    return all(max(h[i] for h in w) - min(h[i] for h in w) < pos_m for i in (1, 2, 3)) and \
        max(abs(wrap_deg(h[4] - w[0][4])) for h in w) < yaw_deg_


def settled_pose(track):
    w = track.window(1.0)
    n = len(w)
    return tuple(sum(h[i] for h in w) / n for i in (1, 2, 3)) + (w[-1][4],)


def wait_for_move_and_settle(node, start, timeout=90.0):
    """Wait until the drone has moved (> 0.2 m or 20 deg) from `start`, then is still again."""
    end = time.time() + timeout
    moved = False
    while time.time() < end:
        h = node.vicon.last()
        if h and not moved:
            moved = math.dist(h[1:4], start[:3]) > 0.2 or abs(wrap_deg(h[4] - start[3])) > 20
        if moved and still(node.vicon) and still(node.ekf, pos_m=0.02, yaw_deg_=2.0):
            return True
        time.sleep(0.1)
    return False


def fmt(d):
    return f'({d[0]:+.2f}, {d[1]:+.2f}, {d[2]:+.2f}) m, yaw {d[3]:+.1f} deg'


def run_steps(node):
    print('Waiting for Vicon and MAVROS poses...', flush=True)
    node.request_streams()
    end = time.time() + 15
    while (node.vicon.age() > 0.5 or node.ekf.age() > 0.5) and time.time() < end:
        time.sleep(0.2)
    if node.vicon.age() > 0.5 or node.ekf.age() > 0.5:
        print(f'FAIL: no data (Vicon age {node.vicon.age():.1f} s, EKF age {node.ekf.age():.1f} s). '
              'Is vicon.launch.py running and the EKF origin set?', flush=True)
        return False
    print('Hold the drone still on the floor...', flush=True)
    while not (still(node.vicon) and still(node.ekf, pos_m=0.02, yaw_deg_=2.0)):
        time.sleep(0.1)
    v, e = settled_pose(node.vicon), settled_pose(node.ekf)
    off = [e[i] - v[i] for i in range(3)] + [wrap_deg(e[3] - v[3])]
    results = []

    def check(name, ok, detail):
        results.append(ok)
        print(f'  [{"PASS" if ok else "FAIL"}] {name}: {detail}', flush=True)

    check('EKF horizontal position = Vicon', math.hypot(off[0], off[1]) < 0.1,
          f'off by ({off[0]:+.3f}, {off[1]:+.3f}) m')
    check('EKF heading = Vicon', abs(off[3]) < 5, f'off by {off[3]:+.1f} deg')
    print(f'  [info] EKF height reads {off[2] * 100:+.1f} cm vs Vicon (datum reset; expected ~ the '
          'resting height of the Vicon object origin)', flush=True)

    for name, text, expected, kind in [s for s in STEPS if s[0] in node.steps]:
        v0, e0 = settled_pose(node.vicon), settled_pose(node.ekf)
        print(f'\n{name}: {text}', flush=True)
        if kind == 'nose' and abs(wrap_deg(v0[3])) < 15:
            print('  (already pointing along +x; turn it away and back, or it will time out)', flush=True)
        if not wait_for_move_and_settle(node, v0):
            check(name, False, 'timed out waiting for the move')
            continue
        v1, e1 = settled_pose(node.vicon), settled_pose(node.ekf)
        dv = [v1[i] - v0[i] for i in range(3)] + [wrap_deg(v1[3] - v0[3])]
        de = [e1[i] - e0[i] for i in range(3)] + [wrap_deg(e1[3] - e0[3])]
        print(f'  Vicon moved {fmt(dv)}\n  EKF   moved {fmt(de)}', flush=True)
        if kind == 'nose':
            check('Vicon object x axis is the nose', abs(wrap_deg(v1[3])) < 15,
                  f'Vicon yaw {v1[3]:+.1f} deg with the nose along +x (expect ~0)')
            check('EKF heading follows', abs(wrap_deg(de[3] - dv[3])) < 5, f'differs by {wrap_deg(de[3] - dv[3]):+.1f} deg')
        elif kind == 'move':
            axis = expected.index(1)
            dist = abs(dv[axis])
            ok_axis = dv[axis] > 0.3 and all(abs(dv[i]) < 0.5 * dist for i in range(3) if i != axis)
            check(f'you moved it along Vicon +{"xyz"[axis]}', ok_axis,
                  'yes' if ok_axis else 'Vicon saw a different direction: redo this step')
            err = math.dist(de[:3], dv[:3])
            check(f'EKF moved the same way (+{"xyz"[axis]})', err < node.tol_m + node.tol_frac * dist,
                  f'difference {err * 100:.1f} cm over {dist:.2f} m')
        else:
            ok_turn = 60 < dv[3] < 120
            check('you turned it left (counter-clockwise)', ok_turn,
                  f'Vicon yaw changed {dv[3]:+.1f} deg' + ('' if ok_turn else ': redo, turning left'))
            check('EKF heading turned the same way', abs(wrap_deg(de[3] - dv[3])) < node.tol_deg,
                  f'EKF {de[3]:+.1f} deg vs Vicon {dv[3]:+.1f} deg')
    print(f'\n{sum(results)}/{len(results)} checks passed', flush=True)
    return all(results)


def watch(node):
    while rclpy.ok():
        v, e = node.vicon.last(), node.ekf.last()
        if v and e:
            print(f'Vicon ({v[1]:+6.2f} {v[2]:+6.2f} {v[3]:+6.2f}) yaw {v[4]:+7.1f}   '
                  f'EKF ({e[1]:+6.2f} {e[2]:+6.2f} {e[3]:+6.2f}) yaw {e[4]:+7.1f}   '
                  f'ages {node.vicon.age() * 1000:4.0f}/{node.ekf.age() * 1000:4.0f} ms', flush=True)
        time.sleep(0.2)


def main():
    rclpy.init()
    node = MocapCheck()
    def spin():
        try:
            rclpy.spin(node)
        except Exception:     # ExternalShutdownException at exit
            pass

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    ok = True
    try:
        if node.watch:
            node.request_streams()
            watch(node)
        else:
            ok = run_steps(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()          # stops the spin thread before the node goes away
        spinner.join(timeout=2.0)
        node.destroy_node()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
