"""Vicon pose -> ArduPilot external navigation, through MAVROS. Same node in SITL and in the lab.

    /vicon/<subject>/<segment>  (PoseStamped, from ros2-vicon-receiver or fake_vicon)
        -> checks (finite, valid quaternion, not stale), rate limit
        -> /mavros/vision_pose/pose  (MAVROS sends VISION_POSITION_ESTIMATE to the FC)

It also sets the EKF origin once (SET_GPS_GLOBAL_ORIGIN): with no GPS, ArduPilot will not
use position modes (Loiter, Guided) until an origin exists.

Losing Vicon in flight (on_loss:=land, the default): after loss_timeout_s without a pose while
armed, it switches the EKF to source set 2 (no horizontal position: EK3_SRC2_POSXY 0) and
commands LAND. Without this the EKF freezes its last position and keeps steering on it while
the drone drifts away (5.6 m in SITL before ArduPilot's own failsafe, 8 s later, landed it).
With no position source, Land holds the drone level and descends. Once poses are back and the
FC is disarmed, it switches back to source set 1. If the Wi-Fi itself drops, these commands
can't reach the FC either; the Pi's kill switch covers that case.

Frames: the pose must be ENU-like (z up) with the body x axis pointing at the drone's nose and
z up. MAVROS converts to NED/FRD. With no compass, ArduPilot's "north" is simply the Vicon +y
axis. The Vicon object's origin should be at the drone's CoG, or set VISO_POS_X/Y/Z.
"""
import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong, SetMode
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data

from mole_mocap.mocap_math import RateLimiter, pose_ok

MAV_CMD_SET_EKF_SOURCE_SET = 42007


class MocapBridge(Node):
    def __init__(self):
        super().__init__('mocap_bridge')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.pose_topic = p('pose_topic', '/vicon/mole/mole')
        rate_hz = p('rate_hz', 50.0)
        self.stale_s = p('max_age_s', 0.25)
        # Default: the SITL home (sitl.launch.py HOME / mole_field.sdf). Any fixed point works
        # indoors; it only anchors the local frame for ArduPilot's lat/lon bookkeeping.
        self.origin = (p('origin_lat', 40.4433), p('origin_lon', -79.9436), p('origin_alt', 280.0))
        self.on_loss = p('on_loss', 'land')          # 'land' or 'none'
        self.loss_timeout = p('loss_timeout_s', 0.5)
        self.fc_state = None
        self.lost_action_taken = False             # source set 2 + LAND sent for this loss

        self.limiter = RateLimiter(rate_hz)
        self.origin_set = False
        self.origin_tries = 0
        self.last_rx = None
        self.forwarded = 0
        self.rejected = 0

        self.vision_pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.origin_pub = self.create_publisher(GeoPointStamped, '/mavros/global_position/set_gp_origin', 10)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GeoPointStamped, '/mavros/global_position/gp_origin', self.on_origin, latched)
        self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, qos_profile_sensor_data)
        self.create_timer(2.0, self.send_origin)
        self.create_timer(1.0, self.watchdog)
        self.create_subscription(State, '/mavros/state', self.on_state, 10)
        self.cmd = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.create_timer(0.1, self.loss_check)
        self.get_logger().info(f'{self.pose_topic} -> /mavros/vision_pose/pose at <= {rate_hz:g} Hz')

    def on_pose(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.last_rx = now
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, q = msg.pose.position, msg.pose.orientation
        if not pose_ok((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)):
            self.rejected += 1
            self.get_logger().warn('invalid pose (NaN or bad quaternion: object lost?), not sent',
                                   throttle_duration_sec=1.0)
            return
        if abs(now - t) > self.stale_s:
            self.rejected += 1
            self.get_logger().warn(
                f'pose stamp is {now - t:+.3f} s off this computer\'s clock (limit {self.stale_s} s), '
                'not sent. Check clock sync / use_sim_time.', throttle_duration_sec=1.0)
            return
        if not self.limiter.accept(t):
            return
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.pose = msg.pose
        self.vision_pub.publish(out)
        self.forwarded += 1

    def send_origin(self):
        if self.origin_set:
            return
        self.origin_tries += 1
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position.latitude, msg.position.longitude, msg.position.altitude = self.origin
        self.origin_pub.publish(msg)
        if self.origin_tries % 5 == 1:
            self.get_logger().info(f'setting EKF origin {self.origin} (try {self.origin_tries}); '
                                   'waiting for the FC to confirm')

    def on_origin(self, msg):
        if not self.origin_set:
            self.get_logger().info(f'FC confirmed EKF origin {msg.position.latitude:.6f}, '
                                   f'{msg.position.longitude:.6f}')
        self.origin_set = True

    def on_state(self, msg):
        self.fc_state = msg

    def source_set(self, n):
        req = CommandLong.Request(command=MAV_CMD_SET_EKF_SOURCE_SET, param1=float(n))
        self.cmd.call_async(req)

    def loss_check(self):
        if self.on_loss != 'land' or self.fc_state is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        lost = self.last_rx is None or now - self.last_rx > self.loss_timeout
        if lost and self.fc_state.armed and not self.lost_action_taken:
            self.lost_action_taken = True
            self.source_set(2)
            self.set_mode.call_async(SetMode.Request(custom_mode='LAND'))
            self.get_logger().error(f'Vicon lost for > {self.loss_timeout} s in flight: EKF source set 2 '
                                    '(no position) and LAND sent')
        elif not lost and not self.fc_state.armed and self.lost_action_taken:
            self.lost_action_taken = False
            self.source_set(1)
            self.get_logger().info('Vicon back and disarmed: EKF source set 1 (Vicon) again')

    def watchdog(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_rx is None:
            self.get_logger().warn(f'no pose on {self.pose_topic} yet', throttle_duration_sec=5.0)
        elif now - self.last_rx > 0.5:
            self.get_logger().error(f'no pose for {now - self.last_rx:.1f} s: FC will lose position',
                                    throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = MocapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
