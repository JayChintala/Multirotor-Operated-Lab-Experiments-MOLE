"""Upload a square inclusion fence, centred on the Vicon origin, to the FC through MAVROS.

    ros2 run mole_mocap lab_fence --ros-args -p half_x_m:=1.25 -p half_y_m:=1.25

Waits for the EKF origin (set by mocap_bridge), converts the square's corners from Vicon metres
(x east, y north, as ArduPilot sees them with no compass) to lat/lon around that origin, and
pushes them as one polygon on /mavros/geofence/push. Same node in SITL and in the lab.
It only matters with FENCE_TYPE including bit 2 (lab_limits.parm); then the FC will not arm
until the polygon is there.
"""
import rclpy
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from mole_mocap.mocap_math import offset_latlon

MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001


class LabFence(Node):
    def __init__(self):
        super().__init__('lab_fence')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        hx, hy = p('half_x_m', 1.25), p('half_y_m', 1.25)
        self.corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]   # (east, north) m
        self.origin = None
        self.done = False
        self.client = self.create_client(WaypointPush, '/mavros/geofence/push')
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GeoPointStamped, '/mavros/global_position/gp_origin', self.on_origin, latched)
        self.create_timer(2.0, self.try_push)
        self.get_logger().info(f'{2 * hx:g} x {2 * hy:g} m fence around the Vicon origin; waiting for the EKF origin')

    def on_origin(self, msg):
        self.origin = (msg.position.latitude, msg.position.longitude)

    def try_push(self):
        if self.done or self.origin is None or not self.client.service_is_ready():
            return
        req = WaypointPush.Request()
        for east, north in self.corners:
            lat, lon = offset_latlon(*self.origin, north, east)
            wp = Waypoint(frame=Waypoint.FRAME_GLOBAL, command=MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                          autocontinue=True, param1=float(len(self.corners)), x_lat=lat, y_long=lon)
            req.waypoints.append(wp)
        self.done = True                       # one request in flight at a time
        self.client.call_async(req).add_done_callback(self.on_result)

    def on_result(self, future):
        res = future.result()
        if res is not None and res.success and res.wp_transfered == len(self.corners):
            self.get_logger().info(f'fence uploaded: {len(self.corners)} vertices')
        else:
            self.get_logger().error(f'fence upload failed ({res}); retrying')
            self.done = False


def main():
    rclpy.init()
    node = LabFence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
