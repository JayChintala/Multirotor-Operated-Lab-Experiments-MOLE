"""Simulated Vicon for SITL: Gazebo ground truth -> geometry_msgs/PoseStamped, published on the
same topic and with the same message type as ros2-vicon-receiver, so everything downstream
(mocap_bridge, MAVROS, ArduPilot) runs unchanged between simulation and the real lab.

Real-system effects it adds: lower rate, latency, position/attitude noise, dropouts, and the
tracked volume: outside it (default 3 x 3 m floor centred on the origin, 4 m high) nothing is
published, as when the drone leaves the cameras' view.

    ros2 run mole_mocap fake_vicon --ros-args -p use_sim_time:=true -p latency_ms:=20
    ros2 param set /fake_vicon dropout_probability 1.0     # lose Vicon now (tests); 0.0 restores

The stamp is the time the pose is published (receipt time on the laptop), not the capture
time: the latency shows up as a stale pose, as it would through a network, and ArduPilot
compensates with VISO_DELAY_MS.
"""
import random

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mole_mocap.mocap_math import DelayLine, in_volume, perturb


class FakeVicon(Node):
    def __init__(self):
        super().__init__('fake_vicon')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        truth_topic = p('truth_topic', '/model/mole_quad/odometry')
        namespace = p('namespace', 'vicon')
        subject = p('subject', 'mole')
        segment = p('segment', '') or subject
        rate_hz = p('rate_hz', 100.0)
        self.latency = p('latency_ms', 20.0) / 1000.0
        self.pos_sigma = p('position_noise_m', 0.0005)
        self.ang_sigma = p('angle_noise_rad', 0.002)
        self.dropout = p('dropout_probability', 0.0)
        self.frame_id = p('frame_id', 'vicon')
        self.volume = (p('volume_x_m', 3.0) / 2, p('volume_y_m', 3.0) / 2, p('volume_z_m', 4.0))
        self.rng = random.Random(p('seed', 0))

        self.add_on_set_parameters_callback(self.on_params)   # latency/noise/dropout change live
        self.delay = DelayLine()
        self.topic = f'/{namespace}/{subject}/{segment}'
        self.pub = self.create_publisher(PoseStamped, self.topic, 10)
        self.create_subscription(Odometry, truth_topic, self.on_truth, qos_profile_sensor_data)
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'{truth_topic} -> {self.topic} at {rate_hz:g} Hz, latency {self.latency * 1000:g} ms, '
            f'noise {self.pos_sigma * 1000:g} mm / {self.ang_sigma:g} rad, dropout {self.dropout:g}, '
            f'volume {2 * self.volume[0]:g} x {2 * self.volume[1]:g} x {self.volume[2]:g} m')

    def on_params(self, params):
        live = {'latency_ms': ('latency', 1e-3), 'position_noise_m': ('pos_sigma', 1.0),
                'angle_noise_rad': ('ang_sigma', 1.0), 'dropout_probability': ('dropout', 1.0)}
        for prm in params:
            if prm.name in live:
                attr, scale = live[prm.name]
                setattr(self, attr, float(prm.value) * scale)
                self.get_logger().warn(f'{prm.name} = {prm.value}')
        return SetParametersResult(successful=True)

    def on_truth(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.delay.push(t, msg.pose.pose)

    def tick(self):
        now = self.get_clock().now()
        pose = self.delay.pop_ready(now.nanoseconds * 1e-9, self.latency)
        if pose is None or self.rng.random() < self.dropout:
            return
        p, q = pose.position, pose.orientation
        if not in_volume((p.x, p.y, p.z), *self.volume):
            self.get_logger().warn(f'drone outside the tracked volume at ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m: '
                                   'no pose', throttle_duration_sec=1.0)
            return
        (x, y, z), (qx, qy, qz, qw) = perturb((p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                                              self.pos_sigma, self.ang_sigma, self.rng)
        out = PoseStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.frame_id
        out.pose.position.x, out.pose.position.y, out.pose.position.z = x, y, z
        o = out.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        self.pub.publish(out)


def main():
    rclpy.init()
    node = FakeVicon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
