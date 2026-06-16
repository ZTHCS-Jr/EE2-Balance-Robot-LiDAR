#!/usr/bin/env python3
"""
Subscribes: /wheel/joint_states  (sensor_msgs/JointState)
                  name     = ['left_wheel', 'right_wheel']
                  position = [left_steps, right_steps]   (signed absolute microsteps)
                  stamp    = ESP32 clock (anchored to ROS time by the udp_receiver)
Publishes: /wheel/odom          (nav_msgs/Odometry, frame odom -> base_link)

The EKF (robot_localization) only fuses this node's twist.linear.x
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        steps_per_rev = 200
        microstepping = 16
        wheel_diameter = 0.065
        self.wheel_base = 0.12
        self.left_sign = 1
        self.right_sign = -1
        self.vx_variance = 0.002
        self.wz_variance = 1.0e6
        self.x_variance = 0.001
        self.y_variance = 0.001
        self.yaw_variance = 1.0e6
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.max_dt = 0.5 # Skip integrating yaw in large differences (0.5s)
        self.max_jump = 0.3 # Max jump in positions to remove the glitch of it randomly teloporting
        self.min_dt = 0.005 # Skip integrating yaw in small differences

        # Metres of wheel travel per commanded microstep.
        self.dist_per_step = (math.pi * wheel_diameter) / (steps_per_rev * microstepping)

        # Integrated pose + previous sample.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left = None
        self.prev_right = None
        self.prev_t = None

        self.odom_pub = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.create_subscription(JointState, 'wheel/joint_states', self.joint_cb, 10)

        self.get_logger().info(
            f"wheel_odometry_node: dist_per_step={self.dist_per_step:.6e} m, "
            f"wheel_base={self.wheel_base} m")

    @staticmethod
    def _stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def joint_cb(self, msg):
        try:
            li = msg.name.index('left_wheel')
            ri = msg.name.index('right_wheel')
        except ValueError:
            return
        left = msg.position[li]
        right = msg.position[ri]
        t = self._stamp_to_sec(msg.header.stamp)

        # First sample: seed state, nothing to integrate yet.
        if self.prev_left is None:
            self.prev_left, self.prev_right, self.prev_t = left, right, t
            return

        dt = t - self.prev_t
        d_left = (left - self.prev_left) * self.left_sign * self.dist_per_step
        d_right = (right - self.prev_right) * self.right_sign * self.dist_per_step
        self.prev_left, self.prev_right, self.prev_t = left, right, t

        # Glitching fix so that corrupt/duplicate packets (due to UDP) can cause the EKF and ODOM to spike causing it to teleport in the generated map
        # We just ignore data that is greater than a large jump
        if abs(d_left) > self.max_jump or abs(d_right) > self.max_jump:
            self.get_logger().warn(
                f"wheel-odom glitch rejected: d_left={d_left:.2f} m, "
                f"d_right={d_right:.2f} m (> max_jump {self.max_jump} m)")
            self.publish_odom(msg.header.stamp, 0.0, 0.0)
            return

        # Guard against reordered or late packets causing the timing to be out of sync
        # Simply stop the robot if out of sync
        valid_dt = self.min_dt <= dt <= self.max_dt

        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.wheel_base

        # Integrate pose with the midpoint heading (2nd-order exact for constant-curvature).
        self.x += d_center * math.cos(self.theta + 0.5 * d_theta)
        self.y += d_center * math.sin(self.theta + 0.5 * d_theta)
        self.theta = math.atan2(math.sin(self.theta + d_theta),
                                math.cos(self.theta + d_theta))

        vx = d_center / dt if valid_dt else 0.0
        wz = d_theta / dt if valid_dt else 0.0

        self.publish_odom(msg.header.stamp, vx, wz)

    def publish_odom(self, stamp, vx, wz):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz

        # 6x6 row-major [x, y, z, roll, pitch, yaw]. Only twist.vx is fused by the EKF;
        # heading terms carry huge variance so the EKF never leans on wheel rotation.
        odom.pose.covariance[0] = self.x_variance     # x
        odom.pose.covariance[7] = self.y_variance     # y
        odom.pose.covariance[35] = self.yaw_variance  # yaw
        odom.twist.covariance[0] = self.vx_variance   # vx (trusted)
        odom.twist.covariance[35] = self.wz_variance  # vyaw (distrusted)

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
