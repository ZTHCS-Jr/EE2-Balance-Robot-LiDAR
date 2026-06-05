#!/usr/bin/env python3
"""Differential-drive wheel odometry from commanded stepper counts.

Subscribes  : /wheel/joint_states  (sensor_msgs/JointState)
                  name     = ['left_wheel', 'right_wheel']
                  position = [left_steps, right_steps]   (signed absolute microsteps)
                  stamp    = ESP32 clock (anchored to ROS time by the receiver)
Publishes   : /wheel/odom          (nav_msgs/Odometry, frame odom -> base_link)

The EKF (robot_localization) only fuses this node's twist.linear.x: steppers do
not slip much longitudinally, so vx is trusted (small covariance). Heading from
the wheels is DISTRUSTED (open-loop steppers can drop steps under the dynamic load
of balancing, with no encoder to confirm) -> twist.angular.z gets a huge
covariance and the EKF takes heading from the gyro instead. The full pose is still
integrated and published for debugging / RViz. This node does NOT publish TF; the
EKF owns odom -> base_link.

All robot geometry is exposed as ROS parameters (no magic numbers).
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        # --- Geometry (override at launch; wheel_diameter/wheel_base are placeholders
        #     marked TODO until the measured values are supplied) ---
        self.declare_parameter('steps_per_rev', 200)
        self.declare_parameter('microstepping', 16)
        self.declare_parameter('wheel_diameter', 0.065)   # measured: 6.5 cm
        self.declare_parameter('wheel_base', 0.12)        # measured: 12 cm track width

        # Sign conventions (verify empirically: push the robot forward by hand and
        # confirm both wheel deltas come out positive). step2 (right) is mounted
        # mirrored and negated in firmware, hence the default -1.
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', -1.0)

        # Covariances. vx trusted; wheel heading distrusted.
        self.declare_parameter('vx_variance', 0.002)
        self.declare_parameter('wz_variance', 1.0e6)
        self.declare_parameter('x_variance', 0.001)
        self.declare_parameter('y_variance', 0.001)
        self.declare_parameter('yaw_variance', 1.0e6)

        # Frames + robustness.
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('max_dt', 0.5)  # s; skip integration across long gaps

        steps_per_rev = self.get_parameter('steps_per_rev').value
        microstepping = self.get_parameter('microstepping').value
        wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.left_sign = self.get_parameter('left_sign').value
        self.right_sign = self.get_parameter('right_sign').value
        self.vx_variance = self.get_parameter('vx_variance').value
        self.wz_variance = self.get_parameter('wz_variance').value
        self.x_variance = self.get_parameter('x_variance').value
        self.y_variance = self.get_parameter('y_variance').value
        self.yaw_variance = self.get_parameter('yaw_variance').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_dt = self.get_parameter('max_dt').value

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

        # Guard against reordered / late packets and post-gap dt (a dropped run of
        # packets leaves the absolute counts correct, but the instantaneous velocity
        # over a huge dt would be meaningless): still advance the pose, but report
        # zero twist rather than a garbage spike.
        valid_dt = 0.0 < dt <= self.max_dt

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
