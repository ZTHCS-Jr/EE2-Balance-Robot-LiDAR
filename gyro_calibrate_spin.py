#!/usr/bin/env python3
"""Gyro-scale calibration spin.

Spins the robot in place until the EKF heading (driven by the gyro) BELIEVES it has
turned a target angle (default 180 deg), then stops. You then measure the robot's
ACTUAL physical angle (protractor, floor tiles, tape marks) and set, in
sensors_udp_receiver.py:

    GYRO_SCALE = actual_deg / target_deg          (x the CURRENT GYRO_SCALE if not 1.0)

Intuition: if you ask for "180" and the robot really spun 200 deg, the gyro
UNDER-reports rotation, so GYRO_SCALE = 200/180 = 1.11 (>1). If it only spun 160,
the gyro OVER-reports -> 160/180 = 0.89 (<1).

Self-contained: reads /odometry/filtered for heading and sends {"v","w"} straight to
the Pi over UDP (same protocol/port 31416 as cmd_vel_bridge). Run the mapping stack
only -- do NOT run autonomous_explore/nav2 at the same time (it would fight for cmd):

    # terminal 1  (provides /odometry/filtered + TF)
    ros2 launch ./launcher.py
    # terminal 2
    python3 gyro_calibrate_spin.py --pi-ip 10.140.43.80
    python3 gyro_calibrate_spin.py --pi-ip 10.140.43.80 --deg 180 --rate 0.3

Safeties: it will not command motion until it has received odometry, and it stops
unconditionally after --timeout seconds. Ctrl-C also sends stop. The Pi's own 0.5 s
deadman halts the robot if this script dies.
"""
import argparse
import json
import math
import socket

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    """2D yaw (rad) from a quaternion."""
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


class GyroCalSpin(Node):
    def __init__(self, target_deg, rate, pi_ip, pi_port, timeout):
        super().__init__('gyro_calibrate_spin')
        self.target = math.radians(abs(target_deg))
        self.target_deg = abs(target_deg)
        self.w = abs(rate) if target_deg >= 0 else -abs(rate)
        self.timeout = timeout
        self.pi = (pi_ip, int(pi_port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.prev_yaw = None
        self.accum = 0.0          # signed, unwrapped heading change (rad)
        self.done = False
        self.start = self.get_clock().now()

        self.create_subscription(Odometry, 'odometry/filtered', self._cb, 20)
        self.create_timer(0.05, self._tick)   # 20 Hz, matches the Pi deadman cadence
        self.get_logger().info(
            f"Will spin until the EKF heading turns {self.target_deg:.0f} deg "
            f"({'CCW' if self.w > 0 else 'CW'}) at {abs(rate):.2f} rad/s, then STOP. "
            f"Measure the actual angle and set GYRO_SCALE = actual/{self.target_deg:.0f}.")

    def _cb(self, msg):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.prev_yaw is not None:
            step = math.atan2(math.sin(yaw - self.prev_yaw),
                              math.cos(yaw - self.prev_yaw))   # unwrap each step
            self.accum += step                                  # signed -> wobble cancels
        self.prev_yaw = yaw

    def _send(self, v, w):
        self.sock.sendto(json.dumps({"v": v, "w": w}).encode("utf-8"), self.pi)

    def stop(self):
        for _ in range(5):
            self._send(0.0, 0.0)

    def _tick(self):
        if self.done:
            return
        elapsed = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self.done = True
            self.stop()
            self.get_logger().warn(
                f"TIMEOUT after {self.timeout:.0f}s at believed "
                f"{math.degrees(abs(self.accum)):.1f} deg -- is launcher.py up and "
                f"/odometry/filtered publishing? Stopped.")
            return
        if self.prev_yaw is None:
            return  # wait for odometry before moving (safety)
        if abs(self.accum) >= self.target:
            self.done = True
            self.stop()
            self.get_logger().info(
                f"STOPPED at believed {math.degrees(abs(self.accum)):.1f} deg. "
                f"Now measure the ACTUAL physical angle -> "
                f"GYRO_SCALE = actual/{self.target_deg:.0f}.")
            return
        self._send(0.0, self.w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pi-ip', default='10.140.43.80')
    ap.add_argument('--pi-port', type=int, default=31416)
    ap.add_argument('--deg', type=float, default=180.0, help='believed angle (deg, + = CCW)')
    ap.add_argument('--rate', type=float, default=0.3, help='spin rate magnitude (rad/s)')
    ap.add_argument('--timeout', type=float, default=30.0, help='hard stop (s)')
    args = ap.parse_args()

    rclpy.init()
    node = GyroCalSpin(args.deg, args.rate, args.pi_ip, args.pi_port, args.timeout)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()          # always leave the robot stopped
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
