#!/usr/bin/env python3
"""Offline sanity test for wheel_odometry_node (no hardware/DDS needed).

Feeds synthetic JointState samples straight into joint_cb() and checks the
published Odometry against hand calculations for the real geometry
(wheel_diameter=0.065 m, wheel_base=0.12 m). Run:

    source /opt/ros/humble/setup.bash
    python3 test_wheel_odometry.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rclpy
from sensor_msgs.msg import JointState
from wheel_odometry_node import WheelOdometryNode


def make_js(t, left_count, right_count):
    js = JointState()
    js.header.stamp.sec = int(t)
    js.header.stamp.nanosec = int((t - int(t)) * 1e9)
    js.name = ['left_wheel', 'right_wheel']
    js.position = [float(left_count), float(right_count)]
    return js


def yaw_of(odom):
    return 2.0 * math.atan2(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)


def main():
    rclpy.init()
    node = WheelOdometryNode()
    captured = []
    node.odom_pub.publish = lambda msg: captured.append(msg)   # intercept output

    dps = node.dist_per_step
    wb = node.wheel_base
    print(f"dist_per_step = {dps:.6e} m/microstep   wheel_base = {wb:.3f} m\n")

    def reset():
        node.x = node.y = node.theta = 0.0
        node.prev_left = node.prev_right = node.prev_t = None
        captured.clear()

    ok = True

    # --- Test A: drive straight 2.0 m (forward => left count up, right count down) ---
    reset()
    target = 2.0
    total = target / dps          # total microstep delta per wheel
    N = 20
    node.joint_cb(make_js(0.0, 0, 0))   # seed
    for i in range(1, N + 1):
        node.joint_cb(make_js(i * 0.1, i * total / N, -i * total / N))
    last = captured[-1]
    x, y, yaw, vx = last.pose.pose.position.x, last.pose.pose.position.y, yaw_of(last), last.twist.twist.linear.x
    print(f"[A] straight 2 m : x={x:.3f}  y={y:.4f}  yaw={yaw:.4f} rad  vx={vx:.3f} m/s")
    a_ok = abs(x - 2.0) < 0.01 and abs(y) < 1e-6 and abs(yaw) < 1e-6
    print(f"    expect x~2.000, y~0, yaw~0, vx~2.0   -> {'PASS' if a_ok else 'FAIL'}\n")
    ok &= a_ok

    # --- Test B: in-place +90 deg turn (CCW, +yaw) ---
    reset()
    d_theta = math.pi / 2.0
    dL = -d_theta * wb / 2.0      # left wheel travel (m)
    dR = +d_theta * wb / 2.0      # right wheel travel (m)
    countL = dL / (node.left_sign * dps)
    countR = dR / (node.right_sign * dps)
    N = 18
    node.joint_cb(make_js(0.0, 0, 0))
    for i in range(1, N + 1):
        node.joint_cb(make_js(i * 0.1, i * countL / N, i * countR / N))
    last = captured[-1]
    x, y, yaw = last.pose.pose.position.x, last.pose.pose.position.y, yaw_of(last)
    print(f"[B] in-place +90: x={x:.4f}  y={y:.4f}  yaw={yaw:.4f} rad ({math.degrees(yaw):.1f} deg)")
    b_ok = abs(yaw - math.pi / 2.0) < 1e-3 and abs(x) < 0.05 and abs(y) < 0.05
    print(f"    expect yaw~+1.571 (90 deg), x~0, y~0   -> {'PASS' if b_ok else 'FAIL'}\n")
    ok &= b_ok

    # --- Test C: dropped-packet gap => pose still advances, twist not a garbage spike ---
    reset()
    node.joint_cb(make_js(0.0, 0, 0))
    node.joint_cb(make_js(0.1, 1000, -1000))      # normal step
    node.joint_cb(make_js(5.0, 2000, -2000))      # 4.9 s gap (> max_dt=0.5)
    last = captured[-1]
    moved = last.pose.pose.position.x
    vx = last.twist.twist.linear.x
    print(f"[C] post-gap     : x={moved:.4f} m  vx={vx:.3f} m/s")
    c_ok = moved > 0.0 and vx == 0.0
    print(f"    expect x advanced, vx forced 0 (no spike)   -> {'PASS' if c_ok else 'FAIL'}\n")
    ok &= c_ok

    rclpy.shutdown()
    print("RESULT:", "ALL PASS" if ok else "FAILURES PRESENT")
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
