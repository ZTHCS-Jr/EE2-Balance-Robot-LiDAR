#!/usr/bin/env python3
"""Bridge /cmd_vel (ROS, on the laptop/WSL) -> UDP velocity commands to the Pi.

The Pi's client.py listens on UDP cmd_port and turns {"v","w"} into the ESP32's
V:/A: commands. This node is the only thing that drives the robot from ROS, so it
is deliberately conservative:

  * It re-sends the latest /cmd_vel at a fixed rate (the Pi expects a steady stream
    and has its own deadman if this node dies).
  * If /cmd_vel goes stale (publisher stopped) it sends ZEROS rather than repeating
    the last non-zero command -- so a crashed explorer cannot leave the robot
    driving. Two stop layers: stale -> zeros here, and bridge-dead -> deadman on Pi.

Uses only stdlib sockets (no websocket dependency).
"""
import json
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('pi_ip', '10.144.216.133')  # SET to the robot Pi's LAN IP
        self.declare_parameter('pi_port', 31416)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('stale_timeout', 0.3)       # s; older -> send zeros

        self.pi_ip = self.get_parameter('pi_ip').value
        self.pi_port = int(self.get_parameter('pi_port').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        rate = float(self.get_parameter('rate').value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.v = 0.0
        self.w = 0.0
        self.last_rx = None  # rclpy Time of last /cmd_vel

        self.create_subscription(Twist, 'cmd_vel', self._cb, 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"cmd_vel_bridge -> {self.pi_ip}:{self.pi_port} at {rate:.0f} Hz "
            f"(zeros after {self.stale_timeout}s stale)")

    def _cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_rx = self.get_clock().now()

    def _tick(self):
        fresh = (self.last_rx is not None and
                 (self.get_clock().now() - self.last_rx).nanoseconds < self.stale_timeout * 1e9)
        v, w = (self.v, self.w) if fresh else (0.0, 0.0)
        try:
            self.sock.sendto(json.dumps({"v": v, "w": w}).encode("utf-8"),
                             (self.pi_ip, self.pi_port))
        except OSError as e:
            self.get_logger().warning(f"UDP send failed: {e}")

    def stop(self):
        # Best-effort: tell the Pi to halt before we exit.
        for _ in range(3):
            try:
                self.sock.sendto(json.dumps({"v": 0.0, "w": 0.0}).encode("utf-8"),
                                 (self.pi_ip, self.pi_port))
            except OSError:
                break


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
