#!/usr/bin/env python3
"""Bridge /cmd_vel (nav2, on the laptop/WSL) -> UDP velocity commands to the Pi.

nav2's controller publishes /cmd_vel; the Pi's client.py cmd_listener turns the
{"v","w"} UDP message into the ESP32's V:/A: commands. Deliberately conservative:

  * Re-sends the latest /cmd_vel at a fixed rate (the Pi expects a steady stream and
    has its own deadman if this node dies).
  * If /cmd_vel goes stale (nav2 stopped publishing) it sends ZEROS rather than
    repeating the last command -> a crashed/cancelled nav cannot leave the robot
    driving. Two stop layers: stale -> zeros here, bridge-dead -> deadman on the Pi.

Stdlib sockets only (no websocket dependency).
"""
import json
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('pi_ip', '10.140.43.80')  # SET to the robot Pi's LAN IP
        self.declare_parameter('pi_port', 31416)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('stale_timeout', 0.3)       # s; older -> send zeros
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        self.pi_ip = self.get_parameter('pi_ip').value
        self.pi_port = int(self.get_parameter('pi_port').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        rate = float(self.get_parameter('rate').value)
        topic = self.get_parameter('cmd_vel_topic').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.v = 0.0
        self.w = 0.0
        self.last_rx = None  # rclpy Time of last /cmd_vel

        self.create_subscription(Twist, topic, self._cb, 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"cmd_vel_bridge: {topic} -> {self.pi_ip}:{self.pi_port} at {rate:.0f} Hz "
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
