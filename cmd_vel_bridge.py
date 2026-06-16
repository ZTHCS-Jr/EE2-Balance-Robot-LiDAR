#!/usr/bin/env python3

# Converts nav2 vel control to commands to the Pi
# Subscribes to twist and sends to Pi over UDP to a different port
import json
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        
        self.pi_ip = '192.168.0.196'
        self.pi_port = 31416
        self.stale_timeout = float(1) # Timeout count down
        rate = float(20.0) # Transmission rate
        topic = 'cmd_vel'

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.v = 0.0
        self.w = 0.0
        self.last_rx = None  # rclpy Time of last /cmd_vel

        self.create_subscription(Twist, topic, self._cb, 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"cmd_vel_bridge: {topic} -> {self.pi_ip}:{self.pi_port} at {rate:.0f} Hz ")

    # Subscriber callback for Twist msg
    def _cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_rx = self.get_clock().now()

    # Sends data on every timer tick (depends on rate)
    def _tick(self):
        current = (self.last_rx is not None and (self.get_clock().now() - self.last_rx).nanoseconds < self.stale_timeout * 1e9)
        
        if current:
            v, w = (self.v, self.w) 
        else:
            v, w = (0.0, 0.0)
        try:
            self.sock.sendto(json.dumps({"v": v, "w": w}).encode("utf-8"), (self.pi_ip, self.pi_port))
        except OSError as e:
            self.get_logger().warning(f"UDP send failed: {e}")

    def stop(self):
        for _ in range(3):
            try:
                self.sock.sendto(json.dumps({"v": 0.0, "w": 0.0}).encode("utf-8"), (self.pi_ip, self.pi_port))
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
