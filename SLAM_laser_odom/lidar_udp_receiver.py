import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import json
import math

class UDPLidarNode(Node):
    def __init__(self):
        super().__init__('udp_lidar_receiver')
        
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        
        self.UDP_PORT = 31415
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 0.0.0.0 to listens to all incoming network traffic
        self.sock.bind(('0.0.0.0', self.UDP_PORT)) 
        self.sock.setblocking(False) 
        
        self.timer = self.create_timer(0.05, self.timer_callback) # 20hz checking
        self.get_logger().info(f"Listening on UDP port {self.UDP_PORT}...")

    def timer_callback(self):
        try:
            data, addr = self.sock.recvfrom(65536) 
            scanData = json.loads(data.decode('utf-8'))
            
            self.publish_laserscan(scanData)
            
        except BlockingIOError:
            pass # wait if no data received
        except Exception as e:
            self.get_logger().warning(f"UDP Error: {e}")

    def publish_laserscan(self, scanData):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / 360.0
        msg.range_min = 0.02 # min 2cm limit
        msg.range_max = 2.0  # custom max 2m limit (hardware limit is 12m)
        
        # array initialized to infinity
        ranges = [float('inf')] * 360
        
        # data_pair contains [angle, radius] arriving from the Pi
        for deg_str, data_pair in scanData.items():
            deg = int(deg_str)
            if 0 <= deg < 360:
                try:
                    radius_mm = data_pair[1] 
                    ranges[deg] = float(radius_mm) / 1000.0 # Convert mm to meters
                except (IndexError, TypeError):
                    continue # Skip if data format is corrupt
                
        msg.ranges = ranges
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UDPLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()