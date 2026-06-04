import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
import socket
import json
import math

class UDPLidarNode(Node):
    def __init__(self):
        super().__init__('udp_lidar_receiver')
        
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        self.UDP_PORT = 31415
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 0.0.0.0 listens to all incoming network traffic
        self.sock.bind(('0.0.0.0', self.UDP_PORT)) 
        self.sock.setblocking(False) 
        
        self.timer = self.create_timer(0.01, self.timer_callback) # 20hz checking
        self.get_logger().info(f"Listening on UDP port {self.UDP_PORT}...")

    def timer_callback(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(65536) 
                incoming_json = json.loads(data.decode('utf-8'))
                
                # same timestamp for sensors for perfect fusion
                current_time = self.get_clock().now().to_msg()
                
                if "lidar" in incoming_json:
                    self.publish_laserscan(incoming_json["lidar"], current_time)
                    
                if "imu" in incoming_json:
                    self.publish_imu(incoming_json["imu"], current_time)
                
            except BlockingIOError:
                break # Queue is empty, exit loop
            except Exception as e:
                self.get_logger().warning(f"UDP Error: {e}")
                break

    def publish_laserscan(self, scanData, current_time):
        msg = LaserScan()
        msg.header.stamp = current_time
        msg.header.frame_id = 'laser_frame'
        
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / 360.0
        msg.range_min = 0.1 # min 2cm limit
        msg.range_max = 2.0  # custom max 2m limit
        
        ranges = [float('inf')] * 360
        
        for deg_str, data_pair in scanData.items():
            deg = int(deg_str)
            if 0 <= deg < 360:
                try:
                    radius_mm = data_pair[1] 
                    ranges[deg] = float(radius_mm) / 1000.0 
                except (IndexError, TypeError):
                    continue 
                
        msg.ranges = ranges
        self.scan_pub.publish(msg)

    def publish_imu(self, imu_data, current_time):
        msg = Imu()
        msg.header.stamp = current_time
        msg.header.frame_id = 'base_link'
        
        raw_gyro = float(imu_data.get("gyro_x", 0.0))
        
        print(f"GYRO: {raw_gyro:.4f}")
        
        # deadzone
        if abs(raw_gyro) < 0.06:
            raw_gyro = 0.0
            
        # map ros z axiss
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = -raw_gyro # (Keep the negative if inverted!)
        
        # EKF trust z axis
        msg.angular_velocity_covariance[0] = -1.0 
        msg.angular_velocity_covariance[4] = -1.0 
        msg.angular_velocity_covariance[8] = 0.01 
        
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0

        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UDPLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()