import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
import socket
import json
import math

# --- Empirical conventions: confirm with the Step 0 diagnostic, flip if needed ---
# LD19 reports angle clockwise; ROS LaserScan expects counter-clockwise (REP-103).
# A mirrored scan inverts rotation and fights the gyro heading -> doubled walls.
# Verify: rotate the robot CCW; /scan features should sweep CCW. If not, flip this.
REVERSE_SCAN_DIRECTION = True
# Sign of the gyro yaw so that CCW rotation is POSITIVE (REP-103).
# Verify: rotate exactly 90 deg CCW; /imu/data yaw should read ~ +1.57. If it
# reads ~ -1.57, flip this to -1.0.
GYRO_SIGN = 1.0
# ---------------------------------------------------------------------------------


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
        msg.range_min = 0.1  # 0.1 m minimum
        msg.range_max = 8.0  # 8 m (LD19 hardware does 12 m); far returns pin down rotation

        ranges = [float('inf')] * 360

        for deg_str, data_pair in scanData.items():
            deg = int(deg_str)
            if 0 <= deg < 360:
                try:
                    radius_mm = data_pair[1]
                    idx = (360 - deg) % 360 if REVERSE_SCAN_DIRECTION else deg
                    ranges[idx] = float(radius_mm) / 1000.0
                except (IndexError, TypeError):
                    continue

        msg.ranges = ranges
        self.scan_pub.publish(msg)

    def publish_imu(self, imu_data, current_time):
        msg = Imu()
        msg.header.stamp = current_time
        msg.header.frame_id = 'base_link'
        
        # The Pi sends absolute integrated yaw + the raw rate, both in rad / rad/s.
        yaw = float(imu_data.get("yaw", 0.0)) * GYRO_SIGN
        rate = float(imu_data.get("rate", 0.0)) * GYRO_SIGN

        # Absolute heading as a 2D quaternion so the EKF can actually fuse yaw.
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = rate

        # Covariances: element 0 must NOT be -1 -- that flag tells robot_localization
        # to ignore the WHOLE orientation / angular-velocity vector (this is why the
        # old config never actually fused the gyro). Mark roll/pitch unknown (large),
        # give yaw a real, tight variance.
        msg.orientation_covariance[0] = 1e6   # roll unknown (not fused)
        msg.orientation_covariance[4] = 1e6   # pitch unknown (not fused)
        msg.orientation_covariance[8] = 0.01  # yaw variance

        msg.angular_velocity_covariance[0] = 1e6    # roll rate unknown (not fused)
        msg.angular_velocity_covariance[4] = 1e6    # pitch rate unknown (not fused)
        msg.angular_velocity_covariance[8] = 0.002  # yaw-rate variance

        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UDPLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()