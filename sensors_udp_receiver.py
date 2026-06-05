import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import socket
import json
import math
import time

class UDPSensorNode(Node):
    def __init__(self):
        super().__init__('udp_sensor_receiver')
        
        self.lidar_pub_ = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_pub_ = self.create_publisher(Imu, 'imu/data', 10)
        self.odom_pub_ = self.create_publisher(Odometry, 'wheel/odometry', 10)
        
        self.UDP_PORT = 31415
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 0.0.0.0 listens to all incoming network traffic
        self.sock.bind(('0.0.0.0', self.UDP_PORT)) 
        self.sock.setblocking(False) 
        
        # robot physical parameters - can adjust
        self.WHEEL_RADIUS = 0.0325
        self.TRACK_WIDTH = 0.12
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_left_rad = 0.0
        self.last_right_rad = 0.0
        self.last_time = time.time()
        
        self.timer = self.create_timer(0.05, self.timer_callback) 
        self.get_logger().info(f"Listening for LiDAR, IMU, and Odom on UDP port {self.UDP_PORT}...")

    def timer_callback(self):
            while True:
                try:
                    data, addr = self.sock.recvfrom(65536) 
                    parsed_data = json.loads(data.decode('utf-8'))
                    
                    if 'lidar' in parsed_data:
                        self.publish_laserscan(parsed_data['lidar'])
                    elif 'imu' in parsed_data:
                        self.publish_imu(parsed_data['imu'])
                    elif 'odom' in parsed_data:
                        self.publish_odometry(parsed_data['odom'])
                        
                except BlockingIOError:
                    break
                except Exception as e:
                    pass # fail silently on malformed packets

    def publish_odometry(self, odom_data):
        current_time = time.time()
        dt = current_time - self.last_time
        # if getSpeed() in sender:
        # left_rad = odom_data.get('left_rad', 0.0)
        # right_rad = odom_data.get('right_rad', 0.0)
        
        # d_left_rad = left_rad - self.last_left_rad
        # d_right_rad = right_rad - self.last_right_rad
        
        # d_left = d_left_rad * self.WHEEL_RADIUS
        # d_right = d_right_rad * self.WHEEL_RADIUS
        
        # if getPosition in sender:
        left_steps = odom_data.get('left_steps', 0)
        right_steps = odom_data.get('right_steps', 0)
        
        # steps to radians
        # 200 steps/rev * 16 microsteps = 3200 steps per full rotation
        STEPS_PER_REV = 3200.0
        left_rad = (left_steps / STEPS_PER_REV) * 2.0 * math.pi
        right_rad = (right_steps / STEPS_PER_REV) * 2.0 * math.pi
        
        # how much the wheels turned since last tick
        d_left_rad = left_rad - self.last_left_rad
        d_right_rad = right_rad - self.last_right_rad
        
        # physical distance each wheel traveled this tick
        d_left = d_left_rad * self.WHEEL_RADIUS
        d_right = d_right_rad * self.WHEEL_RADIUS

        # drive kinematics
        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.TRACK_WIDTH
        
        # update x, y, theta
        self.x += d_center * math.cos(self.th + (d_theta / 2.0))
        self.y += d_center * math.sin(self.th + (d_theta / 2.0))
        self.th += d_theta
        
        # velocity calc
        v_x = d_center / dt if dt > 0 else 0.0
        v_th = d_theta / dt if dt > 0 else 0.0
        
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        # convert theta to quaternion
        msg.pose.pose.orientation.z = math.sin(self.th / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.th / 2.0)
        
        msg.twist.twist.linear.x = v_x
        msg.twist.twist.angular.z = v_th
        
        self.odom_pub_.publish(msg)

        self.last_left_rad = left_rad
        self.last_right_rad = right_rad
        self.last_time = current_time

    def publish_laserscan(self, scanData):
        msg = LaserScan()
        msg.header.stamp = current_time
        msg.header.frame_id = 'laser_frame'
        
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / 360.0
        msg.range_min = 0.02 # min 2cm limit
        msg.range_max = 2.0  # custom max 2m limit
        
        ranges = [float('inf')] * 360
        
        for deg_str, data_pair in scanData.items():
            deg = int(deg_str)
            if 0 <= deg < 360:
                try:
                    radius_mm = data_pair[1] 
                    mirrored_deg = 359 - deg
                    ranges[mirrored_deg] = float(radius_mm) / 1000.0 
                except (IndexError, TypeError):
                    continue 
                
        msg.ranges = ranges
        self.lidar_pub_.publish(msg)

def publish_imu(self, imu_data):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        raw_rate = float(imu_data.get("gyro_x", 0.0))
        print(f"GYRO RATE: {raw_rate:.4f}")
            
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = raw_rate 
        
        msg.angular_velocity_covariance[0] = -1.0 
        msg.angular_velocity_covariance[4] = -1.0 
        msg.angular_velocity_covariance[8] = 0.01 
        
        msg.orientation_covariance[0] = -1.0

        self.imu_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UDPSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()