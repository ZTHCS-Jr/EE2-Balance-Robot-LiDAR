import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan, Imu, JointState
import socket
import json
import math

# Reverses direction of laser scan
REVERSE_SCAN_DIRECTION = True
# Changes the direction of gyro reading
GYRO_SIGN = 1.0
# Scale gyro ranges since it's off sometimes (calibrate with gyro_calibrate_spin)
GYRO_SCALE = 1.167
# Ignore scanes done at greater of 5 deg tilt angle
PITCH_GATE = 0.09  # ~5 deg
# Range around robot it doesn't detect as it sees the wire around it
SELF_CLIP_RANGE = 0.15
# Had an issue where the laser scan was too delayed so I measured the delay and modified the order the scans were updated to account for this
SCAN_LATENCY = 0.0 


class UDPSensorNode(Node):
    def __init__(self):
        super().__init__('udp_lidar_receiver')

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        # Wheel odemetry is also received here and published das joint_states
        self.wheel_pub = self.create_publisher(JointState, 'wheel/joint_states', 10)

        self.latest_pitch = 0.0   # Current robot pitch

        self.UDP_PORT = 31415
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.UDP_PORT))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.01, self.timer_callback)  # drain socket at 100 Hz
        self.get_logger().info(f"Listening on UDP port {self.UDP_PORT}...")

    def timer_callback(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(65536)
                incoming_json = json.loads(data.decode('utf-8'))

                if "lidar" in incoming_json:
                    # Lidar packets carry no ESP32 timestamp. Back-date the stamp by
                    # SCAN_LATENCY so the scan lines up with the heading the robot
                    # had mid-sweep, not the over-rotated heading at arrival 
                    # NOTE: Might have fixed this issue but extra info carys :)
                    stamp = (self.get_clock().now() - Duration(seconds=SCAN_LATENCY)).to_msg()
                    self.publish_laserscan(incoming_json["lidar"], stamp)

                if "odom" in incoming_json:
                    self.publish_odom(incoming_json["odom"])

            except BlockingIOError:
                break  # Queue is empty, exit loop
            except Exception as e:
                self.get_logger().warning(f"UDP Error: {e}")
                break

    def publish_laserscan(self, scanData, current_time):
        # Ignore any scans carried out at pitches too high
        if abs(self.latest_pitch) > PITCH_GATE:
            return

        msg = LaserScan()
        msg.header.stamp = current_time
        msg.header.frame_id = 'laser_frame'

        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / 360.0
        msg.range_min = SELF_CLIP_RANGE  # drop self-hits closer than the robot's own structure
        msg.range_max = 8.0  

        ranges = [float('inf')] * 360

        for deg_str, data_pair in scanData.items():
            deg = int(deg_str)
            if 0 <= deg < 360:
                try:
                    r = float(data_pair[1]) / 1000.0
                    if r < SELF_CLIP_RANGE:
                        continue  # self-hit ignore
                    idx = (360 - deg) % 360 if REVERSE_SCAN_DIRECTION else deg
                    ranges[idx] = r
                except (IndexError, TypeError):
                    continue

        msg.ranges = ranges
        self.lidar_pub_.publish(msg)

    def publish_odom(self, odom):
        # Packet: {"t_ms", "left_steps", "right_steps", "yaw_rate", "pitch"}.
        # Every piece of data from esp32 if given the same timeframe by ROS-now which fixes a bunch of latency bugs we had so everything 
        # Operating in the same tiem frame and not delayed from each other

        stamp = self.get_clock().now().to_msg()
        self.latest_pitch = float(odom.get("pitch", 0.0))  # gates /scan (see PITCH_GATE)

        # Raw signed microstep counts (distance calculated by odom node)
        js = JointState()
        js.header.stamp = stamp
        js.name = ['left_wheel', 'right_wheel']
        js.position = [float(odom["left_steps"]), float(odom["right_steps"])]
        self.wheel_pub.publish(js)

        # Gyro yaw rate
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'

        imu.orientation_covariance[0] = -1.0  # no absolute orientation provided

        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = float(odom["yaw_rate"]) * GYRO_SIGN * GYRO_SCALE
        imu.angular_velocity_covariance[0] = 1e6    # roll rate unknown (not fused)
        imu.angular_velocity_covariance[4] = 1e6    # pitch rate unknown (not fused)
        imu.angular_velocity_covariance[8] = 0.002  # yaw-rate variance

        imu.linear_acceleration_covariance[0] = -1.0  # no linear acceleration fused

        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = UDPSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
