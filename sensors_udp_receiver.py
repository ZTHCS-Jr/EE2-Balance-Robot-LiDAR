import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan, Imu, JointState
import socket
import json
import math

# --- Empirical conventions: confirm with the Step 0 diagnostic, flip if needed ---
# LD19 reports angle clockwise; ROS LaserScan expects counter-clockwise (REP-103).
# A mirrored scan inverts rotation and fights the gyro heading -> doubled walls.
# Verify: rotate the robot CCW; /scan features should sweep CCW. If not, flip this.
REVERSE_SCAN_DIRECTION = True
# Sign of the gyro yaw rate so that CCW rotation is POSITIVE (REP-103).
# Verify: rotate the robot CCW; /imu/data angular_velocity.z should be > 0.
# If it reads negative, flip this to -1.0.
GYRO_SIGN = 1.0
# ---------------------------------------------------------------------------------


class UDPLidarNode(Node):
    def __init__(self):
        super().__init__('udp_lidar_receiver')

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        # Raw wheel counts for the wheel_odometry_node. JointState carries a stamp,
        # so the timestamp travels with the data.
        self.wheel_pub = self.create_publisher(JointState, 'wheel/joint_states', 10)

        # ESP32-millis() -> ROS-time anchor, set on the first odom packet. Stamping
        # wheel/IMU messages with the embedded ESP32 clock (rather than arrival time)
        # lets the wheel odometry node derive dt immune to UDP/processing jitter and
        # robust to dropped packets (gaps simply show as a larger dt; the absolute
        # step counts self-heal across drops). millis() wraps after ~49.7 days, which
        # is far beyond any mapping session, so wrap handling is omitted.
        self._t0_ros = None       # rclpy Time captured at the first odom packet
        self._t0_esp_ms = None    # ESP32 millis() value at that first packet

        self.UDP_PORT = 31415
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 0.0.0.0 listens to all incoming network traffic
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
                    # Lidar packets carry no ESP32 timestamp; stamp on arrival.
                    self.publish_laserscan(incoming_json["lidar"],
                                           self.get_clock().now().to_msg())

                if "odom" in incoming_json:
                    self.publish_odom(incoming_json["odom"])

            except BlockingIOError:
                break  # Queue is empty, exit loop
            except Exception as e:
                self.get_logger().warning(f"UDP Error: {e}")
                break

    def _esp_stamp(self, t_ms):
        """Map an ESP32 millis() value onto ROS time via a fixed first-packet anchor."""
        if self._t0_ros is None:
            self._t0_ros = self.get_clock().now()
            self._t0_esp_ms = t_ms
        elapsed_ns = int((t_ms - self._t0_esp_ms) * 1_000_000)
        return (self._t0_ros + Duration(nanoseconds=elapsed_ns)).to_msg()

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

    def publish_odom(self, odom):
        """One consolidated ESP32 packet -> raw wheel counts + gyro yaw rate.

        Packet: {"t_ms", "left_steps", "right_steps", "yaw_rate"}.
        Both messages are stamped with the ESP32 clock so downstream dt is accurate.
        """
        stamp = self._esp_stamp(int(odom["t_ms"]))

        # Raw signed microstep counts -> /wheel/joint_states (geometry applied later
        # by the parameterized wheel_odometry_node).
        js = JointState()
        js.header.stamp = stamp
        js.name = ['left_wheel', 'right_wheel']
        js.position = [float(odom["left_steps"]), float(odom["right_steps"])]
        self.wheel_pub.publish(js)

        # Gyro yaw rate -> /imu/data (frame imu_link). Only angular_velocity.z is
        # trusted. orientation_covariance[0] = -1 tells robot_localization to ignore
        # the absolute orientation entirely (we fuse the *rate*, not absolute yaw, so
        # heading cannot inherit the MPU's integration drift). linear acceleration is
        # likewise flagged unused (a balancing body leaks gravity/vibration into it).
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'

        imu.orientation_covariance[0] = -1.0  # no absolute orientation provided

        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = float(odom["yaw_rate"]) * GYRO_SIGN
        imu.angular_velocity_covariance[0] = 1e6    # roll rate unknown (not fused)
        imu.angular_velocity_covariance[4] = 1e6    # pitch rate unknown (not fused)
        imu.angular_velocity_covariance[8] = 0.002  # yaw-rate variance

        imu.linear_acceleration_covariance[0] = -1.0  # no linear acceleration fused

        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = UDPLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
