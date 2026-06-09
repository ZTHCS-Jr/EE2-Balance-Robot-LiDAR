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
# This flips the scan's HANDEDNESS (a mirror). Wrong choice => the built map is a
# MIRROR IMAGE of the real room (scan still self-consistent, so translation looks
# fine -- only rotation/chirality reveals it). 2026-06: False gave a mirrored map AND
# rotation doubling (the mirrored scan fights the +CCW gyro) -> set True, the correct
# CW(LD19)->CCW(REP-103) conversion. NB: the "rotate CCW, scan sweeps CW" check is a
# red herring -- in a robot-fixed view the scan SHOULD sweep opposite the body. The
# real test: the map must NOT be mirrored. (True's old doubling was scan latency,
# now handled by SCAN_LATENCY below -- not a direction problem.)
REVERSE_SCAN_DIRECTION = True
# Sign of the gyro yaw rate so that CCW rotation is POSITIVE (REP-103).
# Verify: rotate the robot CCW; /imu/data angular_velocity.z should be > 0.
# If it reads negative, flip this to -1.0.
GYRO_SIGN = 1.0
# Gyro yaw-rate SCALE. The MPU6050 rate sensitivity is a few % off, so the EKF heading
# rotates a bit faster/slower than the body -> on a pure in-place spin the map walls fan
# out (rotated doubling) even at low speed. Calibrate with this at 1.0:
#   ros2 run tf2_ros tf2_echo odom base_link        # prints yaw (RPY) in degrees
# rotate the robot a precise known angle (e.g. 180 deg against a wall, or several 90 deg
# steps and sum), read the reported yaw change, then set:
#   GYRO_SCALE = true_angle / reported_angle        # >1 if it UNDER-rotates, <1 if over
# 2026-06 calib: spun to a believed 180 deg, measured 210 deg actual -> 210/180 = 1.167
# (gyro under-reports ~14%; heading is gyro-only, so this directly de-doubles spins).
GYRO_SCALE = 1.167
# Pitch scan-gate: drop /scan whenever the balancing body is tilted more than this
# (rad). A tilted 2D lidar plane measures walls at the wrong range and smears the map.
PITCH_GATE = 0.09  # ~5 deg
# Near-range self-hit clip (m): the LiDAR sees the robot's own structure (a fixed return
# ~0.13 m out at a CONSTANT bearing). Drop returns closer than this so they never reach
# slam -- otherwise slam smears a ring of phantom obstacles around the robot as it turns,
# boxing the explorer in ("no reachable frontiers"). Keep it above the self-structure but
# below real obstacles; the explorer avoids real walls well beyond it.
SELF_CLIP_RANGE = 0.10
# Scan timestamp back-dating (s). The Pi accumulates a full LD19 revolution
# (~100 ms) before sending, so by arrival the gyro/EKF heading has already
# rotated past where the scan was actually captured -> slam places the scan
# over-rotated and walls "overshoot then snap back", doubling on fast turns.
# Stamping the scan this far in the PAST makes slam look up the heading the
# robot really had mid-sweep, cancelling the overshoot. Tune: if walls still
# lead the turn, raise it; if they now lag, lower it. Set 0.0 to disable.
SCAN_LATENCY = 0.0  # DISABLED for now: Pi JPEG encode is threaded so real lag is small;
                    # an over-large back-date skews the map on turns. Re-tune only if
                    # rotation doubling returns (measure spin-stop lag, set SCAN_LATENCY=that).
# ---------------------------------------------------------------------------------


class UDPLidarNode(Node):
    def __init__(self):
        super().__init__('udp_lidar_receiver')

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        # Raw wheel counts for the wheel_odometry_node. JointState carries a stamp,
        # so the timestamp travels with the data.
        self.wheel_pub = self.create_publisher(JointState, 'wheel/joint_states', 10)

        self.latest_pitch = 0.0   # latest body pitch (rad) from ODOM; gates /scan

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
                    # Lidar packets carry no ESP32 timestamp. Back-date the stamp by
                    # SCAN_LATENCY so the scan lines up with the heading the robot
                    # had mid-sweep, not the over-rotated heading at arrival (see top).
                    stamp = (self.get_clock().now() -
                             Duration(seconds=SCAN_LATENCY)).to_msg()
                    self.publish_laserscan(incoming_json["lidar"], stamp)

                if "odom" in incoming_json:
                    self.publish_odom(incoming_json["odom"])

            except BlockingIOError:
                break  # Queue is empty, exit loop
            except Exception as e:
                self.get_logger().warning(f"UDP Error: {e}")
                break

    def publish_laserscan(self, scanData, current_time):
        # Pitch scan-gate: a tilted balancing body throws the 2D scan plane off, so
        # skip this scan entirely rather than feed slam/nav a smeared one.
        if abs(self.latest_pitch) > PITCH_GATE:
            return

        msg = LaserScan()
        msg.header.stamp = current_time
        msg.header.frame_id = 'laser_frame'

        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / 360.0
        msg.range_min = SELF_CLIP_RANGE  # drop self-hits closer than the robot's own structure
        msg.range_max = 8.0  # 8 m (LD19 hardware does 12 m); far returns pin down rotation

        ranges = [float('inf')] * 360

        for deg_str, data_pair in scanData.items():
            deg = int(deg_str)
            if 0 <= deg < 360:
                try:
                    r = float(data_pair[1]) / 1000.0
                    if r < SELF_CLIP_RANGE:
                        continue  # self-hit / too-close -> leave as inf (no return)
                    idx = (360 - deg) % 360 if REVERSE_SCAN_DIRECTION else deg
                    ranges[idx] = r
                except (IndexError, TypeError):
                    continue

        msg.ranges = ranges
        self.scan_pub.publish(msg)

    def publish_odom(self, odom):
        """One consolidated ESP32 packet -> raw wheel counts + gyro yaw rate.

        Packet: {"t_ms", "left_steps", "right_steps", "yaw_rate", "pitch"}.
        Stamped with ROS-now -- the SAME clock as /scan and slam's map->odom -- so the
        whole TF tree is time-consistent. nav2's strict tf2 MessageFilter requires this;
        an earlier ESP32-anchored stamp put odom->base_link on a different timeline,
        which slam tolerated but nav2 saw as a ~1.2 s `Transform data too old`. The
        embedded t_ms is no longer used for stamping; wheel_odometry_node derives dt
        from these ROS-now stamps (its dt<=0 guard handles same-tick packets).
        """
        stamp = self.get_clock().now().to_msg()
        self.latest_pitch = float(odom.get("pitch", 0.0))  # gates /scan (see PITCH_GATE)

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
        imu.angular_velocity.z = float(odom["yaw_rate"]) * GYRO_SIGN * GYRO_SCALE
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
