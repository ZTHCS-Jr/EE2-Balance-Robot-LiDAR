#!/usr/bin/env python3
"""Lightweight frontier explorer for autonomous mapping.

Reads slam_toolbox's /map and the lidar /scan, finds frontiers (free cells next to
unknown space), greedily drives toward the nearest one with simple reactive
obstacle avoidance, and publishes /cmd_vel. Deliberately simple (no global planner)
and conservative -- suited to a slow balancing robot driven over WiFi.

Safety: gated by the /explore/enable service (std_srvs/SetBool), DEFAULT OFF. While
disabled it commands zero velocity. Combined with the bridge's stale->zero behaviour
and the Pi-side deadman, there are three independent ways the robot stops.

    ros2 service call /explore/enable std_srvs/srv/SetBool "{data: true}"   # start
    ros2 service call /explore/enable std_srvs/srv/SetBool "{data: false}"  # stop
"""
import math

import numpy as np
from scipy import ndimage

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Motion limits (conservative for a balancing robot).
        self.declare_parameter('control_rate', 5.0)
        self.declare_parameter('v_fwd', 0.07)       # m/s forward creep (slow -> less scan smear)
        self.declare_parameter('w_max', 0.2)        # rad/s max turn (slow -> less rotational smear)
        self.declare_parameter('w_gain', 0.8)       # heading P-gain (gentle -> small abrupt turn steps)
        self.declare_parameter('turn_sign', -1.0)   # physical turn dir: -1 if +angular.z turns the robot RIGHT (matches face_seeker)
        self.declare_parameter('heading_tol', 0.4)  # rad; above -> rotate in place
        self.declare_parameter('goal_tol', 0.2)     # m; frontier considered reached
        self.declare_parameter('settle_time', 0.7)  # s; hold still after a turn for a clean scan

        # Obstacle avoidance (front sector only). Kept tight so a sub-metre corridor's
        # SIDE walls don't read as a frontal obstacle (which made the robot rock in place).
        self.declare_parameter('stop_dist', 0.25)       # m; front-sector obstacle -> stop & turn
                                                         # (must be > receiver SELF_CLIP_RANGE 0.20)
        self.declare_parameter('front_angle', 0.5)      # rad half-width of front sector (~29 deg)
        # Wide hard-stop arc: catches a head-on wall even if the lidar's forward is a little
        # misaligned. (These two were referenced but never declared in the original code.)
        self.declare_parameter('emergency_angle', 1.0)  # rad half-width of the wide hard-stop arc
        self.declare_parameter('emergency_dist', 0.35)   # m; nearest in wide arc -> stop & turn
                                                         # (must be > receiver SELF_CLIP_RANGE 0.20)
        self.declare_parameter('scan_forward_offset', 0.0)  # rad; scan angle that points along the
                                                            # robot's +x. Set this if the lidar's 0 deg
                                                            # is not mounted facing forward.
        self.declare_parameter('debug', True)           # log front/near range + nearest bearing

        # Frontier filtering.
        self.declare_parameter('min_frontier_size', 5)   # cells
        self.declare_parameter('blacklist_radius', 0.3)  # m

        # Stuck detection.
        self.declare_parameter('stuck_dist', 0.05)  # m progress over...
        self.declare_parameter('stuck_time', 5.0)   # ...this many s -> escape

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        g = self.get_parameter
        self.v_fwd = g('v_fwd').value
        self.w_max = g('w_max').value
        self.w_gain = g('w_gain').value
        self.turn_sign = g('turn_sign').value
        self.heading_tol = g('heading_tol').value
        self.goal_tol = g('goal_tol').value
        self.settle_time = g('settle_time').value
        self.stop_dist = g('stop_dist').value
        self.front_angle = g('front_angle').value
        self.emergency_angle = g('emergency_angle').value
        self.emergency_dist = g('emergency_dist').value
        self.scan_forward_offset = g('scan_forward_offset').value
        self.debug = g('debug').value
        self.min_frontier_size = int(g('min_frontier_size').value)
        self.blacklist_radius = g('blacklist_radius').value
        self.stuck_dist = g('stuck_dist').value
        self.stuck_time = g('stuck_time').value
        self.map_frame = g('map_frame').value
        self.base_frame = g('base_frame').value
        self.control_rate = g('control_rate').value

        self.enabled = False
        self.map = None
        self.scan = None
        self.target = None             # (x, y) in map frame
        self.blacklist = []            # [(x, y), ...]
        self._done_logged = False
        self._escape_ticks = 0
        self._settle_ticks = 0
        self._avoid_dir = 0.0          # latched turn direction while avoiding (anti-rock hysteresis)
        self._was_rotating = False
        self._progress_pose = None
        self._progress_time = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(OccupancyGrid, 'map', self._map_cb, 1)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 1)
        self.create_service(SetBool, 'explore/enable', self._enable_cb)
        self.create_timer(1.0 / self.control_rate, self._control_step)

        self.get_logger().info("frontier_explorer ready (DISABLED). "
                               "Enable: ros2 service call /explore/enable std_srvs/srv/SetBool \"{data: true}\"")

    # --- callbacks ---
    def _map_cb(self, msg):
        self.map = msg

    def _scan_cb(self, msg):
        self.scan = msg

    def _enable_cb(self, req, resp):
        self.enabled = req.data
        if not self.enabled:
            self.target = None
            self._publish(0.0, 0.0)
        else:
            self._done_logged = False
            self._progress_pose = None
            self._progress_time = self.get_clock().now()
        resp.success = True
        resp.message = "exploration " + ("enabled" if self.enabled else "disabled")
        self.get_logger().info(resp.message)
        return resp

    # --- helpers ---
    def _publish(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        # turn_sign maps our +w (CCW per ROS) to the robot's actual turn direction
        w = self.turn_sign * w
        t.angular.z = float(max(-self.w_max, min(self.w_max, w)))
        self.cmd_pub.publish(t)

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        return (tf.transform.translation.x, tf.transform.translation.y, yaw)

    def _front_obstacle(self):
        """Inspect /scan ahead of the robot.

        Returns (front_min, near_min, open_dir, nearest_bearing):
          front_min       -- nearest range within +/-front_angle of forward (normal slow/turn)
          near_min        -- nearest range within +/-emergency_angle (wide hard-stop arc; catches
                             head-on walls even if the lidar forward is a little misaligned)
          open_dir        -- +1 (left/CCW) or -1 (right) toward more open space (ROS convention;
                             _publish applies turn_sign to convert to the robot's physical direction)
          nearest_bearing -- bearing of the closest return in the wide arc (rad; 0 = forward)
        scan_forward_offset shifts the angle reference if the lidar 0 deg is not along base_link +x.
        """
        s = self.scan
        n = len(s.ranges)
        if n == 0:
            return math.inf, math.inf, 1.0, 0.0
        ang = s.angle_min + np.arange(n) * s.angle_increment - self.scan_forward_offset
        ang = (ang + math.pi) % (2.0 * math.pi) - math.pi  # wrap to [-pi, pi], 0 = robot forward
        r = np.array(s.ranges, dtype=float)
        valid = np.isfinite(r) & (r > s.range_min)

        front = valid & (np.abs(ang) < self.front_angle)
        front_min = r[front].min() if np.any(front) else math.inf

        near = valid & (np.abs(ang) < self.emergency_angle)
        if np.any(near):
            idx = np.where(near)[0]
            j = idx[np.argmin(r[idx])]
            near_min, nearest_bearing = r[j], ang[j]
        else:
            near_min, nearest_bearing = math.inf, 0.0

        # Decide turn direction: compare clearance on left (+) vs right (-).
        left = valid & (ang > 0.2) & (ang < 1.6)
        right = valid & (ang < -0.2) & (ang > -1.6)
        left_clear = r[left].mean() if np.any(left) else 0.0
        right_clear = r[right].mean() if np.any(right) else 0.0
        open_dir = 1.0 if left_clear >= right_clear else -1.0  # +1 = turn left (CCW)
        return front_min, near_min, open_dir, nearest_bearing

    def _path_clear(self, p0, p1, info, occ):
        """True if the straight line p0->p1 (world m) crosses no occupied cell."""
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        H, W = occ.shape
        steps = max(1, int(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / res))
        for i in range(steps + 1):
            t = i / steps
            col = int((p0[0] + t * (p1[0] - p0[0]) - ox) / res)
            row = int((p0[1] + t * (p1[1] - p0[1]) - oy) / res)
            if 0 <= row < H and 0 <= col < W and occ[row, col]:
                return False
        return True

    def _select_frontier(self, pose):
        m = self.map
        info = m.info
        res = info.resolution
        W, H = info.width, info.height
        grid = np.array(m.data, dtype=np.int16).reshape(H, W)

        free = grid == 0
        unknown = grid == -1
        occ = grid >= 50               # occupied cells (walls)
        unknown_dil = ndimage.binary_dilation(unknown)
        frontier = free & unknown_dil
        if not np.any(frontier):
            return None

        labeled, n = ndimage.label(frontier)
        if n == 0:
            return None
        sizes = ndimage.sum(frontier, labeled, range(1, n + 1))
        centroids = ndimage.center_of_mass(frontier, labeled, range(1, n + 1))  # (row, col)

        best = None
        best_d = math.inf
        for size, (row, col) in zip(sizes, centroids):
            if size < self.min_frontier_size:
                continue
            wx = info.origin.position.x + (col + 0.5) * res
            wy = info.origin.position.y + (row + 0.5) * res
            if self._blacklisted(wx, wy):
                continue
            # Only pursue frontiers reachable in a straight line (this greedy explorer
            # has no global planner, so don't aim a goal through a wall).
            if not self._path_clear((pose[0], pose[1]), (wx, wy), info, occ):
                continue
            d = math.hypot(wx - pose[0], wy - pose[1])
            if d < best_d:
                best_d, best = d, (wx, wy)
        return best

    def _blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < self.blacklist_radius for bx, by in self.blacklist)

    def _update_stuck(self, pose):
        now = self.get_clock().now()
        if self._progress_pose is None:
            self._progress_pose = pose
            self._progress_time = now
            return False
        if math.hypot(pose[0] - self._progress_pose[0], pose[1] - self._progress_pose[1]) > self.stuck_dist:
            self._progress_pose = pose
            self._progress_time = now
            return False
        return (now - self._progress_time) > Duration(seconds=self.stuck_time)

    # --- main loop ---
    def _control_step(self):
        if not self.enabled:
            return
        if self.map is None or self.scan is None:
            self._publish(0.0, 0.0)
            return
        pose = self._robot_pose()
        if pose is None:
            self._publish(0.0, 0.0)
            self.get_logger().warning("no map->base_link TF yet", throttle_duration_sec=2.0)
            return

        # Escape maneuver (rotate in place) after being stuck.
        if self._escape_ticks > 0:
            self._escape_ticks -= 1
            self._publish(0.0, self.w_max)
            if self._escape_ticks == 0:
                self.target = None
                self._avoid_dir = 0.0
                self._settle_ticks = max(1, int(self.settle_time * self.control_rate))
                self._progress_pose = None
                self._progress_time = self.get_clock().now()
            return

        # Settle after a turn: hold still briefly so slam gets a clean, undistorted
        # scan before translating (cheap fix for rotational scan smear / doubled walls).
        if self._settle_ticks > 0:
            self._settle_ticks -= 1
            self._publish(0.0, 0.0)
            return

        # Reactive obstacle avoidance takes priority.
        front_min, near_min, open_dir, bearing = self._front_obstacle()
        if self.debug:
            self.get_logger().info(
                f"front_min={front_min:.2f}m near_min={near_min:.2f}m "
                f"nearest_bearing={math.degrees(bearing):+.0f}deg "
                f"(0=forward; if a wall you're driving into shows a large bearing, "
                f"set scan_forward_offset)", throttle_duration_sec=1.0)
        if near_min < self.emergency_dist or front_min < self.stop_dist:
            # Latch the turn direction on entering avoidance and hold it until the path
            # clears, so we don't oscillate left/right (rock) in a symmetric corridor.
            if self._avoid_dir == 0.0:
                self._avoid_dir = open_dir
            self._publish(0.0, self._avoid_dir * self.w_max)
            return
        self._avoid_dir = 0.0   # path ahead is clear

        # Pick / refresh a frontier target.
        if self.target is None or self._blacklisted(*self.target):
            self.target = self._select_frontier(pose)
        if self.target is None:
            self._publish(0.0, 0.0)
            if not self._done_logged:
                self.get_logger().info("exploration complete: no reachable frontiers")
                self._done_logged = True
            return

        # Stuck? blacklist this target and rotate to escape.
        if self._update_stuck(pose):
            self.get_logger().info(f"stuck near target {self.target}; blacklisting & escaping")
            self.blacklist.append(self.target)
            self.target = None
            self._escape_ticks = max(1, int(2.0 * self.control_rate))
            return

        # Go-to-goal.
        dx = self.target[0] - pose[0]
        dy = self.target[1] - pose[1]
        dist = math.hypot(dx, dy)
        if dist < self.goal_tol:
            self.target = None  # reached; pick a new one next tick
            self._publish(0.0, 0.0)
            return

        yaw_err = normalize_angle(math.atan2(dy, dx) - pose[2])
        if abs(yaw_err) > self.heading_tol:
            # Rotate in place to face the target.
            self._was_rotating = True
            self._publish(0.0, self.w_gain * yaw_err)
            return
        if self._was_rotating:
            # Just finished a turn -> settle for a clean scan before driving.
            self._was_rotating = False
            self._settle_ticks = max(1, int(self.settle_time * self.control_rate))
            self._publish(0.0, 0.0)
            return
        # Aligned: creep forward with gentle heading correction.
        self._publish(self.v_fwd, self.w_gain * yaw_err)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
