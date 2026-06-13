#!/usr/bin/env python3
"""Vision-driven face seeker: roam the map, approach a face, identify it, mark it.

Run alongside launcher.py (provides /scan, /map, TF) and cmd_vel_bridge (relays
/cmd_vel to the Pi). Detections come from the server's /ws/ui broadcast. Gated OFF
until: ros2 service call /seek/enable std_srvs/srv/SetBool "{data: true}"
"""
import asyncio
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

import websockets


class FaceSeeker(Node):
    def __init__(self):
        super().__init__('face_seeker')

        # variables
        self.server_ws = 'ws://10.232.9.34:8000/ws/ui'
        self.frame_w = 1000.0             # video width (bbox coord space)
        self.frame_h = 1000.0             # video height
        self.v_fwd = 0.15                 # m/s forward creep
        self.w_max = 0.5                  # rad/s max turn
        self.w_search = 0.2               # rad/s search-rotate speed (slow so faces linger)
        self.accel_v = 0.3                # m/s^2 ramp; smooths lurches (balance bot)
        self.accel_w = 1.0                # rad/s^2 ramp
        self.steer_gain = 0.8             # bbox x-error -> turn rate
        self.turn_sign = -1.0             # physical turn dir: -1 if +angular.z turns the robot RIGHT
        self.w_approach = 0.2             # rad/s max turn while approaching (low -> no lag overshoot)
        self.min_face_frac = 0.15         # ignore boxes shorter than this*frame_h (skips far/background faces)
        self.approach_face_frac = 0.6     # box this tall -> close enough (backup stop)
        self.approach_stop_dist = 0.50    # m; front obstacle -> as close as map allows
        self.identify_time = 4.0          # s; hold still & confirm reg/unknown
        self.front_angle = 0.5            # rad half-width of front sector (wander avoidance)
        self.stop_angle = 0.2             # rad half-width for approach-stop (narrow: ignore side/self hits)
        self.min_valid_range = 0.20       # m; ignore closer returns (the robot's own structure)
        self.avoid_dist = 0.30            # m; wander avoidance (> SELF_CLIP 0.20)
        self.clear_factor = 1.5           # keep turning until front > avoid_dist*this
        self.visit_radius = 0.6           # m; don't re-greet within this of a marked spot
        self.detection_stale = 1.5        # s; CPU detector is slow, tolerate gaps
        self.fresh_for_steer = 0.6        # s; only steer/commit on detections fresher than this
        self.lost_timeout = 2.5           # s; no target in APPROACH -> back to search
        self.search_rotate_time = 10.0    # s; scan-in-place before wandering
        self.wander_time = 4.0            # s; drive before scanning again
        self.move_on_turn_time = 4.0      # s; turn-away after greeting

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'people_markers', 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.create_service(SetBool, 'seek/enable', self._on_enable)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- state ---
        self.enabled = False
        self.scan = None
        self.faces_lock = threading.Lock()
        self.latest_faces = []
        self.faces_stamp = 0.0
        self.got_first_msg = False
        self.state = 'SEARCH_ROTATE'
        self.state_t0 = time.time()
        self.dt = 0.1                    # control period (10 Hz timer below)
        self.cur_v = 0.0                 # slew-limited command state
        self.cur_w = 0.0
        self.avoiding = False            # wander avoidance latch (with hysteresis)
        self.avoid_dir = 1.0
        self.id_votes = {}               # name -> count, tallied during IDENTIFY
        self.visited_names = set()       # greeted registered people (dedup by name)
        self.visited_locs = []           # greeted locations (dedup unknowns by place)
        self.markers = MarkerArray()
        self.marker_id = 0
        self.last_diag = 0.0

        threading.Thread(target=self._ws_thread, daemon=True).start()
        self.create_timer(self.dt, self._tick)
        self.get_logger().info('face_seeker ready (disabled; call /seek/enable)')

    # ---- callbacks ----
    def _on_scan(self, msg):
        self.scan = msg

    def _on_enable(self, req, resp):
        self.enabled = req.data
        self.cur_v = self.cur_w = 0.0    # hard zero, no ramp-down on disable
        self.cmd_pub.publish(Twist())
        self._enter('SEARCH_ROTATE')
        resp.success = True
        resp.message = 'seeking' if req.data else 'idle'
        self.get_logger().info(resp.message)
        return resp

    # ---- detection stream (server /ws/ui; video frames share it and are skipped) ----
    def _ws_thread(self):
        asyncio.run(self._ws_loop())

    async def _ws_loop(self):
        while True:
            try:
                async with websockets.connect(self.server_ws, max_size=None) as ws:
                    self.get_logger().info(f'detections: connected to {self.server_ws}')
                    async for msg in ws:
                        if isinstance(msg, bytes):
                            continue                      # video frame, not a detection
                        try:
                            obj = json.loads(msg)
                        except ValueError:
                            continue
                        if obj.get('type') == 'detections':
                            with self.faces_lock:
                                self.latest_faces = obj.get('faces', [])
                                self.faces_stamp = time.time()
                            if not self.got_first_msg:
                                self.got_first_msg = True
                                self.get_logger().info('detections: first message received')
            except Exception as e:
                self.get_logger().warning(f'detections ws down ({e}); retrying in 2s')
                await asyncio.sleep(2.0)

    def _faces(self):
        with self.faces_lock:
            if time.time() - self.faces_stamp > self.detection_stale:
                return []
            return list(self.latest_faces)

    # ---- helpers ----
    def _send(self, v, w):
        # turn_sign maps our +w (CCW per ROS) to the robot's actual turn direction
        w = self.turn_sign * w
        # slew-limit so the balance robot ramps instead of lurching
        w = max(-self.w_max, min(self.w_max, w))
        dv, dw = self.accel_v * self.dt, self.accel_w * self.dt
        self.cur_v += max(-dv, min(dv, v - self.cur_v))
        self.cur_w += max(-dw, min(dw, w - self.cur_w))
        t = Twist()
        t.linear.x = self.cur_v
        t.angular.z = self.cur_w
        self.cmd_pub.publish(t)

    def _enter(self, state):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}')
        self.state = state
        self.state_t0 = time.time()

    def _sector_min(self, half):
        # nearest finite return within +/-half of forward (0 rad)
        if self.scan is None:
            return float('inf')
        best = float('inf')
        a, inc = self.scan.angle_min, self.scan.angle_increment
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or r < self.min_valid_range:
                continue                      # drop self-hits / too-close returns
            ang = math.atan2(math.sin(a + i * inc), math.cos(a + i * inc))
            if abs(ang) <= half and r < best:
                best = r
        return best

    def _side_clearer(self):
        # +1 turn left (CCW) if the left side is more open, else -1
        if self.scan is None:
            return 1.0
        a, inc = self.scan.angle_min, self.scan.angle_increment
        lmin = rmin = float('inf')
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or r < self.min_valid_range:
                continue
            ang = math.atan2(math.sin(a + i * inc), math.cos(a + i * inc))
            if 0.3 < ang < 1.6:
                lmin = min(lmin, r)
            elif -1.6 < ang < -0.3:
                rmin = min(rmin, r)
        return 1.0 if lmin >= rmin else -1.0

    def _pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
            return tf.transform.translation.x, tf.transform.translation.y, yaw
        except Exception:
            return None

    def _near_visited(self):
        p = self._pose()
        if p is None:
            return False
        return any(math.hypot(p[0] - vx, p[1] - vy) < self.visit_radius
                   for vx, vy in self.visited_locs)

    def _pick_target(self, faces):
        # largest big-enough face whose name we haven't greeted yet
        best, best_area = None, 0.0
        for f in faces:
            b = f.get('bbox')
            if not b or len(b) < 4:
                continue
            h = b[3] - b[1]
            if h < self.min_face_frac * self.frame_h:
                continue
            if f.get('name') and f['name'] in self.visited_names:
                continue
            area = (b[2] - b[0]) * h
            if area > best_area:
                best, best_area = f, area
        return best

    def _x_err(self, face):
        b = face['bbox']
        cx = 0.5 * (b[0] + b[2])
        return (cx - 0.5 * self.frame_w) / (0.5 * self.frame_w)

    def _diag(self, faces):
        # 1s heartbeat: state + what it's chasing + motion, so tuning isn't blind
        now = time.time()
        if now - self.last_diag < 1.0:
            return
        self.last_diag = now
        age = now - self.faces_stamp if self.faces_stamp else -1.0
        front = self._sector_min(self.front_angle)
        tgt = self._pick_target(faces)
        if tgt is not None:
            nm = tgt.get('name') or 'unknown'
            h = (tgt['bbox'][3] - tgt['bbox'][1]) / self.frame_h
            info = f'target={nm} h={h:.2f} x_err={self._x_err(tgt):+.2f}'
        else:
            hmax = max(((f['bbox'][3] - f['bbox'][1]) / self.frame_h
                        for f in faces if f.get('bbox')), default=0.0)
            info = f'no target (faces={len(faces)} max_h={hmax:.2f} need>{self.min_face_frac:.2f})'
        self.get_logger().info(
            f'[{self.state}] {info} front={front:.2f}m det_age={age:.1f}s '
            f'v={self.cur_v:+.2f} w={self.cur_w:+.2f}')

    # ---- control loop ----
    def _tick(self):
        if not self.enabled:
            return
        if self.scan is None:
            return                      # don't move blind
        faces = self._faces()
        self._diag(faces)
        front = self._sector_min(self.front_angle)
        if self.state == 'SEARCH_ROTATE':
            self._search_rotate(faces)
        elif self.state == 'SEARCH_WANDER':
            self._search_wander(faces, front)
        elif self.state == 'APPROACH':
            self._approach(faces)
        elif self.state == 'IDENTIFY':
            self._identify(faces)
        elif self.state == 'MOVE_ON':
            self._move_on()

    def _search_rotate(self, faces):
        target = self._pick_target(faces)
        if target is not None and not self._near_visited():
            # face in view: stop sweeping past it; commit only on a fresh fix
            if time.time() - self.faces_stamp < self.fresh_for_steer:
                self._enter('APPROACH')
            else:
                self._send(0.0, 0.0)
            return
        self._send(0.0, self.w_search)
        if time.time() - self.state_t0 > self.search_rotate_time:
            self.avoiding = False
            self._enter('SEARCH_WANDER')

    def _search_wander(self, faces, front):
        target = self._pick_target(faces)
        if (target is not None and not self._near_visited()
                and time.time() - self.faces_stamp < self.fresh_for_steer):
            self._enter('APPROACH')
            return
        # hysteresis: start avoiding below avoid_dist, keep turning until clearly open
        if self.avoiding:
            if front > self.avoid_dist * self.clear_factor:
                self.avoiding = False
        elif front < self.avoid_dist:
            self.avoiding = True
            self.avoid_dir = self._side_clearer()
        if self.avoiding:
            self._send(0.0, self.avoid_dir * self.w_search)
        else:
            self._send(self.v_fwd, 0.0)
        if time.time() - self.state_t0 > self.wander_time:
            self._enter('SEARCH_ROTATE')

    
    def _approach(self, faces):
        target = self._pick_target(faces)
        fresh = target is not None and (time.time() - self.faces_stamp) < self.fresh_for_steer
        if not fresh:                            # no fresh fix: hold still, don't chase a stale bbox
            self._send(0.0, 0.0)
            if time.time() - self.state_t0 > self.lost_timeout:
                self._enter('SEARCH_ROTATE')
            return
        self.state_t0 = time.time()              # fresh sighting -> keep alive
        x_err = self._x_err(target)
        h_frac = (target['bbox'][3] - target['bbox'][1]) / self.frame_h
        ahead = self._sector_min(self.stop_angle)   # only stop for something dead-ahead
        if ahead < self.approach_stop_dist or h_frac >= self.approach_face_frac:
            self.get_logger().info(f'reached target (ahead={ahead:.2f}m h={h_frac:.2f}) -> identifying')
            self.id_votes = {}
            self._send(0.0, 0.0)
            self._enter('IDENTIFY')
            return

        raw_w = -self.steer_gain * x_err
        w = max(-self.w_approach, min(self.w_approach, raw_w))
        v = self.v_fwd * (1.0 - 0.5 * abs(x_err))
        
        self._send(v, w)

    def _identify(self, faces):
        self._send(0.0, 0.0)                         # hold still while confirming
        # tally the centred face each tick (skip ticks where nothing is detected)
        tgt = self._pick_target(faces)
        if tgt is not None:
            key = tgt.get('name') or '__unknown__'
            self.id_votes[key] = self.id_votes.get(key, 0) + 1
        if time.time() - self.state_t0 < self.identify_time:
            return                                   # keep watching for >= identify_time
        if not self.id_votes:                        # face vanished during the hold
            self.get_logger().info('lost face during identify -> moving on')
            self._enter('MOVE_ON')
            return
        best = max(self.id_votes, key=self.id_votes.get)
        total = sum(self.id_votes.values())
        registered = best != '__unknown__'
        name = best if registered else None
        self._mark(name)
        if registered:
            self.visited_names.add(name)
        self.get_logger().info(
            f'identified {"registered " + name if registered else "UNKNOWN"} '
            f'({self.id_votes[best]}/{total} votes over {self.identify_time:.0f}s)')
        self._enter('MOVE_ON')

    def _move_on(self):
        dt = time.time() - self.state_t0
        if dt < 0.8:
            self._send(-self.v_fwd, 0.0)         # back off
        elif dt < 0.8 + self.move_on_turn_time:
            self._send(0.0, self.w_search)       # turn away so they leave the FOV
        else:
            self._enter('SEARCH_ROTATE')

    def _mark(self, name):
        p = self._pose()
        if p is None:
            self.get_logger().warning('no map->base_link TF; marker skipped')
            return
        x, y, yaw = p
        d = min(self._sector_min(self.front_angle), self.approach_stop_dist)
        px, py = x + d * math.cos(yaw), y + d * math.sin(yaw)
        if any(math.hypot(px - vx, py - vy) < self.visit_radius for vx, vy in self.visited_locs):
            return                               # already marked this spot
        self.visited_locs.append((px, py))
        registered = name is not None
        dot = Marker()
        dot.header.frame_id = 'map'
        dot.header.stamp = self.get_clock().now().to_msg()
        dot.ns = 'people'
        dot.id = self.marker_id
        dot.type = Marker.SPHERE
        dot.action = Marker.ADD
        dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = px, py, 0.1
        dot.pose.orientation.w = 1.0
        dot.scale.x = dot.scale.y = dot.scale.z = 0.2
        dot.color.a = 1.0
        dot.color.g = 1.0 if registered else 0.0
        dot.color.r = 0.0 if registered else 1.0
        label = Marker()
        label.header.frame_id = 'map'
        label.header.stamp = dot.header.stamp
        label.ns = 'people_labels'
        label.id = self.marker_id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x, label.pose.position.y, label.pose.position.z = px, py, 0.35
        label.pose.orientation.w = 1.0
        label.scale.z = 0.2
        label.color.a = label.color.r = label.color.g = label.color.b = 1.0
        label.text = name if registered else 'Unknown'
        self.markers.markers.append(dot)
        self.markers.markers.append(label)
        self.marker_id += 1
        self.marker_pub.publish(self.markers)

    def destroy_node(self):
        try:
            self.cmd_pub.publish(Twist())        # raw zero on shutdown
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceSeeker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
