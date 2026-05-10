#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan, Range
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class CooldownZone:
    x: float
    y: float
    radius: float
    until_ns: int


@dataclass
class BamseRecord:
    x: float
    y: float
    z: float
    max_temp: float


class BamseLocator(Node):
    def __init__(self):
        super().__init__('bamse_locator')

        # Topics & frames
        self.thermal_topic = self.declare_parameter('thermal_topic', '/thermal/pixel_raw').value
        self.yolo_det_topic = self.declare_parameter('yolo_det_topic', '/yolo/detections').value
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.ir_front_topic = self.declare_parameter('ir_front_topic', '/ir/front').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value
        self.thermal_frame = self.declare_parameter('thermal_frame', 'thermal_camera').value

        # Termisk
        self.temp_scale = float(self.declare_parameter('temp_scale', 100.0).value)
        self.temp_offset = float(self.declare_parameter('temp_offset', -273.15).value)
        self.thermal_hfov_rad = float(self.declare_parameter('thermal_hfov_rad', 0.95).value)
        self.heat_threshold_c = float(self.declare_parameter('heat_threshold_c', 30.0).value)
        self.temp_band_min_c = float(self.declare_parameter('temp_band_min_c', 30.0).value)
        self.temp_band_max_c = float(self.declare_parameter('temp_band_max_c', 42.0).value)
        self.min_blob_pixels = int(self.declare_parameter('min_blob_pixels', 10).value)
        self.max_blob_pixels = int(self.declare_parameter('max_blob_pixels', 8000).value)
        self.blob_vertical_max_frac = float(self.declare_parameter('blob_vertical_max_frac', 0.65).value)
        self.max_aspect_ratio = float(self.declare_parameter('max_aspect_ratio', 1.6).value)

        # YOLO-bekreftelse
        self.require_yolo_confirm = bool(self.declare_parameter('require_yolo_confirm', False).value)
        self.teddy_class_id = int(self.declare_parameter('teddy_class_id', 77).value)
        self.yolo_confirm_window_s = float(self.declare_parameter('yolo_confirm_window_s', 1.5).value)
        self.yolo_min_score = float(self.declare_parameter('yolo_min_score', 0.30).value)

        # Front IR
        self.use_front_ir = bool(self.declare_parameter('use_front_ir', False).value)
        self.ir_trigger_max_range_m = float(self.declare_parameter('ir_trigger_max_range_m', 0.10).value)

        # Nav2 / approach
        self.nav_action_name = self.declare_parameter('nav_action_name', 'navigate_to_pose').value
        self.approach_step_m = float(self.declare_parameter('approach_step_m', 0.7).value)
        self.approach_min_step_m = float(self.declare_parameter('approach_min_step_m', 0.20).value)
        self.approach_replan_period_s = float(self.declare_parameter('approach_replan_period_s', 1.5).value)
        self.stop_distance_m = float(self.declare_parameter('stop_distance_m', 0.35).value)
        self.confirm_blob_fraction = float(self.declare_parameter('confirm_blob_fraction', 0.15).value)
        self.confirm_hold_s = float(self.declare_parameter('confirm_hold_s', 1.0).value)
        self.confirm_timeout_s = float(self.declare_parameter('confirm_timeout_s', 8.0).value)
        self.max_lost_frames = int(self.declare_parameter('max_lost_frames', 20).value)

        # Explore-integrasjon
        self.pause_explore = bool(self.declare_parameter('pause_explore', True).value)
        self.explore_resume_topic = self.declare_parameter('explore_resume_topic', '/explore/resume').value

        # Markering
        self.mark_offset_x = float(self.declare_parameter('mark_offset_x', 0.15).value)
        self.mark_offset_z = float(self.declare_parameter('mark_offset_z', 0.10).value)
        self.cooldown_radius_m = float(self.declare_parameter('cooldown_radius_m', 1.5).value)
        self.cooldown_time_s = float(self.declare_parameter('cooldown_time_s', 5.0).value)
        self.false_positive_cooldown_radius_m = float(
            self.declare_parameter('false_positive_cooldown_radius_m', 0.8).value
        )
        self.false_positive_cooldown_time_s = float(
            self.declare_parameter('false_positive_cooldown_time_s', 30.0).value
        )
        self.republish_markers_period_s = float(self.declare_parameter('republish_markers_period_s', 1.0).value)

        # Debug
        self.publish_debug_image = bool(self.declare_parameter('publish_debug_image', True).value)
        self.state_topic = self.declare_parameter('state_topic', '/bamse/state').value

        # Callback groups / thread safety
        self.sensor_cbg = MutuallyExclusiveCallbackGroup()
        self.action_cbg = MutuallyExclusiveCallbackGroup()

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/bamse/pose', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/bamse/markers', 10)
        self.debug_pub = self.create_publisher(Image, '/bamse/debug_image', 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        explore_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.explore_pub = self.create_publisher(Bool, self.explore_resume_topic, explore_qos)

        # Subscribers
        self.create_subscription(Image, self.thermal_topic, self.cb_thermal, 10, callback_group=self.sensor_cbg)
        self.create_subscription(Detection2DArray, self.yolo_det_topic, self.cb_yolo, 10, callback_group=self.sensor_cbg)
        self.create_subscription(LaserScan, self.scan_topic, self.cb_scan, 10, callback_group=self.sensor_cbg)
        self.create_subscription(Range, self.ir_front_topic, self.cb_ir, 10, callback_group=self.sensor_cbg)

        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name, callback_group=self.action_cbg)

        # Timers
        self.create_timer(1.0, self.publish_state_heartbeat)
        self.create_timer(self.republish_markers_period_s, self.publish_markers)

        # State
        self.state = 'SEARCH'
        self.state_enter_ns = self.now_ns()
        self.confirm_hold_start_ns: Optional[int] = None
        self.last_replan_ns = 0
        self.last_scan: Optional[LaserScan] = None
        self.last_ir: Optional[Range] = None
        self.yolo_hits: List[int] = []
        self.bamses: List[BamseRecord] = []
        self.cooldowns: List[CooldownZone] = []

        self.current_goal_handle = None
        self.nav_abort_streak = 0
        self.lost_frames = 0
        self.nav_warned_once = False
        self.robot_frame_fallback_tried = False

        self.latest_blob = None
        self.latest_temp = None
        self.latest_blob_fraction = 0.0
        self.latest_max_temp = float('nan')

        self.publish_state(force=True)
        self.get_logger().info('bamse_locator startet')

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def publish_state(self, force: bool = False):
        if force:
            self.get_logger().info(f'State -> {self.state}')
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def publish_state_heartbeat(self):
        self.publish_state(force=False)

    def set_state(self, new_state: str):
        if new_state == self.state:
            return
        self.state = new_state
        self.state_enter_ns = self.now_ns()
        self.publish_state(force=True)

    def cb_scan(self, msg: LaserScan):
        self.last_scan = msg

    def cb_ir(self, msg: Range):
        self.last_ir = msg

    def cb_yolo(self, msg: Detection2DArray):
        now_ns = self.now_ns()
        self.yolo_hits = [t for t in self.yolo_hits if now_ns - t <= int(self.yolo_confirm_window_s * 1e9)]
        for det in msg.detections:
            for res in det.results:
                try:
                    cls_id = int(res.hypothesis.class_id)
                except ValueError:
                    continue
                if cls_id == self.teddy_class_id and float(res.hypothesis.score) >= self.yolo_min_score:
                    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                    if stamp_ns <= 0:
                        stamp_ns = now_ns
                    self.yolo_hits.append(stamp_ns)
                    break

    def cb_thermal(self, msg: Image):
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
        temp_c = raw.astype(np.float32) / self.temp_scale + self.temp_offset

        blob = self.extract_blob(temp_c)
        self.latest_blob = blob
        self.latest_temp = temp_c

        if blob is None:
            if self.state == 'APPROACH':
                self.lost_frames += 1
                if self.lost_frames > self.max_lost_frames:
                    self.false_positive_abort('Mistet varmesignal')
            self.publish_debug(msg)
            return

        (x, y, bw, bh, area, max_temp, centroid_u, centroid_v) = blob
        self.latest_blob_fraction = area / float(temp_c.shape[0] * temp_c.shape[1])
        self.latest_max_temp = max_temp

        thermal_frame = msg.header.frame_id if msg.header.frame_id else self.thermal_frame

        filter_ok = self.apply_filters(blob, temp_c.shape)
        if not filter_ok:
            if self.state == 'APPROACH':
                self.lost_frames += 1
                if self.lost_frames > self.max_lost_frames:
                    self.false_positive_abort('Filtre feilet under APPROACH')
            self.publish_debug(msg)
            return

        yaw_in_base = self.centroid_to_yaw_in_base(centroid_u, centroid_v, temp_c.shape[1], temp_c.shape[0], thermal_frame)
        if yaw_in_base is None:
            self.publish_debug(msg)
            return

        lidar_range = self.lookup_scan_range(yaw_in_base)

        if self.is_in_cooldown_zone(yaw_in_base, lidar_range):
            self.publish_debug(msg)
            return

        self.lost_frames = 0
        confirm_condition = self.confirm_condition(lidar_range)

        if self.state == 'SEARCH':
            self.start_approach(yaw_in_base, lidar_range)
        elif self.state == 'APPROACH':
            if confirm_condition:
                self.enter_confirm()
            else:
                self.replan_approach_if_needed(yaw_in_base, lidar_range)
        elif self.state == 'CONFIRM':
            self.process_confirm(confirm_condition)
        elif self.state == 'COOLDOWN':
            if self.now_ns() - self.state_enter_ns >= int(self.cooldown_time_s * 1e9):
                self.resume_explore()
                self.set_state('SEARCH')

        self.publish_debug(msg)

    def extract_blob(self, temp_c: np.ndarray):
        mask = (temp_c >= self.heat_threshold_c).astype(np.uint8) * 255
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        best = None
        for i in range(1, n_labels):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < self.min_blob_pixels or area > self.max_blob_pixels:
                continue
            x = int(stats[i, cv2.CC_STAT_LEFT])
            y = int(stats[i, cv2.CC_STAT_TOP])
            w = int(stats[i, cv2.CC_STAT_WIDTH])
            h = int(stats[i, cv2.CC_STAT_HEIGHT])
            idx = labels == i
            max_temp = float(np.max(temp_c[idx]))
            c = centroids[i]
            cand = (x, y, w, h, area, max_temp, float(c[0]), float(c[1]))
            if best is None or area > best[4]:
                best = cand
        return best

    def apply_filters(self, blob, shape: Tuple[int, int]) -> bool:
        _, y, bw, bh, area, max_temp, _, _ = blob
        h_img, _ = shape

        if not (self.temp_band_min_c <= max_temp <= self.temp_band_max_c):
            return False
        if not (self.min_blob_pixels <= area <= self.max_blob_pixels):
            return False
        vertical_cut = h_img * (1.0 - self.blob_vertical_max_frac)
        if y < vertical_cut:
            return False
        if bw <= 0:
            return False
        if bh / float(bw) > self.max_aspect_ratio:
            return False
        return True

    def centroid_to_yaw_in_base(self, u: float, v: float, w: int, h: int, thermal_frame: str) -> Optional[float]:
        hfov = self.thermal_hfov_rad
        vfov = hfov * (h / max(w, 1))
        fx = (w / 2.0) / math.tan(hfov / 2.0)
        fy = (h / 2.0) / math.tan(vfov / 2.0)
        cx = (w - 1) / 2.0
        cy = (h - 1) / 2.0

        x_opt = (u - cx) / fx
        y_opt = (v - cy) / fy
        vec_thermal = np.array([1.0, -x_opt, -y_opt], dtype=np.float64)

        tf = self.lookup_transform_robot_from_thermal(thermal_frame)
        if tf is None:
            return None

        q = tf.transform.rotation
        vec_base = self.quat_to_rot(q.x, q.y, q.z, q.w) @ vec_thermal
        if np.linalg.norm(vec_base[:2]) < 1e-6:
            return None
        return math.atan2(vec_base[1], vec_base[0])

    def lookup_transform_robot_from_thermal(self, thermal_frame: str):
        timeout = Duration(seconds=0.2)
        try:
            return self.tf_buffer.lookup_transform(self.robot_frame, thermal_frame, rclpy.time.Time(), timeout)
        except TransformException as ex:
            if self.robot_frame == 'base_link' and not self.robot_frame_fallback_tried:
                self.robot_frame_fallback_tried = True
                self.robot_frame = 'base_footprint'
                self.get_logger().warn(
                    'base_link ikke funnet, faller tilbake til base_footprint',
                    throttle_duration_sec=2.0,
                )
                try:
                    return self.tf_buffer.lookup_transform(self.robot_frame, thermal_frame, rclpy.time.Time(), timeout)
                except TransformException:
                    pass
            self.get_logger().warn(f'TF thermal->robot feilet: {ex}', throttle_duration_sec=2.0)
            return None

    def lookup_scan_range(self, yaw: float) -> float:
        if self.last_scan is None or len(self.last_scan.ranges) == 0:
            return float('inf')

        scan = self.last_scan
        yaw = max(scan.angle_min, min(scan.angle_max, yaw))
        idx = (yaw - scan.angle_min) / scan.angle_increment
        i0 = max(0, min(int(math.floor(idx)), len(scan.ranges) - 1))
        i1 = min(i0 + 1, len(scan.ranges) - 1)

        r0 = scan.ranges[i0]
        r1 = scan.ranges[i1]
        if not math.isfinite(r0) and not math.isfinite(r1):
            return float('inf')
        if not math.isfinite(r0):
            return float(r1)
        if not math.isfinite(r1):
            return float(r0)
        return float((1.0 - (idx - i0)) * r0 + (idx - i0) * r1)

    def confirm_condition(self, lidar_range: float) -> bool:
        ir_triggered = self.front_ir_triggered()
        lidar_triggered = math.isfinite(lidar_range) and lidar_range <= self.stop_distance_m
        blob_triggered = self.latest_blob_fraction >= self.confirm_blob_fraction
        return ir_triggered or lidar_triggered or blob_triggered

    def front_ir_triggered(self) -> bool:
        if not self.use_front_ir or self.last_ir is None:
            return False
        if not math.isfinite(self.last_ir.range):
            return False
        return self.last_ir.range <= self.ir_trigger_max_range_m

    def start_approach(self, yaw_in_base: float, lidar_range: float):
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            if not self.nav_warned_once:
                self.get_logger().warn('Nav2 action server ikke tilgjengelig, holder SEARCH', throttle_duration_sec=2.0)
                self.nav_warned_once = True
            return
        self.nav_warned_once = False
        self.pause_explore_if_needed()
        self.nav_abort_streak = 0
        self.send_approach_goal(yaw_in_base, lidar_range)
        self.last_replan_ns = self.now_ns()
        self.set_state('APPROACH')

    def replan_approach_if_needed(self, yaw_in_base: float, lidar_range: float):
        if self.now_ns() - self.last_replan_ns < int(self.approach_replan_period_s * 1e9):
            return
        self.send_approach_goal(yaw_in_base, lidar_range)
        self.last_replan_ns = self.now_ns()

    def send_approach_goal(self, yaw_in_base: float, lidar_range: float):
        pose = self.lookup_robot_pose_in_map()
        if pose is None:
            return

        _, _, rz, rw, tx, ty, tz = pose
        yaw_goal = self.quat_to_yaw(rz, rw) + yaw_in_base

        step = self.approach_step_m
        if math.isfinite(lidar_range) and lidar_range < self.approach_step_m * 1.3:
            step = max(self.approach_min_step_m, lidar_range - self.stop_distance_m)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(tx + step * math.cos(yaw_goal))
        goal.pose.pose.position.y = float(ty + step * math.sin(yaw_goal))
        goal.pose.pose.position.z = float(tz)
        goal.pose.pose.orientation.z = float(math.sin(yaw_goal / 2.0))
        goal.pose.pose.orientation.w = float(math.cos(yaw_goal / 2.0))

        self.cancel_current_goal()
        self.nav_client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().warn(f'Nav2 goal response feilet: {ex}', throttle_duration_sec=2.0)
            return

        if not goal_handle.accepted:
            self.nav_abort_streak += 1
            if self.nav_abort_streak >= 3 and self.state == 'APPROACH':
                self.false_positive_abort('Nav2 avviste/aborterte gjentatt')
            return

        self.current_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.on_goal_result)

    def on_goal_result(self, future):
        try:
            status = future.result().status
        except Exception as ex:
            self.get_logger().warn(f'Nav2 result feilet: {ex}', throttle_duration_sec=2.0)
            return
        if status == GoalStatus.STATUS_ABORTED and self.state == 'APPROACH':
            self.nav_abort_streak += 1
            if self.nav_abort_streak >= 3:
                self.false_positive_abort('Nav2 aborterte 3 ganger')
        elif status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED):
            self.nav_abort_streak = 0

    def cancel_current_goal(self):
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None

    def enter_confirm(self):
        self.cancel_current_goal()
        self.stop_robot()
        self.confirm_hold_start_ns = None
        self.set_state('CONFIRM')

    def process_confirm(self, confirm_condition: bool):
        now_ns = self.now_ns()
        if now_ns - self.state_enter_ns > int(self.confirm_timeout_s * 1e9):
            self.false_positive_abort('CONFIRM timeout')
            return
        if not confirm_condition:
            self.confirm_hold_start_ns = None
            return
        if self.confirm_hold_start_ns is None:
            self.confirm_hold_start_ns = now_ns
            return
        if now_ns - self.confirm_hold_start_ns < int(self.confirm_hold_s * 1e9):
            return
        if self.require_yolo_confirm and not self.has_recent_yolo_teddy(now_ns):
            return
        self.mark_bamse()

    def has_recent_yolo_teddy(self, now_ns: int) -> bool:
        limit_ns = int(self.yolo_confirm_window_s * 1e9)
        self.yolo_hits = [t for t in self.yolo_hits if now_ns - t <= limit_ns]
        return len(self.yolo_hits) > 0

    def mark_bamse(self):
        pose = self.lookup_robot_pose_in_map()
        if pose is None:
            self.false_positive_abort('Mangler robot-pose i map ved MARK')
            return

        _, _, rz, rw, tx, ty, _ = pose
        yaw_robot = self.quat_to_yaw(rz, rw)
        rec = BamseRecord(
            x=float(tx + self.mark_offset_x * math.cos(yaw_robot)),
            y=float(ty + self.mark_offset_x * math.sin(yaw_robot)),
            z=float(self.mark_offset_z),
            max_temp=float(self.latest_max_temp),
        )
        self.bamses.append(rec)

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.map_frame
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = rec.x
        pose_msg.pose.position.y = rec.y
        pose_msg.pose.position.z = rec.z
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

        self.add_cooldown_zone(rec.x, rec.y, self.cooldown_radius_m, self.cooldown_time_s)
        self.publish_markers()
        self.get_logger().info(f'MARK bamse #{len(self.bamses)} @ ({rec.x:.2f}, {rec.y:.2f}) max={rec.max_temp:.1f}C')
        self.set_state('MARK')
        self.set_state('COOLDOWN')

    def false_positive_abort(self, reason: str):
        self.get_logger().info(f'Aborterer som false positive: {reason}')
        self.cancel_current_goal()
        self.stop_robot()
        pose = self.lookup_robot_pose_in_map()
        if pose is not None:
            _, _, _, _, tx, ty, _ = pose
            self.add_cooldown_zone(tx, ty, self.false_positive_cooldown_radius_m, self.false_positive_cooldown_time_s)
        self.resume_explore()
        self.confirm_hold_start_ns = None
        self.lost_frames = 0
        self.nav_abort_streak = 0
        self.set_state('SEARCH')

    def pause_explore_if_needed(self):
        if self.pause_explore:
            self.explore_pub.publish(Bool(data=False))

    def resume_explore(self):
        if self.pause_explore:
            self.explore_pub.publish(Bool(data=True))

    def lookup_robot_pose_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time(), Duration(seconds=0.2)
            )
        except TransformException as ex:
            self.get_logger().warn(f'TF map->robot feilet: {ex}', throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (q.x, q.y, q.z, q.w, t.x, t.y, t.z)

    def add_cooldown_zone(self, x: float, y: float, radius: float, duration_s: float):
        self.cooldowns.append(
            CooldownZone(x=float(x), y=float(y), radius=float(radius), until_ns=self.now_ns() + int(duration_s * 1e9))
        )

    def is_in_cooldown_zone(self, yaw_in_base: float, lidar_range: float) -> bool:
        now_ns = self.now_ns()
        self.cooldowns = [c for c in self.cooldowns if c.until_ns > now_ns]
        if not self.cooldowns:
            return False

        pose = self.lookup_robot_pose_in_map()
        if pose is None:
            return False
        _, _, rz, rw, tx, ty, _ = pose
        yaw_robot = self.quat_to_yaw(rz, rw)
        d = max(0.2, lidar_range if math.isfinite(lidar_range) else self.approach_step_m)
        px = tx + d * math.cos(yaw_robot + yaw_in_base)
        py = ty + d * math.sin(yaw_robot + yaw_in_base)
        for zone in self.cooldowns:
            if (px - zone.x) ** 2 + (py - zone.y) ** 2 <= zone.radius ** 2:
                return True
        return False

    def publish_markers(self):
        arr = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = self.map_frame
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        arr.markers.append(delete_all)

        stamp = self.get_clock().now().to_msg()
        for i, b in enumerate(self.bamses):
            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = stamp
            sphere.ns = 'bamse'
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = b.x
            sphere.pose.position.y = b.y
            sphere.pose.position.z = b.z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.25
            sphere.scale.y = 0.25
            sphere.scale.z = 0.25
            sphere.color.r = 1.0
            sphere.color.a = 1.0
            arr.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = stamp
            text.ns = 'bamse_text'
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = b.x
            text.pose.position.y = b.y
            text.pose.position.z = b.z + 0.30
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f'Bamse #{i + 1}\n{b.max_temp:.1f}°C'
            arr.markers.append(text)

        self.markers_pub.publish(arr)

    def publish_debug(self, src_msg: Image):
        if not self.publish_debug_image or self.latest_temp is None:
            return

        norm = np.clip((self.latest_temp - 15.0) / 30.0, 0.0, 1.0)
        color = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
        state_colors = {
            'SEARCH': (0, 255, 0),
            'APPROACH': (0, 255, 255),
            'CONFIRM': (0, 165, 255),
            'MARK': (0, 0, 255),
            'COOLDOWN': (255, 128, 0),
        }
        if self.latest_blob is not None:
            x, y, bw, bh, _, _, cu, cv = self.latest_blob
            cv2.rectangle(color, (x, y), (x + bw, y + bh), (255, 255, 255), 2)
            cv2.circle(color, (int(cu), int(cv)), 3, (255, 255, 255), -1)
        c = state_colors.get(self.state, (255, 255, 255))
        cv2.putText(color, f'STATE: {self.state}', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2)
        cv2.putText(color, f'Bamser: {len(self.bamses)}', (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2)

        out = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
        out.header = src_msg.header
        self.debug_pub.publish(out)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    @staticmethod
    def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
        n = x * x + y * y + z * z + w * w
        if n < 1e-12:
            return np.eye(3, dtype=np.float64)
        s = 2.0 / n
        xx, yy, zz = x * x * s, y * y * s, z * z * s
        xy, xz, yz = x * y * s, x * z * s, y * z * s
        wx, wy, wz = w * x * s, w * y * s, w * z * s
        return np.array(
            [
                [1.0 - (yy + zz), xy - wz, xz + wy],
                [xy + wz, 1.0 - (xx + zz), yz - wx],
                [xz - wy, yz + wx, 1.0 - (xx + yy)],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def quat_to_yaw(z: float, w: float) -> float:
        return 2.0 * math.atan2(z, w)


def main(args=None):
    rclpy.init(args=args)
    node = BamseLocator()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
