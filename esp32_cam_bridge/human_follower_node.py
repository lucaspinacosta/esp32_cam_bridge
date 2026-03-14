#!/usr/bin/env python3
"""Detect a person in the ESP32 camera feed and track them with Nav2."""

import math
import threading
from typing import Optional, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class HumanFollowerNode(Node):
    """Track the largest detected person and hand targets to Nav2."""

    def __init__(self) -> None:
        """Initialize subscriptions, publishers, Nav2 client, and detector."""
        super().__init__('human_follower')

        self.declare_parameter('image_topic', '/esp32/camera/image_raw')
        self.declare_parameter('led_state_topic', '/esp32/led_state')
        self.declare_parameter('status_topic', '/human_follower/status')
        self.declare_parameter('goal_topic', '/human_follower/goal_pose')
        self.declare_parameter('image_qos_reliability', 'reliable')
        self.declare_parameter('process_fps', 5.0)
        self.declare_parameter('lost_timeout_sec', 1.0)
        self.declare_parameter('target_height_fraction', 0.35)
        self.declare_parameter('camera_horizontal_fov_deg', 62.0)
        self.declare_parameter('desired_follow_distance_m', 1.0)
        self.declare_parameter('min_goal_distance_m', 0.0)
        self.declare_parameter('max_goal_distance_m', 2.0)
        self.declare_parameter('goal_update_period_sec', 0.75)
        self.declare_parameter('goal_position_tolerance_m', 0.20)
        self.declare_parameter('goal_yaw_tolerance_rad', 0.25)
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_detection_weight', 0.3)
        self.declare_parameter('hog_scale', 1.05)
        self.declare_parameter('cancel_nav_on_lost', True)

        self.image_topic = self.get_parameter(
            'image_topic').get_parameter_value().string_value
        self.led_state_topic = self.get_parameter(
            'led_state_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter(
            'status_topic').get_parameter_value().string_value
        self.goal_topic = self.get_parameter(
            'goal_topic').get_parameter_value().string_value
        self.image_qos_reliability = self.get_parameter(
            'image_qos_reliability'
        ).get_parameter_value().string_value.lower()
        self.process_fps = self.get_parameter(
            'process_fps').get_parameter_value().double_value
        self.lost_timeout_sec = self.get_parameter(
            'lost_timeout_sec').get_parameter_value().double_value
        self.target_height_fraction = self.get_parameter(
            'target_height_fraction').get_parameter_value().double_value
        self.camera_horizontal_fov_deg = self.get_parameter(
            'camera_horizontal_fov_deg'
        ).get_parameter_value().double_value
        self.desired_follow_distance_m = self.get_parameter(
            'desired_follow_distance_m'
        ).get_parameter_value().double_value
        self.min_goal_distance_m = self.get_parameter(
            'min_goal_distance_m'
        ).get_parameter_value().double_value
        self.max_goal_distance_m = self.get_parameter(
            'max_goal_distance_m'
        ).get_parameter_value().double_value
        self.goal_update_period_sec = self.get_parameter(
            'goal_update_period_sec'
        ).get_parameter_value().double_value
        self.goal_position_tolerance_m = self.get_parameter(
            'goal_position_tolerance_m'
        ).get_parameter_value().double_value
        self.goal_yaw_tolerance_rad = self.get_parameter(
            'goal_yaw_tolerance_rad'
        ).get_parameter_value().double_value
        self.nav_action_name = self.get_parameter(
            'nav_action_name').get_parameter_value().string_value
        self.global_frame = self.get_parameter(
            'global_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value
        self.min_detection_weight = self.get_parameter(
            'min_detection_weight').get_parameter_value().double_value
        self.hog_scale = self.get_parameter(
            'hog_scale').get_parameter_value().double_value
        self.cancel_nav_on_lost = self.get_parameter(
            'cancel_nav_on_lost').get_parameter_value().bool_value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=self.get_reliability_policy(),
        )

        self.bridge = CvBridge()
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.led_state_pub = self.create_publisher(
            String, self.led_state_topic, 10
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, self.goal_topic, 10
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self, NavigateToPose, self.nav_action_name
        )

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.last_detection_time = None
        self.last_goal_sent_time = None
        self.last_goal_pose = None
        self.last_led_state = ''
        self.nav_goal_handle = None
        self.goal_response_future = None

        period = 1.0 / max(self.process_fps, 1.0)
        self.process_timer = self.create_timer(period, self.process_frame)

        self.publish_led_state('searching')
        self.publish_status('waiting_for_images')

    def get_reliability_policy(self) -> ReliabilityPolicy:
        """Translate the image QoS parameter to a ROS reliability policy."""
        if self.image_qos_reliability == 'best_effort':
            return ReliabilityPolicy.BEST_EFFORT

        if self.image_qos_reliability != 'reliable':
            self.get_logger().warn(
                'Invalid image_qos_reliability value '
                f'"{self.image_qos_reliability}", using reliable'
            )
            self.image_qos_reliability = 'reliable'

        return ReliabilityPolicy.RELIABLE

    def image_callback(self, msg: Image) -> None:
        """Store the latest incoming image for detector processing."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'Image conversion failed: {exc}')
            return

        with self.frame_lock:
            self.latest_frame = frame

    def process_frame(self) -> None:
        """Run person detection and update Nav2 with a new target pose."""
        with self.frame_lock:
            if self.latest_frame is None:
                self.handle_missing_detection('waiting_for_images')
                return

            frame = self.latest_frame.copy()

        detection = self.detect_person(frame)
        if detection is None:
            self.handle_missing_detection('no_person_detected')
            return

        self.last_detection_time = self.get_clock().now()

        goal_pose = self.build_goal_pose(frame, detection)
        if goal_pose is None:
            return

        self.goal_pub.publish(goal_pose)

        if not self.should_send_goal(goal_pose):
            self.publish_led_state('tracking')
            self.publish_status('tracking_person awaiting_goal_refresh')
            return

        self.send_nav_goal(goal_pose)

    def detect_person(self, frame) -> Optional[Tuple[int, int, int, int]]:
        """Detect the largest person-like bounding box in the frame."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rects, weights = self.hog.detectMultiScale(
            gray,
            winStride=(8, 8),
            padding=(8, 8),
            scale=self.hog_scale,
        )

        best_rect = None
        best_score = -1.0
        for rect, weight in zip(rects, weights):
            score = float(weight)
            if score < self.min_detection_weight:
                continue

            x, y, w, h = [int(value) for value in rect]
            area = w * h
            weighted_area = area * score
            if weighted_area > best_score:
                best_rect = (x, y, w, h)
                best_score = weighted_area

        return best_rect

    def build_goal_pose(
        self,
        frame,
        detection: Tuple[int, int, int, int],
    ) -> Optional[PoseStamped]:
        """Convert the current detection into a Nav2 goal pose."""
        x, y, w, h = detection
        frame_height, frame_width = frame.shape[:2]

        bbox_height_fraction = h / float(frame_height)
        center_x = x + (w / 2.0)
        center_error = ((center_x / frame_width) - 0.5) * 2.0
        angle_rad = center_error * math.radians(
            self.camera_horizontal_fov_deg / 2.0
        )

        estimated_target_distance = (
            self.desired_follow_distance_m *
            self.target_height_fraction /
            max(bbox_height_fraction, 1e-3)
        )
        goal_distance = self.clamp(
            estimated_target_distance - self.desired_follow_distance_m,
            self.min_goal_distance_m,
            self.max_goal_distance_m,
        )

        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.publish_led_state('searching')
            self.publish_status(f'tf_lookup_failed:{exc}')
            return None

        base_x = transform.transform.translation.x
        base_y = transform.transform.translation.y
        base_yaw = self.yaw_from_quaternion(transform.transform.rotation)

        rel_x = goal_distance * math.cos(angle_rad)
        rel_y = goal_distance * math.sin(angle_rad)

        goal_x = (
            base_x +
            (math.cos(base_yaw) * rel_x) -
            (math.sin(base_yaw) * rel_y)
        )
        goal_y = (
            base_y +
            (math.sin(base_yaw) * rel_x) +
            (math.cos(base_yaw) * rel_y)
        )
        goal_yaw = self.normalize_angle(base_yaw + angle_rad)

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.global_frame
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = self.quaternion_from_yaw(goal_yaw)

        self.publish_status(
            'tracking_person'
            f' center_error={center_error:.3f}'
            f' bbox_height_fraction={bbox_height_fraction:.3f}'
            f' estimated_distance={estimated_target_distance:.3f}'
            f' goal_distance={goal_distance:.3f}'
        )
        return goal_pose

    def should_send_goal(self, goal_pose: PoseStamped) -> bool:
        """Rate-limit and deduplicate Nav2 goal updates."""
        now = self.get_clock().now()

        if self.last_goal_pose is None or self.last_goal_sent_time is None:
            return True

        elapsed = (now - self.last_goal_sent_time).nanoseconds / 1e9
        if elapsed < self.goal_update_period_sec:
            return False

        dx = (
            goal_pose.pose.position.x -
            self.last_goal_pose.pose.position.x
        )
        dy = (
            goal_pose.pose.position.y -
            self.last_goal_pose.pose.position.y
        )
        distance_delta = math.hypot(dx, dy)

        current_yaw = self.yaw_from_quaternion(goal_pose.pose.orientation)
        last_yaw = self.yaw_from_quaternion(
            self.last_goal_pose.pose.orientation
        )
        yaw_delta = abs(self.normalize_angle(current_yaw - last_yaw))

        return (
            distance_delta >= self.goal_position_tolerance_m or
            yaw_delta >= self.goal_yaw_tolerance_rad
        )

    def send_nav_goal(self, goal_pose: PoseStamped) -> None:
        """Send a new goal to Nav2."""
        if self.goal_response_future is not None:
            if not self.goal_response_future.done():
                return
            self.goal_response_future = None

        if not self.nav_client.wait_for_server(timeout_sec=0.0):
            self.publish_led_state('searching')
            self.publish_status('nav2_action_unavailable')
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        self.publish_led_state('following')
        self.goal_response_future = self.nav_client.send_goal_async(goal)
        self.goal_response_future.add_done_callback(
            self.goal_response_callback
        )
        self.last_goal_sent_time = self.get_clock().now()
        self.last_goal_pose = goal_pose

    def goal_response_callback(self, future) -> None:
        """Handle Nav2 goal acceptance or rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.nav_goal_handle = None
            self.last_goal_pose = None
            self.last_goal_sent_time = None
            self.publish_led_state('lost')
            self.publish_status('nav_goal_rejected')
            return

        self.nav_goal_handle = goal_handle
        self.publish_status('nav_goal_accepted')

    def handle_missing_detection(self, reason: str) -> None:
        """Cancel or hold navigation when no person is currently detected."""
        if self.last_detection_time is None:
            self.publish_led_state('searching')
            self.publish_status(reason)
            return

        elapsed = (
            self.get_clock().now() - self.last_detection_time
        ).nanoseconds / 1e9

        if elapsed >= self.lost_timeout_sec:
            self.publish_led_state('lost')
            self.publish_status(f'lost_person elapsed={elapsed:.2f}')
            if self.cancel_nav_on_lost:
                self.cancel_nav_goal()
            return

        self.publish_led_state('searching')
        self.publish_status(f'searching elapsed={elapsed:.2f}')

    def cancel_nav_goal(self) -> None:
        """Cancel the current Nav2 goal if one is active."""
        if self.nav_goal_handle is None:
            return

        cancel_future = self.nav_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)
        self.nav_goal_handle = None
        self.last_goal_pose = None
        self.last_goal_sent_time = None

    def cancel_done_callback(self, future) -> None:
        """Handle Nav2 goal cancel completion."""
        try:
            future.result()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f'Nav goal cancel failed: {exc}')

    def publish_led_state(self, state: str) -> None:
        """Publish LED state changes only when the value actually changes."""
        if state == self.last_led_state:
            return

        msg = String()
        msg.data = state
        self.led_state_pub.publish(msg)
        self.last_led_state = state

    def publish_status(self, text: str) -> None:
        """Publish a human follower status string."""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        """Clamp a numeric value to the requested range."""
        return max(minimum, min(value, maximum))

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Wrap an angle to the [-pi, pi] interval."""
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def yaw_from_quaternion(quaternion: Quaternion) -> float:
        """Compute yaw from a geometry_msgs quaternion."""
        siny_cosp = (
            2.0 *
            (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        )
        cosy_cosp = (
            1.0 -
            2.0 *
            (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        )
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def quaternion_from_yaw(yaw: float) -> Quaternion:
        """Build a geometry_msgs quaternion from yaw only."""
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion


def main(args=None) -> None:
    """Run the human follower node."""
    rclpy.init(args=args)
    node = HumanFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
