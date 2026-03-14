#!/usr/bin/env python3
"""Detect a person in the ESP32 camera feed and follow them."""

import threading
from typing import Optional, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class HumanFollowerNode(Node):
    """Track the largest detected person and drive the robot toward them."""

    def __init__(self) -> None:
        """Initialize subscriptions, publishers, parameters, and detector."""
        super().__init__('human_follower')

        self.declare_parameter('image_topic', '/esp32/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('led_state_topic', '/esp32/led_state')
        self.declare_parameter('status_topic', '/human_follower/status')
        self.declare_parameter('image_qos_reliability', 'reliable')
        self.declare_parameter('process_fps', 5.0)
        self.declare_parameter('lost_timeout_sec', 1.0)
        self.declare_parameter('target_height_fraction', 0.35)
        self.declare_parameter('center_deadband', 0.08)
        self.declare_parameter('turn_slowdown_threshold', 0.35)
        self.declare_parameter('angular_gain', 1.2)
        self.declare_parameter('linear_gain', 0.8)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('min_detection_weight', 0.3)
        self.declare_parameter('hog_scale', 1.05)
        self.declare_parameter('stop_when_no_detection', True)

        self.image_topic = self.get_parameter(
            'image_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic').get_parameter_value().string_value
        self.led_state_topic = self.get_parameter(
            'led_state_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter(
            'status_topic').get_parameter_value().string_value
        self.image_qos_reliability = self.get_parameter(
            'image_qos_reliability'
        ).get_parameter_value().string_value.lower()
        self.process_fps = self.get_parameter(
            'process_fps').get_parameter_value().double_value
        self.lost_timeout_sec = self.get_parameter(
            'lost_timeout_sec').get_parameter_value().double_value
        self.target_height_fraction = self.get_parameter(
            'target_height_fraction').get_parameter_value().double_value
        self.center_deadband = self.get_parameter(
            'center_deadband').get_parameter_value().double_value
        self.turn_slowdown_threshold = self.get_parameter(
            'turn_slowdown_threshold'
        ).get_parameter_value().double_value
        self.angular_gain = self.get_parameter(
            'angular_gain').get_parameter_value().double_value
        self.linear_gain = self.get_parameter(
            'linear_gain').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter(
            'max_angular_speed').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter(
            'max_linear_speed').get_parameter_value().double_value
        self.min_linear_speed = self.get_parameter(
            'min_linear_speed').get_parameter_value().double_value
        self.min_detection_weight = self.get_parameter(
            'min_detection_weight').get_parameter_value().double_value
        self.hog_scale = self.get_parameter(
            'hog_scale').get_parameter_value().double_value
        self.stop_when_no_detection = self.get_parameter(
            'stop_when_no_detection').get_parameter_value().bool_value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=self.get_reliability_policy(),
        )

        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.led_state_pub = self.create_publisher(
            String, self.led_state_topic, 10
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos
        )

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.last_detection_time = None
        self.last_led_state = ''

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
        """Run person detection on the latest frame and publish motion."""
        with self.frame_lock:
            if self.latest_frame is None:
                self.handle_missing_detection('waiting_for_images')
                return

            frame = self.latest_frame.copy()

        detection = self.detect_person(frame)
        if detection is None:
            self.handle_missing_detection('no_person_detected')
            return

        x, y, w, h = detection
        now = self.get_clock().now()
        self.last_detection_time = now

        frame_height, frame_width = frame.shape[:2]
        center_x = x + (w / 2.0)
        center_error = ((center_x / frame_width) - 0.5) * 2.0
        bbox_height_fraction = h / float(frame_height)
        distance_error = self.target_height_fraction - bbox_height_fraction

        angular_speed = 0.0
        if abs(center_error) > self.center_deadband:
            angular_speed = -self.angular_gain * center_error

        linear_speed = max(distance_error * self.linear_gain, 0.0)
        alignment = 1.0 - (
            abs(center_error) / max(self.turn_slowdown_threshold, 1e-3)
        )
        alignment = self.clamp(alignment, 0.0, 1.0)
        linear_speed *= alignment

        if linear_speed > 0.0:
            linear_speed = max(linear_speed, self.min_linear_speed)

        cmd = Twist()
        cmd.angular.z = self.clamp(
            angular_speed, -self.max_angular_speed, self.max_angular_speed
        )
        cmd.linear.x = self.clamp(
            linear_speed, 0.0, self.max_linear_speed
        )
        self.cmd_vel_pub.publish(cmd)

        if cmd.linear.x > 0.01:
            self.publish_led_state('following')
        else:
            self.publish_led_state('tracking')

        self.publish_status(
            'tracking_person'
            f' center_error={center_error:.3f}'
            f' bbox_height_fraction={bbox_height_fraction:.3f}'
            f' linear_x={cmd.linear.x:.3f}'
            f' angular_z={cmd.angular.z:.3f}'
        )

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

    def handle_missing_detection(self, reason: str) -> None:
        """Stop or search when no person is currently detected."""
        if self.last_detection_time is None:
            self.publish_led_state('searching')
            self.publish_status(reason)
            if self.stop_when_no_detection:
                self.publish_stop()
            return

        elapsed = (
            self.get_clock().now() - self.last_detection_time
        ).nanoseconds / 1e9

        if elapsed >= self.lost_timeout_sec:
            self.publish_led_state('lost')
            self.publish_status(f'lost_person elapsed={elapsed:.2f}')
            if self.stop_when_no_detection:
                self.publish_stop()
            return

        self.publish_led_state('searching')
        self.publish_status(f'searching elapsed={elapsed:.2f}')
        if self.stop_when_no_detection:
            self.publish_stop()

    def publish_stop(self) -> None:
        """Publish a zero-velocity command."""
        self.cmd_vel_pub.publish(Twist())

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
