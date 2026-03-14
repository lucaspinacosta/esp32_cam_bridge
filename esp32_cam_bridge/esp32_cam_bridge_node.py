#!/usr/bin/env python3
"""ROS 2 bridge for an ESP32-CAM HTTP stream and LED API."""

import threading
from typing import Dict, Optional, Tuple

import cv2
import requests
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String


class Esp32CamBridgeNode(Node):
    """Publish ESP32-CAM frames and forward LED commands over HTTP."""

    def __init__(self) -> None:
        """Initialize parameters, ROS interfaces, and timers."""
        super().__init__('esp32_cam_bridge')

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter('esp32_ip', '192.168.1.181')
        self.declare_parameter('control_port', 80)
        self.declare_parameter('stream_port', 81)
        self.declare_parameter('stream_path', '/stream')
        self.declare_parameter('image_topic', '/esp32/camera/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/esp32/camera/camera_info'
        )
        self.declare_parameter('frame_id', 'esp32_camera')
        self.declare_parameter('publish_fps', 15.0)
        self.declare_parameter('http_timeout_sec', 0.5)
        self.declare_parameter('image_qos_reliability', 'reliable')
        self.declare_parameter(
            'jpeg_quality_note',
            'ESP32 should stream JPEG/MJPEG'
        )

        self.esp32_ip = self.get_parameter(
            'esp32_ip').get_parameter_value().string_value
        self.control_port = self.get_parameter(
            'control_port').get_parameter_value().integer_value
        self.stream_port = self.get_parameter(
            'stream_port').get_parameter_value().integer_value
        self.stream_path = self.get_parameter(
            'stream_path').get_parameter_value().string_value
        self.image_topic = self.get_parameter(
            'image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter(
            'camera_info_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.publish_fps = self.get_parameter(
            'publish_fps').get_parameter_value().double_value
        self.http_timeout_sec = self.get_parameter(
            'http_timeout_sec').get_parameter_value().double_value
        self.image_qos_reliability = self.get_parameter(
            'image_qos_reliability'
        ).get_parameter_value().string_value.lower()

        self.stream_url = (
            f'http://{self.esp32_ip}:{self.stream_port}{self.stream_path}'
        )
        self.led_url = f'http://{self.esp32_ip}:{self.control_port}/led'
        self.led_off_url = (
            f'http://{self.esp32_ip}:{self.control_port}/led/off'
        )
        self.status_url = f'http://{self.esp32_ip}:{self.control_port}/status'

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        reliability_policy = self.get_reliability_policy()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=reliability_policy,
        )

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(
            Image, self.image_topic, sensor_qos
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.camera_info_topic, sensor_qos
        )
        self.status_pub = self.create_publisher(
            String, '/esp32/camera/status', 10)

        self.led_rgba_sub = self.create_subscription(
            ColorRGBA,
            '/esp32/led_rgba',
            self.led_rgba_callback,
            10
        )

        self.led_state_sub = self.create_subscription(
            String,
            '/esp32/led_state',
            self.led_state_callback,
            10
        )

        # ----------------------------
        # Runtime state
        # ----------------------------
        self.cap = None
        self.cap_lock = threading.RLock()
        self.frame_lock = threading.Lock()
        self.last_led: Optional[Tuple[int, int, int, int]] = None
        self.latest_frame = None
        self.capture_stop_event = threading.Event()
        self.capture_thread = threading.Thread(
            target=self.capture_loop,
            name='esp32_capture',
            daemon=True,
        )

        self.state_map: Dict[str, Tuple[int, int, int, int]] = {
            'off': (0, 0, 0, 0),
            'idle': (255, 255, 255, 50),
            'ready': (255, 255, 255, 20),
            'connected': (0, 0, 255, 60),
            'tracking': (0, 255, 0, 70),
            'searching': (255, 120, 0, 70),
            'lost': (255, 0, 0, 80),
            'following': (180, 0, 255, 80),
        }

        # ----------------------------
        # Timers
        # ----------------------------
        period = 1.0 / max(self.publish_fps, 1.0)
        self.frame_timer = self.create_timer(
            period, self.frame_timer_callback
        )
        self.health_timer = self.create_timer(
            5.0, self.health_timer_callback
        )

        # ----------------------------
        # Startup
        # ----------------------------
        self.get_logger().info(f'ESP32 stream URL: {self.stream_url}')
        self.get_logger().info(
            'Image QoS reliability: '
            f'{self.image_qos_reliability}'
        )
        self.send_led_named_state('ready')
        self.open_stream()
        self.capture_thread.start()

    # --------------------------------------------------
    # Stream handling
    # --------------------------------------------------
    def open_stream(self) -> None:
        """Open or reopen the ESP32 video stream."""
        with self.cap_lock:
            if self.cap is not None:
                self.cap.release()
                self.cap = None

            self.get_logger().info('Opening ESP32 stream...')
            cap = cv2.VideoCapture(self.stream_url)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            if not cap.isOpened():
                self.get_logger().error(
                    f'Failed to open stream: {self.stream_url}')
                self.clear_latest_frame()
                self.publish_status('stream_open_failed')
                return

            self.cap = cap
            self.get_logger().info('ESP32 stream opened.')
            self.publish_status('stream_opened')
            self.send_led_named_state('connected')

    def capture_loop(self) -> None:
        """Continuously read frames and keep only the latest one."""
        while not self.capture_stop_event.is_set():
            with self.cap_lock:
                cap = self.cap

            if cap is None:
                self.open_stream()
                self.capture_stop_event.wait(0.25)
                continue

            ok, frame = cap.read()

            if not ok or frame is None:
                self.get_logger().warn(
                    'Frame read failed. Reconnecting stream...'
                )
                self.clear_latest_frame()
                self.publish_status('frame_read_failed')
                self.open_stream()
                self.capture_stop_event.wait(0.1)
                continue

            with self.frame_lock:
                self.latest_frame = frame

    def frame_timer_callback(self) -> None:
        """Publish the latest available frame to ROS topics."""
        with self.frame_lock:
            if self.latest_frame is None:
                return

            frame = self.latest_frame.copy()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)
        self.camera_info_pub.publish(
            self.build_camera_info(
                width=frame.shape[1],
                height=frame.shape[0],
                stamp=stamp,
            )
        )

    def health_timer_callback(self) -> None:
        """Poll the ESP32 health endpoint and publish status updates."""
        try:
            response = requests.get(
                self.status_url, timeout=self.http_timeout_sec)
            if response.ok:
                self.publish_status(f'esp32_ok:{response.text}')
            else:
                self.publish_status(
                    f'esp32_http_error:{response.status_code}'
                )
        except requests.RequestException as exc:
            self.publish_status(f'esp32_unreachable:{exc}')

    # --------------------------------------------------
    # LED control
    # --------------------------------------------------
    def led_rgba_callback(self, msg: ColorRGBA) -> None:
        """Translate an RGBA message into an HTTP LED command."""
        r = int(max(0.0, min(1.0, msg.r)) * 255.0)
        g = int(max(0.0, min(1.0, msg.g)) * 255.0)
        b = int(max(0.0, min(1.0, msg.b)) * 255.0)
        brightness = int(max(0.0, min(1.0, msg.a)) * 255.0)

        self.send_led(r, g, b, brightness)

    def led_state_callback(self, msg: String) -> None:
        """Translate a named LED state message into an HTTP LED command."""
        self.send_led_named_state(msg.data.strip().lower())

    def send_led_named_state(self, state: str) -> None:
        """Send a predefined LED color/brightness state to the ESP32."""
        if state not in self.state_map:
            self.get_logger().warn(f'Unknown LED state: {state}')
            return

        r, g, b, brightness = self.state_map[state]
        self.send_led(r, g, b, brightness)

    def send_led(self, r: int, g: int, b: int, brightness: int) -> None:
        """Send an LED command only when the target state changes."""
        new_value = (r, g, b, brightness)

        if self.last_led == new_value:
            return

        try:
            if brightness <= 0 or (r == 0 and g == 0 and b == 0):
                response = requests.get(
                    self.led_off_url, timeout=self.http_timeout_sec)
            else:
                response = requests.get(
                    self.led_url,
                    params={
                        'r': r,
                        'g': g,
                        'b': b,
                        'brightness': brightness,
                    },
                    timeout=self.http_timeout_sec
                )

            response.raise_for_status()
            self.last_led = new_value
        except requests.RequestException as exc:
            self.get_logger().warn(f'LED HTTP request failed: {exc}')

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------
    def get_reliability_policy(self) -> ReliabilityPolicy:
        """Translate the configured QoS reliability string to ROS policy."""
        if self.image_qos_reliability == 'best_effort':
            return ReliabilityPolicy.BEST_EFFORT

        if self.image_qos_reliability != 'reliable':
            self.get_logger().warn(
                'Invalid image_qos_reliability value '
                f'"{self.image_qos_reliability}", using reliable'
            )
            self.image_qos_reliability = 'reliable'

        return ReliabilityPolicy.RELIABLE

    def clear_latest_frame(self) -> None:
        """Drop any cached frame when the stream becomes unavailable."""
        with self.frame_lock:
            self.latest_frame = None

    def build_camera_info(
        self,
        width: int,
        height: int,
        stamp,
    ) -> CameraInfo:
        """Build a minimal uncalibrated CameraInfo message."""
        cx = (width - 1) / 2.0
        cy = (height - 1) / 2.0
        focal_length = float(max(width, height))

        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = width
        msg.height = height
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [
            focal_length, 0.0, cx,
            0.0, focal_length, cy,
            0.0, 0.0, 1.0,
        ]
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            focal_length, 0.0, cx, 0.0,
            0.0, focal_length, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    def publish_status(self, text: str) -> None:
        """Publish a bridge status string."""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        """Release the video stream before shutting down the node."""
        self.capture_stop_event.set()
        if self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)

        with self.cap_lock:
            if self.cap is not None:
                self.cap.release()
                self.cap = None

        return super().destroy_node()


def main(args=None) -> None:
    """Run the ESP32 camera bridge node."""
    rclpy.init(args=args)
    node = Esp32CamBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
