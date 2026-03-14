# esp32_cam_bridge

ROS 2 Python package for:

- bridging an ESP32-S3 camera MJPEG stream into ROS 2 image topics,
- controlling the ESP32 onboard LED over HTTP,
- and running a basic human-following controller from the camera feed.

This package is designed around the updated ESP32 firmware in
[`esp32_s3_firmware.txt`](./esp32_s3_firmware.txt), where:

- control, status, and LED endpoints run on port `80`,
- MJPEG streaming runs on port `81`.

## Package Contents

- `esp32_cam_bridge`
  Reads the ESP32 MJPEG stream and publishes:
  - `/esp32/camera/image_raw`
  - `/esp32/camera/camera_info`
  - `/esp32/camera/status`

  It also listens for LED commands on:
  - `/esp32/led_rgba`
  - `/esp32/led_state`

- `human_follower`
  Subscribes to the camera image, detects the largest visible person with
  OpenCV HOG, and publishes velocity commands on `/cmd_vel`.

- `esp32_camera_follow.launch.py`
  Launches both nodes together using YAML config files.

## Requirements

- ROS 2 Jazzy
- `cv_bridge`
- OpenCV
- `requests`

Package dependencies are declared in `package.xml`.

## Build

```bash
cd /home/lucaspinacosta/criarte_ws
colcon build --packages-select esp32_cam_bridge
source install/setup.bash
```

## Run

Launch both nodes:

```bash
ros2 launch esp32_cam_bridge esp32_camera_follow.launch.py
```

Run nodes individually:

```bash
ros2 run esp32_cam_bridge esp32_cam_bridge
ros2 run esp32_cam_bridge human_follower
```

## Config Files

- [`config/esp32_cam_bridge.yaml`](./config/esp32_cam_bridge.yaml)
- [`config/human_follower.yaml`](./config/human_follower.yaml)

Override them at launch time:

```bash
ros2 launch esp32_cam_bridge esp32_camera_follow.launch.py \
  bridge_config:=/absolute/path/to/esp32_cam_bridge.yaml \
  follower_config:=/absolute/path/to/human_follower.yaml
```

## ESP32 API Assumptions

The bridge expects the ESP32 firmware to expose:

- `GET /status` on port `80`
- `GET /led` on port `80`
- `GET /led/off` on port `80`
- MJPEG stream on `http://<esp32_ip>:81/stream`

Default bridge parameters already match that layout.

## Important Topics

Published:

- `/esp32/camera/image_raw`
- `/esp32/camera/camera_info`
- `/esp32/camera/status`
- `/human_follower/status`
- `/cmd_vel`

Subscribed:

- `/esp32/led_rgba`
- `/esp32/led_state`
- `/esp32/camera/image_raw`

## TF

The camera bridge publishes images with frame id `esp32_camera`.

Example static TF:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link \
  --child-frame-id esp32_camera
```

Optional optical frame:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll -1.5708 --pitch 0 --yaw -1.5708 \
  --frame-id esp32_camera \
  --child-frame-id esp32_camera_optical_frame
```

## Main Parameters

### `esp32_cam_bridge`

- `esp32_ip`
- `control_port`
- `stream_port`
- `stream_path`
- `image_topic`
- `camera_info_topic`
- `frame_id`
- `publish_fps`
- `http_timeout_sec`
- `image_qos_reliability`

### `human_follower`

- `image_topic`
- `cmd_vel_topic`
- `process_fps`
- `lost_timeout_sec`
- `target_height_fraction`
- `center_deadband`
- `turn_slowdown_threshold`
- `angular_gain`
- `linear_gain`
- `max_angular_speed`
- `max_linear_speed`
- `min_linear_speed`
- `min_detection_weight`
- `hog_scale`
- `stop_when_no_detection`

## Notes

- The `human_follower` node currently drives `/cmd_vel` directly. It is not yet a
  Nav2 goal-based tracker.
- Person detection uses OpenCV HOG. It is lightweight, but accuracy is limited
  compared with a modern neural detector.
- `CameraInfo` is currently a valid uncalibrated estimate, not a real camera
  calibration.
