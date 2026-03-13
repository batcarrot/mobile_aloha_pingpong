# mobile_aloha_pingpong

ROS 2 (Jazzy) workspace for the mobile ALOHA ping-pong setup: Trossen arms, dual FLIR Blackfly S cameras (720×540 @ 200 Hz), and ball position from stereo detection.

## Repository layout

| Package | Role |
|---------|------|
| **trossen_arm_bringup** | Arm bringup only: controllers, mobile_ai launch, RViz, demos. Optional single FLIR in URDF via `use_flir_camera`. |
| **mobile_aloha_camera** | Dual FLIR camera launch and config (720×540 @ 200 Hz), run script. |
| **ball_detection_ros** | Ball position node (subscribes to cam images, publishes 3D position). Uses logic from robotpingpong/camera. |

Plus the rest of `trossen_arm_ros` (description, hardware, moveit, controllers) and `trossen_arm_description`, `trossen_arm`.

## Prerequisites

- ROS 2 Jazzy
- [Spinnaker Camera Driver](https://github.com/ros-drivers/flir_camera_driver) (e.g. `ros-jazzy-spinnaker-camera-driver`)
- GenTL setup for USB discovery (run `ros2 run spinnaker_camera_driver linux_setup_flir` if needed, then reboot)

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to mobile_aloha_camera ball_detection_ros
source install/setup.bash
```

## Dual FLIR cameras (mobile_aloha_camera)

Launch both Blackfly S cameras (cam0, cam1) with 720×540 @ 200 Hz:

```bash
ros2 launch mobile_aloha_camera flir_dual_camera.launch.py
```

Or use the helper script (sets GenTL env then launches):

```bash
ros2 run mobile_aloha_camera run_flir_dual.sh
```

**Launch arguments:** `use_cam0`, `use_cam1`, `cam0_serial` (default `'25505853'`), `cam1_serial` (default `'25505854'`).

**Topics:** `/cam0/image_raw`, `/cam0/camera_info`, `/cam1/image_raw`, `/cam1/camera_info`.

**Config:** `mobile_aloha_camera/config/flir_dual_camera.yaml`, `camera_serials.yaml`.

## Ball position (ball_detection_ros)

Subscribes to `/cam0/image_raw` and `/cam1/image_raw`, runs robotpingpong-style detection (background subtract, epipolar intersection, optional Kalman), publishes 3D position. Calibration: use a `calibration.npz` from robotpingpong/camera (e.g. from `cam_localization.py`).

**Run with cameras already running:**

```bash
ros2 run ball_detection_ros ball_position_node.py --ros-args -p calibration_file:=/path/to/calibration.npz
```

**Launch cameras + ball node together:**

```bash
ros2 launch ball_detection_ros ball_position.launch.py calibration_file:=/path/to/calibration.npz

this vvvvv
ros2 launch ball_detection_ros ball_position.launch.py calibration_file:=/home/trossen-ai/Downloads/calibration.npz
```

**Output topics:** `ball_position_node/ball_position` (`geometry_msgs/PointStamped`), `ball_position_node/ball_velocity` (`geometry_msgs/Vector3`). Parameters: `median_frames`, `no_ball_reset_frames`, `max_intersect_error`, `use_kalman`; optional `cam0_topic`, `cam1_topic`.

## Arms (trossen_arm_bringup)

Bring up mobile_ai (dual arms, optional single camera in URDF, RViz):

```bash
ros2 launch trossen_arm_bringup mobile_ai.launch.py
```

See package launch and config for controller and hardware options.
