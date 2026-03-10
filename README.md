# mobile_aloha_pingpong

ROS 2 (Jazzy) workspace for the mobile ALOHA ping-pong setup: Trossen arms and dual FLIR Blackfly S cameras (720×540 @ 200 Hz) for robotpingpong calibration and ball detection.

## Prerequisites

- ROS 2 Jazzy
- [Spinnaker Camera Driver](https://github.com/ros-drivers/flir_camera_driver) (e.g. `ros-jazzy-spinnaker-camera-driver`)
- GenTL setup for USB discovery (run `ros2 run spinnaker_camera_driver linux_setup_flir` if needed, then reboot)

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select trossen_arm_bringup
source install/setup.bash
```

## Dual FLIR cameras

Launch both Blackfly S cameras (cam0, cam1) with the project config (720×540, 200 Hz):

```bash
source install/setup.bash
ros2 launch trossen_arm_bringup flir_dual_camera.launch.py
```

Or use the helper script (sets `SPINNAKER_GENTL64_CTI` then launches):

```bash
ros2 run trossen_arm_bringup run_flir_dual.sh
```

**Launch arguments**

| Argument       | Default     | Description              |
|----------------|-------------|--------------------------|
| `use_cam0`     | `true`      | Launch cam0              |
| `use_cam1`     | `true`      | Launch cam1              |
| `cam0_serial`  | `'25505853'`| Serial for cam0 (string) |
| `cam1_serial`  | `'25505854'`| Serial for cam1 (string) |

Example: run only cam0 for testing:

```bash
ros2 launch trossen_arm_bringup flir_dual_camera.launch.py use_cam1:=false
```

**Topics**

- `/cam0/image_raw` — `sensor_msgs/msg/Image`
- `/cam0/camera_info` — `sensor_msgs/msg/CameraInfo`
- `/cam1/image_raw` — `sensor_msgs/msg/Image`
- `/cam1/camera_info` — `sensor_msgs/msg/CameraInfo`

**View images**

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/cam0/image_raw` or `/cam1/image_raw` from the dropdown.

**Config**

- `trossen_arm_bringup/config/flir_dual_camera.yaml` — frame rate, resolution, exposure, throughput limit.
- `trossen_arm_bringup/config/camera_serials.yaml` — reference for cam0/cam1 serials (robotpingpong calibration).

If cameras are not found, ensure `SPINNAKER_GENTL64_CTI` points to the Spinnaker GenTL `.cti` file (the launch file sets it automatically; the script does too).

