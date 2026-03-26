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

first terminal
```

ros2 launch ball_detection_ros ball_position.launch.py calibration_file:=/home/trossen-ai/Downloads/calibration.npz

```
second term
```
ros2 launch trossen_arm_bringup trossen_arm.launch.py ip_address:=192.168.1.5
```
if you want sim:
```
ros2 launch trossen_arm_bringup trossen_arm.launch.py ros2_control_hardware_type:=mock_components
```
third term
```
ros2 run mpc_ros mpc_node.py
```

### MPC bag recording and MCAP conversion

Record `/mpc_step` to a timestamped folder under `~/bags` (each run gets its own `mpc_run_YYYYMMDD_HHMMSS` directory):

```bash
ros2 bag record -o ~/bags/mpc_run_$(date +%Y%m%d_%H%M%S) /mpc_step
```

Convert that recording to the `mpc_data.pkl`-style pickle. From the workspace root, set `--dir` to the **folder that contains the `.mcap` you want**—usually the run directory for the latest (or chosen) bag, e.g. replace the path with your own `mpc_run_*` folder name:

```bash
.venv/bin/python conversion.py --dir /home/trossen-ai/bags/mpc_run_20260325_205317
```

The script picks the newest `.mcap` in that directory (by the `YYYYMMDD_HHMMSS` in the filename) and writes `pkl/mpc_data_<timestamp>.pkl` next to it. Requires `mcap` and `mcap-ros2-support` in the project venv (`python3 -m venv .venv && .venv/bin/pip install mcap mcap-ros2-support numpy`).

if you only want prediction
```
ros2 run mpc_ros ball_prediction_node.py
```
rviz:
```
ros2 run rviz2 rviz2
```
incase calibration 

install sdk full and python sdk from: https://www.teledynevisionsolutions.com/products/spinnaker-sdk/?model=Spinnaker%20SDK&vertical=machine%20vision&segment=iis

python=3.10

install my repo with `pip install -e .`


python -m robotpingpong.camera.calibration_web.py

anytime you change something
in all the terminal
```
colcon build
source install/setup.bash
```