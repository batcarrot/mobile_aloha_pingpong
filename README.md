# mobile_aloha_pingpong

ROS 2 (Jazzy) workspace for the mobile ALOHA ping-pong setup: Trossen arms, dual FLIR Blackfly S cameras (720×540 @ 200 Hz), and ball position from stereo detection.

## Repository layout

| Package | Role |
|---------|------|
| **trossen_arm_bringup** | Arm bringup only: controllers, mobile_ai launch, RViz, demos. Optional single FLIR in URDF via `use_flir_camera`. |
| **mobile_aloha_camera** | Dual FLIR camera launch and config (720×540 @ 200 Hz), run script. |
| **ball_detection_ros** | Ball position (FLIR stereo), optional D405 RGB ball filter, RGB debug capture (`d405_calibration_snapshot.py`). Uses logic from robotpingpong/camera. |

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

Subscribes to `/cam0/image_raw` and `/cam1/image_raw`, runs robotpingpong-style detection (background subtract, epipolar intersection, optional Kalman), and publishes map-frame outputs. Supply a `calibration.npz` that includes **cam0**, **cam1**, and (for D405) **cam2** (see [Camera calibration](#camera-calibration) below).

**Run with cameras already running:**

```bash
ros2 run ball_detection_ros ball_position_node.py --ros-args -p calibration_file:=/path/to/calibration.npz
```

**Launch FLIR cameras + ball node together:**

```bash
ros2 launch ball_detection_ros ball_position.launch.py \
  calibration_file:=/path/to/calibration.npz
```

**Same launch with optional D405 RGB ball filter** (requires **D405 running** with color topics, and **cam2** in the npz; gates `~/ball_pos`, `~/stereo_ball_hypothesis`, and state updates on HSV in the projected ROI):

```bash
ros2 launch ball_detection_ros ball_position.launch.py \
  calibration_file:=/home/trossen-ai/robotpingpong/robotpingpong/data/calibration.npz \
  use_rgb_ball_filter:=true
```

Filter-related parameters (all on `ball_position_node`): `rgb_topic` (default `/d405/d405/color/image_rect_raw`), `camera_info_topic`, `rgb_sync_max_dt`, `rgb_roi_radius`, `rgb_min_good_fraction`, `rgb_gate_allow_without_image`, and HSV knobs `rgb_h_orange_lo` / `rgb_h_orange_hi` / etc. Full RGB debug (FLIR + D405 + captures): `ros2 launch ball_detection_ros ball_rgb_debug.launch.py` (supports the same `use_rgb_ball_filter` launch arg).

**Output topics:** `ball_position_node/ball_pos` (`geometry_msgs/PointStamped`, map frame), `ball_position_node/ball_state` (`ball_state_msgs/BallState`), and optionally `ball_position_node/stereo_ball_hypothesis` (`geometry_msgs/PointStamped`, `stereo_calib`). Other parameters: `median_frames`, `no_ball_reset_frames`, `max_intersect_error`, `use_kalman`, `cam0_topic`, `cam1_topic`.

## Camera calibration

**Goal:** The FLIR stereo model and D405 **cam2** intrinsics/extrinsics must match the **same pixel streams** you use in ROS (resolution, raw vs rectified).

1. **FLIR (cam0 / cam1)**  
   Keep using your robotpingpong flow (e.g. `python -m robotpingpong.camera.calibration_web` or equivalent) for the table quad and `obj_corners`. Per-camera `K` in the npz should match **720×540** (or whatever `flir_dual_camera` publishes).

2. **D405 aligned with `realsense2_camera`**  
   The driver often publishes **`/d405/d405/color/image_rect_raw`** and **`/d405/d405/color/camera_info`** (not `color/image_raw`). For intrinsics that match ROS **without** running calibration inside ROS every time, grab one snapshot after `d405_color.launch.py` is running:

   ```bash
   ros2 run ball_detection_ros d405_calibration_snapshot.py --ros-args -p output_dir:=/tmp/d405_cal_snap
   ```

   That writes **`d405_frame.png`** (same pixels as `image_rect_raw`) and **`d405_intrinsics.json`** (`K`, `dist`, width, height). Use that image for RealSense corner clicks and pass **`K` / `dist`** into your `calibrate()` / web tool as **`realsense_K`** / **`realsense_dist`** so **`cam2`** in `calibration.npz` matches the ROS rectified stream.

3. **Profile parity**  
   Match **`depth_module.color_profile`** in `mobile_aloha_camera/launch/d405_color.launch.py` (default **848x480x30**) to the resolution you used when saving **`d405_frame.png`** / building **cam2**.

4. **Optional `T_cam2_from_stereo`**  
   If FLIR and D405 quads use different object-frame definitions, store a **4×4** **`T_cam2_from_stereo`** in the npz; otherwise consumers assume identity between stereo and D405 PnP frames.

**Spinnaker / GenTL** (FLIR): see [Prerequisites](#prerequisites) and Teledyne Spinnaker SDK install notes at the end of this file if cameras are not discovered.

## Arms (trossen_arm_bringup)

Bring up mobile_ai (dual arms, optional single camera in URDF, RViz):

```bash
ros2 launch trossen_arm_bringup mobile_ai.launch.py
```

See package launch and config for controller and hardware options.

**Example multi-terminal workflow**

First terminal (stereo + ball; set `calibration_file` to your npz; add `use_rgb_ball_filter:=true` if D405 is up and you want the RGB gate):

```bash
ros2 launch ball_detection_ros ball_position.launch.py \
  calibration_file:=/home/trossen-ai/robotpingpong/robotpingpong/data/calibration.npz
```

Second terminal:

```bash
ros2 launch trossen_arm_bringup trossen_arm.launch.py ip_address:=192.168.1.5
```

If you want sim:

```bash
ros2 launch trossen_arm_bringup trossen_arm.launch.py ros2_control_hardware_type:=mock_components
```

Third terminal:

```bash
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
### FLIR SDK / offline calibration web tool

Install the Spinnaker SDK and Python bindings from [Teledyne Spinnaker](https://www.teledynevisionsolutions.com/products/spinnaker-sdk/?model=Spinnaker%20SDK&vertical=machine%20vision&segment=iis) (your environment may use Python 3.10 for that stack).

Install your robotpingpong repo (editable): `pip install -e .`

Run the browser calibration tool (FLIR + optional RealSense in that repo):

```bash
python -m robotpingpong.camera.calibration_web
```

For D405 intrinsics that match **`realsense2_camera`** in ROS, prefer capturing **`d405_frame.png` / `d405_intrinsics.json`** with `d405_calibration_snapshot.py` (see [Camera calibration](#camera-calibration)) and feeding those values into your npz pipeline.

anytime you change something
in all the terminal
```
colcon build
source install/setup.bash
```