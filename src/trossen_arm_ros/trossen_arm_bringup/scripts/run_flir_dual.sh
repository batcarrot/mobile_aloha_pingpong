#!/usr/bin/env bash
# Run FLIR dual-camera launch with SPINNAKER_GENTL64_CTI set so the driver can discover USB cameras.
# Use this if cameras are still "not found" (e.g. SetEnvironmentVariable in launch not applied in time).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# GenTL path: from spinnaker_camera_driver share, go up to install, then lib/spinnaker-gentl
GENTL_CTI=$(ros2 pkg prefix spinnaker_camera_driver 2>/dev/null)/lib/spinnaker-gentl/Spinnaker_GenTL.cti
if [[ ! -f "$GENTL_CTI" ]]; then
  echo "Spinnaker GenTL not found at $GENTL_CTI. Is ros-jazzy-spinnaker-camera-driver installed?"
  exit 1
fi
export SPINNAKER_GENTL64_CTI="$GENTL_CTI"
exec ros2 launch trossen_arm_bringup flir_dual_camera.launch.py "$@"
