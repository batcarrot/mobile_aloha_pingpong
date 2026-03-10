#!/usr/bin/env bash

GENTL_CTI=$(ros2 pkg prefix spinnaker_camera_driver 2>/dev/null)/lib/spinnaker-gentl/Spinnaker_GenTL.cti
if [[ ! -f "$GENTL_CTI" ]]; then
  echo "Spinnaker GenTL not found at $GENTL_CTI. Is ros-jazzy-spinnaker-camera-driver installed?"
  exit 1
fi
export SPINNAKER_GENTL64_CTI="$GENTL_CTI"
exec ros2 launch mobile_aloha_camera flir_dual_camera.launch.py "$@"
