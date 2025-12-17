#!/usr/bin/env bash
set -e

if [ $# -ne 1 ]; then
  echo "Usage: $0 <output_path/bag_name>"
  echo "Example:"
  echo "  $0 ../data/first"
  exit 1
fi

BAG_OUT="$1"

echo "[INFO] Recording rosbag to: $BAG_OUT"
echo "[INFO] Press Ctrl+C to stop recording"

ros2 bag record \
  -o "$BAG_OUT" \
  /carla/hero/vehicle_control_cmd \
  /cruise_controller/v_ref \
  /cruise_controller/y_vector \
  /carla/hero/local_velocity \
  /carla/hero/imu
