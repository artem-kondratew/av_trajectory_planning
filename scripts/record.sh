#!/usr/bin/env bash
set -e

DATA_DIR="/data"

if [ $# -ne 1 ]; then
  echo "Usage: $0 <bag_name>"
  echo "Example:"
  echo "  $0 first"
  exit 1
fi

BAG_NAME="$1"
BAG_OUT="${DATA_DIR}/${BAG_NAME}"

# Проверка, что /data существует
if [ ! -d "$DATA_DIR" ]; then
  echo "[ERROR] Directory '$DATA_DIR' does not exist"
  exit 1
fi

# Проверка, что такого бэга ещё нет
if [ -e "$BAG_OUT" ]; then
  echo "[ERROR] Bag '$BAG_OUT' already exists"
  echo "        Choose another name or remove the existing bag"
  exit 1
fi

echo "[INFO] Recording rosbag to: $BAG_OUT"
echo "[INFO] Press Ctrl+C to stop recording"

ros2 bag record \
  -o "$BAG_OUT" \
  /carla/hero/vehicle_control_cmd \
  /cruise_controller/v_ref \
  /cruise_controller/y_vector \
  /carla/hero/local_velocity \
  /carla/hero/imu
