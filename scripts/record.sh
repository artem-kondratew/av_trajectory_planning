#!/usr/bin/env bash
set -e

DATA_DIR="/data"

# Check arguments
if [ $# -ne 2 ]; then
  echo "Usage: $0 <bag_name> <topics_file>"
  echo
  echo "Example:"
  echo "  $0 cc_two_levels /topics.txt"
  exit 1
fi

BAG_NAME="$1"
TOPICS_FILE="$2"
BAG_OUT="${DATA_DIR}/${BAG_NAME}"

# Check that data directory exists
if [ ! -d "$DATA_DIR" ]; then
  echo "[ERROR] Directory '$DATA_DIR' does not exist"
  exit 1
fi

# Check that topics file exists
if [ ! -f "$TOPICS_FILE" ]; then
  echo "[ERROR] Topics file '$TOPICS_FILE' does not exist"
  exit 1
fi

# Check that topics file is not empty
if [ ! -s "$TOPICS_FILE" ]; then
  echo "[ERROR] Topics file '$TOPICS_FILE' is empty"
  exit 1
fi

echo "[INFO] Recording rosbag to: $BAG_OUT"
echo "[INFO] Using topics from: $TOPICS_FILE"
echo "[INFO] Press Ctrl+C to stop recording"

# Record rosbag using topics from file
ros2 bag record $(awk '{print $1}' "$TOPICS_FILE") -o "$BAG_OUT"
