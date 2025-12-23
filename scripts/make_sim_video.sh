#!/usr/bin/env bash

set -e

if [ $# -lt 1 ]; then
    echo "Usage: $0 <frames_dir> [fps] [output]"
    echo "Example: $0 /data/cc_two_levels 30 output.mp4"
    exit 1
fi

FRAMES_DIR="$(realpath "$1")"
FPS="${2:-30}"
OUTPUT="${3:-output.mp4}"

if [ ! -d "$FRAMES_DIR" ]; then
    echo "Error: directory does not exist: $FRAMES_DIR"
    exit 1
fi

TMP_LIST="$(mktemp)"

echo "[INFO] Collecting frames from: $FRAMES_DIR"
echo "[INFO] FPS: $FPS"
echo "[INFO] Output: $OUTPUT"

find "$FRAMES_DIR" -name "t_*.png" \
    | sort -V \
    | awk '{print "file '\''" $0 "'\''"}' \
    > "$TMP_LIST"

FRAME_COUNT=$(wc -l < "$TMP_LIST")
echo "[INFO] Frames: $FRAME_COUNT"

if [ "$FRAME_COUNT" -eq 0 ]; then
    echo "Error: no frames found"
    rm "$TMP_LIST"
    exit 1
fi

ffmpeg -y \
    -r "$FPS" \
    -f concat -safe 0 \
    -i "$TMP_LIST" \
    -pix_fmt yuv420p \
    "$OUTPUT"

rm "$TMP_LIST"

echo "[OK] Video generated: $OUTPUT"

