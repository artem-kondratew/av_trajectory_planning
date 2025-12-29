#!/usr/bin/env python3

import argparse
from pathlib import Path
import pandas as pd

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_topics_file(path: str) -> dict[str, type]:
    topic_type_map = {}

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            topic, msg_type_str = line.split()
            topic_type_map[topic] = get_message(msg_type_str)

    return topic_type_map


def extract_fields(topic: str, msg) -> dict:
    """
    Extract fields based on TOPIC semantics, not message class.
    """
    row = {}

    # ---------- vehicle control ----------
    if "vehicle_control_cmd" in topic:
        row.update({
            "throttle": msg.throttle,
            "steer": msg.steer,
            "brake": msg.brake,
            "hand_brake": msg.hand_brake,
            "reverse": msg.reverse,
            "gear": msg.gear,
            "manual_gear": msg.manual_gear_shift,
        })

    # ---------- Float64-like ----------
    elif topic.endswith(("v_ref", "velocity", "dist", "dist_ref")):
        row["data"] = msg.data

    # ---------- Vector3 ----------
    elif topic.endswith(("y_vector", "position")):
        row.update({
            "x": msg.x,
            "y": msg.y,
            "z": msg.z,
        })

    # ---------- IMU ----------
    elif topic.endswith("imu"):
        row.update({
            "ax": msg.linear_acceleration.x,
            "ay": msg.linear_acceleration.y,
            "az": msg.linear_acceleration.z,
            "gx": msg.angular_velocity.x,
            "gy": msg.angular_velocity.y,
            "gz": msg.angular_velocity.z,
        })

    return row


def main():
    parser = argparse.ArgumentParser(
        description="Convert rosbag2 to parquet using topic-based semantics"
    )

    parser.add_argument("bag_path", type=str)
    parser.add_argument("--topics-file", required=True)
    parser.add_argument("--out", default=None)

    args = parser.parse_args()

    topic_type_map = read_topics_file(args.topics_file)
    topics = set(topic_type_map.keys())

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag_path, storage_id="sqlite3"),
        ConverterOptions("", "")
    )

    rows = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in topics:
            continue

        msg_type = topic_type_map[topic]
        msg = deserialize_message(data, msg_type)

        row = {
            "t": t,
            "topic": topic,
            "type": msg_type.__name__,
        }

        row.update(extract_fields(topic, msg))
        rows.append(row)

    if not rows:
        raise RuntimeError("No messages matched selected topics")

    df = pd.DataFrame(rows)

    bag_path = Path(args.bag_path).resolve()
    out_path = (
        Path(args.out).resolve()
        if args.out
        else bag_path.with_suffix(".parquet")
    )

    df.to_parquet(out_path, engine="pyarrow")

    print(f"[OK] Saved parquet to: {out_path}")
    print(f"[INFO] Rows: {len(df)}")
    print(f"[INFO] Topics:")
    for t in sorted(df["topic"].unique()):
        print(f"  {t}")


if __name__ == "__main__":
    main()
