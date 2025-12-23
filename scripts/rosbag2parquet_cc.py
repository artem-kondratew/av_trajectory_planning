#!/usr/bin/env python3

import argparse
from pathlib import Path

import pandas as pd

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_topics_file(path: str) -> dict[str, type]:
    """
    Reads topics file of format:
    <topic_name> <ros_msg_type>

    Example:
    /carla/hero/imu sensor_msgs/msg/Imu
    """
    topic_type_map = {}

    with open(path, "r") as f:
        for line in f:
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) != 2:
                raise RuntimeError(
                    f"Invalid line in topics file (expected: <topic> <type>):\n{line}"
                )

            topic, msg_type_str = parts
            msg_type = get_message(msg_type_str)

            topic_type_map[topic] = msg_type

    return topic_type_map


def extract_fields(msg) -> dict:
    """
    Extracts fields from message into flat dict.
    This is explicit on purpose (no reflection magic).
    """
    row = {}

    # Vehicle control
    if msg.__class__.__name__ == "CarlaEgoVehicleControl":
        row.update({
            "throttle": msg.throttle,
            "steer": msg.steer,
            "brake": msg.brake,
            "hand_brake": msg.hand_brake,
            "reverse": msg.reverse,
            "gear": msg.gear,
            "manual_gear": msg.manual_gear_shift,
        })

    # Float64
    elif hasattr(msg, "data"):
        row["data"] = msg.data

    # Vector3
    elif msg.__class__.__name__ == "Vector3":
        row.update({
            "x": msg.x,
            "y": msg.y,
            "z": msg.z,
        })

    # Twist
    elif msg.__class__.__name__ == "Twist":
        row.update({
            "vx": msg.linear.x,
            "vy": msg.linear.y,
            "vz": msg.linear.z,
            "wx": msg.angular.x,
            "wy": msg.angular.y,
            "wz": msg.angular.z,
        })

    # IMU
    elif msg.__class__.__name__ == "Imu":
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
        description="Convert rosbag2 to parquet (topics + types from file)"
    )

    parser.add_argument(
        "bag_path",
        type=str,
        help="Path to rosbag2 directory"
    )

    parser.add_argument(
        "--topics-file",
        required=True,
        help="Text file: <topic> <msg_type>"
    )

    parser.add_argument(
        "--out",
        default=None,
        help="Output parquet file (default: next to bag)"
    )

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
            "t": t,                         # nanoseconds
            "topic": topic,
            "type": msg_type.__name__,
        }

        row.update(extract_fields(msg))
        rows.append(row)

    if not rows:
        raise RuntimeError("No messages matched selected topics")

    df = pd.DataFrame(rows)

    bag_path = Path(args.bag_path).resolve()
    if args.out is None:
        out_path = bag_path.with_suffix(".parquet")
    else:
        out_path = Path(args.out).resolve()

    df.to_parquet(out_path, engine="pyarrow")

    print(f"[OK] Saved parquet to: {out_path}")
    print(f"[INFO] Rows: {len(df)}")
    print(f"[INFO] Topics: {sorted(df['topic'].unique())}")


if __name__ == "__main__":
    main()
