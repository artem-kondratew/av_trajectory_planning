#!/usr/bin/env python3

import argparse
from pathlib import Path

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message

from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

import pandas as pd


TOPIC_TYPE_MAP = {
    "/carla/hero/vehicle_control_cmd": CarlaEgoVehicleControl,
    "/cruise_controller/v_ref": Float64,
    "/cruise_controller/y_vector": Vector3,
    "/carla/hero/local_velocity": Twist,
    "/carla/hero/imu": Imu,
}


def read_topics_file(path: str) -> list[str]:
    topics = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            topics.append(line)
    return topics


def main():
    parser = argparse.ArgumentParser(
        description="Convert rosbag2 to parquet (selected topics)"
    )
    parser.add_argument(
        "bag_path",
        type=str,
        help="Path to rosbag2 directory"
    )
    parser.add_argument(
        "--topics-file",
        required=True,
        help="Text file with list of topics"
    )
    parser.add_argument(
        "--out",
        default=None,
        help="Output parquet file (default: next to bag)"
    )

    args = parser.parse_args()

    topics = read_topics_file(args.topics_file)

    unknown = [t for t in topics if t not in TOPIC_TYPE_MAP]
    if unknown:
        raise RuntimeError(
            "Unknown topic(s) without message type mapping:\n"
            + "\n".join(unknown)
        )

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

        msg_type = TOPIC_TYPE_MAP[topic]
        msg = deserialize_message(data, msg_type)

        row = {
            "t": t,        # nanoseconds
            "topic": topic,
        }

        if topic == "/carla/hero/vehicle_control_cmd":
            row.update({
                "throttle": msg.throttle,
                "steer": msg.steer,
                "brake": msg.brake,
                "hand_brake": msg.hand_brake,
                "reverse": msg.reverse,
                "gear": msg.gear,
                "manual_gear": msg.manual_gear_shift,
            })

        elif topic == "/cruise_controller/v_ref":
            row["v_ref"] = msg.data

        elif topic == "/cruise_controller/y_vector":
            row.update({
                "dv": msg.x,
                "a": msg.y,
                "j": msg.z,
            })

        elif topic == "/carla/hero/local_velocity":
            row.update({
                "vx": msg.linear.x,
                "vy": msg.linear.y,
                "vz": msg.linear.z,
                "wx": msg.angular.x,
                "wy": msg.angular.y,
                "wz": msg.angular.z,
            })

        elif topic == "/carla/hero/imu":
            row.update({
                "ax": msg.linear_acceleration.x,
                "ay": msg.linear_acceleration.y,
                "az": msg.linear_acceleration.z,
                "gx": msg.angular_velocity.x,
                "gy": msg.angular_velocity.y,
                "gz": msg.angular_velocity.z,
            })

        rows.append(row)

    df = pd.DataFrame(rows)

    bag_path = Path(args.bag_path).resolve()

    if args.out is None:
        out_path = bag_path.with_suffix(".parquet")
    else:
        out_path = Path(args.out).resolve()

    df.to_parquet(out_path, engine="pyarrow")

    print(f"[OK] Saved parquet to: {out_path}")


if __name__ == "__main__":
    main()
