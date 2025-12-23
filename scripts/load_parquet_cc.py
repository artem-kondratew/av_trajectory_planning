from typing import Dict
from pathlib import Path

import pandas as pd
import pyarrow.parquet as pq


# ------------------------------------------------------------

def read_topics_file(path: str) -> Dict[str, str]:
    """
    Reads topics file:
        <topic> <msg_type>

    Returns:
        dict: topic -> msg_type
    """
    topics = {}

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) < 2:
                raise RuntimeError(f"Invalid topics file line: {line}")

            topic, msg_type = parts[0], parts[1]
            topics[topic] = msg_type

    return topics


# ------------------------------------------------------------

def topic_alias(topic: str) -> str:
    """
    Convert ROS topic to dataframe key.
    """
    if topic.endswith("vehicle_control_cmd"):
        return "control"
    if topic.endswith("/v_ref"):
        return "v_ref"
    if topic.endswith("/y_vector"):
        return "y_vector"
    if topic.endswith("/velocity"):
        return "velocity"
    if topic.endswith("/imu"):
        return "imu"

    # fallback
    return topic.replace("/", "_").strip("_")


# ------------------------------------------------------------

def load_topic_dataframes(
    parquet_path: str,
    topics_file: str
) -> Dict[str, pd.DataFrame]:
    """
    Load rosbag parquet and split into DataFrames
    based strictly on topics_file.
    """

    table = pq.read_table(parquet_path)
    df = table.to_pandas()

    topics = read_topics_file(topics_file)
    result: Dict[str, pd.DataFrame] = {}

    for topic, msg_type in topics.items():
        alias = topic_alias(topic)
        df_topic = df[df["topic"] == topic]

        if df_topic.empty:
            continue

        # ---------------- control ----------------
        if msg_type.endswith("CarlaEgoVehicleControl"):
            result[alias] = df_topic[[
                "t",
                "throttle",
                "steer",
                "brake",
                "hand_brake",
                "reverse",
                "gear",
                "manual_gear",
            ]].reset_index(drop=True)

        # ---------------- Float64 ----------------
        elif msg_type.endswith("Float64"):
            result[alias] = df_topic[[
                "t",
                "data",
            ]].reset_index(drop=True)

        # ---------------- Vector3 ----------------
        elif msg_type.endswith("Vector3"):
            result[alias] = df_topic[[
                "t",
                "x",
                "y",
                "z",
            ]].reset_index(drop=True)

        # ---------------- IMU ----------------
        elif msg_type.endswith("Imu"):
            result[alias] = df_topic[[
                "t",
                "ax",
                "ay",
                "az",
                "gx",
                "gy",
                "gz",
            ]].reset_index(drop=True)

        else:
            raise RuntimeError(
                f"Unsupported message type: {msg_type} (topic {topic})"
            )

    return result
