from typing import Dict
import pandas as pd
import pyarrow.parquet as pq


def read_topics_file(path: str) -> Dict[str, str]:
    topics = {}

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) != 2:
                raise RuntimeError(f"Invalid topics file line: {line}")

            topic, msg_type = parts
            topics[topic] = msg_type

    return topics


def topic_to_key(topic: str) -> str:
    """
    Unique dataframe key for each topic.
    """
    return topic.lstrip("/").replace("/", "__")


def load_topic_dataframes(
    parquet_path: str,
    topics_file: str
) -> Dict[str, pd.DataFrame]:

    table = pq.read_table(parquet_path)
    df = table.to_pandas()

    topics = read_topics_file(topics_file)
    result: Dict[str, pd.DataFrame] = {}

    for topic, msg_type in topics.items():
        key = topic_to_key(topic)
        df_topic = df[df["topic"] == topic]

        if df_topic.empty:
            continue

        # ---------------- control ----------------
        if msg_type.endswith("CarlaEgoVehicleControl"):
            result[key] = df_topic[[
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
            result[key] = df_topic[[
                "t",
                "data",
            ]].reset_index(drop=True)

        # ---------------- Vector3 ----------------
        elif msg_type.endswith("Vector3"):
            result[key] = df_topic[[
                "t",
                "x",
                "y",
                "z",
            ]].reset_index(drop=True)

        # ---------------- IMU ----------------
        elif msg_type.endswith("Imu"):
            result[key] = df_topic[[
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
