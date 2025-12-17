from typing import Dict
import pandas as pd
import pyarrow.parquet as pq


def load_topic_dataframes(filepath: str) -> Dict[str, pd.DataFrame]:
    """
    Load rosbag parquet and split into separate DataFrames per topic.

    Returns:
        dict with keys:
          - 'control'
          - 'v_ref'
          - 'y_vector'
          - 'velocity'
          - 'imu'
    """

    table = pq.read_table(filepath)
    df = table.to_pandas()

    result = {}

    # ---------------- control ----------------
    ctrl = df[df["topic"] == "/carla/hero/vehicle_control_cmd"]
    result["control"] = ctrl[[
        "t",
        "throttle",
        "steer",
        "brake",
        "hand_brake",
        "reverse",
        "gear",
        "manual_gear",
    ]].reset_index(drop=True)

    # ---------------- v_ref ----------------
    vref = df[df["topic"] == "/cruise_controller/v_ref"]
    result["v_ref"] = vref[[
        "t",
        "v_ref",
    ]].reset_index(drop=True)

    # ---------------- y_vector ----------------
    y = df[df["topic"] == "/cruise_controller/y_vector"]
    result["y_vector"] = y[[
        "t",
        "dv",
        "a",
        "j",
    ]].reset_index(drop=True)

    # ---------------- velocity ----------------
    vel = df[df["topic"] == "/carla/hero/local_velocity"]
    result["velocity"] = vel[[
        "t",
        "vx",
        "vy",
        "vz",
        "wx",
        "wy",
        "wz",
    ]].reset_index(drop=True)

    # ---------------- imu ----------------
    imu = df[df["topic"] == "/carla/hero/imu"]
    result["imu"] = imu[[
        "t",
        "ax",
        "ay",
        "az",
        "gx",
        "gy",
        "gz",
    ]].reset_index(drop=True)

    return result
