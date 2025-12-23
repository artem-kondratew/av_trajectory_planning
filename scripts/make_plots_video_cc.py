#!/usr/bin/env python3

import argparse
from typing import Tuple

import numpy as np
import pandas as pd
import cv2
import matplotlib.pyplot as plt

from pathlib import Path

from load_parquet import load_topic_dataframes


# ============================================================
# utils
# ============================================================

def extract_time(df: pd.DataFrame) -> np.ndarray:
    """Convert ROS nanoseconds to seconds"""
    return df["t"].to_numpy() * 1e-9


def sync_timeseries(
    ref_time: np.ndarray,
    ref_data: np.ndarray,
    data_time: np.ndarray,
    data: np.ndarray,
    fix_time: bool = True,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Align data to reference timeline using zero-order hold
    """
    idx = np.searchsorted(ref_time, data_time, side="right") - 1
    valid = idx >= 0

    t_sync = data_time[valid]
    ref_sync = ref_data[idx[valid]]
    data_sync = data[valid]

    if fix_time and len(t_sync) > 0:
        t_sync -= t_sync[0]

    return t_sync, ref_sync, data_sync



def make_output_name(parquet_path: str, suffix: str = "_plot.mp4") -> str:
    p = Path(parquet_path)
    stem = p.stem  # cc_two_levels_rosbag

    if stem.endswith("_rosbag"):
        stem = stem.removesuffix("_rosbag")

    return str(p.with_name(stem + suffix))


# ============================================================
# main
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Generate video with velocity plots"
    )

    parser.add_argument(
        "--parquet",
        required=True,
        help="Path to parquet file"
    )
    parser.add_argument(
        "--topics-file",
        required=True,
        help="Path to topics list file"
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Video FPS"
    )
    parser.add_argument(
        "--window",
        type=float,
        default=10.0,
        help="Time window (seconds)"
    )

    args = parser.parse_args()

    # --------------------------------------------------------
    # load data
    # --------------------------------------------------------

    dfs = load_topic_dataframes(args.parquet, args.topics_file)

    v_ref = dfs["v_ref"]
    velocity = dfs["velocity"]

    ms2kmh = 3.6

    v_ref_time = extract_time(v_ref)
    v_ref_data = v_ref["data"].to_numpy() * ms2kmh

    v_time = extract_time(velocity)
    v_data = velocity["data"].to_numpy() * ms2kmh

    t, v_ref_sync, v_sync = sync_timeseries(
        v_ref_time, v_ref_data,
        v_time, v_data,
        fix_time=True
    )

    if len(t) == 0:
        raise RuntimeError("No synchronized data")

    # --------------------------------------------------------
    # matplotlib setup
    # --------------------------------------------------------

    plt.ioff()

    fig, ax = plt.subplots(figsize=(10, 4))

    ax.plot(
        t,
        v_ref_sync,
        label="reference velocity",
        color="red",
    )

    line_v, = ax.plot(
        [],
        [],
        label="vehicle velocity",
        color="blue",
        lw=2,
    )

    cursor = ax.axvline(
        0.0,
        color="black",
        linestyle="--",
        alpha=0.6,
    )

    ax.set_xlabel("time, s")
    ax.set_ylabel("velocity, km/h")
    ax.grid(True)
    ax.legend()

    fig.tight_layout()

    # --------------------------------------------------------
    # video writer
    # --------------------------------------------------------

    fig.canvas.draw()
    width, height = fig.canvas.get_width_height()

    output_path = make_output_name(args.parquet)

    writer = cv2.VideoWriter(
        output_path,
        cv2.VideoWriter_fourcc(*"mp4v"),
        args.fps,
        (width, height),
    )

    # --------------------------------------------------------
    # render loop
    # --------------------------------------------------------

    last_xlim = None

    for t_frame in t:
        mask = t <= t_frame
        if not np.any(mask):
            continue

        line_v.set_data(t[mask], v_sync[mask])
        cursor.set_xdata([t_frame])

        xmin = max(0.0, t_frame - args.window)
        xmax = t_frame + 0.5

        if last_xlim != (xmin, xmax):
            ax.set_xlim(xmin, xmax)
            last_xlim = (xmin, xmax)

        ax.set_ylim(-10, 120)

        fig.canvas.draw()

        buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
        img = buf.reshape((height, width, 4))      # RGBA
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

        writer.write(img)


    writer.release()
    plt.close(fig)

    print(f"[OK] Video saved to: {output_path}")


if __name__ == "__main__":
    main()
