#!/usr/bin/env python3

import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def get_topic(cursor, topic_name):
    cursor.execute("SELECT id, type FROM topics WHERE name=?", (topic_name,))
    row = cursor.fetchone()
    if row is None:
        raise RuntimeError(f"{topic_name} not found")
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", help="Path to rosbag2 folder")
    parser.add_argument("output_prefix", default="data", help="Prefix for output files")
    args = parser.parse_args()

    # --- найти db ---
    db_files = [f for f in os.listdir(args.bag_path) if f.endswith(".db3")]
    if not db_files:
        raise RuntimeError("No .db3 file found")

    db_path = os.path.join(args.bag_path, db_files[0])

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # --- topics ---
    stanley_topic = '/carla/hero/stanley_state'
    control_topic = '/carla/hero/vehicle_control_cmd'
    offset_topic  = '/carla/hero/rear_axle_lane_offset'

    stanley_id, stanley_type = get_topic(cursor, stanley_topic)
    control_id, control_type = get_topic(cursor, control_topic)
    offset_id, offset_type   = get_topic(cursor, offset_topic)

    stanley_msg = get_message(stanley_type)
    control_msg = get_message(control_type)
    offset_msg  = get_message(offset_type)

    print(f"Loaded topics:")
    print(stanley_topic)
    print(control_topic)
    print(offset_topic)

    # ======================
    #   STANLEY
    # ======================
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (stanley_id,))

    t_arr = []
    yaw_vehicle_arr = []
    yaw_path_arr = []
    v_arr = []

    for _, data in cursor.fetchall():
        msg = deserialize_message(data, stanley_msg)

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        t_arr.append(t)
        yaw_vehicle_arr.append(msg.yaw_vehicle)
        yaw_path_arr.append(msg.yaw_path)
        v_arr.append(msg.v)

    # ======================
    #   OFFSET (e)
    # ======================
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (offset_id,))

    offset_t = []
    e_arr = []

    for _, data in cursor.fetchall():
        msg = deserialize_message(data, offset_msg)

        # std_msgs/Float64 → просто data
        offset_t.append(_)  # timestamp из db
        e_arr.append(msg.data)

    # ======================
    #   CONTROL
    # ======================
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (control_id,))

    steer_t = []
    steer = []

    for _, data in cursor.fetchall():
        msg = deserialize_message(data, control_msg)

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        steer_t.append(t)
        steer.append(-msg.steer)

    conn.close()

    # ======================
    #   NUMPY
    # ======================
    t_arr = np.array(t_arr)
    t_arr -= t_arr[0]

    yaw_vehicle_arr = np.array(yaw_vehicle_arr)
    yaw_path_arr = np.array(yaw_path_arr)
    v_arr = np.array(v_arr)

    offset_t = np.array(offset_t)
    offset_t = offset_t / 1e9  # db timestamp → seconds
    offset_t -= offset_t[0]
    e_arr = np.array(e_arr)

    steer_t = np.array(steer_t)
    steer_t -= steer_t[0]
    steer = np.array(steer)

    # unwrap yaw
    yaw_vehicle_arr = np.arctan2(np.sin(yaw_vehicle_arr), np.cos(yaw_vehicle_arr))
    yaw_path_arr = np.arctan2(np.sin(yaw_path_arr), np.cos(yaw_path_arr))

    theta = yaw_path_arr - yaw_vehicle_arr
    theta = np.arctan2(np.sin(theta), np.cos(theta))

    # ======================
    #   INTERPOLATION
    # ======================
    e_interp = np.interp(t_arr, offset_t, e_arr)
    steer_interp = np.interp(t_arr, steer_t, steer)

    # ======================
    #   STANLEY
    # ======================
    k = 1.0
    v_safe = np.maximum(v_arr, 0.1)
    delta_stanley = theta + np.arctan2(k * e_interp, v_safe)

    # ======================
    #   REAL STEER
    # ======================
    MAX_STEER_DEG = 70.0
    steer_deg = steer * MAX_STEER_DEG

    # ======================
    #   PLOTS
    # ======================
    plt.figure(figsize=(12, 12))

    plt.subplot(3,1,1)
    plt.plot(t_arr, -e_interp, color='b')
    plt.ylabel('e [m]')
    plt.title('Rear axle offset')
    plt.grid()
    plt.ylim(-1.0, 1.0)

    plt.subplot(3,1,2)
    plt.plot(t_arr, theta * 180/np.pi, color='b')
    plt.ylabel('theta [deg]')
    plt.title('Heading error')
    plt.grid()
    plt.ylim(-10, 10)

    plt.subplot(3,1,3)
    n = min(t_arr.shape[0], steer_deg.shape[0])
    plt.plot(t_arr[:n], steer_deg[:n], label='real steer', color='b')
    #plt.plot(t_arr, steer_interp, label='real steer', color='b')
    #plt.plot(t_arr, delta_stanley * 180/np.pi, color='b')
    #plt.plot(t_arr, theta * 180/np.pi)
    plt.ylabel('steer [deg]')
    plt.xlabel('time [s]')
    plt.title('Steering angle')
    plt.legend()
    plt.grid()
    plt.ylim(-8, 8)

    #plt.ylim(-33, 30)

    plt.tight_layout()
    plt.show()

    # ======================
    #   SAVE ARRAYS
    # ======================
    prefix = args.output_prefix

    np.save(f"{prefix}_t.npy", t_arr)
    np.save(f"{prefix}_e.npy", -e_interp)
    np.save(f"{prefix}_theta.npy", theta)
    np.save(f"{prefix}_steer.npy", steer_deg)
    np.save(f"{prefix}_v.npy", v_arr)

    print(f"Saved arrays with prefix: {prefix}")


if __name__ == "__main__":
    main()
