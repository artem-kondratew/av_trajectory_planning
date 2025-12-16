from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from carla_msgs.msg import CarlaEgoVehicleControl
import pandas as pd

bag_path = "./first"
topic_name = "/carla/hero/vehicle_control_cmd"

reader = SequentialReader()
reader.open(
    StorageOptions(uri=bag_path, storage_id='sqlite3'),
    ConverterOptions('', '')
)

rows = []

while reader.has_next():
    topic, data, t = reader.read_next()

    if topic != topic_name:
        continue

    msg = deserialize_message(data, CarlaEgoVehicleControl)

    rows.append({
        "t": t,                     # ns
        "throttle": msg.throttle,
        "steer": msg.steer,
        "brake": msg.brake,
        "hand_brake": msg.hand_brake,
        "reverse": msg.reverse,
        "gear": msg.gear,
        "manual_gear": msg.manual_gear_shift,
    })

df = pd.DataFrame(rows)
df.to_parquet("control.parquet", engine="pyarrow")
