import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq


def load_parquet_data(filepath: str, schema: pa.schema) -> pd.DataFrame:
    return pq.read_table(filepath, schema=schema).to_pandas()


def load_control_data(filepath: str) -> pd.DataFrame:
    """
    CarlaEgoVehicleControl parquet loader
    """

    schema = pa.schema([
        ("t", pa.int64()),            # timestamp [ns]
        ("throttle", pa.float32()),   # [0..1]
        ("steer", pa.float32()),      # [-1..1]
        ("brake", pa.float32()),      # [0..1]
        ("hand_brake", pa.bool_()),
        ("reverse", pa.bool_()),
        ("gear", pa.int32()),
        ("manual_gear", pa.bool_()),
    ])

    df = load_parquet_data(filepath, schema)
    return df
