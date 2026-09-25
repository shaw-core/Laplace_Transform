#!/usr/bin/env python3
"""Load a trial recorded by imu_logger.py into a pandas DataFrame.

Reads the CSV plus its sidecar _meta.json and applies the recorded gyro
bias offsets (subtracted from the corresponding imu{n}_gyro_{axis} columns).
The raw CSV on disk is never modified.

Usage:
    python load_trial.py data/20260925_S1_01_walk.csv
"""

import argparse
import json
from pathlib import Path

import pandas as pd


def load_trial(csv_path):
    csv_path = Path(csv_path)
    meta_path = csv_path.with_name(csv_path.stem + "_meta.json")
    if not meta_path.exists():
        raise FileNotFoundError(f"No sidecar meta file found at {meta_path}")

    with open(meta_path) as f:
        meta = json.load(f)

    df = pd.read_csv(csv_path, parse_dates=["pc_time"])

    bias = meta.get("gyro_bias", {})
    for i in (1, 2, 3):
        imu_bias = bias.get(f"imu{i}", {})
        for axis in ("x", "y", "z"):
            col = f"imu{i}_gyro_{axis}"
            offset = imu_bias.get(f"gyro_{axis}")
            if offset is not None and col in df.columns:
                df[col] = df[col] - offset

    return df, meta


def main():
    parser = argparse.ArgumentParser(description="Load an imu_logger.py trial CSV + meta JSON")
    parser.add_argument("csv_path", help="Path to the trial CSV file")
    args = parser.parse_args()

    df, meta = load_trial(args.csv_path)

    print(f"Loaded {len(df)} rows from {args.csv_path}")
    print(f"Subject={meta.get('subject')} Condition={meta.get('condition')} Trial={meta.get('trial')}")
    print(f"Sample rate (actual): {meta.get('actual_sample_rate_hz')} Hz")
    print(f"Segment map: {meta.get('segment_map')}")
    print()
    print(df.head())


if __name__ == "__main__":
    main()
