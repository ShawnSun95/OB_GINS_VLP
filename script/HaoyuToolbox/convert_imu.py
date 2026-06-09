#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convert MTi text log to a 7-column IMU format:

    t gyr_x gyr_y gyr_z acc_x acc_y acc_z

Changes from the original version:
1. Only keep the first 7 output columns: time, angular velocity, acceleration.
2. Gyroscope and accelerometer values are formatted to 6 decimal places.
3. Time is no longer read from the input file. Instead, it is generated as:
       t = start_time + row_index * sample_interval
   with default start_time=5.300 and sample_interval=0.005 (200 Hz).
   Time is formatted to 3 decimal places.

Input example:
- Comment lines starting with //
- Header line with fields like:
  PacketCounter SampleTimeFine Year Month Day Second Acc_X Acc_Y Acc_Z Gyr_X Gyr_Y Gyr_Z Roll Pitch Yaw
- Tab or whitespace separated data rows
"""

import argparse
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--input", required=True, help="Raw MTi txt file path")
    p.add_argument("--output", required=True, help="Converted output file path")
    p.add_argument(
        "--start-time",
        type=float,
        default=5.300,
        help="Output start time in seconds, default 5.300",
    )
    p.add_argument(
        "--sample-interval",
        type=float,
        default=0.005,
        help="Time interval between rows in seconds, default 0.005 (200 Hz)",
    )
    return p.parse_args()


def main():
    args = parse_args()
    in_path = Path(args.input)
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Read raw lines, skip blanks
    with in_path.open("r", encoding="utf-8", errors="ignore") as f:
        raw_lines = [ln.strip() for ln in f if ln.strip()]

    # Remove comment lines beginning with //
    data_lines = [ln for ln in raw_lines if not ln.startswith("//")]
    if not data_lines:
        raise RuntimeError("No data lines found after removing comment lines.")

    # Find header
    header_idx = None
    for i, ln in enumerate(data_lines):
        if "Acc_X" in ln and "Acc_Y" in ln and "Acc_Z" in ln and "Gyr_X" in ln and "Gyr_Y" in ln and "Gyr_Z" in ln:
            header_idx = i
            break
    if header_idx is None:
        raise RuntimeError("Cannot find header line with required fields.")

    header = data_lines[header_idx].replace("\t", " ").split()
    rows = data_lines[header_idx + 1 :]

    required = ["Acc_X", "Acc_Y", "Acc_Z", "Gyr_X", "Gyr_Y", "Gyr_Z"]
    for k in required:
        if k not in header:
            raise RuntimeError(f"Missing required column: {k}")

    idx = {name: header.index(name) for name in header}

    valid_row_index = 0
    with out_path.open("w", encoding="utf-8", newline="\n") as fw:
        fw.write("# t gyr_x gyr_y gyr_z acc_x acc_y acc_z\n")

        for ln in rows:
            toks = ln.replace("\t", " ").split()
            if len(toks) < len(header):
                # Skip broken/incomplete lines
                continue

            t = args.start_time + valid_row_index * args.sample_interval

            gyr_x = float(toks[idx["Gyr_X"]])
            gyr_y = float(toks[idx["Gyr_Y"]])
            gyr_z = float(toks[idx["Gyr_Z"]])
            acc_x = float(toks[idx["Acc_X"]])
            acc_y = float(toks[idx["Acc_Y"]])
            acc_z = float(toks[idx["Acc_Z"]])

            fw.write(
                f"{t:.3f} {gyr_x:.6f} {gyr_y:.6f} {gyr_z:.6f} {acc_x:.6f} {acc_y:.6f} {acc_z:.6f}\n"
            )
            valid_row_index += 1

    print(f"Done. Wrote: {out_path}")


if __name__ == "__main__":
    main()

"""
How to use:
python3 script/HaoyuToolbox/convert_imu.py \
    --input dataset/261127/MT_07782A2B_015-000.txt \
    --output dataset/261127/imu_20251127_4_test.txt \
    --start-time 5.300 \
    --sample-interval 0.005
"""