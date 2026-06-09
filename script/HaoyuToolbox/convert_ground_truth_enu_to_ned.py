#!/usr/bin/env python3
"""Convert ground-truth coordinates from ENU to NED.

Input file format:
    t E N U ...

Output file format:
    t N E D ...

Only the first four columns are transformed. Remaining columns, if any, are
kept in their original order.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def convert_enu_to_ned(data: np.ndarray) -> np.ndarray:
    if data.ndim != 2 or data.shape[1] < 4:
        raise ValueError("Input file must contain at least 4 columns: t E N U")

    out = data.copy()
    out[:, 1] = data[:, 2]
    out[:, 2] = data[:, 1]
    out[:, 3] = -data[:, 3]
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Input ground truth file, columns: t E N U ...")
    parser.add_argument("--output", required=True, help="Output file, columns: t N E D ...")
    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)

    data = np.loadtxt(input_path)
    converted = convert_enu_to_ned(data)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(output_path, converted, fmt="%.6f")

    print(f"Saved converted ground truth to: {output_path}")


if __name__ == "__main__":
    main()

"""
how to use:
python3 script/HaoyuToolbox/convert_ground_truth_enu_to_ned.py --input dataset/251127_6/ground_truth.txt --output dataset/251127_6/ground_truth_ned.txt
"""
