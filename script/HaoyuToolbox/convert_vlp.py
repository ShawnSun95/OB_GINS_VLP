#!/usr/bin/env python3
"""Convert raw VLP intensity sequence into RSS txt following VLPtoolbox logic.

Output format per line:
    time x y z RSS1 RSS2 RSS3 RSS4 RSS5 RSS6 STD1 STD2 STD3 STD4 STD5 STD6

This script mirrors the offline MATLAB preprocessing in VLPtoolbox:
- trim the first/last raw sample by default
- use a 1 s FFT window with sliding output at ``rss_rate`` Hz
- reuse the first/last full window near the two boundaries
- optionally apply a Hamming window and the matching 1.852 gain compensation
- extract RSS from the exact FFT bin used by the MATLAB code

The exported x/y/z columns remain placeholder zeros so the output can be fed
into the downstream OB_GINS_VLP pipeline.
"""

from __future__ import annotations

import argparse
import math
import re

import numpy as np


DEFAULT_FHZ = [735, 215, 640, 305, 865, 520]
DEFAULT_FS = 2000.0
DEFAULT_WINDOW_DURATION = 1.0
DEFAULT_RSS_RATE = 10
DEFAULT_STD = np.asarray([0.1, 0.9, 0.1, 0.1, 0.1, 0.1], dtype=float)
INTEGER_SECOND_TOL = 1e-9


def parse_fhz(fhz_text: str) -> np.ndarray:
    vals = [v.strip() for v in fhz_text.split(",") if v.strip()]
    if not vals:
        raise ValueError("--fhz cannot be empty")
    return np.asarray([float(v) for v in vals], dtype=float)


def load_raw_series(path: str) -> np.ndarray:
    """Load raw intensity sequence from txt/csv-like files.

    Supports:
    - one value per line
    - comma-separated values in one line or multiple lines
    - mixed commas + whitespace
    - optional leading/trailing commas
    """
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    cleaned = text.strip().strip(",")
    if not cleaned:
        raise ValueError(f"Input file is empty: {path}")

    tokens = [tok for tok in re.split(r"[\s,]+", cleaned) if tok]
    if not tokens:
        raise ValueError(f"No numeric values found in input file: {path}")

    try:
        raw = np.asarray([float(tok) for tok in tokens], dtype=float)
    except ValueError as exc:
        raise ValueError(
            "Failed to parse raw input as numeric sequence. "
            "Please ensure file contains only numbers separated by commas/whitespace."
        ) from exc

    return raw


def extract_rss_from_window(
    samples: np.ndarray,
    fhz: np.ndarray,
    fs: float,
    *,
    use_hamming: bool,
) -> np.ndarray:
    """Compute multi-channel RSS from one FFT window."""
    n = len(samples)
    if n <= 0:
        raise ValueError("FFT window length must be > 0")

    if use_hamming:
        samples = samples * np.hamming(n)

    f = np.fft.fft(samples, n=n)
    mag = np.abs(f) * 2 / n
    if use_hamming:
        mag *= 1.852

    rss = np.zeros(len(fhz), dtype=float)
    for i, freq in enumerate(fhz):
        idx = int(freq * n / fs)
        if idx < 0 or idx >= len(mag):
            raise ValueError(
                f"Frequency index out of bounds for LED#{i+1}: freq={freq}, idx={idx}, fft_len={len(mag)}"
            )
        rss[i] = mag[idx] / 1.27
    return rss


def preprocess_vlptoolbox(
    vlp: np.ndarray,
    fhz: np.ndarray,
    fs: float,
    dt: float,
    rss_rate: int,
    *,
    use_hamming: bool,
    start_time: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate RSS using the same sliding-window logic as VLPtoolbox."""
    if rss_rate <= 0:
        raise ValueError("rss_rate must be > 0")

    n_total = len(vlp)
    n = int(round(dt * fs))
    if n <= 0:
        raise ValueError("FFT window length N must be > 0. Check fs and window_duration.")
    if n_total < n:
        raise ValueError(f"Input length {n_total} is shorter than one FFT window ({n}).")

    num_point = math.floor(n_total / n) * rss_rate
    if num_point <= 0:
        raise ValueError("No RSS output can be generated from the provided raw data.")

    step = n / rss_rate
    times = start_time + np.arange(1, num_point + 1, dtype=float) * dt / rss_rate
    fft_result = np.zeros((num_point, len(fhz)), dtype=float)

    for k in range(num_point):
        if k < rss_rate / 2:
            start_idx = 0
        elif n + (k - rss_rate / 2) * step >= n_total:
            start_idx = n_total - n
        else:
            start_idx = int(round((k - rss_rate / 2) * step))

        end_idx = start_idx + n
        s = vlp[start_idx:end_idx]
        fft_result[k, :] = extract_rss_from_window(s, fhz, fs, use_hamming=use_hamming)

    return times, fft_result


def keep_integer_second_rows(
    times: np.ndarray,
    fft_result: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Keep only samples whose timestamps fall on integer seconds."""
    mask = np.isclose(times, np.round(times), atol=INTEGER_SECOND_TOL, rtol=0.0)
    return np.round(times[mask]), fft_result[mask]


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert raw VLP intensity to RSS txt following VLPtoolbox.")
    parser.add_argument("--input", required=True, help="Raw VLP txt/dat file path (single-column intensity sequence).")
    parser.add_argument("--output", required=True, help="Output txt file path.")
    parser.add_argument(
        "--fhz",
        default=None,
        help="LED modulation frequencies in Hz, comma-separated. If omitted, default is 735,215,640,305,865,520.",
    )
    parser.add_argument("--fs", type=float, default=None, help="Sampling frequency in Hz. If omitted, default is 2000.")
    parser.add_argument(
        "--window-duration",
        type=float,
        default=None,
        help="FFT window duration in seconds. Defaults to 1.0.",
    )
    parser.add_argument(
        "--start-time",
        type=float,
        default=0.0,
        help="Time offset added to the VLPtoolbox timestamps t=(1:num_point)*dt/rss_rate.",
    )
    parser.add_argument(
        "--rss-rate",
        type=int,
        default=DEFAULT_RSS_RATE,
        help="RSS sampling rate in Hz. Defaults to 10 to match VLPtoolbox.",
    )
    parser.add_argument(
        "--trim-ends",
        action="store_true",
        default=True,
        help="Drop first and last sample (same as MATLAB: fid3=fid3(2:end-1)). Default: enabled.",
    )
    parser.add_argument(
        "--no-trim-ends",
        action="store_true",
        help="Disable trimming of the first and last raw sample.",
    )
    parser.add_argument(
        "--no-hamming",
        action="store_true",
        help="Disable the Hamming window and 1.852 gain compensation.",
    )
    args = parser.parse_args()

    raw = load_raw_series(args.input)

    do_trim = args.trim_ends and not args.no_trim_ends
    if do_trim:
        if len(raw) < 3:
            raise ValueError("Cannot trim first/last sample: input length < 3")
        raw = raw[1:-1]

    fhz = parse_fhz(args.fhz) if args.fhz is not None else np.asarray(DEFAULT_FHZ, dtype=float)
    fs = args.fs if args.fs is not None else DEFAULT_FS
    window_duration = args.window_duration if args.window_duration is not None else DEFAULT_WINDOW_DURATION
    use_hamming = not args.no_hamming

    t, fft_result = preprocess_vlptoolbox(
        raw,
        fhz,
        fs,
        window_duration,
        args.rss_rate,
        use_hamming=use_hamming,
        start_time=args.start_time,
    )
    t, fft_result = keep_integer_second_rows(t, fft_result)

    xyz = np.zeros((len(t), 3), dtype=float)
    std_mat = np.tile(DEFAULT_STD, (len(t), 1))
    out_mat = np.hstack([t.reshape(-1, 1), xyz, fft_result, std_mat])

    fmt = ["%.2f"] + ["%.3f"] * 9 + ["%.1f"] * 6
    np.savetxt(args.output, out_mat, fmt=fmt, delimiter="\t")

    print(f"Saved {len(t)} rows to: {args.output}")


if __name__ == "__main__":
    main()

"""
how to use:
python3 script/HaoyuToolbox/convert_vlp.py --input dataset/261127/20251127_174137_93.txt --output dataset/261127/vlp_20251127_4.txt
"""
