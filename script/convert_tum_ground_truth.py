#!/usr/bin/env python3
"""Align a TUM trajectory to an indoor ENU initial pose and export t N E D R P Y.

Input: t x y z qx qy qz qw (timestamps in seconds, positions in metres).
--xyz is the desired FIRST pose position in indoor ENU, in metres.
--rpy is the desired FIRST pose orientation in indoor ENU, in degrees,
using Rz(yaw) @ Ry(pitch) @ Rx(roll). Zero angles align the initial body
axes with the indoor axes; positive yaw rotates East toward North.

The first TUM pose is removed before applying the desired initial pose:
    p_enu = xyz + R_rpy @ R_tum_first.T @ (p_tum - p_tum_first)
The same alignment is applied to every orientation. Output positions are NED;
output roll, pitch, yaw are ENU angles in degrees, matching --rpy.
Positions use linear interpolation and orientations use quaternion SLERP.
Uniform timestamps are multiples of 1 / rate (10 Hz: .00, .10, .20, ...),
without extrapolating beyond the input trajectory.
"""

from __future__ import annotations

import argparse
from fractions import Fraction
from pathlib import Path

import numpy as np


def finite_float(value: str) -> float:
    number = float(value)
    if not np.isfinite(number):
        raise argparse.ArgumentTypeError("Expected a finite number")
    return number


def load_tum(path: Path) -> np.ndarray:
    data = np.loadtxt(path, ndmin=2)
    if data.shape[0] == 0 or data.shape[1] != 8:
        raise ValueError("TUM input must contain rows of t x y z qx qy qz qw")
    if not np.isfinite(data).all():
        raise ValueError("TUM input contains NaN or infinite values")
    if np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError("TUM timestamps must be strictly increasing")
    return data


def quaternion_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Convert TUM quaternion(s) (qx, qy, qz, qw) to body-to-world matrices."""
    x, y, z, w = np.moveaxis(normalize_quaternions(quaternion), -1, 0)
    return np.stack([
        1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w),
        2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w),
        2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y),
    ], axis=-1).reshape(quaternion.shape[:-1] + (3, 3))


def normalize_quaternions(quaternions: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(quaternions, axis=-1, keepdims=True)
    if not np.isfinite(norms).all() or np.any(norms < 1e-12):
        raise ValueError("All TUM quaternions must have a nonzero finite norm")
    return quaternions / norms


def interpolate_quaternions(
    times: np.ndarray, quaternions: np.ndarray, target: np.ndarray,
) -> np.ndarray:
    """Interpolate each pair along its shortest rotation, including q/-q pairs."""
    quaternions = normalize_quaternions(quaternions)
    if len(times) == 1:
        return np.repeat(quaternions, len(target), axis=0)
    left = np.clip(np.searchsorted(times, target, side="right") - 1, 0, len(times) - 2)
    fraction = ((target - times[left]) / (times[left + 1] - times[left]))[:, None]
    q0, q1 = quaternions[left], quaternions[left + 1]
    dot = np.sum(q0 * q1, axis=1, keepdims=True)
    q1 = np.where(dot < 0, -q1, q1)
    angle = np.arccos(np.clip(np.abs(dot), 0, 1))
    sine = np.sin(angle)
    # The limit for identical rotations is normalized linear interpolation.
    weight0 = np.divide(np.sin((1 - fraction) * angle), sine,
                        out=1 - fraction, where=sine > 1e-8)
    weight1 = np.divide(np.sin(fraction * angle), sine,
                        out=fraction.copy(), where=sine > 1e-8)
    return normalize_quaternions(weight0 * q0 + weight1 * q1)


def rpy_matrix(rpy_degrees: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = np.deg2rad(rpy_degrees)
    sr, sp, sy = np.sin([roll, pitch, yaw])
    cr, cp, cy = np.cos([roll, pitch, yaw])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_rpy(matrices: np.ndarray) -> np.ndarray:
    """Extract ZYX roll/pitch/yaw in degrees; at gimbal lock choose roll = 0."""
    cos_pitch = np.hypot(matrices[:, 0, 0], matrices[:, 1, 0])
    roll = np.arctan2(matrices[:, 2, 1], matrices[:, 2, 2])
    pitch = np.arctan2(-matrices[:, 2, 0], cos_pitch)
    yaw = np.arctan2(matrices[:, 1, 0], matrices[:, 0, 0])
    locked = cos_pitch < 1e-10
    roll = np.where(locked, 0, roll)
    yaw = np.where(locked, np.arctan2(-matrices[:, 0, 1], matrices[:, 1, 1]), yaw)
    return np.rad2deg(np.column_stack((roll, pitch, yaw)))


def align_positions(tum: np.ndarray, xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    """Place the first pose at xyz/rpy in ENU, then convert positions to NED."""
    rotation = rpy_matrix(rpy) @ quaternion_matrix(tum[0, 4:8]).T
    enu = (tum[:, 1:4] - tum[0, 1:4]) @ rotation.T + xyz
    return np.column_stack((enu[:, 1], enu[:, 0], -enu[:, 2]))


def sample_times(times: np.ndarray, rate: float, reference_nav: Path | None) -> np.ndarray:
    if reference_nav is not None:
        target = np.loadtxt(reference_nav, usecols=(0,), ndmin=1)
        if target.size == 0 or not np.isfinite(target).all():
            raise ValueError("Reference nav file must contain finite timestamps")
        if np.any(np.diff(target) <= 0):
            raise ValueError("Reference nav timestamps must be strictly increasing")
        target = target[(target >= times[0]) & (target <= times[-1])]
        if target.size == 0:
            raise ValueError("No reference nav timestamps fall within the TUM time range")
        return target

    if rate == 0:
        return times.copy()
    if 1 / rate < np.max(np.abs(np.spacing(times))):
        raise ValueError("Sampling rate is too high for the timestamp precision")
    # Include a candidate on each side, then clip using actual timestamps.
    # This also preserves boundary samples when time * rate rounds slightly.
    first_tick = int(np.floor(times[0] * rate))
    last_tick = int(np.ceil(times[-1] * rate))
    target = (first_tick + np.arange(last_tick - first_tick + 1, dtype=float)) / rate
    target = target[(target >= times[0]) & (target <= times[-1])]
    if target.size == 0:
        raise ValueError("No aligned sampling timestamps fall within the TUM time range; use --rate 0 to keep original samples")
    if np.any(np.diff(target) <= 0):
        raise ValueError("Sampling rate is too high for the timestamp precision")
    return target


def convert(
    tum: np.ndarray,
    xyz: np.ndarray,
    rpy: np.ndarray,
    *,
    rate: float = 10.0,
    time_origin: float = 0.0,
    time_offset: float = 0.0,
    reference_nav: Path | None = None,
) -> np.ndarray:
    if not np.isfinite(rate) or rate < 0:
        raise ValueError("--rate must be nonnegative; use 0 to keep original samples")
    positions = align_positions(tum, xyz, rpy)
    times = tum[:, 0] - time_origin + time_offset
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Time conversion must produce finite, strictly increasing timestamps")
    target = sample_times(times, rate, reference_nav)
    # Work relative to the first timestamp to reduce rounding with Unix seconds.
    relative_times = times - times[0]
    relative_target = target - times[0]
    sampled = np.column_stack([
        np.interp(relative_target, relative_times, positions[:, axis])
        for axis in range(3)
    ])
    quaternions = interpolate_quaternions(relative_times, tum[:, 4:8], relative_target)
    rotation = rpy_matrix(rpy) @ quaternion_matrix(tum[0, 4:8]).T
    angles = matrix_rpy(rotation @ quaternion_matrix(quaternions))
    return np.column_stack((target, sampled, angles))


def timestamp_format(rate: float, reference_nav: Path | None) -> str:
    """Print decimal grids exactly, rather than Unix-second float artifacts."""
    if rate == 0 or reference_nav is not None:
        return "%.9f"
    denominator = (1 / Fraction(str(rate))).denominator
    twos = fives = 0
    while denominator % 2 == 0:
        denominator //= 2
        twos += 1
    while denominator % 5 == 0:
        denominator //= 5
        fives += 1
    decimals = max(2, twos, fives) if denominator == 1 else 9
    return f"%.{decimals}f"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", type=Path, required=True, help="Input TUM file")
    parser.add_argument("--output", type=Path, required=True,
                        help="Output t N E D roll pitch yaw (NED positions in metres, ENU angles in degrees)")
    parser.add_argument("--xyz", nargs=3, type=finite_float, default=[6.3, 2.25, 1.01],
                        metavar=("E", "N", "U"), help="First pose position in indoor ENU, metres (default: 6.3 2.25 1.01)")
    parser.add_argument("--rpy", nargs=3, type=finite_float, default=[0.0, 0.0, 0.0],
                        metavar=("ROLL", "PITCH", "YAW"), help="First pose orientation in indoor ENU, degrees (default: 0 0 0)")
    sampling = parser.add_mutually_exclusive_group()
    sampling.add_argument("--rate", type=finite_float, default=10.0,
                          help="Uniform output rate in Hz, aligned to multiples of 1/rate (default: 10; 0 keeps original samples)")
    sampling.add_argument("--reference-nav", type=Path,
                          help="Instead interpolate at nav timestamps within the converted TUM time range")
    parser.add_argument("--time-origin", type=finite_float, default=0.0,
                        help="Seconds subtracted from TUM timestamps (default: 0, retain original clock)")
    parser.add_argument("--time-offset", type=finite_float, default=0.0,
                        help="Seconds added after subtracting --time-origin (default: 0)")
    args = parser.parse_args()

    try:
        protected_paths = [args.input]
        if args.reference_nav is not None:
            protected_paths.append(args.reference_nav)
        if args.output.resolve() in [path.resolve() for path in protected_paths]:
            raise ValueError("Output must not overwrite the input TUM or reference nav file")
        tum = load_tum(args.input)
        result = convert(tum, np.asarray(args.xyz), np.asarray(args.rpy), rate=args.rate,
                         time_origin=args.time_origin, time_offset=args.time_offset,
                         reference_nav=args.reference_nav)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        header = (
            "t N E D roll pitch yaw (seconds, NED metres, ENU degrees)\n"
            f"initial ENU xyz: {args.xyz}; initial ENU rpy (deg): {args.rpy}\n"
            f"time = TUM time - {args.time_origin} + {args.time_offset}"
        )
        time_fmt = timestamp_format(args.rate, args.reference_nav)
        np.savetxt(args.output, result, fmt=[time_fmt] + ["%.9f"] * 6, header=header)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    print(f"Input: {len(tum)} poses, {tum[-1, 0] - tum[0, 0]:.6f} s")
    if len(tum) > 1:
        print(f"Input median rate: {1 / np.median(np.diff(tum[:, 0])):.6f} Hz")
    sampling_description = (
        f"reference nav timestamps ({args.reference_nav})" if args.reference_nav is not None
        else "original samples" if args.rate == 0 else f"uniform {args.rate:g} Hz"
    )
    print(f"Output: {len(result)} poses, {sampling_description}")
    print(f"Output time range: {time_fmt % result[0, 0]} .. {time_fmt % result[-1, 0]}")
    print(f"First input pose mapped to NED: {args.xyz[1]:.6f} {args.xyz[0]:.6f} {-args.xyz[2]:.6f}")
    print("Output orientation: ENU roll pitch yaw, degrees (initial input orientation set by --rpy)")
    print(f"Saved: {args.output}")
    print("Use --ground_truth_frame ned with either plotting script.")


if __name__ == "__main__":
    main()
