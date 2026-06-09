#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_nav_file(path: str) -> np.ndarray:
    data = np.loadtxt(path, usecols=tuple(range(10)))
    if data.ndim == 1:
        data = data[np.newaxis, :]
    return data


def load_ground_truth(path: str, frame: str) -> np.ndarray:
    data = np.loadtxt(path, usecols=(0, 1, 2, 3))
    if data.ndim == 1:
        data = data[np.newaxis, :]

    if frame == "auto":
        frame = "ned" if "_ned" in Path(path).stem.lower() else "enu"

    if frame == "enu":
        out = np.column_stack((data[:, 0], data[:, 2], data[:, 1], -data[:, 3]))
    elif frame == "ned":
        out = data.copy()
    else:
        raise ValueError(f"Unsupported ground-truth frame: {frame}")

    return out


def crop_to_common_time(reference: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ref_timestamps = reference[:, 0]
    target_timestamps = target[:, 0]

    common_start = max(ref_timestamps[0], target_timestamps[0])
    common_end = min(ref_timestamps[-1], target_timestamps[-1])
    if common_start > common_end + 1e-6:
        raise ValueError(
            f"No common time slot: reference [{ref_timestamps[0]}, {ref_timestamps[-1]}], "
            f"target [{target_timestamps[0]}, {target_timestamps[-1]}]"
        )

    ref_mask = (ref_timestamps >= common_start - 1e-6) & (ref_timestamps <= common_end + 1e-6)
    target_mask = (target_timestamps >= common_start - 1e-6) & (target_timestamps <= common_end + 1e-6)
    return reference[ref_mask], target[target_mask]


def nearest_time_indices(reference_times: np.ndarray, target_times: np.ndarray) -> np.ndarray:
    indices = np.searchsorted(target_times, reference_times, side="left")
    best_indices = np.zeros_like(indices)

    for i, idx in enumerate(indices):
        candidates = []
        if idx > 0:
            candidates.append(idx - 1)
        if idx < len(target_times):
            candidates.append(idx)
        best_indices[i] = min(candidates, key=lambda j: abs(target_times[j] - reference_times[i]))

    return best_indices


def compute_error_metrics(gt_ned: np.ndarray, opt_ned: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    xyz_error = opt_ned - gt_ned
    error_3d = np.linalg.norm(xyz_error, axis=1)
    return xyz_error, error_3d


def print_metrics(xyz_error: np.ndarray, error_3d: np.ndarray) -> None:
    rmse_xyz = np.sqrt(np.mean(np.square(xyz_error), axis=0))
    max_abs_xyz = np.max(np.abs(xyz_error), axis=0)

    print("3D error statistics in NED frame (optimized - ground truth):")
    print(f"mean error 3d: {np.mean(error_3d):.6f} m")
    print(f"rmse error 3d: {np.sqrt(np.mean(np.square(error_3d))):.6f} m")
    print(f"max error 3d : {np.max(error_3d):.6f} m")
    print(
        "mean xyz error: "
        f"N={np.mean(xyz_error[:, 0]):.6f} m, "
        f"E={np.mean(xyz_error[:, 1]):.6f} m, "
        f"D={np.mean(xyz_error[:, 2]):.6f} m"
    )
    print(
        "rmse xyz error: "
        f"N={rmse_xyz[0]:.6f} m, "
        f"E={rmse_xyz[1]:.6f} m, "
        f"D={rmse_xyz[2]:.6f} m"
    )
    print(
        "max abs xyz  : "
        f"N={max_abs_xyz[0]:.6f} m, "
        f"E={max_abs_xyz[1]:.6f} m, "
        f"D={max_abs_xyz[2]:.6f} m"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot 3D trajectories and compute 3D position error."
    )
    parser.add_argument(
        "--initial_poses",
        default="",
        help="Optional initial pose file, columns: t N E D vx vy vz roll pitch yaw",
    )
    parser.add_argument(
        "--optimized_poses",
        required=True,
        help="Optimized pose file, columns: t N E D vx vy vz roll pitch yaw",
    )
    parser.add_argument(
        "--ground_truth",
        required=True,
        help="Ground-truth file, columns: t E N U or t N E D",
    )
    parser.add_argument(
        "--ground_truth_frame",
        choices=("auto", "enu", "ned"),
        default="auto",
        help="Frame of the ground-truth file. Default auto treats *_ned.txt as NED, otherwise ENU.",
    )
    parser.add_argument(
        "--save",
        default="",
        help="Optional path for saving the generated figure.",
    )
    args = parser.parse_args()

    poses_original = load_nav_file(args.initial_poses) if args.initial_poses else None
    poses_optimized = load_nav_file(args.optimized_poses)
    ground_truth = load_ground_truth(args.ground_truth, args.ground_truth_frame)

    gt_cropped, opt_cropped = crop_to_common_time(ground_truth, poses_optimized)
    matched_indices = nearest_time_indices(gt_cropped[:, 0], opt_cropped[:, 0])
    matched_optimized = opt_cropped[matched_indices]

    xyz_error, error_3d = compute_error_metrics(gt_cropped[:, 1:4], matched_optimized[:, 1:4])
    print_metrics(xyz_error, error_3d)

    fig = plt.figure(figsize=(14, 10))

    ax1 = fig.add_subplot(2, 2, 1, projection="3d")
    ax1.plot(
        gt_cropped[:, 1],
        gt_cropped[:, 2],
        gt_cropped[:, 3],
        "--",
        label="Ground Truth",
        color="red",
    )
    ax1.plot(
        matched_optimized[:, 1],
        matched_optimized[:, 2],
        matched_optimized[:, 3],
        label="Optimized",
        color="blue",
    )
    if poses_original is not None:
        original_cropped, _ = crop_to_common_time(poses_original, ground_truth)
        ax1.plot(
            original_cropped[:, 1],
            original_cropped[:, 2],
            original_cropped[:, 3],
            label="Initial",
            color="green",
            alpha=0.6,
        )
    ax1.set_title("3D Trajectory Comparison")
    ax1.set_xlabel("North (m)")
    ax1.set_ylabel("East (m)")
    ax1.set_zlabel("Down (m)")
    ax1.legend()

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(gt_cropped[:, 0], xyz_error[:, 0], label="North error")
    ax2.plot(gt_cropped[:, 0], xyz_error[:, 1], label="East error")
    ax2.plot(gt_cropped[:, 0], xyz_error[:, 2], label="Down error")
    ax2.set_title("Axis-wise Error in NED")
    ax2.set_xlabel("Timestamp")
    ax2.set_ylabel("Error (m)")
    ax2.grid(True)
    ax2.legend()

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(gt_cropped[:, 0], error_3d, color="black")
    ax3.set_title("3D Position Error")
    ax3.set_xlabel("Timestamp")
    ax3.set_ylabel("Distance (m)")
    ax3.grid(True)

    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(gt_cropped[:, 0], gt_cropped[:, 1], "--", label="GT North")
    ax4.plot(gt_cropped[:, 0], gt_cropped[:, 2], "--", label="GT East")
    ax4.plot(gt_cropped[:, 0], gt_cropped[:, 3], "--", label="GT Down")
    ax4.plot(gt_cropped[:, 0], matched_optimized[:, 1], label="OPT North")
    ax4.plot(gt_cropped[:, 0], matched_optimized[:, 2], label="OPT East")
    ax4.plot(gt_cropped[:, 0], matched_optimized[:, 3], label="OPT Down")
    ax4.set_title("Aligned Position Components")
    ax4.set_xlabel("Timestamp")
    ax4.set_ylabel("Position (m)")
    ax4.grid(True)
    ax4.legend(ncol=2)

    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=200, bbox_inches="tight")
        print(f"figure saved to: {args.save}")

    plt.show()


if __name__ == "__main__":
    main()

"""
how to use:
python3 script/plot_results_3d.py \
    --optimized_poses dataset/251127_5/OB_GINS_TXT.nav \
    --ground_truth dataset/251127_5/ground_truth.txt
"""
