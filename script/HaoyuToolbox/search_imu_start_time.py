#!/usr/bin/env python3
"""Search the best IMU start time by regenerating IMU files and scoring nav error.

This script keeps the original VLP file unchanged. For each candidate start time,
it regenerates a temporary IMU file via ``convert_imu.py``, writes a temporary
config file that points to that IMU file, runs ``ob_gins_vlp``, and evaluates the
mean 2D trajectory error against ground truth.
"""

from __future__ import annotations

import argparse
import csv
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np


DEFAULT_TOPK = 5
TIME_TOL = 1e-9


@dataclass
class CandidateResult:
    start_time: float
    imu_path: Path
    status: str
    mean_2d_error: float | None
    sample_count: int
    note: str = ""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Search IMU start time by regenerating IMU files and running OB_GINS_VLP."
    )
    parser.add_argument("--config", required=True, help="Path to the base YAML config.")
    parser.add_argument("--raw-imu", required=True, help="Path to the raw MTi log for convert_imu.py.")
    parser.add_argument("--gt", required=True, help="Path to ground truth file.")
    parser.add_argument(
        "--workdir",
        default=".",
        help="Repository root used to run commands. Defaults to current directory.",
    )
    parser.add_argument(
        "--center-start-time",
        type=float,
        default=None,
        help="Search center. If omitted, read the first timestamp from the configured IMU file.",
    )
    parser.add_argument(
        "--span",
        type=float,
        default=1.0,
        help="Search half-span in seconds. Defaults to 1.0.",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=0.05,
        help="Search step in seconds. Defaults to 0.05.",
    )
    parser.add_argument(
        "--sample-interval",
        type=float,
        default=0.005,
        help="IMU sample interval passed to convert_imu.py. Defaults to 0.005.",
    )
    parser.add_argument(
        "--keep-generated",
        action="store_true",
        help="Keep generated candidate IMU files. Default is to delete them after the search.",
    )
    parser.add_argument(
        "--topk",
        type=int,
        default=DEFAULT_TOPK,
        help=f"How many best candidates to print. Defaults to {DEFAULT_TOPK}.",
    )
    parser.add_argument(
        "--runner",
        default="./bin/ob_gins_vlp",
        help="Path to the OB_GINS_VLP executable relative to workdir.",
    )
    parser.add_argument(
        "--nav-file",
        default=None,
        help="Override the nav output file. Defaults to <outputpath>/OB_GINS_TXT.nav from config.",
    )
    return parser.parse_args()


def parse_scalar(config_text: str, key: str) -> str:
    pattern = re.compile(rf"(?m)^[ \t]*{re.escape(key)}[ \t]*:[ \t]*([^\n#]+)")
    match = pattern.search(config_text)
    if not match:
        raise RuntimeError(f"Cannot find '{key}' in config.")
    return match.group(1).strip().strip('"').strip("'")


def replace_scalar(config_text: str, key: str, value: str) -> str:
    pattern = re.compile(rf"(?m)^([ \t]*{re.escape(key)}[ \t]*:[ \t]*)([^\n#]*)(.*)$")
    match = pattern.search(config_text)
    if not match:
        raise RuntimeError(f"Cannot replace '{key}' in config.")
    return config_text[: match.start()] + f"{match.group(1)}\"{value}\"{match.group(3)}" + config_text[match.end() :]


def first_valid_time(path: Path) -> float:
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            return float(stripped.split()[0])
    raise RuntimeError(f"No valid data rows found in {path}")


def generate_candidates(center: float, span: float, step: float) -> list[float]:
    if step <= 0:
        raise ValueError("--step must be > 0")
    if span < 0:
        raise ValueError("--span must be >= 0")

    count = int(round((2 * span) / step))
    values = [center - span + i * step for i in range(count + 1)]
    rounded = sorted({round(v + 0.0, 3) for v in values})
    return rounded


def load_gt_enu_to_ned(path: Path) -> tuple[np.ndarray, np.ndarray]:
    gt = np.loadtxt(path, usecols=(0, 1, 2, 3))
    if gt.ndim == 1:
        gt = gt.reshape(1, -1)
    t = gt[:, 0]
    e = gt[:, 1]
    n = gt[:, 2]
    u = gt[:, 3]
    ned = np.column_stack([n, e, -u])
    return t, ned


def load_nav_ned(path: Path) -> tuple[np.ndarray, np.ndarray]:
    nav = np.loadtxt(path, usecols=(0, 1, 2, 3))
    if nav.ndim == 1:
        nav = nav.reshape(1, -1)
    return nav[:, 0], nav[:, 1:4]


def nearest_time_match(src_t: np.ndarray, dst_t: np.ndarray) -> np.ndarray:
    idx = np.searchsorted(dst_t, src_t)
    idx = np.clip(idx, 1, len(dst_t) - 1)
    left = idx - 1
    choose = np.where(np.abs(dst_t[idx] - src_t) < np.abs(dst_t[left] - src_t), idx, left)
    return choose


def compute_mean_2d_error(gt_path: Path, nav_path: Path) -> tuple[float, int]:
    gt_t, gt_ned = load_gt_enu_to_ned(gt_path)
    nav_t, nav_pos = load_nav_ned(nav_path)

    common_start = max(gt_t[0], nav_t[0])
    common_end = min(gt_t[-1], nav_t[-1])
    if common_start > common_end + TIME_TOL:
        raise RuntimeError(
            f"No common time span between GT [{gt_t[0]:.3f}, {gt_t[-1]:.3f}] and NAV [{nav_t[0]:.3f}, {nav_t[-1]:.3f}]"
        )

    gt_mask = (gt_t >= common_start - TIME_TOL) & (gt_t <= common_end + TIME_TOL)
    nav_mask = (nav_t >= common_start - TIME_TOL) & (nav_t <= common_end + TIME_TOL)
    gt_t = gt_t[gt_mask]
    gt_ned = gt_ned[gt_mask]
    nav_t = nav_t[nav_mask]
    nav_pos = nav_pos[nav_mask]

    if len(gt_t) == 0 or len(nav_t) == 0:
        raise RuntimeError("No samples remain after common-window cropping.")

    gt_idx = nearest_time_match(nav_t, gt_t)
    gt_match = gt_ned[gt_idx]
    h_err = np.linalg.norm(nav_pos[:, :2] - gt_match[:, :2], axis=1)
    return float(h_err.mean()), int(len(h_err))


def csv_safe_time_tag(start_time: float) -> str:
    return f"{start_time:.3f}".replace("-", "m").replace(".", "p")


def run_command(cmd: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=cwd,
        check=False,
        text=True,
        capture_output=True,
    )


def write_csv(results: list[CandidateResult], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["start_time", "status", "mean_2d_error", "sample_count", "imu_file", "note"])
        for item in results:
            writer.writerow(
                [
                    f"{item.start_time:.3f}",
                    item.status,
                    "" if item.mean_2d_error is None else f"{item.mean_2d_error:.6f}",
                    item.sample_count,
                    str(item.imu_path),
                    item.note,
                ]
            )


def main() -> int:
    args = parse_args()
    workdir = Path(args.workdir).resolve()
    config_path = Path(args.config).resolve()
    raw_imu_path = Path(args.raw_imu).resolve()
    gt_path = Path(args.gt).resolve()
    runner_path = Path(args.runner)

    if not config_path.exists():
        raise FileNotFoundError(f"Config not found: {config_path}")
    if not raw_imu_path.exists():
        raise FileNotFoundError(f"Raw IMU file not found: {raw_imu_path}")
    if not gt_path.exists():
        raise FileNotFoundError(f"Ground truth file not found: {gt_path}")

    config_text = config_path.read_text(encoding="utf-8")
    base_imu_rel = parse_scalar(config_text, "imufile")
    output_rel = parse_scalar(config_text, "outputpath")
    base_imu_path = (workdir / base_imu_rel).resolve()
    output_dir = (workdir / output_rel).resolve()
    nav_path = Path(args.nav_file).resolve() if args.nav_file else (output_dir / "OB_GINS_TXT.nav")
    csv_path = output_dir / "imu_start_time_search.csv"

    if not base_imu_path.exists():
        raise FileNotFoundError(f"Configured IMU file not found: {base_imu_path}")

    center = args.center_start_time if args.center_start_time is not None else first_valid_time(base_imu_path)
    candidates = generate_candidates(center, args.span, args.step)

    print(f"Base config      : {config_path}")
    print(f"Raw IMU input    : {raw_imu_path}")
    print(f"Configured IMU   : {base_imu_path}")
    print(f"Ground truth     : {gt_path}")
    print(f"Nav output       : {nav_path}")
    print(f"Search center    : {center:.3f} s")
    print(f"Search span/step : +/-{args.span:.3f} s / {args.step:.3f} s")
    print(f"Candidates       : {len(candidates)}")

    results: list[CandidateResult] = []
    generated_imu_paths: list[Path] = []

    with tempfile.TemporaryDirectory(prefix="imu_sync_search_") as tmpdir_str:
        tmpdir = Path(tmpdir_str)
        for idx, start_time in enumerate(candidates, start=1):
            tag = csv_safe_time_tag(start_time)
            imu_filename = f"{base_imu_path.stem}_sync_{tag}{base_imu_path.suffix}"
            imu_path = output_dir / imu_filename
            generated_imu_paths.append(imu_path)

            convert_cmd = [
                sys.executable,
                str((workdir / "script/HaoyuToolbox/convert_imu.py").resolve()),
                "--input",
                str(raw_imu_path),
                "--output",
                str(imu_path),
                "--start-time",
                f"{start_time:.3f}",
                "--sample-interval",
                f"{args.sample_interval:.6f}",
            ]
            convert_proc = run_command(convert_cmd, workdir)
            if convert_proc.returncode != 0:
                results.append(
                    CandidateResult(
                        start_time=start_time,
                        imu_path=imu_path,
                        status="convert_failed",
                        mean_2d_error=None,
                        sample_count=0,
                        note=convert_proc.stderr.strip() or convert_proc.stdout.strip(),
                    )
                )
                print(f"[{idx}/{len(candidates)}] {start_time:.3f}s convert failed")
                continue

            temp_config_text = replace_scalar(config_text, "imufile", str(imu_path.relative_to(workdir)))
            temp_config_path = tmpdir / f"{config_path.stem}_{tag}.yaml"
            temp_config_path.write_text(temp_config_text, encoding="utf-8")

            run_proc = run_command([str(runner_path), str(temp_config_path)], workdir)
            if run_proc.returncode != 0:
                results.append(
                    CandidateResult(
                        start_time=start_time,
                        imu_path=imu_path,
                        status="run_failed",
                        mean_2d_error=None,
                        sample_count=0,
                        note=run_proc.stderr.strip() or run_proc.stdout.strip(),
                    )
                )
                print(f"[{idx}/{len(candidates)}] {start_time:.3f}s run failed")
                continue

            if not nav_path.exists():
                results.append(
                    CandidateResult(
                        start_time=start_time,
                        imu_path=imu_path,
                        status="missing_nav",
                        mean_2d_error=None,
                        sample_count=0,
                        note=f"Expected nav file not found: {nav_path}",
                    )
                )
                print(f"[{idx}/{len(candidates)}] {start_time:.3f}s missing nav")
                continue

            try:
                mean_2d_error, sample_count = compute_mean_2d_error(gt_path, nav_path)
            except Exception as exc:  # noqa: BLE001
                results.append(
                    CandidateResult(
                        start_time=start_time,
                        imu_path=imu_path,
                        status="score_failed",
                        mean_2d_error=None,
                        sample_count=0,
                        note=str(exc),
                    )
                )
                print(f"[{idx}/{len(candidates)}] {start_time:.3f}s score failed")
                continue

            results.append(
                CandidateResult(
                    start_time=start_time,
                    imu_path=imu_path,
                    status="ok",
                    mean_2d_error=mean_2d_error,
                    sample_count=sample_count,
                )
            )
            print(
                f"[{idx}/{len(candidates)}] {start_time:.3f}s ok "
                f"mean_2d={mean_2d_error:.4f}m samples={sample_count}"
            )

    write_csv(results, csv_path)

    ok_results = [item for item in results if item.status == "ok" and item.mean_2d_error is not None]
    ok_results.sort(key=lambda item: item.mean_2d_error)

    print()
    print(f"CSV saved to: {csv_path}")
    if ok_results:
        print("Best candidates:")
        for rank, item in enumerate(ok_results[: max(args.topk, 1)], start=1):
            print(
                f"  {rank}. start_time={item.start_time:.3f} s "
                f"mean_2d={item.mean_2d_error:.4f} m samples={item.sample_count} imu={item.imu_path.name}"
            )
    else:
        print("No valid candidate produced a score.")

    if not args.keep_generated:
        for path in generated_imu_paths:
            if path.exists():
                path.unlink()

    return 0 if ok_results else 1


if __name__ == "__main__":
    raise SystemExit(main())

"""
how to use:
python3 script/HaoyuToolbox/search_imu_start_time.py \
    --config config/251127_5.yaml \
    --raw-imu dataset/251127_5/MT_07782A2B_016-000.txt \
    --gt dataset/251127_5/ground_truth.txt \
    --workdir .
"""
