#!/usr/bin/python
#
# Plots the results from the 3D pose graph optimization. It will draw a line
# between consecutive vertices.  The commandline expects three optional filenames:
#
#   ./plot_results.py --initial_poses optional --optimized_poses optional --ground_truth optional
# The positioning error of "optimized_poses" will be evaluated based on the "ground_truth" file.


from pathlib import Path

import matplotlib
import numpy as np
import sys
from optparse import OptionParser
from matplotlib.patches import Ellipse

parser = OptionParser()
parser.add_option("--initial_poses", dest="initial_poses",
                  default="", help="The filename that contains the original poses.")
parser.add_option("--optimized_poses", dest="optimized_poses",
                  default="", help="The filename that contains the optimized poses.")
parser.add_option("--ground_truth", dest="ground_truth",
                  default="", help="The filename that contains the ground truth.")
parser.add_option("--ground_truth_frame", type="choice", choices=("ned", "neu", "enu"),
                  default="ned", help="Reference frame: ned (unchanged, default), "
                  "neu (negate Z), or enu (swap X/Y and negate Z). "
                  "Navigation output is assumed to be NED.")
parser.add_option("--show", action="store_true", default=False,
                  help="Show plot windows after saving (default: save only).")
(options, args) = parser.parse_args()

if not options.optimized_poses:
  parser.error("--optimized_poses is required")
if not options.show:
  matplotlib.use("Agg")
import matplotlib.pyplot as plot

output_file = Path(options.optimized_poses)
figures_to_save = []

# Read the original and optimized poses files.
poses_original = None
if options.initial_poses != '':
  poses_original = np.genfromtxt(options.initial_poses, usecols = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9))

poses_optimized = None
if options.optimized_poses != '':
  poses_optimized = np.genfromtxt(options.optimized_poses, usecols = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9))
  
ground_truth = None
if options.ground_truth != '' and options.optimized_poses != '':
  ground_truth = np.genfromtxt(options.ground_truth, usecols = (0, 1, 2, 3))

  # Convert only reference positions to the navigation output's NED frame.
  # Apply before both plotting and error evaluation; never modify the input file.
  ground_truth = np.atleast_2d(ground_truth)
  if options.ground_truth_frame == "enu":
    ground_truth[:, [1, 2]] = ground_truth[:, [2, 1]]
  if options.ground_truth_frame in ("neu", "enu"):
    ground_truth[:, 3] *= -1
  print(f"ground truth frame: {options.ground_truth_frame} -> ned")

  # 提取时间戳
  gt_timestamps = ground_truth[:, 0]
  opt_timestamps = poses_optimized[:, 0]

  # 共同的时间窗口
  gt_start, gt_end = gt_timestamps[0], gt_timestamps[-1]
  opt_start, opt_end = opt_timestamps[0], opt_timestamps[-1]

  common_start = max(gt_start, opt_start)
  common_end = min(gt_end, opt_end)
  if common_start > common_end + 1e-6:
    raise ValueError(f"No common time slot: GT [{gt_start}, {gt_end}], OPT [{opt_start}, {opt_end}]")

  gt_mask = (gt_timestamps >= common_start - 1e-6) & (gt_timestamps <= common_end + 1e-6)
  opt_mask = (opt_timestamps >= common_start - 1e-6) & (opt_timestamps <= common_end + 1e-6)
  gt_indices = np.where(gt_mask)[0]
  opt_indices = np.where(opt_mask)[0]

  gt_cropped = ground_truth[gt_indices, :]
  opt_cropped = poses_optimized[opt_indices, :]

  # 使用 numpy.searchsorted 找到每个优化位姿时间戳在 ground_truth 中最接近的时间戳的索引
  indices = np.searchsorted(opt_cropped[:, 0], gt_cropped[:, 0], side='left')

  # 处理边界情况
  distances = []

  for i, idx in enumerate(indices):
    gt_time = gt_cropped[i, 0]
    # Extract X, Y, and Z for Ground Truth
    gt_x, gt_y, gt_z = gt_cropped[i, 1], gt_cropped[i, 2], gt_cropped[i, 3]
    
    # Determine candidate indices
    candidates = []
    if idx > 0:
        candidates.append(idx - 1)
    if idx < len(opt_cropped[:, 0]):
        candidates.append(idx)
    
    # Find the closest timestamp in optimized poses
    best_idx = min(candidates, key=lambda j: abs(opt_cropped[j, 0] - gt_time))
    
    # Extract X, Y, and Z for Optimized Pose
    opt_x, opt_y, opt_z = opt_cropped[best_idx, 1], opt_cropped[best_idx, 2], opt_cropped[best_idx, 3]
    
    # Calculate 3D Euclidean distance
    distance = np.sqrt((opt_x - gt_x)**2 + (opt_y - gt_y)**2 + (opt_z - gt_z)**2)
    distances.append(distance)

  fig = plot.figure(figsize=(10, 8))
  figures_to_save.append((fig, "position_error"))

  # 用颜色区分位置分量，用线型区分数据来源。
  ax1 = fig.add_subplot(2, 1, 1)
  for column, component, color in ((1, 'X', 'C0'), (2, 'Y', 'C1'), (3, 'Z', 'C2')):
    ax1.plot(opt_timestamps, poses_optimized[:, column],
             color=color, label=f'Optimized {component}')
    if poses_original is not None:
      ax1.plot(poses_original[:, 0], poses_original[:, column], ':',
               color=color, label=f'Initial {component}')
    ax1.plot(gt_timestamps, ground_truth[:, column], '--',
             color=color, label=f'Ground Truth {component}')
  ax1.set_title('Position Components')
  ax1.set_xlabel('Timestamp')
  ax1.set_ylabel('Position (m)')
  ax1.legend(ncol=3)
  ax1.grid(True)

  ax2 = fig.add_subplot(2, 1, 2)
  ax2.plot(gt_cropped[:, 0], distances, label='3D Position Error')
  ax2.set_title('3D Position Error')
  ax2.set_xlabel('Timestamp')
  ax2.set_ylabel('Distance (m)')
  ax2.legend()
  ax2.grid(True)

  plot.tight_layout()

  err = np.mean(distances)
  print('mean 3D error:', err)
  print('RMSE 3D error:', np.sqrt(np.mean(np.square(distances))))

# Plots the results for the specified poses.
fig=plot.figure()
figures_to_save.append((fig, "trajectory_2d"))
if poses_original is not None:
  plot.plot(poses_original[:, 2], poses_original[:, 1], '*-', label="Original",
            alpha=0.5, color="green")

if poses_optimized is not None:
  plot.plot(poses_optimized[:, 2], poses_optimized[:, 1], '*-', label="Optimized",
            alpha=0.5, color="blue")

if ground_truth is not None:
  plot.plot(ground_truth[:, 2], ground_truth[:, 1], '--', color="red", label="Ground Truth")

plot.title('2D Position Comparison (m)')
plot.grid(True)
plot.axis('equal')
plot.legend(loc='best')

# 三轴速度曲线：三个分量共享坐标轴。
fig1, ax_velocity = plot.subplots(figsize=(10, 5))
figures_to_save.append((fig1, "velocity"))
for column, component in ((4, 'X'), (5, 'Y'), (6, 'Z')):
  ax_velocity.plot(poses_optimized[:, 0], poses_optimized[:, column],
                   label=f'Velocity {component}')
ax_velocity.set_title('Triaxial Velocity Curves')
ax_velocity.set_xlabel('Time (s)')
ax_velocity.set_ylabel('Velocity (m/s)')
ax_velocity.legend()
ax_velocity.grid(True)
fig1.tight_layout()

# 三轴姿态角曲线：导航文件中的角度单位为度。
fig2, ax_attitude = plot.subplots(figsize=(10, 5))
figures_to_save.append((fig2, "attitude"))
for column, component in ((7, 'Roll (X)'), (8, 'Pitch (Y)'), (9, 'Yaw (Z)')):
  ax_attitude.plot(poses_optimized[:, 0], poses_optimized[:, column], label=component)
ax_attitude.set_title('Triaxial Attitude Angle Curves')
ax_attitude.set_xlabel('Time (s)')
ax_attitude.set_ylabel('Angle (deg)')
ax_attitude.legend()
ax_attitude.grid(True)
fig2.tight_layout()

# Save every figure beside the optimized poses, then optionally show windows.
for figure, name in figures_to_save:
  save_path = output_file.parent / f"{output_file.stem}_{name}.png"
  figure.savefig(save_path, dpi=200, bbox_inches="tight")
  print(f"figure saved to: {save_path}")

if options.show:
  plot.show()
plot.close("all")
