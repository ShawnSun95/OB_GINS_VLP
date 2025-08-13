#!/usr/bin/python
#
# Plots the results from the 3D pose graph optimization. It will draw a line
# between consecutive vertices.  The commandline expects three optional filenames:
#
#   ./plot_results.py --initial_poses optional --optimized_poses optional --ground_truth optional
# The positioning error of "optimized_poses" will be evaluated based on the "ground_truth" file.


import matplotlib.pyplot as plot
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
(options, args) = parser.parse_args()

# Read the original and optimized poses files.
poses_original = None
if options.initial_poses != '':
  poses_original = np.genfromtxt(options.initial_poses, usecols = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9))

poses_optimized = None
if options.optimized_poses != '':
  poses_optimized = np.genfromtxt(options.optimized_poses, usecols = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9))
  
ground_truth = None
if options.ground_truth != '':
  ground_truth = np.genfromtxt(options.ground_truth, usecols = (0, 1, 2, 3))

  # 提取时间戳
  gt_timestamps = ground_truth[:, 0]
  opt_timestamps = poses_optimized[:, 0] + gt_timestamps[0]

  # 使用 numpy.searchsorted 找到每个优化位姿时间戳在 ground_truth 中最接近的时间戳的索引
  # 注意：searchsorted 找的是插入位置，我们取左边或右边最近的
  indices = np.searchsorted(gt_timestamps, opt_timestamps, side='left')

  # 处理边界情况
  # 对于每个索引，比较左边和右边（如果存在）哪个时间戳更接近
  distances = []

  for i, idx in enumerate(indices):
    opt_time = opt_timestamps[i]
    opt_x, opt_y = poses_optimized[i, 1], poses_optimized[i, 2]
    
    # 确定比较的候选索引
    candidates = []
    if idx > 0:
        candidates.append(idx - 1)
    if idx < len(gt_timestamps):
        candidates.append(idx)
    
    # 找到时间戳最接近的 ground truth 行
    best_idx = min(candidates, key=lambda j: abs(gt_timestamps[j] - opt_time))
    gt_x, gt_y = ground_truth[best_idx, 1], ground_truth[best_idx, 2]
    
    # 计算欧几里得距离
    distance = np.sqrt((opt_x - gt_x)**2 + (opt_y - gt_y)**2)
    distances.append(distance)

  fig = plot.figure(figsize=(8, 10))

  # 子图 1
  ax1 = fig.add_subplot(4, 1, 1)
  ax1.plot(opt_timestamps, poses_optimized[:, 1], label='Optimized')
  ax1.plot(gt_timestamps, ground_truth[:, 1], label='Ground Truth')
  ax1.set_title('Position X')
  ax1.set_xlabel('Timestamp')
  ax1.set_ylabel('Position (m)')
  ax1.legend()

  # 子图 2
  ax2 = fig.add_subplot(4, 1, 2)
  ax2.plot(opt_timestamps, poses_optimized[:, 2], label='Optimized')
  ax2.plot(gt_timestamps, ground_truth[:, 2], label='Ground Truth')
  ax2.set_title('Position Y')
  ax2.set_xlabel('Timestamp')
  ax2.set_ylabel('Position (m)')
  ax2.legend()

  # 子图 3
  ax3 = fig.add_subplot(4, 1, 3)
  ax3.plot(opt_timestamps, poses_optimized[:, 3], label='Optimized')
  ax3.plot(gt_timestamps, ground_truth[:, 3], label='Ground Truth')
  ax3.set_title('Position Z')
  ax3.set_xlabel('Timestamp')
  ax3.set_ylabel('Position (m)')
  ax3.legend()

  # 子图 4
  ax4 = fig.add_subplot(4, 1, 4)
  ax4.plot(opt_timestamps, distances)
  ax4.set_title('2D Position Error')
  ax4.set_xlabel('Timestamp')
  ax4.set_ylabel('Distance (m)')

  plot.tight_layout()  # 自动调整子图参数，使之填充整个图像区域

  err = np.mean(distances)
  print('mean error:',err)

# Plots the results for the specified poses.
fig=plot.figure()
if poses_original is not None:
  plot.plot(poses_original[:, 2], poses_original[:, 1], '*-', label="Original",
            alpha=0.5, color="green")

if poses_optimized is not None:
  plot.plot(poses_optimized[:, 2], poses_optimized[:, 1], '*-', label="Optimized",
            alpha=0.5, color="blue")

if ground_truth is not None:
  plot.plot(ground_truth[:, 2], ground_truth[:, 1], '--', color="red")

plot.title('2D Position Comparison (m)')
plot.grid(True)
plot.axis('equal')
plot.legend(loc='best')

# 三轴速度曲线
fig1 = plot.figure(figsize=(8, 8))

plot.subplot(3, 1, 1)
plot.plot(poses_optimized[:, 0], poses_optimized[:, 4], label='Velocity X')
plot.ylabel('Vx (m/s)')
plot.title('Triaxial Velocity Curves')
plot.legend()
plot.grid(True)

plot.subplot(3, 1, 2)
plot.plot(poses_optimized[:, 0], poses_optimized[:, 5], label='Velocity Y')
plot.ylabel('Vy (m/s)')
plot.legend()
plot.grid(True)

plot.subplot(3, 1, 3)
plot.plot(poses_optimized[:, 0], poses_optimized[:, 6], label='Velocity Z')
plot.ylabel('Vz (m/s)')
plot.xlabel('Time (s)')
plot.legend()
plot.grid(True)

plot.tight_layout()

# 三轴姿态角曲线（如 roll, pitch, yaw）
fig2 = plot.figure(figsize=(8, 8))

plot.subplot(3, 1, 1)
plot.plot(poses_optimized[:, 0], poses_optimized[:, 7], label='Roll (X)')
plot.ylabel('Angle (rad)')
plot.title('Triaxial Attitude Angle Curves')
plot.legend()
plot.grid(True)

plot.subplot(3, 1, 2)
plot.plot(poses_optimized[:, 0], poses_optimized[:, 8], label='Pitch (Y)')
plot.ylabel('Angle (rad)')
plot.legend()
plot.grid(True)

plot.subplot(3, 1, 3)
plot.plot(poses_optimized[:, 0], poses_optimized[:, 9], label='Yaw (Z)')
plot.ylabel('Angle (rad)')
plot.xlabel('Time (s)')
plot.legend()
plot.grid(True)

plot.tight_layout()

# Show the plot and wait for the user to close.
plot.show()
