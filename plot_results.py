#!/usr/bin/python
#
# Plots the results from the 3D pose graph optimization. It will draw a line
# between consecutive vertices.  The commandline expects two optional filenames:
#
#   ./plot_results.py --initial_poses optional --optimized_poses optional --ground_truth optional


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
  pointnum = poses_optimized.shape[0]
  err = 0
  n = 0
  for j in range(pointnum):
    time_floor=round(poses_optimized[j,0])
    if(abs(time_floor-poses_optimized[j,0])<0.02):
    	bias = (ground_truth[time_floor+1,1] - poses_optimized[j,1])**2+(ground_truth[time_floor+1,2] - poses_optimized[j,2])**2
    	err = err + np.sqrt(bias)
    	n=n+1
  err = err / n
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


plot.axis('equal')
plot.legend(loc='best')

fig=plot.figure()
plot.subplot(3, 1, 1)
plot.plot(poses_optimized[:, 0], poses_optimized[:, 4])
plot.subplot(3, 1, 2)
plot.plot(poses_optimized[:, 0], poses_optimized[:, 5])
plot.subplot(3, 1, 3)
plot.plot(poses_optimized[:, 0], poses_optimized[:, 6])

fig=plot.figure()
plot.subplot(3, 1, 1)
plot.plot(poses_optimized[:, 0], poses_optimized[:, 7])
plot.subplot(3, 1, 2)
plot.plot(poses_optimized[:, 0], poses_optimized[:, 8])
plot.subplot(3, 1, 3)
plot.plot(poses_optimized[:, 0], poses_optimized[:, 9])

# Show the plot and wait for the user to close.
plot.show()

