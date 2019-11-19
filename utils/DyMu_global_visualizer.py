# -*- coding: utf-8 -*-

#========================DyMu Results Visualizer===============================
#           Visualization Tool to graphically represent maps and path
#            Author: J. Ricardo Sanchez Ibanez (ricardosan@uma.es)
#==============================================================================

import numpy as np
import matplotlib.pyplot as plt

costMap = np.loadtxt(open("data/results/GlobalCostMap.txt"), skiprows=0)
costMap[np.where(costMap==-1)] = np.amax(costMap)

total_cost = np.loadtxt(open("data/results/TotalCostMap.txt"), skiprows=0)
total_cost[np.where(total_cost==-1)] = np.amax(total_cost)

path = np.loadtxt(open("data/results/Path.txt"), skiprows=0)

fig1, (ax1, ax2) = plt.subplots(1, 2, tight_layout=True)
plot1 = ax1.contourf(costMap, 40, cmap = 'Reds')
ax1.set_aspect('equal')
ax1.plot(path[:,0],path[:,1],'b')
ax1.set_xlabel('X-axis')
ax1.set_ylabel('Y-axis')
cb1 = fig1.colorbar(plot1, ax = ax1, orientation = 'horizontal')
cb1.ax.set_title('Cost')

plot2 = ax2.contourf(total_cost, 100, cmap = 'viridis', alpha = .5)
ax2.contour(total_cost, 100, cmap = 'viridis')
ax2.plot(path[:,0],path[:,1],'r')
ax2.set_aspect('equal')
ax2.set_xlabel('X-axis')
ax2.set_ylabel('Y-axis')
cb2 = fig1.colorbar(plot2, ax = ax2, orientation = 'horizontal')
cb2.ax.set_title('Total Cost')

plt.show()
