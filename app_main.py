import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import random
import itertools
import copy
import os
import json

from scipy import interpolate
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from sklearn.cluster import KMeans
from collections import defaultdict
from utils import *

from MissionScenario import MissionScenario as Scenario
from ZoningProblem import ZoningSolution, ZoningAlgorithm, RoutingAlgorithm



with open('in-out-examples/mission-input.json') as json_file:
  jsonresp = json.load(json_file)

scenario = Scenario.load(jsonresp)

z_algo = ZoningAlgorithm(scenario)
z_algo.search()


zoning_sol = scenario.zoning_sol


## zone plygones
zone_polygons = []
for i in range(1, scenario.nb_agents+1):
  zone_poly = zoning_sol.separators[i-1] + zoning_sol.separators[i][::-1]
  zone_polygons.append(zone_poly)

## routes
r_algo = RoutingAlgorithm(scenario)
[smooth_routes, routes] = r_algo.search()

## combined solution
solution = {}
solution['zone_polygons'] = zone_polygons
solution['routes'] = routes
print(solution)




#=====================================
# PLOT LATEST SOLUTION:
#=====================================
hexcolours = ['#000000',
  '#6efa88',
  '#6ec7fa',
  '#f86efa',
  '#fac06e',
  '#fa6e6e',
  '#6efa88',
  '#6ec7fa',
  '#f86efa',
  '#fac06e',
  '#fa6e6e',
]

x = scenario.targets_x
y = scenario.targets_y
nb_targets = scenario.nb_targets
(xb, yb) = scenario.base
(xn, yn) = scenario.exit
(xe, ye) = scenario.centroid
Re = scenario.radius_internal

(xc,yc) = scenario.centre
SR = scenario.radius


fig = plt.figure()
fig.set_size_inches(9, 11)
ax = fig.add_subplot(111)
ax.grid(False)
# plot base station
ax.scatter(xb, yb, color='b', marker='^', s=100)
ax.text(xb, yb, 'starting point', size=15, zorder=1, color='k')
# plot next station
ax.scatter(xn, yn, color='g', marker='^', s=100)
ax.text(xn, yn, 'ending point', size=15, zorder=1, color='k')

# sample points around safety circle:
for v in np.arange(0, 2*math.pi, math.pi/64.0):
  rnd = np.random.rand()
  theta = math.pi + v
  tx = xe + Re * math.cos(theta)
  ty = ye + Re * math.sin(theta)
  ax.scatter(tx, ty, color='y', marker='.', s=6)

nb_separators = scenario.nb_agents + 1

for i in range(nb_separators):
  for j in range(scenario.nb_wp):
    if zoning_sol.solution[j]==1:
      ax.scatter(scenario.sep_waypoints_x[j], scenario.sep_waypoints_y[j], color='#888', marker='x', s=50)
      # plot separators
      sep = zoning_sol.separators[i]

      for j in range(len(sep)-1):
        p1 = sep[j]
        p2 = sep[j+1]
        xline = (p1[0], p2[0])
        yline = (p1[1], p2[1])
        ax.plot(xline, yline, linestyle = '--', color='#888', linewidth=.5)

for zone_id, nodes in zoning_sol.zones.items():
  for node in nodes:
    p = scenario.priorities[node]
    ax.scatter(x[node], y[node], c=hexcolours[zone_id], s=p*20)

#=====================================

## ZONE POLYGONES
for i in range(scenario.nb_agents):
  #zone_poly = zoning_sol.separators[i-1] + zoning_sol.separators[i][::-1]
  #print(zone_poly)
  zone_poly = zone_polygons[i]
  for xy in zone_poly:
    ax.scatter(xy[0]+np.random.rand(),xy[1]+np.random.rand(), color=hexcolours[i+1],marker='x')



## PATHS
for i in range(1, scenario.nb_agents+1):
  
  # plot original route
  cx, cy = zip(*routes[i-1])
  cx = np.r_[cx]
  cy = np.r_[cy]
  ax.plot(cx, cy, c='g', linewidth=1)

  
  cx, cy = zip(*smooth_routes[i-1])
  cx = np.r_[cx]
  cy = np.r_[cy]
  ax.plot(cx, cy, c='r', linewidth=1)


plt.show()
#=====================================
