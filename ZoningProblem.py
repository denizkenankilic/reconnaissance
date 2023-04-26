import numpy as np
import math
import copy
import itertools
import requests

from scipy import interpolate
from sklearn.cluster import KMeans
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from collections import defaultdict
from scipy.spatial import ConvexHull, convex_hull_plot_2d

from utils import *
#from MissionScenario import MissionScenario


#==================================
# Zoning problem
#==================================
class ZoningAlgorithm:

  #====================================
  # Initialise Zoning problem instance
  #====================================
  def __init__(self, scenario=None, solution=None):
    self.scenario = scenario

  #====================================
  # Solution generation
  #====================================
  def search(self):

    #print('sep alloc:', scenario.zoning_sol)
    Zbest = self.scenario.zoning_sol
    Zbest.evaluate()
    improved = True
    it = 0
    while improved:
      it += 1
      Z = Zbest
      improved = False
      for i in range(1, self.scenario.nb_wp-1):
        for j in range(i+1, self.scenario.nb_wp-1):
          if Z.solution[i]==Z.solution[j]:
            continue
          soln = copy.deepcopy(Z.solution)
          # swap
          tmp=soln[i]
          soln[i]=soln[j]
          soln[j]=tmp
          Zn = ZoningSolution(self.scenario, soln)
          Zn.evaluate()
          if Zn.dev_coverage < Zbest.dev_coverage:
            Zbest = Zn
            print("Iteration "+str(it)+' - coverage: '+ str(Zbest.dev_coverage), end='\r')
            improved = True
    self.scenario.zoning_sol = Zbest
    print()
    # extract convex envolope
    targets = np.array(list(map(list, self.scenario.targets)))
    hull = ConvexHull(targets)
    envelope = np.array(targets[hull.simplices]).tolist()
    
    return [Zbest, envelope]
    #print(Zbest.solution)
    #plot_zoning_sol(scenario, Zbest)


class ZoningSolution:
  #==================================
  # Mission scenario constructor
  #==================================
  def __init__(self, scenario=None, solution=None):
    self.scenario = scenario
    self.solution = solution
    self.zones = defaultdict(list)
    self.separators = []

    nb_targets = self.scenario.nb_targets
    nb_agents = self.scenario.nb_agents
    nb_separators = self.scenario.nb_agents + 1
    nb_wp = self.scenario.nb_wp
    (xb,yb) = self.scenario.base
    (xn,yn) = self.scenario.exit

    #=====================
    # evaluate solution
    #=====================
    lin_separators = []
    for j in range(0, nb_wp):
      if self.solution[j]==1:
        lin_separators.append( [(xb,yb), (self.scenario.sep_waypoints_x[j], self.scenario.sep_waypoints_y[j]), (xn,yn)] )

    for i in range(nb_separators):
      #for j in range(nb_wp):
      #  if self.zoning_sol.solution[j]==1:
      self.separators.append(smooth_line(lin_separators[i], s=0, k=2))
      #    break

    node_zone = np.zeros(nb_targets, dtype=int)
    for j in range(nb_targets):
      for i in range(1, nb_agents+1):
        zone_poly = self.separators[i-1] + self.separators[i][::-1]
        node = self.scenario.targets[j]
        point = Point(node)
        polygon = Polygon(zone_poly)
        if polygon.contains(point):
          node_zone[j] = i

    for k in range(1, nb_agents+1):
      #print('K:',k)
      self.zones[k] = []
    for i in range(nb_targets):
      self.zones[node_zone[i]].append(i)
    #print('init done!')


  def __str__(self):
    s = str(self.solution)
    s += "\n"
    return s


  def evaluate(self):

    #Z = self.zones
    goal_coverage = self.scenario.nb_targets/self.scenario.nb_agents
    goal_score = sum(self.scenario.priorities)/self.scenario.nb_agents
    #print("goal #targets/zone:", goal_coverage)

    self.dev_coverage = 0
    self.dev_score = 0
    #total_targets_covered = 0
    for key,z in self.zones.items():

      nb_targets_zone = len(z)
      #total_targets_covered += nb_targets_zone

      self.dev_coverage += abs( goal_coverage - nb_targets_zone )

      # zone 0 exists => targets not covered
      if key==0:
        self.dev_coverage += nb_targets_zone
        #return 1

      # if zone is empty, assign high deviation as a penalty
      if nb_targets_zone == 0:
        self.dev_coverage = self.scenario.nb_targets
        return 2
      #qi = len(z)
      #print(key,qi)
      
      #self.dev_score += abs( goal_score-)

    # if some task are out of coverage zones, assign high deviation as a penalty
    #print("nb targets=", self.scenario.nb_targets," | nb assigned=", total_targets_covered)
    #if total_targets_covered != self.scenario.nb_targets:
    #  #print("COVERAGE ISSUES - nb targets=", self.scenario.nb_targets," | nb assigned=", total_targets_covered)
    #  self.dev_coverage = self.scenario.nb_targets + 1
    #return dev_coverage





class RoutingAlgorithm:

  def __init__(self, scenario=None, solution=None):
    self.scenario = scenario


  def search(self):

    sep_wp = np.where(self.scenario.zoning_sol.solution==1)[0]
    wp_x = self.scenario.sep_waypoints_x
    wp_y = self.scenario.sep_waypoints_y
    targets = np.array(self.scenario.targets)

    routes = []
    routes_taskset = []
    sm_paths = []
    dropped_tasks = []

    for i in range(1, self.scenario.nb_agents+1):

      # get zone data
      zone_ids = self.scenario.zoning_sol.zones[i]
      zone = targets[zone_ids]
      #print('>>', zone_ids, list(zone),'\n')

      #========================================
      # GENERATE BASELINE PATH
      #========================================
      p1 = (wp_x[sep_wp[i-1]], wp_y[sep_wp[i-1]])
      p2 = (wp_x[sep_wp[i]], wp_y[sep_wp[i]])
      parts = 10
      cx, cy = np.linspace(p1[0], p2[0], parts+1)[1:-1], np.linspace(p1[1], p2[1], parts+1)[1:-1]
      route_wps = list(zip(cx,cy))
      
      min_dist = math.inf
      zone_middle = ( (wp_x[sep_wp[i-1]]+wp_x[sep_wp[i]])/2, (wp_y[sep_wp[i-1]]+wp_y[sep_wp[i]])/2 )
      for rwp in route_wps:
        #lin_path = np.array([self.scenario.base, zone_middle, self.scenario.exit])
        #sm_path = np.array(smooth_line(lin_path, nb_samples=7))
        #sm_paths.append(sm_path)
        dist = 0
        for tr_id in zone_ids:
          dist += calc_distance(targets[tr_id], rwp)#*self.scenario.priorities[tr_id]

        if dist < min_dist:
          min_dist = dist
          zone_middle = rwp
          #print('>>',zone_middle)

      #========================================
      # construct and smooth path
      #========================================
      #zone_middle = ( (wp_x[sep_wp[i-1]]+wp_x[sep_wp[i]])/2, (wp_y[sep_wp[i-1]]+wp_y[sep_wp[i]])/2 )
      lin_path = np.array([self.scenario.base, zone_middle, self.scenario.exit])
      sm_path = np.array(smooth_line(lin_path, nb_samples=7))
      sm_paths.append(sm_path)
      
      ## GENERATE ROUTE
      route = []
      route_tasks = []
      for k in range(len(sm_path)-1):

        max_score = -1
        max_tr = None
        #max_tr_id = -1
        best_dist = math.inf
        route.append(tuple(sm_path[k]))
        #route_tasks.append(k)

        for tr_id in zone_ids:

          [dist, inline] = distancePointLine(targets[tr_id], sm_path[k], sm_path[k+1])
          if not inline:
            continue

          # to ensure selecting tasks within proximity using zone width
          if dist > calc_distance( (wp_x[sep_wp[i-1]], wp_y[sep_wp[i-1]]) , (wp_x[sep_wp[i]], wp_y[sep_wp[i]]) )/2:
            continue

          score = self.scenario.priorities[tr_id]
          if score>max_score or score==max_score and dist<best_dist:
            max_score = score
            best_dist = dist
            max_tr = tuple(targets[tr_id])
            max_tr_id =  tr_id

        if max_tr==None:
          max_tr_id=-1
          continue
        if max_tr in route:
          max_tr_id=-1
          route.pop(-1)
          #route_tasks.pop(-1)
          continue

        route[-1] = max_tr
        route_tasks.append(max_tr_id)

        
        # fix route if angle is too narrow
        if len(route)>2:
          '''
          angle = calc_angle(route[-2],route[-3], route[-1])
          turn_radius = calc_distance_real(route[-3],route[-2]) + calc_distance_real(route[-2],route[-1])
          bank_angle = calc_phi(self.scenario.agents[i-1]['velocity'], turn_radius, self.scenario.agents[i-1]['gravity_acc'])
          if angle < bank_angle:
            #ax.text(route[-2][0], route[-2][1], '%.2f'%angle, size=12, zorder=1, color='r')
            dropped_tasks.append([route[-2], angle])
            route.pop(-2)
          '''
          [feasible, angle] = self.feasible_angle(route, self.scenario.agents[i-1], -2)
          if not feasible:
            dropped_tasks.append([route[-2], angle])
            route.pop(-2)

      routes.append(route)
      routes_taskset.append(route_tasks)


    smooth_routes = []
    for i in range(1, self.scenario.nb_agents+1):
      sm_path = sm_paths[i-1]
      route = routes[i-1]
      route.append(sm_path[-1])
      # smooth routes
      #if calc_distance(self.scenario.base, self.scenario.centre) > self.scenario.radius:
      smooth_route = smooth_line(route, s=0, k=2)
      smooth_routes.append(smooth_route)

    return [smooth_routes, routes, dropped_tasks, routes_taskset]


  # route is gievn as node IDs
  def feasible_angle(self, route, profile, i):

    angle = calc_angle(route[i],route[i-1], route[i+1])
    turn_radius = calc_distance(route[i-1],route[i]) + calc_distance(route[i],route[i+1])
    bank_angle = calc_phi(profile['velocity'], turn_radius, profile['gravity_acc'])
    if angle < bank_angle:
      #dropped_tasks.append([route[-2], angle])
      #route.pop(-2)
      return [False, angle]
    return [True, None]



  # evaluate route coverage
  def evaluate_route(self, routes):

    nb_targets = len(self.scenario.targets)
    assigned_tasks = np.zeros(nb_targets, dtype=int)
    #print(assigned_tasks)
    for i in range(self.scenario.nb_agents):
      profile = self.scenario.agents[i]
      A = profile['altitude']
      alpha = profile['range-angle']
      b = A*np.tan(alpha)

      # calculate distance between route (set of segments) and task
      route = routes[i]
      for j in range(nb_targets):
        for k in range(len(route)-1):
          if calc_distance_pl(self.scenario.targets[j], [route[k], route[k+1]] ) < b:
            #ax.scatter(targets[j][0], targets[j][1], color='k', marker='x', s=20)
            assigned_tasks[j] = 1
            break
    
    #=====================================
    #for i in range(nb_targets):
    #  print(i,assigned_tasks[i],end=' | ')
    #print()
    #print(sum(assigned_tasks),'/',nb_targets)
    
    acc_score = sum([self.scenario.priorities[i]*assigned_tasks[i] for i in range(nb_targets)])
    return [acc_score, assigned_tasks]
