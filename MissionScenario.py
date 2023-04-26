import numpy as np
import math
import copy
import itertools
import requests
import random

from scipy import interpolate
from sklearn.cluster import KMeans
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from collections import defaultdict

from utils import *
from MissionPlan import MissionPlan, MissionPath
from ZoningProblem import ZoningSolution, ZoningAlgorithm


#==================================
# Mission scenario
#==================================
class MissionScenario:

  #================================================================================
  # Mission scenario constructor
  #================================================================================
  def __init__(self, targets=None, base=None, exit=None, nb_agents=None, agent_speed=None, 
               agent_speed_in_zone=None, priorities=None, agents=None, heatmap=None):

    self.heatmap = heatmap

    self.base = base
    self.exit = exit

    self.targets = targets # array of tuples
    self.priorities = priorities

    self.agents = agents
    self.nb_agents = len(agents)

    self.nb_targets = len(self.targets)
    self.nb_clusters = self.nb_agents

    # deduce overall centroid
    self.targets_x = [v[0] for v in self.targets]
    self.targets_y = [v[1] for v in self.targets]
    self.centroid = (np.sum(self.targets_x)/self.nb_targets, np.sum(self.targets_y)/self.nb_targets)

    # calculate internal and external radius
    self.calc_threat_perimeter()

    # generate waypoints to separate zones
    self.sample_separator_waypoints()

    # put targets in clusters
    self.init_zones()


  #================================================================================
  # string simplified representation of a scenario (scenario summary)
  #================================================================================
  def __str__(self):
    s = ''
    s += '# targets:' + str(self.nb_targets) + '\n'
    s += '# agents:' + str(self.nb_agents) + '\n'
    s += 'Agent speed:' + str(self.agent_speed) + '\n'
    return s


  #================================================================================
  # load a mission scenario from (json) dictionary
  #================================================================================
  @classmethod
  def load(cls, jsonresp):
    start_location = ()
    exit_location = ()

    target_locations = []
    priorities = []

    agents = None

    for key, val in jsonresp.items():
      if key=='start_location':
        start_location = tuple(val)
      elif key=='exit_location':
        exit_location = tuple(val)
      elif key=='targets':
        targets = val
      elif key=='agents':
        agents = val

    for t in targets:
      target_locations.append(tuple(t['location']))

    for t in targets:
      priorities.append(t['priority'])

    nb_agents = len(agents)

    # create load
    scenario = cls(targets=target_locations, base=start_location, exit=exit_location, agents=agents, priorities=priorities)
    return scenario


  #================================================================================
  # Generate mission scenario
  #================================================================================
  @staticmethod
  def generate(dist_base_centre=140, dist_exit_centre=100, nb_agents=4, capacity_ratio=0.5, nb_clusters = 5):

    # Generate test map
    map_dim = (50, 50)
    nb_clusters = nb_clusters
    x_set,y_set=[],[]
    for i in range(nb_clusters):
      mu = np.random.uniform(10,50)
      sigma = np.random.uniform(3,10)
      x_set.append(np.random.normal(mu, sigma, 1000) )
      mu = np.random.uniform(10,50)
      sigma = np.random.uniform(3,10)
      y_set.append(np.random.normal(mu, sigma, 1000) )

    x = np.concatenate(x_set)
    y = np.concatenate(y_set)

    heatmap, xedges, yedges = np.histogram2d(x, y, bins=map_dim)

    data_mean = np.mean(heatmap[heatmap>0])

    targets = np.zeros(map_dim, dtype=int)
    for i in range(1,targets.shape[0]-1):
      for j in range(1,targets.shape[1]-1):
        if (heatmap[i,j]>data_mean/2 and np.random.rand()<.5):
          i1 = i + round(np.random.uniform(-1,1))
          j1 = j + round(np.random.uniform(-1,1))
          targets[i1,j1] = heatmap[i1,j1]
        #if (targets[i-1,j]>0 or targets[i+1,j]>0 or targets[i,j-1]>0 or targets[i,j+1]>0 or 
        #    targets[i-1,j-1]>0 or targets[i+1,j-1]>0 or targets[i-1,j+1]>0 or targets[i+1,j+1]>0 ):
        #  targets[i,j] = 0

    # expand map:
    #heatmap = fill_map(heatmap)
    #targets = fill_map(targets)

    # generate targets
    x, y, p = [],[],[]
    tar2 = []
    for i in range(0, targets.shape[0]):
      for j in range(0, targets.shape[1]):
        if (targets[i,j] > 0):

          #print(i,j, '->', targets[i,j])
          delta1 = round(np.random.uniform(-500,500))
          delta2 = round(np.random.uniform(-500,500))
          [xcoord, ycoord] = [i*1000+10000+delta1, 100000+j*1000+delta2]
          tar2.append((xcoord,ycoord))
          x.append(xcoord)
          y.append(ycoord)
          # priorities
          p.append(targets[i,j])
    nb_targets = len(x)



    #=======================
    # calculate centroid
    #=======================
    centroid = (np.sum(x)/nb_targets, np.sum(y)/nb_targets)

    #=======================
    # base station
    #=======================
    (xb, yb) = (20, centroid[1] - dist_base_centre)

    #=======================
    # exit station
    #=======================
    (xn, yn) = (80000, centroid[1] + dist_exit_centre)

    # agent capacity (max time for executing tasks)
    #agent_capacity = (np.sum(d)/nb_agents)*capacity_ratio

    #=======================
    # ageb=t profiles
    #=======================
    agent_profiles = {
      'EF-typhoon':{
        'name': 'typhoon',
        'gravity_acc': 9.81*12,
        'velocity': 693.0556,
        'range-angle': 0.35,
        'altitude': 7000
      },
      'UAV-X':{
        'name': 'UX',
        'gravity_acc': 9.81*10,
        'velocity': 1000,
        'range-angle': 0.35,
        'altitude': 5500
      },
      'Drone-Z':{
        'name': 'DZ',
        'gravity_acc': 9.81*4,
        'velocity': 150,
        'range-angle': 0.7,
        'altitude': 3500
      },
      'UAV-Y':{
        'name': 'G3',
        'gravity_acc': 9.81*4,
        'velocity': 450,
        'range-angle': 0.7,
        'altitude': 2500
      },
    }

    deployed_profiles = []
    for i in range(1, nb_agents+1):
      deployed_profiles.append(random.choice(list(agent_profiles.values())))

    scenario = MissionScenario(targets=tar2, base=(xb,yb), exit=(xn,yn), priorities=p, agents=deployed_profiles, heatmap=heatmap)

    return scenario



  #================================================================================
  # calculate safety circle parameters
  # calculate internal radius
  #================================================================================
  def calc_threat_perimeter(self):
    (xe,ye) = self.centroid
    (xb,yb) = self.base
    (xn,yn) = self.exit

    # cluster and safety radius
    self.radius_cluster = max( [ calc_distance( self.centroid, (self.targets_x[k], self.targets_y[k]) ) for k in range(self.nb_targets) ] )
    self.radius_internal = self.radius_cluster + self.radius_cluster*.01

    # calculate reference angle theta: (0,centroid,starting
    xsign = -1 if xb>xe else 1
    ysign = -1 if yb>ye else 1
    self.theta = calc_angle( self.centroid, (xe-self.radius_internal,ye), self.base )
    #self.theta = math.pi-self.theta if self.theta<0 else self.theta

    # zone centre
    #A = self.centroid[0]+self.radius_internal*math.cos(math.pi+xsign*self.theta)
    #B = self.centroid[1]+self.radius_internal*math.sin(math.pi+ysign*self.theta)
    self.centre = self.centroid#( (A + self.centroid[0])/2.0, (B + self.centroid[1])/2.0 )
    self.radius = self.radius_internal# + self.radius_internal/2.0

    # top and bottom tangents (starting point)
    F = self.centroid[0]-self.radius_internal*math.cos(math.pi+xsign*self.theta)
    G = self.centroid[1]-self.radius_internal*math.sin(math.pi+ysign*self.theta)
    self.top_tg = (F, G)
    
    F = self.centroid[0]+self.radius_internal*math.cos(math.pi+xsign*self.theta)
    G = self.centroid[1]+self.radius_internal*math.sin(math.pi+ysign*self.theta)
    self.bottom_tg = (F, G)


    # egress point reference angle
    xsign = -1 if xn>xe else 1
    ysign = -1 if yn>ye else 1
    self.beta = calc_angle( self.centroid, (xe-self.radius_internal,ye), self.exit )
    #self.beta = math.pi-self.beta if self.beta<0 else self.beta

    # top and bottom tangents (ending point)
    F = self.centroid[0]-self.radius_internal*math.cos(math.pi+xsign*self.beta)
    G = self.centroid[1]-self.radius_internal*math.sin(math.pi+ysign*self.beta)
    self.top_tg_exit = (F, G)

    F = self.centroid[0]+self.radius_internal*math.cos(math.pi+xsign*self.beta)
    G = self.centroid[1]+self.radius_internal*math.sin(math.pi+ysign*self.beta)
    self.bottom_tg_exit = (F, G)

    #print('cc',self.centroid,self.centre)
    #print('rr',self.radius_internal)



  #================================================================================
  # create potential zone separator positions
  #================================================================================
  def sample_separator_waypoints(self, nb_wp = 30):
    (xe,ye) = self.centroid
    (xb,yb) = self.base
    (xn,yn) = self.exit

    sign_theta = -1 if yb>ye else 1
    sign_beta = -1 if yn>ye else 1
    #beta = math.pi-self.beta if self.beta<0 else self.beta
    #theta = math.pi-self.theta if self.theta<0 else self.theta
    self.alpha = abs(sign_beta*self.beta - sign_theta*self.theta)/2 + min([sign_beta*self.beta, sign_theta*self.theta])
    #self.alpha = abs(beta - theta)/2 + min([beta, theta])

    F = self.centroid[0]+self.radius_internal*math.cos(math.pi+self.alpha)
    G = self.centroid[1]+self.radius_internal*math.sin(math.pi+self.alpha)
    self.segment_A = (F, G)
    F = self.centroid[0]+self.radius_internal*math.cos(math.pi+self.alpha+math.pi)
    G = self.centroid[1]+self.radius_internal*math.sin(math.pi+self.alpha+math.pi)
    self.segment_B = (F, G)

    # sample from segment [A,B]
    self.sep_waypoints_x = []
    self.sep_waypoints_y = []

    # calculate separator extermeties based on distance from centroid to [Star,Finish] segment:
    dist_cs = calc_distance_pl(self.centroid, [self.base,self.exit])

    #@if dist_cs > 0:
    sep_max = 1
    sep_min = -1-np.log(1+dist_cs/self.radius)
    #else:
    #  self.sep_max = -dist_cs
    #print("DIST:",dist_cs/self.radius)
    #print("min-max:",self.sep_min,'|', self.sep_max)
    #nb_wp = 20
    sep_step = (sep_max-sep_min)/nb_wp

    for i in np.arange(sep_min, sep_max+0.001, sep_step):
      F = self.centroid[0]+self.radius_internal*math.cos(math.pi+self.alpha)*i
      G = self.centroid[1]+self.radius_internal*math.sin(math.pi+self.alpha)*i
      self.sep_waypoints_x.append(F)
      self.sep_waypoints_y.append(G)


  #================================================================================
  # crate target clusters using K-means based on Euclidean distance
  #================================================================================
  def init_zones(self):
    
    (xb,yb) = self.base
    (xn,yn) = self.exit

    self.nb_wp = len(self.sep_waypoints_x)

    nb_separators = self.nb_agents + 1

    step = math.floor(self.nb_wp/self.nb_agents)
    #print('step:',step, '|', (nb_wp/self.nb_agents) )
    #self.sep_allocation = np.zeros([nb_separators, nb_wp], dtype=int)
    self.zone_alloc = np.zeros(self.nb_wp, dtype=int)
    for j in range(0, math.ceil(self.nb_wp/2), step):
      #print(j, self.zone_alloc)
      if np.sum(self.zone_alloc)>=nb_separators:
        break
      self.zone_alloc[j] = 1
      if np.sum(self.zone_alloc)>=nb_separators:
        break
      self.zone_alloc[-j-1] = 1

    self.zoning_sol = ZoningSolution(self, self.zone_alloc)



