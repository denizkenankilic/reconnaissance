import numpy as np
from scipy import interpolate
import copy

#==================================
# Mission plan
#==================================
class MissionPlan:

  #================================================================================
  # Mission scenario constructor
  #================================================================================
  def __init__(self, paths=[]):
    self.paths = paths
    self.nb_paths = len(paths)

  #================================================================================
  # string representation
  #================================================================================
  def __str__(self):
    return ''


#==================================
# Mission path
#==================================
class MissionPath:

  #================================================================================
  # Mission scenario constructor
  #================================================================================
  def __init__(self, path=[], target_centroid=None, centroid_index=None, r_score=None, p_score=None, smooth_path=[]):
    self.path = path
    self.target_centroid = target_centroid
    self.centroid_index = centroid_index
    # TODO for later use
    self.r_score = r_score
    self.p_score = p_score
    self.smooth_path = smooth_path

  def setPScore(self, p_score):
    self.p_score = p_score

  def setRScore(self, r_score):
    self.r_score = r_score
    

  #================================================================================
  # string representation
  #================================================================================
  def __str__(self):
    s = str(self.target_centroid) + '->'
    for i in range(len(self.path)-1):
      s +=  str(self.path[i])+', '
    s += str(self.path[-1])

    return s


  def smooth(self):

    #arr = copy.deepcopy(self.path)
    arr = copy.deepcopy(self.path)

    #print('ARR:\n',arr)
    cx, cy = zip(*arr)
    cx = np.r_[cx]
    cy = np.r_[cy]
    #centroid_wp = ((cx[0]+cx[1])/2, (cy[0]+cy[1])/2)
    cx = np.insert(cx, 1, (cx[0]+cx[1])/2)
    cy = np.insert(cy, 1, (cy[0]+cy[1])/2)

    # create B-spline function
    #f, u = interpolate.splprep([cx[1:], cy[1:]], s=0, per=False, k=2)
    f, u = interpolate.splprep([cx, cy], s=0, per=False, k=2)
    # create interpolated lists of points
    xint, yint = interpolate.splev(np.linspace(0, 1, len(arr)*5), f)
    #print('SP:',self.paths[i].smooth_path)
    self.smooth_path = []
    self.smooth_path.append(self.path[0])
    for j in range(len(xint)):
      self.smooth_path.append( (xint[j], yint[j]) )
    #self.paths[i].smooth_path.append(self.exit)
