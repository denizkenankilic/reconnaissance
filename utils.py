import numpy as np
import math
from scipy import interpolate

def fill_map(imap):
  map_dim = imap.shape
  omap = np.concatenate((np.zeros(map_dim, dtype=int), imap), axis=0)

  dim2 = (map_dim[0]*2, map_dim[1]*3)
  omap = np.concatenate((np.zeros(dim2, dtype=int), omap), axis=1)

  dim2 = (map_dim[0]*2, map_dim[1]*2)
  omap = np.concatenate((omap, np.zeros(dim2, dtype=int)), axis=1)

  return omap

def calc_distance(A, B):
  return np.linalg.norm( np.array(A) - np.array(B) )

#def calc_distance_real(A, B, scale_factor=1):
#  return scale_factor*calc_distance(A, B)

def calc_distance_pl(pt, line):
  A, B = np.array(line[0]), np.array(line[1])
  C = np.array(pt)
  # if falls out, => penalise
  dist = np.linalg.norm(np.cross(B-A, A-C))/np.linalg.norm(B-A)

  min_dist = min([calc_distance(A,C), calc_distance(B,C)])

  return max([min_dist, dist])


def calc_tangent( centre, radius, ref_point ):
  Cx, Cy = centre
  r = radius
  Px, Py =  ref_point

  dx, dy = Px-Cx, Py-Cy
  dxr, dyr = -dy, dx
  d = math.sqrt(dx*dx+dy*dy)
  if d < r :
    #print('Radius:',r, '| Distance:',d)
    return None
  rho = r/d
  ad = rho*rho
  #print(">>>>>",rho, ad)
  bd = rho*math.sqrt(1-rho*rho)
  T1x = Cx + ad*dx + bd*dxr
  T1y = Cy + ad*dy + bd*dyr
  T2x = Cx + ad*dx - bd*dxr
  T2y = Cy + ad*dy - bd*dyr

  return [(T1x,T1y), (T2x,T2y)]

def calc_intersection( L1,L2 ):
  (a,b) = L1
  (c,d) = L2
  x = (b-d)/(c-a)
  y = b+a*x
  return (x,y)

def calc_angle2(B, A, C):
  a = np.array(A)
  b = np.array(B)
  c = np.array(C)
  ba = a - b
  bc = c - b
  cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
  return np.arccos(cosine_angle)

# calculate the angle <BAC>
def calc_angle(A, B, C):
  a = calc_distance(B, C)
  b = calc_distance(A, C)
  c = calc_distance(A, B)
  if b==0.0 or c==0.0:
    #print('math error!')
    return -4.0

  cos_angle = (b*b+c*c-a*a)/(2*b*c)
  #print('cos:', cos_angle)
  if math.isnan(cos_angle):
    #print('NAN')
    return 3.0

  if cos_angle>1:
    #print(cos_angle,'to 1')
    cos_angle=1
  elif cos_angle<-1:
    #print(cos_angle,'to -1')
    cos_angle=-1

  theta = math.acos( cos_angle )
  return theta


def smooth_line(line, s=0, k=2, nb_samples=5):

  cx, cy = zip(*line)
  cx, cy = np.r_[cx], np.r_[cy]
  #print('CX', cx)
  #print('CY', cy)

  # create B-spline function
  f, u = interpolate.splprep([cx, cy], s=s, per=False, k=k)
  # create interpolated lists of points
  xint, yint = interpolate.splev(np.linspace(0, 1, len(line)*nb_samples), f)
  
  smooth_line = []
  for j in range(len(xint)):
    smooth_line.append( (xint[j], yint[j]) )

  return smooth_line




def lineMagnitude (x1, y1, x2, y2):
    return math.sqrt(math.pow((x2 - x1), 2)+ math.pow((y2 - y1), 2))

#Calc minimum distance from a point and a line segment (i.e. consecutive vertices in a polyline).
#def distancePointLine (px, py, x1, y1, x2, y2):
def distancePointLine (P, A, B):
  (px, py) = P
  (x1, y1) = A
  (x2, y2) = B
  
  #http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/source.vba
  LineMag = lineMagnitude(x1, y1, x2, y2)

  if LineMag < 0.00000001:
    distancePointLine = 9999
    return distancePointLine

  u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
  u = u1 / (LineMag * LineMag)

  in_line = True
  if (u < 0.00001) or (u > 1):
    #// closest point does not fall within the line segment, take the shorter distance
    #// to an endpoint
    ix = lineMagnitude(px, py, x1, y1)
    iy = lineMagnitude(px, py, x2, y2)
    in_line = False
    if ix > iy:
      distancePointLine = iy
    else:
      distancePointLine = ix
  else:
    # Intersecting point is on the line, use the formula
    ix = x1 + u * (x2 - x1)
    iy = y1 + u * (y2 - y1)
    distancePointLine = lineMagnitude(px, py, ix, iy)

  return [distancePointLine, in_line]


def calc_phi(vt, turn_radius, gravity_acc = 9.81):
  phi = np.arctan( (vt*vt)/(turn_radius*gravity_acc) )
  return phi
