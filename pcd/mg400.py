import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

L1 = 284 - (175 + 66)
L2 = 175
L3 = 175
L4 = 66

def ki(j1,j2,j3):
  if(check_joint_limit(j1,j2,j3) == False):
    raise Exception('out of range')

  j1_rad = math.pi * j1 / 180
  j2_rad = math.pi * j2 / 180
  j3_rad = math.pi * j3 / 180

  l =L1 +  L2 * math.sin(j2_rad) + L3 * math.cos(j3_rad) + L4
  z = L2 * math.cos(j2_rad) - L3 * math.sin(j3_rad)

  x = l * math.cos(j1_rad)
  y = l * math.sin(j1_rad)

  return (x,y,z)

def check_joint_limit(j1,j2,j3):

  d_j = j3 - j2

  if (j1 >= -160 and j1 <= 160):
    if (j2 >= -25 and j2 <= 85):
      if (j3 >= -25 and j3 <= 105):
        return True

  return False

def check_coordinate(x,y,z):
  if x < 150 and x > -150:
    if y < 150 and y > -150:
      return False

  return True 


fig = plt.figure()
ax = Axes3D(fig)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_xlim(-500,500)
ax.set_ylim(-500,500)
ax.set_zlim(-200,250)

points = []

for j1 in range(-180,180,10):
  for j2 in range(-180,180,10):
    for j3 in range(-180,180,10):
      try:
        x,y,z = ki(j1,j2,j3)
        if check_coordinate(x,y,z):
          ax.plot([x],[y],[z],marker=".",linestyle='None',color='blue',alpha=0.2)
          points.append([x/1000,y/1000,z/1000])
      except:
        pass

#plt.show()


### change to ply

import open3d as o3d
import numpy as np

xyz = np.array(points)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
o3d.io.write_point_cloud("mg400.ply", pcd)
