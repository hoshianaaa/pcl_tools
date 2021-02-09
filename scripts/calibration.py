
#!/usr/bin/env python
import numpy as np
import copy
import pandas as pd
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from scipy.spatial.transform import Rotation

def matplotlib_plt(X,X2,X3):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    #ax.scatter(X[:,0], X[:,1], X[:,2], c=[0,0,0])
    ax.scatter(X2[:,0], X2[:,1], X2[:,2], c=[0.1,1,1])
    ax.scatter(X3[:,0], X3[:,1], X3[:,2], c=[1,1,0.1])

    plt.show()

data = pd.read_csv('data.csv',header=0)

c = data.iloc[:,0:3].values
d = data.iloc[:,3:6].values
v = np.ones((len(c),1))
print(c)
print(d)
c_add = np.hstack((c,v))
print(c_add)
print(d)


(X, residuals, rank, s) = np.linalg.lstsq(c_add,d,rcond=-1)
R = X[0:3].T
t_ = X[3:4][0]
t = [t_[0],t_[1],t_[2]]
print("R:",R)
print("t:",t)

result = []
for i in c:
    a = np.dot(R,i) + t
    result.append(a)

#print(np.array(result))
matplotlib_plt(c,d,np.array(result))

error_sum = 0
for i in range(len(d)):
  a = (np.dot(R,c[i]) + t)
  b = (d[i])
  u = b - a
  e = np.linalg.norm(u)
  error_sum = error_sum + e

error_mean =  error_sum / len(d)
print("ERROR:",error_mean)

rot = Rotation.from_dcm(R)
quat = rot.as_quat()
euler = rot.as_euler('XYZ')
print(quat)
print(t)
frame1 = "base_link"
frame2 = "camera_depth_optical_frame"
print("rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms")

print("rosrun tf static_transform_publisher " + str(t[0]) + " " + str(t[1]) + " " + str(t[2]) + " " + str(quat[0]) + " " + str(quat[1]) + " " + str(quat[2]) + " " + str(quat[3]) + " " + frame1 + " " + frame2 + " 100") 

print("rosrun tf static_transform_publisher " + str(t[0]) + " " + str(t[1]) + " " + str(t[2]) + " " + str(euler[0]) + " " + str(euler[1]) + " " + str(euler[2]) + " " + frame1 + " " + frame2 + " 100") 
