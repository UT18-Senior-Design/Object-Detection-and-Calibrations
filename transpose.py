import numpy as np
import math 
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import scipy
from mpl_toolkits.mplot3d import Axes3D
import time
from camera_capture import get_image
from velodyne_capture_v3 import init_velo_socket, get_pointcloud
import socket


def minmax_scale(x, i_min, i_max, o_min, o_max):
	return (x-i_min)/float(i_max-i_min)*(o_max-o_min)+o_min

PORT = 2368
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('', PORT))

pcl = get_pointcloud(soc)
X= pcl[:,0]
Y= pcl[:,1]
Z= pcl[:,2]
distance = pcl[:,3]
# filtered_dist = distance[(distance > 2.0) ]
# filtered_dist = filtered_dist[filtered_dist < 6.]
print(len(distance))
# print(len(filtered_dist))
# from resizeimage import resizeimage
#R = np.linalg.inv(R)
#replace the name of this later



# For matrix values
xr = 95 * math.pi/180
yr = 10 * math.pi/180  
zr = 0 * math.pi/180


# start z by 90 y by -90

Xr = np.matrix([[1,0,0],[0,math.cos(xr),-1*math.sin(xr)],[0,math.sin(xr),math.cos(xr)]])
Yr = np.matrix([[math.cos(yr),0,math.sin(yr)],[0,1,0],[-1*math.sin(yr),0,math.cos(yr)]])
Zr = np.matrix([[math.cos(zr),-1*math.sin(zr),0],[math.sin(zr),math.cos(zr),0],[0,0,1]])

F = np.matrix([[935,0,0],[0,935,0],[225,375,1]])

R = np.matmul(Zr,Yr)
R= np.matmul(R,Xr)

T = np.matrix([[1.1],[0],[-1.32]])
img = get_image()
#img= resizeimage.resize_height(img,450, validate=True)
fig,ax = plt.subplots(1)
plt.xlim(0, img.shape[1])
plt.ylim(img.shape[0],0)
ax.imshow(img)

now = time.time()

size= len(X)
X1= np.matrix.transpose(X)
Y1= np.matrix.transpose(Y)
Z1= np.matrix.transpose(Z)
A=[X1,Y1,Z1]
A= np.matrix([X1,Y1 ,Z1])

T1=np.matrix.transpose(T)
T2= np.repeat(T1,size,axis=0)

T2= np.matrix.transpose(T2)

c2 = np.matmul((F), (R))
c2 = .25*np.matmul((c2),(A+T2))

xcenter = 307
ycenter = 207
B = np.square((c2[0,:]-xcenter)) + np.square((c2[1,:]-ycenter))
index = int(np.argmin(B, axis=1))

cmap = plt.get_cmap('brg')
pointsDist = np.asarray(distance)
pointsDist = np.round(minmax_scale(1.0/pointsDist,1.0/75,1.0/1.5,1,255).astype('uint8'))
pointsColor = np.array([cmap(1.0-pdist/255.0)[:3] for pdist in pointsDist])
plt.scatter(np.asarray(c2[0,:]), np.asarray(c2[1,:]), s=0.5, c=pointsColor)
circ = Circle((c2[0,index],c2[1,index]), 5, color='red')
ax.add_patch(circ)
print(distance[index])
plt.show()