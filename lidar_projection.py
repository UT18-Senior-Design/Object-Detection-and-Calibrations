import numpy as np
import math 
import pandas as pd
import matplotlib.pyplot as plt
import datetime
import scipy
import time
import socket
import datetime
import cv2

from velodyne_capture_v3 import init_velo_socket, get_pointcloud
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle

PORT = 2368
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('', PORT))

# For matrix values
xr = 95 * math.pi/180
yr = 0 * math.pi/180  
zr = 0 * math.pi/180

# Get R matrix
Xr = np.matrix([[1,0,0],[0,math.cos(xr),-1*math.sin(xr)],[0,math.sin(xr),math.cos(xr)]])
Yr = np.matrix([[math.cos(yr),0,math.sin(yr)],[0,1,0],[-1*math.sin(yr),0,math.cos(yr)]])
Zr = np.matrix([[math.cos(zr),-1*math.sin(zr),0],[math.sin(zr),math.cos(zr),0],[0,0,1]])
R = np.matmul(Zr,Yr,Xr)

# Transpose matrix?
T = np.matrix([[-2.1],[0],[1.4]])
T1=np.matrix.transpose(T)

cap = cv2.VideoCapture(0)
# fig, ax = plt.subplots(1)
# plt.ion()
# plt.show()

if not cap.isOpened():
	raise IOError("cannot open webcam")

while 1:
	#pcl = get_pointcloud(soc)
	flag, frame = cap.read()
	cv2.imshow('frame', frame)	
	c = cv2.waitKey(1)
	if c == 27:
		break
	# X= pcl[:,0]
	# Y= pcl[:,1]
	# Z= pcl[:,2]
	# distance = pcl[:,3]
	# # make A matrix (x y z)
	# size= len(X)
	# X1= np.matrix.transpose(X)
	# Y1= np.matrix.transpose(Y)
	# Z1= np.matrix.transpose(Z)
	# A= np.matrix([X1, Y1 ,Z1])
	# T2= np.repeat(T1,size,axis=0)
	# T2= np.matrix.transpose(T2)

	# # Multiply matrices (lidar points in pixel coordinates)
	# c2 = 500*np.matmul((R),(A-T2))
	# plt.scatter(np.asarray(c2[0,:]), np.asarray(c2[1,:]), s=0.5, c='red')
cap.release()
cap.destroyAllWindows()