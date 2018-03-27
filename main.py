import numpy as np
import math 
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import scipy
from mpl_toolkits.mplot3d import Axes3D
import time
import socket
from client import init_yolo_socket, get_bounding_boxes
from velodyne_capture_v2 import init_velo_socket, get_pointcloud


'''
code to get xy plane from 3d point cloud + image
inputs: 
	point cloud: 	csv/data data frame
	image: 			.jpg
	
'''

# set up sockets
HOST = "192.168.1.201"
PORT = 2368
TCP_IP = '127.0.0.1'
TCP_PORT = 8080
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('', PORT))


# get x, y, z, and distance data from dataframe 
pcl = get_pointcloud(soc)
X= pcl[:,0]
Y= pcl[:,1]
Z= pcl[:,2]
distance = pcl[:,3]

# GET BOUNDING BOX DATA HERE: (hard coded for now)
numObjects = 0
objects = []
timestamp = 0
while numObjects < 1:
	timestamp, numObjects, objects = get_bounding_boxes(s)


xr = 1.58
yr = -1.42 #-1*math.pi/2.0    
zr = 1.58

# start z by 90 y by -90

# Get R matrix
Xr = np.matrix([[1,0,0],[0,math.cos(xr),-1*math.sin(xr)],[0,math.sin(xr),math.cos(xr)]])
Yr = np.matrix([[math.cos(yr),0,math.sin(yr)],[0,1,0],[-1*math.sin(yr),0,math.cos(yr)]])
Zr = np.matrix([[math.cos(zr),-1*math.sin(zr),0],[math.sin(zr),math.cos(zr),0],[0,0,1]])
R = np.matmul(Zr,Yr,Xr)

# transpose matrix?
T = np.matrix([[-1],[3],[.7]])

# make A matrix (x y z)
size= len(X)
X1= np.matrix.transpose(X)
Y1= np.matrix.transpose(Y)
Z1= np.matrix.transpose(Z)
A= np.matrix([X1, Y1 ,Z1])

# multiply matrices
T1=np.matrix.transpose(T)
T2= np.repeat(T1,size,axis=0)
T2= np.matrix.transpose(T2)
now= time.time()
c2 = 100*np.matmul((R),(A-T2))

# get center of bounding box
xcenter= (objects[0][0]+objects[0][1])/2.0
ycenter= (objects[0][2]+objects[0][3])/2.0


c3= c2
B= np.square((c3[0,:]-xcenter))+ np.square((c3[1,:]-ycenter))

index0= np.argmin(B, axis=1)
print(distance[index0])