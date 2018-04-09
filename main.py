import numpy as np
import math 
import pandas as pd
import matplotlib.pyplot as plt
import datetime
import scipy
import time
import socket
import datetime

from client import init_yolo_socket, get_bounding_boxes
from velodyne_capture_v2 import init_velo_socket, get_pointcloud
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
from collisionNew import Car, collision_detection

'''
code to get lidar point cloud, get bounding boxes for that frame, 
and predict a collision using Kalman filter (in progress)
	
'''

# Initialization info
car_dim = {
        "length": 4.5,
        "width": 2.1
        }

fake_obj = {
        "length": 4.5,
        "width": 2.1,
        "x": 22.0,
        "y": 15.0,
        "angle": 75,
        "speed": 6.0
        }

# Set up sockets:
HOST = "192.168.1.201"
PORT = 2368
TCP_IP = '127.0.0.1'
TCP_PORT = 8080
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('', PORT))

# For matrix values
xr = 95 * math.pi/180
yr = 10 * math.pi/180  
zr = 0 * math.pi/180

# Get R matrix
Xr = np.matrix([[1,0,0],[0,math.cos(xr),-1*math.sin(xr)],[0,math.sin(xr),math.cos(xr)]])
Yr = np.matrix([[math.cos(yr),0,math.sin(yr)],[0,1,0],[-1*math.sin(yr),0,math.cos(yr)]])
Zr = np.matrix([[math.cos(zr),-1*math.sin(zr),0],[math.sin(zr),math.cos(zr),0],[0,0,1]])

F = np.matrix([[935,0,0],[0,935,0],[225,375,1]])

R = np.matmul(Zr,Yr)
R= np.matmul(R,Xr)

# Transpose matrix?
T = np.matrix([[0.9],[0],[-1.32]])
T1=np.matrix.transpose(T)

# Initialize previous x,y,t
prev_x = 0
prev_y = 0
prev_t = 0

# Initialize car objects: 
# Car location and dimensions (CHANGE)
my_car = Car(0,0,car_dim["width"],car_dim["length"],0)
# Detected object (CHANGE)
other_car = Car(0,0,fake_obj["width"],fake_obj["length"],0)

# Get x, y, z, and distance data from point cloud
while 1:
	start = datetime.datetime.now()
	pcl = get_pointcloud(soc)
	elapsed = (datetime.datetime.now() - start).microseconds
	# print('pcl time: ', elapsed)
	X= pcl[:,0]
	Y= pcl[:,1]
	Z= pcl[:,2]
	distance = pcl[:,3]

	# GET BOUNDING BOX DATA HERE:
	#while 1:
	start = datetime.datetime.now()
	timestamp, numObjects, objects = get_bounding_boxes(s)
	print(timestamp, numObjects, objects)
	start = datetime.datetime.now()

	# make A matrix (x y z)
	size= len(X)
	X1= np.matrix.transpose(X)
	Y1= np.matrix.transpose(Y)
	Z1= np.matrix.transpose(Z)
	A= np.matrix([X1, Y1 ,Z1])
	print(A.shape)
	T2= np.repeat(T1,size,axis=0)
	T2= np.matrix.transpose(T2)

	# Multiply matrices (lidar points in pixel coordinates)
	c2 = np.matmul((F), (R))
	c2 = .25*np.matmul((c2),(A+T2))



	#print("matrix calculations: ", (datetime.datetime.now()-start).microseconds)
	#print('objects', numObjects)
	#print(datetime.datetime.now())

	for i in range(numObjects):
		left = objects[i][0]
		right =  objects[i][1]
		top = objects[i][2]
		bottom =  objects[i][3]

		# Center of box
		xcenter = (left+right)/2.0
		ycenter = (top+bottom)/2.0

		# c3 = c2
		# Bounding box
		start = datetime.datetime.now()

		B = np.square((c2[0,:]-xcenter))+ np.square((c2[1,:]-ycenter))

		# Get lidar points in bounding box
		#points = []

		#points = [[X[i], Y[i], distance[i]] for i in range(c2_T.shape[0]) if (c2_T[i,0] > left and c2_T[i,0] < right and c2_T[i,1] > top and c2_T[i,1] < bottom)]
		# for i in range(c2_T.shape[0]):
		# 	if c2_T[i,0] > left and c2_T[i,0] < right and c2_T[i,1] > top and c2_T[i,1] < bottom:
		# 		points.append([X[i], Y[i], distance[i]])
		# elapsed = (datetime.datetime.now() - start).microseconds
		# print(elapsed/1000)
		# print(len(points)) 

		# Get index of lidar point for detected object
		index0 = int(np.argmin(B, axis=1))
		

		#print('y', Y[index0])
		#print("Index of center point is:", index0)

		# printing x,y, and distance for detected objects
		print('x:{:.2f} y:{:.2f} distance: {:.2f}'.format(X[index0], Y[index0], distance[index0]));

		# Get inputs ready for prediction: [x,y,vx,vy,dt]
		x = X[index0]
		y = Y[index0]
		t = time.time()

		# Account for first instance:
		if (prev_t == 0):
			dt = 1
			vx = 0
			vy = 0
		# Else, update:
		else:	
			dt = t - prev_t
			vx = (x - prev_x)/(t_in - prev_t)
			vy = (y_- prev_x)/(t)
			prev_x = x
			prev_y = y
			prev_t = t

		# Code from collisionNew.py:
		other_car.update_locarray([x, y, vx, vy, dt])
		#print('distance: {:.2f}'.format(distance[index0]))  

		my_car.update_speed()
		other_car.update_object()

		if(collision_detection(my_car,other_car)):
			print('ALERT!!!')
		    # alert()
		                                
	print(' ')
