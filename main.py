import numpy as np
import math 
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import scipy
from mpl_toolkits.mplot3d import Axes3D
import time
import socket
import datetime
from client import init_yolo_socket, get_bounding_boxes
from velodyne_capture_v2 import init_velo_socket, get_pointcloud
import datetime

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

#random initial stuff
fake_obj = {
        "length": 4.5,
        "width": 2.1,
        "x": 22.0,
        "y": 15.0,
        "angle": 75,
        "speed": 6.0
        }

# set up sockets
HOST = "192.168.1.201"
PORT = 2368
TCP_IP = '127.0.0.1'
TCP_PORT = 8080
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('', PORT))


# For matrix values
xr = 1.58
yr = -1.42 #-1*math.pi/2.0    
zr = 1.58

# Get R matrix
Xr = np.matrix([[1,0,0],[0,math.cos(xr),-1*math.sin(xr)],[0,math.sin(xr),math.cos(xr)]])
Yr = np.matrix([[math.cos(yr),0,math.sin(yr)],[0,1,0],[-1*math.sin(yr),0,math.cos(yr)]])
Zr = np.matrix([[math.cos(zr),-1*math.sin(zr),0],[math.sin(zr),math.cos(zr),0],[0,0,1]])
R = np.matmul(Zr,Yr,Xr)

# transpose matrix?
T = np.matrix([[-1],[3],[.7]])
T1=np.matrix.transpose(T)

# initialize previous x,y,t
prev_x = 0
prev_y = 0
prev_t = 0

# Initialize car objects: 
# Car location and dimensions (CHANGE)
my_car = Car(0,0,car_dim["width"],car_dim["length"],0)
# Detected object (CHANGE)
other_car = Car(0,0,fake_obj["width"],fake_obj["length"],0)

# get x, y, z, and distance data from dataframe 
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
	T2= np.repeat(T1,size,axis=0)
	T2= np.matrix.transpose(T2)

	# multiply matrices
	c2 = 100*np.matmul((R),(A-T2))

	#print("matrix calculations: ", (datetime.datetime.now()-start).microseconds)
	#print('objects', numObjects)
	# get center of bounding box
	#print(datetime.datetime.now())

	for i in range(numObjects):
		# Center of box
		xcenter= (objects[i][0]+objects[i][1])/2.0
		ycenter= (objects[i][2]+objects[i][3])/2.0
		c3= c2
		# Bounding box
		B= np.square((c3[0,:]-xcenter))+ np.square((c3[1,:]-ycenter))
		# Get index of lidar point for detected object
		index0= int(np.argmin(B, axis=1))

		#print('y', Y[index0])
		#print("Index of center point is:", index0)

		# printing x,y, and distance for detected objects
		# print('x:{:.2f} y:{:.2f} distance: {:.2f}'.format(X[index0], Y[index0], distance[index0]));


		# Get inputs ready for prediction: [x,y,vx,vy,dt]
		x = X[index0]
		y = Y[index0]
		t = time.time()

		# account for first instance:
		if (prev_t == 0):
			dt = 1
			vx = 0
			vy = 0
		else:	
			dt = t - prev_t
			vx = (x - prev_x)/(t_in - prev_t)
			vy = (y_- prev_x)/(t)
			prev_x = x
			prev_y = y
			prev_t = t


		# CODE FROM collisionNew.py
		other_car.update_locarray([x, y, vx, vy, dt])
		print('distance: {:.2f}'.format(distance[index0]))
		# fig = plt.figure(1, figsize=(15, 8))
		# ax = fig.add_subplot(121)
		# ax.set_xlim(-40, 40)
		# ax.set_ylim(-40, 40)  


		my_car.update_speed()
		other_car.update_object()

		# ax.add_patch(PolygonPatch(my_car.get_contour(), fc='#990000', alpha=1))
		# ax.add_patch(PolygonPatch(my_car.get_path(), fc='#990000', alpha=.5))
		# ax.add_patch(PolygonPatch(other_car.get_contour(), fc='#000099', alpha=1))
		# ax.add_patch(PolygonPatch(other_car.get_path(), fc='#000099', alpha=.5))


		if(collision_detection(my_car,other_car)):
			print('ALERT!!!')
		    # alert()
		                                     
		# plt.show()


		#make sure we uncomment out the previous line with distance
	    #print(distance[int(index0)]) since the index is not just an int?
	print(' ')


	# For one object only (no loop)
	# For detection: 
	#	output: x,y, v_x, v_y (of detected object)
	# 	need previous x (m), y(m), and t (seconds) to calculate v's
	# 
	# For prediction: (collisionNew.py code)
	# 	input: x,y, v_x, v_y
	#	
