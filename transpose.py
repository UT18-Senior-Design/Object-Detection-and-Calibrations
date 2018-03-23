import numpy as np
import math 
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import scipy
from mpl_toolkits.mplot3d import Axes3D
import time
import socket


'''
code to get xy plane from 3d point cloud + image
inputs: 
	point cloud: 	csv/data data frame
	image: 			.jpg
	
'''

# set up yolo socket
TCP_PORT = 8080
BUFFER_SIZE = 816
MESSAGE = "Hello, World!"
 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

#R = np.linalg.inv(R)
#replace the name of this later
# GET LIDAR DATA HERE:
df = pd.read_csv("truck.csv")
print(df.shape)

df = df[df['Points_m_XYZ:0']>=0]
df = df[df['distance_m']<=30]

# get x, y, z, and distance data from dataframe 
X= df['Points_m_XYZ:0']
Y= df['Points_m_XYZ:1']
Z= df['Points_m_XYZ:2']
distance = df['distance_m']

# GET BOUNDING BOX DATA HERE: (hard coded for now)
data = s.recv(BUFFER_SIZE)
st = struct.Struct('=IQi200i')
try:
	packet = st.unpack(data)
except:
	continue
print("Start")
timestamp = packet[1]
numObjects = packet[2]
print('Timestamp: ', timestamp, 'NumObjects: ', numObjects)
objects = []
for i in range(numObjects):
	index = 4*i
	left = packet[index+3]
	right = packet[index+4]
	top = packet[index+5]
	bottom = packet[index+6]
	print(left, right, top, bottom)
	objects.append([left,right,top,bottom])
print("End")


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
# get image
img = plt.imread('single_truck.jpg')
# plot image
fig,ax = plt.subplots(1)
plt.xlim(0, 720)
plt.ylim(450,0)
ax.imshow(np.fliplr(img))

# make A matrix (x y z)
X1= X.values
size= len(X1)
X1= np.matrix.transpose(X1)
Y1= Y.values
Y1= np.matrix.transpose(Y1)
Z1= Z.values
Z1= np.matrix.transpose(Z1)
A=[X1,Y1,Z1]
A= np.matrix([X1,Y1 ,Z1])

# multiply matrices
T1=np.matrix.transpose(T)
T2= np.repeat(T1,size,axis=0)
T2= np.matrix.transpose(T2)
now= time.time()
c2 = 100*np.matmul((R),(A-T2))

# plot points?
for i in range(size):    
   circ = Circle((c2[0,i], c2[1,i]), .5, color='blue' )
   ax.add_patch(circ)

# get center of bounding box
xcenter= (x1+x2)/2.0
ycenter= (y1+y2)/2.0


c3= c2
B= np.square((c3[0,:]-xcenter))+ np.square((c3[1,:]-ycenter))

index0= np.argmin(B, axis=1)
print(index0)

# check time elapsed
runtime= time.time()- now
print("Matrix calculation runtime:" ,runtime)

circ = Circle((c2[0,index0], c2[1,index0]), 25, color='red' )
ax.add_patch(circ)
plt.plot([x1,x1],[y1,y2],color ='black')
plt.plot([x2,x2],[y1,y2], color ='black')
plt.plot([x1,x2],[y1,y1], color ='black')
plt.plot([x1,x2],[y2,y2],color ='black')
plt.show()