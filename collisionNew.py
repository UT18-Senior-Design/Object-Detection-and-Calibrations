#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 19 12:07:05 2018

@author:
"""

import shapely.geometry
import shapely.affinity
import matplotlib.pyplot as plt
from descartes import PolygonPatch
import math, random
# import winsound
import numpy as np
import os

delta_t = 3.0



#initialize GPS capture
"""
import socket
import pandas as pd
HOST = "192.168.1.201"
PORT = 8308
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('', PORT))
data = soc.recv(554)
data_string = data[206:270].decode('utf-8')
data_list = data_string.split(",")
"""


#Initialize Kalman stuff
delta_t = 1
F_ = np.array([[1, 0, delta_t, 0],[0, 1, 0, delta_t],[0, 0, 1, 0], [0, 0, 0, 1]])
R_laser = np.eye(4)*0.0225
H_laser = np.array([[1,0,0,0],[0,1,0,0]])
#P_ = np.array([[1,0,0,0],[0,1,0,0],[0,0,1000,0],[0,0,0,1000]])
Q_ = np.array([[0.25*R_laser[0,0]*delta_t**4, 0, 0.5*R_laser[0,0]*delta_t**3,0],
               [0,0.25*R_laser[1,1]*delta_t**4 ,0,0.5*R_laser[0,0]*delta_t**3],
               [0.5*R_laser[0,0]*delta_t**3,0,R_laser[0,0]*delta_t**2,0],
               [0, 0.5*R_laser[0,0]*delta_t**3, 0,R_laser[1,1]*delta_t**2 ]])


#convert knots to meters per second
def kn_to_ms(kn):
    return kn*0.514444
  
#gets data from object tracking module (evan's thing)  
def object_tracking(obj):
    obj.calc()
    return obj.x_

def update_F_and_Q(delta_t):
    F_[0,2] = delta_t
    Q_[1,3] = delta_t

#represents a 2D rotated rectangle, angle is from 0 to 360
class Rectangle:
    def __init__(self, center_x, center_y, width, length, angle):
        self.cx = center_x
        self.cy = center_y
        self.l = length
        self.w = width
        self.angle = angle #0 angle is front to the top
    def get_contour(self):
        w = self.w
        l = self.l
        c = shapely.geometry.box(-w/2.0, -l/2.0, w/2.0, l/2.0)
        rc = shapely.affinity.rotate(c, self.angle)
        return shapely.affinity.translate(rc, self.cx, self.cy)
    #returns intersect area between two rectangles
    def intersection(self, other):
        return self.get_contour().intersection(other.get_contour())
    #returns true if collision
    def collision(self, other):
        overlap = self.intersection(other)
        if (overlap.area > 0):
            return True
        else:
            return False

#extends rectangle
class Car(Rectangle):
    def __init__(self, center_x, center_y, width, length, angle):
        self.cx = center_x
        self.cy = center_y
        self.l = length
        self.w = width
        self.angle = angle
        #new variables
        self.cx_future = self.cx
        self.cy_future = self.cy
        self.speed = 0.0 #meters/second
        #Kalman variables
        self.x_ = np.array([0, 0, 1, 1])
        self.P_ = np.array([[1,0,0,0],[0,1,0,0],[0,0,1000,0],[0,0,0,1000]])
        self.results = np.empty([1,2])
        self.locarray = np.empty([1,2])
    def update_locarray(self, locarray):
        self.locarray = locarray
    def calc(self):  
        #Predict 
        self.x_ = F_.dot(self.x_)
        self.P_ = F_.dot(self.P_).dot(F_.T) + Q_
        # Kalman Update
        self.z = np.array([self.locarray[0],self.locarray[1], self.locarray[2], self.locarray[3]])
        self.y = self.z-self.x_
        self.S = self.P_ + R_laser
        self.K = self.P_.dot(np.linalg.inv(self.S))
        self.x_ = self.x_ + self.K.dot(self.y)
        self.P_ = (np.eye(len(self.P_))-self.K).dot(self.P_)
        self.results = np.append(self.results, [self.x_[0:2]], axis = 0)    
    #adjusts cx_future and cy_future as where the center of car will be after delta_t time
    def update_speed(self):
        #data = soc.recv(554)
        #data_string = data[206:270].decode('utf-8')
        #data_list = data_string.split(",")
        #df = pd.DataFrame(data_list)
        #self.speed = kn_to_ms(float(df.iloc[7][0]))
        
        #self.speed = float(random.randint(1,15))
        self.speed = 0
        
        #self.speed = math.sqrt((self.x_[2]*self.x_[2])+(self.x_[3]*self.x_[3]))
    def update_for_box(self):
        hyp = self.speed*delta_t
        changeiny = hyp*math.sin(math.radians(90.0-(-1.0*self.angle)))
        changeinx = hyp*math.cos(math.radians(90.0-(-1.0*self.angle)))
        self.cy_future = self.cy+changeiny
        self.cx_future = self.cx+changeinx
        return hyp
    #adjusts cx_future and cy_future as where the center of the car's path will be after delta_t time
    def update_for_path(self):
        hyp = self.speed*delta_t
        changeiny = (hyp/2)*math.sin(math.radians(90.0-(-1.0*self.angle)))
        changeinx = (hyp/2)*math.cos(math.radians(90.0-(-1.0*self.angle)))
        self.cy_future = self.cy+changeiny
        self.cx_future = self.cx+changeinx
        return hyp
    def get_contour_future(self):
        self.update_for_box()
        w = self.w
        l = self.l
        c = shapely.geometry.box(-w/2.0, -l/2.0, w/2.0, l/2.0)
        rc = shapely.affinity.rotate(c, self.angle)
        return shapely.affinity.translate(rc, self.cx_future, self.cy_future)
    def get_path(self):
        dist = self.update_for_path()
        w = self.w
        l = dist+self.l
        c = shapely.geometry.box(-w/2.0, -l/2.0, w/2.0, l/2.0)
        rc = shapely.affinity.rotate(c, self.angle)
        return shapely.affinity.translate(rc, self.cx_future, self.cy_future)
    def intersection_future(self, other):
        return self.get_path().intersection(other.get_path())
    def collision_future(self, other):
        overlap = self.intersection_future(other)
        if (overlap.area > 0):
            return True
        else:
            return False
    def update_object(self):
        array = object_tracking(self)
        self.cx = array[0]
        self.cy = array[1]
        vx = array[2]
        vy = array[3]
        
        if vx == 0 and vy == 0:
            angle = 0
        elif vy == 0:
            if (vx > 0):
                angle = -90.0
            elif (vx < 0):
                angle = 90.0
        elif vx == 0:
            if (vy > 0):
                angle = 0
            elif (vy < 0):
                angle = 180
        else:
            angle = math.degrees(math.atan(array[3]/array[2]))
            if vx>0 and vy>0: #quadrant 1 #pos/pos = pos
                angle = (90.0 - angle)*-1.0
            if vx<0 and vy>0: #quadrant 2 #pos/neg = neg
                angle = (90.0 + angle)
            if vx<0 and vy<0: #quadrant 3 #neg/neg = pos
                angle = angle+90
            if vx>0 and vy<0: #quadrant 4 #neg/pos = neg
                angle = (90 + angle) + 180            
        self.angle = angle
        self.speed = abs(math.sqrt((vx*vx)+(vy*vy)))
    


#random initial stuff
car_dim = {
        "length": 12.0,
        "width": 2.0
        }

#random initial stuff
fake_obj = {
        "length": 12.0,
        "width": 2.0,
        "x": 22.0,
        "y": 15.0,
        "angle": 75,
        "speed": 6.0
        }


#returns true if car and obj intersect
def collision_detection(car, obj):
    return car.collision_future(obj)

def alert_linx():
    duration = 1  # second
    freq = 440  # Hz
    os.system('play --no-show-progress --null --channels 1 synth %s sine %f' % (duration, freq))

def alert():
    print ("***CRASH DETECTED***")
    alert_linux()
    frequency = 600 # Set Frequency To 2500 Hertz
    duration = 750  # Set Duration To 1000 ms == 1 second
    # winsound.Beep(frequency, duration)
    # winsound.Beep(frequency, duration)

############## the "MAIN"####################
    

"""GET LOCARRAY INPUT"""
# locarray1 = np.empty([1,3])
# with open("obj_pose-laser-radar-synthetic-ukf-input1.txt","r") as f:
#     for line in f:
#         location = line.lower()
#         location = location.replace("\n","")
#         location = location.split("\t")
#         for n in range(len(location)):
#            location[n] = float(location[n])
#         locarray1 = np.append(locarray1, [location], axis = 0)
# plt.plot(locarray1[1:,0],locarray1[1:,1])
# plt.title("Input")
# plt.show()


# my_car = Car(0,0,car_dim["width"],car_dim["length"],0)
# other_car = Car(0,0,fake_obj["width"],fake_obj["length"],0)
# other_car.update_locarray(locarray)


# fig = plt.figure(1, figsize=(15, 8))
# ax = fig.add_subplot(121)
# ax.set_xlim(-40, 40)
# ax.set_ylim(-40, 40)  


# my_car.update_speed()
# other_car.update_object()

# ax.add_patch(PolygonPatch(my_car.get_contour(), fc='#990000', alpha=1))
# ax.add_patch(PolygonPatch(my_car.get_path(), fc='#990000', alpha=.5))
# ax.add_patch(PolygonPatch(other_car.get_contour(), fc='#000099', alpha=1))
# ax.add_patch(PolygonPatch(other_car.get_path(), fc='#000099', alpha=.5))


# if(collision_detection(my_car,other_car)):
#     alert()

                                     
# plt.show()



