import socket
import math
import time
import datetime
from datetime import datetime
import numpy as np
import pandas as pd
import os
import struct

HOST = "192.168.1.201"
PORT = 2368

vertical_angle = [-15, 1, -13, -3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
count = 0
count1 = 0
initial_angle = 0

def init_velo_socket():
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.bind(('', PORT))

#data_buff = pd.DataFrame(columns=['x', 'y', 'z', 'distance'])
def get_pointcloud(soc):
    prev_time = datetime.now()
    data_buff = []
    count = 0

    while True:
        # get data from port
        data = soc.recv(1248)
        #print('pcl size: ', len(data))
        # get all data except the last 2 bytes
        raw_data = data[:-2]
        count += 1
        #print('Avg for one packet in microseconds: ', (datetime.now()-prev_time).microseconds/count)
        if(count==90):
            #print('Time for one pcl in microseconds: ', (datetime.now()-prev_time).microseconds)
            prev_time = datetime.now()

            #data_buff =  pd.DataFrame(columns=['x', 'y', 'z', 'distance'])
            #data_buff = []
            #reset counter
            count = 0
            break


        # for each of the 12 data blocks: 
        for k in range(12):

            # kth block:
            offset = k * 100
            # print('Block #: {}'.format(k))

            '''
            Get azimuth data (see data packet sheet)
            Get values, reverse bytes, combine, converte to decimal, divide by 100
            '''
            azimuth = raw_data[2+offset] | (raw_data[3+offset] << 8)
            azimuth = azimuth / 100
            alpha = azimuth * np.pi / 180.0
            # print(azimuth)

            for i in range(2):
                for j in range(16):
                    distance = raw_data[4+j*3+i*48+offset] | (raw_data[5+j*3+offset] << 8)
                    distance = distance / 500
                    reflectivity = data[6+j*3+offset]

                    # how to get alpha angle?
                    omega = vertical_angle[j] * np.pi / 180.0
                    x = distance*math.cos(omega)*math.sin(alpha)
                    y = distance*math.cos(omega)*math.cos(alpha)
                    z = distance*math.sin(omega)

                    # limit fov 
                    #if(45 <= azimuth <= 135):
                        # print('angle1: %f\tangle2: %f\t x: %f\ty: %f\tz: %f\tdistance: %f\t' % (azimuth, azimuth2, x, y, z, distance))
                   # data_buff.loc[count1] = [x, y, z, distance]
                    data_buff.append([x,y,z,distance])
                    

    return np.array(data_buff)


# Code to test point clouds: 
print('point cloud test')              
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('', PORT))
np.savetxt('pcl.csv', get_pointcloud(soc), delimiter=',')
soc.close()


