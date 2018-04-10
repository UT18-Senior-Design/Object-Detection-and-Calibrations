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

LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16

EXPECTED_PACKET_TIME = 0.001327  # valid only in "the strongest return mode"
EXPECTED_SCAN_DURATION = 0.1
DISTANCE_RESOLUTION = 0.002
ROTATION_RESOLUTION = 0.01
ROTATION_MAX_UNITS = 36000


def init_velo_socket():
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.bind(('', PORT))

#data_buff = pd.DataFrame(columns=['x', 'y', 'z', 'distance'])

def calc(dis, azimuth, laser_id, timestamp):
    R = dis * DISTANCE_RESOLUTION
    omega = LASER_ANGLES[laser_id] * np.pi / 180.0
    alpha = azimuth / 100.0 * np.pi / 180.0
    X = R * np.cos(omega) * np.sin(alpha)
    Y = R * np.cos(omega) * np.cos(alpha)
    Z = R * np.sin(omega)
    return [X, Y, Z, R]

def get_pointcloud(soc):
    prev_time = datetime.now()
    data_buff = []
    count = 0
    timestamp = 0
    time_offset = 0

    while True:
        # get data from port
        data = soc.recv(1248)
        count += 1
        if count == 90:
            count = 0
            break
        #print('pcl size: ', len(data))
        # get all data except the last 2 bytes
        raw_data = data[:-2]
        for offset in xrange(0, 1200, 100):
            flag, azimuth = struct.unpack_from("<HH", raw_data, offset)
            #assert flag == 0xEEFF, hex(flag)
            for step in xrange(2):
                #seq_index += 1
                # azimuth += step
                # azimuth %= ROTATION_MAX_UNITS
                
                prev_azimuth = azimuth
                # H-distance (2mm step), B-reflectivity (0
                arr = struct.unpack_from('<' + "HB" * 16, data, offset + 4 + step * 48)
                for i in xrange(NUM_LASERS):
                    #time_offset = (55.296 * seq_index + 2.304 * i) / 1000000.0
                    if arr[i * 2] != 0:
                        x, y, z, dist = calc(arr[i * 2], azimuth, i, timestamp + time_offset)
                        if y > 0:
                            data_buff.append([x, y, z, dist])
                    

    return np.array(data_buff)


# Code to test point clouds: 
print('point cloud test')              
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('', PORT))
np.savetxt('pcl.csv', get_pointcloud(soc), delimiter=',')
soc.close()


