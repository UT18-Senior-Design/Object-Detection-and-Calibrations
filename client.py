#!/usr/bin/env python

from __future__ import print_function
import socket
import struct
import binascii
import array
import numpy as np
import time
#import cv2 

TCP_IP = '127.0.0.1'
TCP_PORT = 8080
BUFFER_SIZE = 816

def init_yolo_socket():
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))

def get_bounding_boxes(s):
	objects = []
	while 1:
		data = s.recv(BUFFER_SIZE)
		#print('yolo size: ', len(data))
		st = struct.Struct('=IQi200i')
		try:
			packet = st.unpack(data)
		except:
			continue
		#print("Start")
		#print('received packet #', packet[0])
		timestamp = packet[1]
		numObjects = packet[2]
		#print('Timestamp: ', timestamp, 'NumObjects: ', numObjects)
		for i in range(numObjects):
			index = 4*i
			left = packet[index+3]
			right = packet[index+4]
			top = packet[index+5]
			bottom = packet[index+6]
			objects.append([left,right,top,bottom])
			#print(left, right, top, bottom)
		#print("End")
		break
	return timestamp, numObjects, objects


def close_yolo_socket():
	s.close()

if __name__ == "__main__":
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	timestamps = []
	while 1:
		time.sleep(0.1)
		timestamp, numObjects, objects = get_bounding_boxes(s)
		#timestamps.append(timestamp)
	# diff = []
	# for i in range(len(timestamps)-1):
	# 	time_diff = timestamps[i+1] - timestamps[i]
	# 	print(time_diff)
	# 	diff.append(time_diff)
