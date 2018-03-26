#!/usr/bin/env python

from __future__ import print_function
import socket
import struct
import binascii
import array
import cv2 

TCP_IP = '127.0.0.1'
TCP_PORT = 8080
BUFFER_SIZE = 816

def init_yolo_socket():
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))

def get_bounding_boxes():
	objects = []
	while 1:
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
		for i in range(numObjects):
			index = 4*i
			left = packet[index+3]
			right = packet[index+4]
			top = packet[index+5]
			bottom = packet[index+6]
			objects.append([left,right,top,bottom])
			print(left, right, top, bottom)
		print("End")
		break
	return timestamp, numObjects, objects


def close_yolo_socket():
	s.close()