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
MESSAGE = "Hello, World!"
 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

# s.send(MESSAGE)
left = 0
right = 0
top = 0
bottom = 0
# tracker = cv2.TrackerMIL_create()
# video = cv2.VideoCapture(0)
while 1:
	data = s.recv(BUFFER_SIZE)
	# convert(data)
	#new_data = [hex(ord(i)) for i in data]
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
		print(left, right, top, bottom)
	print("End")

# ok,frame = video.read()
# bbox=(right, top,left, bottom)
# ok = tracker.init(frame,bbox)
# while 1:
# 	ok, frame = video.read()
# 	if not ok:
# 		break
# 	ok, bbox = tracker.update(frame)
# 	if ok:
# 		p1 = (int(bbox[0]), int(bbox[1]))
# 		p2 = (int(bbox[0]+bbox[2]), int(bbox[1]+bbox[3]))
# 		cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
# 	else:
# 		cv2.putText(frame, "tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

# 	cv2.imshow("Tracking", frame)


s.close()