from socket import *
import struct
import numpy as np
import cv2
import threading
from queue import Queue

q = Queue(maxsize=10)
 
s = socket(AF_INET,SOCK_DGRAM)
#s.setblocking(0)
s.bind(('',8000))

teensy_addr = ("169.254.1.1", 8888)

s.sendto(b'Hi!',teensy_addr)

def read_uint12(data_chunk):
	data = np.frombuffer(data_chunk, dtype=np.uint8)
	fst_uint8, mid_uint8, lst_uint8 = np.reshape(data, (data.shape[0] // 3, 3)).astype(np.uint16).T
	fst_uint12 = (fst_uint8 << 4) + (mid_uint8 >> 4)
	snd_uint12 = ((mid_uint8 % 16) << 8) + lst_uint8
	return np.reshape(np.concatenate((fst_uint12[:,None], snd_uint12[:,None]), axis=1), 2* fst_uint12.shape[0])

#while True: 
print("waiting to receive")

# data comes in broken into 4 chunks due to max rx packet size (mtu or something)


def make_frames():
	frame = np.zeros([365,365],dtype=np.uint16)
	frame_chunk = np.zeros([365,50],dtype=np.uint16)
	n = 0
	while True:
		for i in range(0,50):
			data1,client = s.recvfrom(1472)
			data2,client = s.recvfrom(1472)
			data3,client = s.recvfrom(1472)
			data4,client = s.recvfrom(1472)
			print(len(data1))
			print(len(data2))
			print(len(data3))
			print(len(data4))
			data = data1 + data2 + data3 + data4
			vals = read_uint12(data)
			frame_chunk[:,i] = 2**16 - 2**4*vals[::10] # downsample 10x and scale to 16 bits
			print(i)
		n += 1
		frame = np.roll(frame,50,axis=1)
		frame[:,:50] = np.flip(frame_chunk,axis=1)
		q.put(frame)

	
t1 = threading.Thread(target=make_frames)
t1.start()

while True:
	cv2.imshow('TCD1304',q.get())
	c = cv2.waitKey(1)
	if c ==ord('q'):
		break
	

