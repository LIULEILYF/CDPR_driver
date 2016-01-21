#!/usr/bin/env python

import socket
import time


TCP_IP = '127.0.0.1'
TCP_PORT = 50001
BUFFER_SIZE = 255
MESSAGE = "Hello, World!"
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))	
for i in range(60):
	s.send(str(i))	
	data = s.recv(BUFFER_SIZE)
	print "received data:", data
	time.sleep(0.2)
s.close()


