#!/usr/bin/env python

import socket
import time

TCP_IP = '127.0.0.1'
TCP_PORT = 50001
BUFFER_SIZE = 1024
MESSAGE = "Hello, World!"
i=0
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
for i in range(60):
	s.send(str(i))	
	print "i=",i
	time.sleep(1)

s.close()
print "received data:", data
