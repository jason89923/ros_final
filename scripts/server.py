#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import socket
import time
import rospy 
from std_msgs.msg import String
from ros_final.srv import *

HOST = '127.0.0.1'
PORT = 7000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((HOST, PORT))
s.listen(5)
print('server start at: %s:%s' % (HOST, PORT))
print('wait for connection...')
state = ''

def Ros_driver_client(degree: int):
    rospy.wait_for_service('rosky_control')
    try:
        client = rospy.ServiceProxy('rosky_control', driving_control)
        request = driving_controlRequest(angle_degree=degree, velocity=0.17, mode='absolute')
        resp1 = client(request)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



while True:
    conn, addr = s.accept()
    print('connected by ' + str(addr))

    while True:
        message = conn.recv(1024)
        if len(message) == 0: # connection closed
            conn.close()
            print('client closed connection.')
            break
        if message.decode() == 'STOP':
            Ros_driver_client(0)
        else:
            Ros_driver_client(180)
        conn.send('ok'.encode())