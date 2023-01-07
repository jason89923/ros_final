#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import socket
import time
import rospy 
from std_msgs.msg import String
from ros_final.srv import *

HOST = '192.168.1.224'
PORT = 7000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((HOST, PORT))
s.listen(5)
print('server start at: %s:%s' % (HOST, PORT))
print('wait for connection...')
state = ''

def Ros_driver_client():
    rospy.wait_for_service('rosky_control')
    try:
        client = rospy.ServiceProxy('rosky_control', driving_control)
        request = driving_controlRequest(angle_degree=170, velocity=0.17)
        resp1 = client(request)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



while True:
    conn, addr = s.accept()
    print('connected by ' + str(addr))

    while True:
        indata = conn.recv(1024)
        if len(indata) == 0: # connection closed
            conn.close()
            print('client closed connection.')
            break
        print('recv: ' + indata.decode())
        if state != indata.decode() :
            state = indata.decode()
            Ros_driver_client()

        #return_info = 'spin complete'
        #return_info = 'echo: ' + return_info.decode()
        #conn.send(return_info.encode())