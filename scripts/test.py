#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-import rospy
import rospy 
from ros_final.srv import *
import time

def Ros_driver_client(degree):
    rospy.wait_for_service('rosky_control')
    try:
        client = rospy.ServiceProxy('rosky_control', driving_control)
        request = driving_controlRequest(angle_degree=degree, velocity=0.9)
        resp1 = client(request)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        
if __name__ == '__main__':
    for i in range(20):
        #for i in [0, 90, 180, 270]:
        Ros_driver_client(170)
        print('done')
        time.sleep(1.5)