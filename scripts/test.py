#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-import rospy
import rospy 
from ros_final.srv import *
import time

def Ros_driver_client():
    rospy.wait_for_service('rosky_control')
    try:
        client = rospy.ServiceProxy('rosky_control', driving_control)
        request = driving_controlRequest(angle_degree=180, velocity=0.17)
        resp1 = client(request)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        
if __name__ == '__main__':
    for i in range(20):
        Ros_driver_client()
        time.sleep(1)