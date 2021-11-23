#class for creation and running etc of sinusoidal signals node

import sys
import roslib
import rospy
import cv2
import numpy as np

def signal_publisher():
    t0 = rospy.get_time()
    curr_time = np.array([rospy.get_time()]) - t0
    #publish to /robot/joint-x-position_controller/command
    pi = np.pi
    joint_2 = pi/2 * np.sin(curr_time * pi/15)
    joint_3 = pi/2 * np.sin(curr_time * pi/20)
    joint4 = pi/2 * np.sin(curr_time * pi/18)


