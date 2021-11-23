#!/usr/bin/env python3
#file for creation and running etc of sinusoidal signals node

import sys
import roslib
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64

def v1signal_publisher():

    #initialise node
    rospy.init_node('signal_publisher', anonymous=True)
    rate = rospy.Rate(50)

    #initialise publishers
    joint_2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    joint_3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    joint_4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    #get necessary values, constants
    t0 = rospy.get_time()

    while not rospy.is_shutdown():
        curr_time = np.array([rospy.get_time()]) - t0
        pi = np.pi
        #calculate joint positions
        joint_2 = pi/2 * np.sin(curr_time * pi/15)
        joint_3 = pi/2 * np.sin(curr_time * pi/20)
        joint_4 = pi/2 * np.sin(curr_time * pi/18)

        targetj2 = Float64()
        targetj3 = Float64()
        targetj4 = Float64()

        targetj2.data = joint_2
        targetj3.data = joint_3
        targetj4.data = joint_4

        joint_2_pub.publish(targetj2)
        joint_3_pub.publish(targetj3)
        joint_4_pub.publish(targetj4)
        rate.sleep()



#run code if node is called
if __name__ == '__main__':
    try:
        v1signal_publisher()
    except rospy.ROSInterruptException:
        pass



