#!/usr/bin/env python3

#This is a file
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class control:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)

        # subscriptions
        self.image_sub1 = rospy.Subscriber("joint_angle_1", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("joint_angle_3", Image, self.callback2)


        #publishers
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)


        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    #use methods implemented in vision_2 class to detect the position of robot end-effector
    def detect_end_effector(self, image):
        a = self.vision_2.pixel_to_meter(image)
        endPos = a * (self.vision_2.detect_green(image) - self.vision_2.detect_red(image))
        return endPos

    def forward_kinematics(self, image):
        joints = self.vision_2.detect_joint_angles(image)



