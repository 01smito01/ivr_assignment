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
        #calculate each entry for the array to return
        effector1 = 3.2 * np.sin(joints[0]) * np.sin(joints[1]) + 2.8*(np.cos(joints[0])*np.sin(joints[2]) + np.sin(joints[0]*np.sin(joints[1]*np.cos(joints[2]))))
        effector2 = 2.8 * (np.sin(joints[0]) * np.sin(joints[2]) - np.cos(joints[0]) * np.sin(joints[1]) * np.cos(joints[2]))
        effector3 = -3.2 * (np.cos(joints[0]) * np.sin(joints[1])) * 2.8 * np.cos(joints[1]) * np.cos(joints[2]) + 3.2 * np.cos(joints[1]) +4
        end_effector = np.array([effector1, effector2, effector3])
        return end_effector

    def jacobian(self, image):
        joints = self.vision_2.detect_joint_angles(image)
        #calculate each jacobian matrix element individually (for readability)
        j11 = np.sin(joints[1]) * np.cos(joints[2])
        j12 = joints[0] * np.cos(joints[1]) * np.cos(joints[2])
        j13 = -joints[0] * np.sin(joints[1]) + np.sin(joints[2])
        j21 = np.sin(joints[1]) * np.sin(joints[2])
        j22 = joints[0] * np.cos(joints[1]) * np.sin(joints[2])
        j23 = joints[0] * np.sin(joints[1]) * np.cos(joints[2])
        j31 = np.cos(joints[1])
        j32 = -joints[0] * np.sin(joints[1])
        j33 = 0
        jacobian = np.array([j11, j12, j13], [j21, j22, j23], [j31, j32, j33])
        return jacobian





