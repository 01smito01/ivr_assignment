#!/usr/bin/env python3

# solution for Section 2.1. This file should initiate a node that subscribes to image topics:
# ”/camera1/robot/image raw”
# ”/camera2/robot/image raw”
# and publishes the 3 joint state topics:
# ”joint angle 2”
# ”joint angle 3”
# ”joint angle 4”
import math

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        r = rospy.Rate(100)

        # subscriptions
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)

        # publishers

        self.robot_joint2_pub = rospy.Publisher("joint_angle_2", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("joint_angle_4", Float64, queue_size=10)
        self.bridge = CvBridge()

        # blob location init

        self.red = np.array([0.0, 0.0, 0.0])
        self.blue = [0.0, 0.0, 0.0]
        self.green = [0.0, 0.0, 0.0]
        self.yellow = [0.0, 0.0, 0.0]

    def detect_red(self, image):
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if (M['m00'] != 0.0):
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx = 0
            cy = 0
        return np.array([cx, cy])

    def detect_green(self, image):
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if (M['m00'] != 0.0):
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx = 0
            cy = 0
        return np.array([cx, cy])

    def detect_blue(self, image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if (M['m00'] != 0.0):
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx = 0
            cy = 0
        return np.array([cx, cy])

    def detect_yellow(self, image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if (M['m00'] != 0.0):
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx = 0
            cy = 0
        return np.array([cx, cy])

    def pixel_to_meter(self, image):
        c1 = self.detect_green(image)
        c2 = self.detect_yellow(image)
        distance = np.sum((c1 - c2) ** 2)
        return 4 / np.sqrt(distance)

    def detect_joint_angles(self, image):
        a = self.pixel_to_meter(image)
        center = a * self.detect_green(image)
        c1Pos = a * self.detect_yellow(image)
        c2Pos = a * self.detect_blue(image)
        c3Pos = a * self.detect_red(image)

        ja1 = np.arctan2(center[0] - c1Pos[0], center[1] - c1Pos[1])
        ja2 = np.arctan2(c1Pos[0] - c2Pos[0], c1Pos[1] - c2Pos[1]) - ja1
        ja3 = np.arctan2(c2Pos[0] - c3Pos[0], c2Pos[1] - c3Pos[1]) - ja2 - ja1

        return np.array([ja1, ja2, ja3])



    def get_xz(self, blob, detector):
        temp = blob
        detection = detector
        temp[0] = detection[0]
        temp[2] = detection[1]
        return np.array(temp)

    def get_yz(self, blob, detector):
        temp = blob
        detection = detector
        temp[1] = detection[0]
        temp[2] = detection[1]
        return np.array(temp)

    def reject_outlier(self, old, new):
        if abs(old - new) > 0.1*old:
            return old
        else:
            return new
    def vec_angle(self, base, end_a, end_b):
        v1 = end_a - base
        v2 = end_b - base
        v3 = np.cross(v1, v2)  # Cross product for sign detection


        n1 = v1/np.linalg.norm(v1)
        n2 = v2/np.linalg.norm(v2)
        dot = np.dot(n1, n2)
        return np.arccos(dot)

    def dotproduct(self, vec1, vec2):
        return sum((a*b) for a, b in zip(vec1, vec2))

    def length(self, vec):
        return math.sqrt(self.dotproduct(vec, vec))


    # Recieve data from camera 1, process it, and publish
    def callback1(self, data):
        # Receive the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.red = self.get_yz(self.red, self.detect_red(self.cv_image1))
        self.green = self.get_yz(self.green, self.detect_green(self.cv_image1))
        self.blue = self.get_yz(self.blue, self.detect_blue(self.cv_image1))
        self.yellow = self.get_yz(self.yellow, self.detect_yellow(self.cv_image1))


        joint_data = self.detect_joint_angles(self.cv_image1)

        self.joints = Float64MultiArray()
        self.joints = joint_data

        # Publish the results
        try:
            self.robot_joint2_pub.publish(self.joints[0])
            self.robot_joint3_pub.publish(self.joints[1])
            # self.robot_joint4_pub.publish(self.joints[2])
        except CvBridgeError as e:
            print(e)

    # Recieve data from camera 2, process it, and publish
    def callback2(self, data):
        # Receive the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        im1 = cv2.imshow('Camera 1', self.cv_image1)
        im2 = cv2.imshow('Camera 2', self.cv_image2)
        cv2.waitKey(3)

        joint_data = self.detect_joint_angles(self.cv_image2)

        self.red = self.get_xz(self.red, self.detect_red(self.cv_image2))
        self.green = self.get_xz(self.green, self.detect_green(self.cv_image2))
        self.blue = self.get_xz(self.blue, self.detect_blue(self.cv_image2))
        self.yellow = self.get_xz(self.yellow, self.detect_yellow(self.cv_image2))

        self.joints = Float64MultiArray()
        self.joints = joint_data
        # print(self.vec_angle(self.blue, self.yellow, self.red))
        # Publish the results
        try:
            self.robot_joint2_pub.publish(self.joints[0])
            self.robot_joint3_pub.publish(self.joints[1])
            self.robot_joint4_pub.publish(self.vec_angle(self.blue, self.yellow, self.red))  # self.joints[2])
        except CvBridgeError as e:
            print(e)


# call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
