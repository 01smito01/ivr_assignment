#!/usr/bin/env python3

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
        self.red = np.array([0.0, 0.0, 0.0])  # obvs nonsense
        self.blue = np.array([0.0, 0.0, 0.0])

        # green doesn't move and yellow doesn't move enough to be worth updating. so these won't be updated like ever.
        self.green = np.array([400.0, 400.0, 535.0])
        self.yellow = np.array([400.0, 400.0, 432.0])
        self.pixel_to_meter = 4 / ((535 - 432) ** 2) ** 0.5  # kind of assumes this is equal for each camera

    # Detection code mostly copied from the labs, w/ some additional smartness when we can't see blobs
    def detect_red(self, image):
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if (M['m00'] != 0.0):
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:  # ASSUMPTION: if you can't see the blob, pretend it's where you last saw it
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

    def detect_joint_angles(self, image):
        a = self.pixel_to_meter
        center = a * self.green
        c1Pos = a * self.yellow
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

    def vec_angle(self, base, end_a, end_b):
        v1 = end_a - base
        v2 = end_b - base

        n1 = v1 / np.linalg.norm(v1)
        n2 = v2 / np.linalg.norm(v2)
        dot = np.dot(n1, n2)
        angle = np.arccos(dot)
        cross = np.cross(v1, v2)

        if np.dot(cross, cross) < 0:
            angle = -angle
        return angle

    def dotproduct(self, vec1, vec2):
        return sum((a * b) for a, b in zip(vec1, vec2))

    def length(self, vec):
        return math.sqrt(self.dotproduct(vec, vec))

    def joint3side(self):
        angle = self.vec_angle(self.yellow, self.green, self.blue)
        print(self.blue[1])
        if self.blue[1] > 400:
            return angle - math.pi
        else:
            return -angle + math.pi

    # Recieve data from camera 1, process it, and publish
    def callback1(self, data):
        # Receive the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.red = self.get_yz(self.red, self.detect_red(self.cv_image1))
        self.blue = self.get_yz(self.blue, self.detect_blue(self.cv_image1))

        joint_data = self.detect_joint_angles(self.cv_image1)

        self.joints = Float64MultiArray()
        self.joints = joint_data

        # this one doesn't publish because I can't get them both to play nice

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
        self.blue = self.get_xz(self.blue, self.detect_blue(self.cv_image2))

        self.joints = Float64MultiArray()
        self.joints = joint_data
        # print(self.vec_angle(self.blue, self.yellow, self.red))
        # Publish the results
        try:
            self.robot_joint2_pub.publish(self.joints[0])
            self.robot_joint3_pub.publish(self.joint3side())  # self.joints[1])
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
