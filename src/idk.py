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
pi = np.pi


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        r = rospy.Rate(100)  # attempt to stop the 1st callback being used like 3x as often as the 2nd

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
    def detect_red(self, image, caller):
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if M['m00'] != 0.0:  # check blob is visible
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:  # ASSUMPTION: if you can't see the blob, pretend it's where you last saw it
            if caller == 1:
                cx = self.red[1]
            else:
                cx = self.red[0]
            cy = self.red[2]
        return np.array([cx, cy])

    def detect_blue(self, image, caller):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if M['m00'] != 0.0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            if caller == 1:
                cx = self.blue[1]
            else:
                cx = self.blue[0]
            cy = self.blue[2]
        return np.array([cx, cy])

    def detect_joint_angles(self, image, caller):
        a = self.pixel_to_meter
        center = a * self.green
        c1Pos = a * self.yellow
        c2Pos = a * self.detect_blue(image, caller)
        c3Pos = a * self.detect_red(image, caller)

        ja1 = np.arctan2(center[0] - c1Pos[0], center[1] - c1Pos[1])
        ja2 = np.arctan2(c1Pos[0] - c2Pos[0], c1Pos[1] - c2Pos[1]) - ja1
        ja3 = np.arctan2(c2Pos[0] - c3Pos[0], c2Pos[1] - c3Pos[1]) - ja2 - ja1

        return np.array([ja1, ja2, ja3])

    # coords from our cameras
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

    # Takes 2 vectors, returns an unsigned angle between them
    def vec_angle(self, ab, ac):
        n1 = ab / np.linalg.norm(ab)
        n2 = ab / np.linalg.norm(ab)
        dot = np.dot(n1, n2)
        angle = np.arccos(dot)
        return angle

    def abvector(self, a, b):
        return b - a

    def joint2(self):
        # calculate movement around y


        angle = 111.1
        return angle

    def joint3(self):
        angle = self.vec_angle(self.abvector(self.yellow, self.green), self.abvector(self.yellow, self.blue))
        if angle > pi/2:
            angle = pi - angle
        elif angle < -pi/2:
            angle = angle - pi
        return angle

    def joint4(self):
        angle = self.vec_angle(self.abvector(self.blue, self.yellow), self.abvector(self.blue, self.red))
        return angle

    # makes coordinates relative to the green blob
    def base_relative(self, point):
        x = point[0] - self.green[0]
        y = point[1] - self.green[1]
        z = point[2] - self.green[2]
        return np.array([x, y, z])

    def calculate_joint_angles(self):
        x_unit = np.array([1, 0, 0])
        y_unit = np.array([0, 1, 0])
        z_unit = np.array([0, 0, 1])

        r = self.base_relative(self.red)
        g = self.base_relative(self.green)
        b = self.base_relative(self.blue)
        y = self.base_relative(self.yellow)
        yb = b - y

        newframe = np.cross(y_unit, yb)






    # Recieve data from camera 1, process it
    def callback1(self, data):
        # Receive the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.red = self.get_yz(self.red, self.detect_red(self.cv_image1, 1))
        self.blue = self.get_yz(self.blue, self.detect_blue(self.cv_image1, 1))
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

        joint_data = self.detect_joint_angles(self.cv_image2, 2)

        self.red = self.get_xz(self.red, self.detect_red(self.cv_image2, 2))
        self.blue = self.get_xz(self.blue, self.detect_blue(self.cv_image2, 2))

        self.joints = Float64MultiArray()
        self.joints = joint_data
        # print(self.vec_angle(self.blue, self.yellow, self.red))
        # Publish the results
        try:
            self.robot_joint2_pub.publish(self.joints[0])
            self.robot_joint3_pub.publish(self.joint3())  # self.joints[1])
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
