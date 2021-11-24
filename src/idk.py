#!/usr/bin/env python3

# solution for Section 2.1. This file should initiate a node that subscribes to image topics:
# ”/camera1/robot/image raw”
# ”/camera2/robot/image raw”
# and publishes the 3 joint state topics:
# ”joint angle 2”
# ”joint angle 3”
# ”joint angle 4”

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

        # subscriptions
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)

        #publishers

        self.robot_joint2_pub = rospy.Publisher("joint_angle_2", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("joint_angle_4", Float64, queue_size=10)
        self.test_pub = rospy.Publisher("red_blob", Float64MultiArray, queue_size=10)

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # blob location init

        self.red = [0.0, 0.0, 0.0]
        self.blue = [0.0, 0.0, 0.0]
        self.green = [0.0, 0.0, 0.0]
        self.yellow = [0.0, 0.0, 0.0]

    def detect_red(self, image):
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if(M['m00']!=0.0):
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
        if(M['m00']!=0.0):
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
        if(M['m00']!=0.0):
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
        if(M['m00']!=0.0):
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx = 0
            cy = 0
        return np.array([cx, cy])

    def pixel_to_meter(self, image):
        c1 = self.detect_yellow(image)
        c2 = self.detect_blue(image)
        distance = np.sum((c1 - c2) ** 2)
        return 2.8 / np.sqrt(distance)

    def detect_joint_angles(self, image):
        a = self.pixel_to_meter(image)
        center = a * self.detect_green(image)
        c1Pos = a * self.detect_yellow(image)
        c2Pos = a * self.detect_blue(image)
        c3Pos = a * self.detect_red(image)

        ja1 = np.arctan2(center[0] - c1Pos[0], center[1] - c1Pos[1])
        ja2 = np.arctan2(c1Pos[0] - c2Pos[0], c1Pos[1] - c2Pos[1]) -ja1
        ja3 = np.arctan2(c2Pos[0] - c3Pos[0], c2Pos[1] - c3Pos[1]) - ja2 - ja1

        return np.array([ja1, ja2, ja3])

    def update_blob_position(self, image, colour, camera):
        if colour == 'red':
            temp = self.detect_red(image)
            if camera == 1: # Camera 1 has vertical z and horizontal y
                self.red[1] = temp[0]
                self.red[2] = temp[1]
            elif camera == 2: # Camera 2 has vertical z and horizontal x
                self.red[0] = temp[0]
                self.red[2] = temp[1]
        elif colour == 'green':
            temp = self.detect_green(image)
            if camera == 1:
                self.green[1] = temp[0]
                self.green[2] = temp[1]
            elif camera == 2:
                self.green[0] = temp[0]
                self.green[2] = temp[1]
        elif colour == 'blue':
            temp = self.detect_blue(image)
            if camera == 1:
                self.blue[1] = temp[0]
                self.blue[2] = temp[1]
            elif camera == 2:
                self.blue[0] = temp[0]
                self.blue[2] = temp[1]
        elif colour == 'yellow':
            temp = self.detect_yellow(image)
            if camera == 1:
                self.yellow[1] = temp[0]
                self.yellow[2] = temp[1]
            elif camera == 2:
                self.yellow[0] = temp[0]
                self.yellow[2] = temp[1]


    # Recieve data from camera 1, process it, and publish
    def callback1(self, data):
        # Receive the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)

        im1 = cv2.imshow('window1', self.cv_image1)
        cv2.waitKey(3)

        joint_data = self.detect_joint_angles(self.cv_image1)
        self.update_blob_position(self.cv_image1, 'red', 1)

        tst = Float64MultiArray()
        tst.data = np.array(self.red)

        self.joint4 = Float64()
        self.joint2 = Float64()
        self.joint2 = joint_data[0]
        self.joint3 = Float64()
        self.joint3 = joint_data[1]
        self.joint4 = joint_data[2]


        # Publish the results
        try:
            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
            self.test_pub.publish(tst)
        except CvBridgeError as e:
            print(e)

    # Recieve data from camera 2, process it, and publish
    def callback2(self, data):
        # Receive the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)
        im2 = cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(3)


        joint_data = self.detect_joint_angles(self.cv_image2)

        self.update_blob_position(self.cv_image2, 'red', 2)

        tst = Float64MultiArray()
        tst.data = np.array(self.red)

        self.joint2 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()

        self.joint2 = joint_data[0]
        self.joint3 = joint_data[1]
        self.joint4 = joint_data[2]

        # Publish the results
        try:
            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
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
