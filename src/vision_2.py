#!/usr/bin/env python3

# solution for Section 2.2. This file should initiate a node that subscribes to image topics:
# ”/camera1/robot/image raw”
# ”/camera2/robot/image raw”
# and publishes the 3 joint state topics:
# ”joint angle 1”
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


class vision_2:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)

        # subscriptions
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)


        #publishers
        self.robot_joint1_pub = rospy.Publisher("joint_angle_1", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("joint_angle_4", Float64, queue_size=10)


        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    def detect_red(self, image):
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def detect_green(self, image):
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def detect_blue(self, image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def detect_yellow(self, image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx, cy])

    def pixel_to_meter(self, image):
        c1 = self.detect_blue(image)
        c2 = self.detect_green(image)
        distance = np.sum((c1 - c2) ** 2)
        return 3 / np.sqrt(distance)

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
        cv2.waitKey(1)

        self.joints = Float64MultiArray()
        self.joints.data = self.detect_joint_angles(self.cv_image1)

        # Publish the results
        try:
            # self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
            self.robot_joint1_pub.publish(self.joints.data[0])
            self.robot_joint3_pub.publish(self.joints.data[1])
            self.robot_joint4_pub.publish(self.joints.data[2])
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
        cv2.waitKey(1)

        self.joints = Float64MultiArray()
        self.joints.data = self.detect_joint_angles(self.cv_image2)
        # Publish the results
        try:
            # self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            self.robot_joint1_pub.publish(self.joints.data[0])
            self.robot_joint3_pub.publish(self.joints.data[1])
            self.robot_joint4_pub.publish(self.joints.data[2])
        except CvBridgeError as e:
            print(e)

# call the class
def main(args):
    ic = vision_2()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)