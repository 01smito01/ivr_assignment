#!/usr/bin/env python3

# solution for Section 2.2. This file should initiate a node that subscribes to image topics:
# ”/camera1/robot/image raw”
# ”/camera2/robot/image raw”
# and publishes the 3 joint state topics:
# ”joint angle 1”
# ”joint angle 3”
# ”joint angle 4”


import roslib
import math
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
pi = math.pi

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

        # initial positions for blobs (won't ever change for green and yellow)
        self.green = np.array([400.0, 400.0, 535.0])
        self.yellow = np.array([400.0, 400.0, 432.0])
        self.red = np.array([400.0, 400.0, 350.0])
        self.blue = np.array([400.0, 400.0, 278.0])

        self.p2m = self.pixel_to_meter()


        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

    def detect_red(self, image, caller):
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if M['m00'] != 0.0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:  # ASSUMPTION: if you can't see the blob, pretend it's where you last saw it
            if caller == 1:
                cx = self.red[1]
            elif caller == 2:
                cx = self.red[0]
            else:  # This shouldn't ever happen
                print("Warning: trying to pass data from a nonexistent camera")
                cx = 0
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
            elif caller == 2:
                cx = self.blue[0]
            else:  # This shouldn't ever happen
                print("Warning: trying to pass data from a nonexistent camera")
                cx = 0
            cy = self.red[2]
        return np.array([cx, cy])

    def pixel_to_meter(self):
        c1 = self.yellow
        c2 = self.green
        distance = np.sum((c1 - c2) ** 2)
        return 4 / np.sqrt(distance)

    def unit_vector(self, vector):
        return vector / np.linalg.norm(vector)

    # thank you stackoverflow, very cool!
    def angle(self, v1, v2):
        v1u = self.unit_vector(v1)
        v2u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1u, v2u), -1.0, 1.0))

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

    def detect_joint_angles(self, image, caller):
        a = self.pixel_to_meter()
        center = a * self.yellow
        c1Pos = a * self.green
        c2Pos = a * self.detect_blue(image, caller)
        c3Pos = a * self.detect_red(image, caller)

        ja1 = np.arctan2(center[0] - c1Pos[0], center[1] - c1Pos[1])
        ja2 = np.arctan2(c1Pos[0] - c2Pos[0], c1Pos[1] - c2Pos[1]) -ja1
        ja3 = np.arctan2(c2Pos[0] - c3Pos[0], c2Pos[1] - c3Pos[1]) - ja2 - ja1

        return np.array([ja1, ja2, ja3])

    def j1(self):
        fy = np.array([0, 0, 0])  # Fake yellow
        fx = np.array([0,0,0])  # Fake vector along x axis
        fy[0], fy[1], fy[2] = 400, 400, self.blue[2]
        fx[0], fx[1], fx[2] = 600, 400, self.blue[2]

        fyb = self.blue - fy
        fyfx = fx - fy
        angle = self.angle(fyfx, fyb)

        if angle > pi:
            angle = pi - angle
        if angle < -pi:
            angle = -pi - angle

        test = np.cross(fyfx, fyb)
        print(test)
        if test[2] < 0:
            angle = -angle
        return angle


    def j3(self):
        yellowgreen = self.green - self.yellow
        yellowblue = self.blue - self.yellow
        yg_yz = np.array([yellowgreen[1], yellowgreen[2]])
        yb_yz = np.array([yellowblue[1], yellowblue[2]])
        angle = self.angle(yg_yz, yb_yz)
        if angle > pi/2:
            angle = pi - angle
        if angle < -pi/2:
            angle = -pi - angle
        if self.blue[1] > 400:
            angle = -angle
        return angle

    def j4(self):
        blueyellow = self.yellow - self.blue
        bluered = self.red - self.blue
        angle = self.angle(bluered, blueyellow)
        if angle > pi/2:
            angle = pi - angle
        if angle < -pi/2:
            angle = -pi - angle

        # Simplistic quadrant maths to get correct sign
        if self.blue[0] < 400 and self.blue[1] < 400:
           angle = -angle
        return angle

    # Recieve data from camera 1, process it
    def callback1(self, data):
        # Receive the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.red = self.get_yz(self.red, self.detect_red(self.cv_image1, 1))
        self.blue = self.get_yz(self.blue, self.detect_blue(self.cv_image1, 1))
        # only publish with callback 2 due to some nasty concurrency issues

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

        self.red = self.get_xz(self.red, self.detect_red(self.cv_image2, 2))
        self.blue = self.get_xz(self.blue, self.detect_blue(self.cv_image2, 2))

        # Publish the results
        try:
            self.robot_joint1_pub.publish(self.j1())  # joint_angles[0])
            self.robot_joint3_pub.publish(self.j3())
            self.robot_joint4_pub.publish(self.j4())
        except CvBridgeError as e:
            print(e)

# call the class
def main(args):
    v2 = vision_2()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
