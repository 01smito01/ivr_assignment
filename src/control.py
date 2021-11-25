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
        rate = rospy.Rate(50)

        # subscriptions
        self.joint1_sub = rospy.Subscriber("joint_angle_1", Float64, self.callback1)
        self.joint3_sub = rospy.Subscriber("joint_angle_3", Float64, self.callback2)
        self.joint4_sub = rospy.Subscriber("joint_angle_4", Float64, self.callback3)

        #subscribe to target_pos topic to get target positions for IK implementation
        self.target_sub = rospy.Subscriber("/target_control/target_pos", Float64MultiArray, self.callback)

        #publishers
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)


        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        #initialize errors etc
        self.time_initial = rospy.get_time()
        self.time_prev = np.array([rospy.get_time()], dtype='floatt64')
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.errorderived = np.array([0.0, 0.0, 0.0], dtype='float64')


        #initialise
        self.target = Float64MultiArray()
        self.joint1 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()
        self.joints = np.array([self.joint1, self.joint3, self.joint4])

    #use methods implemented in vision_2 class to detect the position of robot end-effector

    def forward_kinematics(self, joints):
        #calculate each entry for the array to return
        effector1 = 3.2 * np.sin(joints[0]) * np.sin(joints[1]) + 2.8*(np.cos(joints[0])*np.sin(joints[2]) + np.sin(joints[0]*np.sin(joints[1]*np.cos(joints[2]))))
        effector2 = 2.8 * (np.sin(joints[0]) * np.sin(joints[2]) - np.cos(joints[0]) * np.sin(joints[1]) * np.cos(joints[2]))
        effector3 = -3.2 * (np.cos(joints[0]) * np.sin(joints[1])) * 2.8 * np.cos(joints[1]) * np.cos(joints[2]) + 3.2 * np.cos(joints[1]) +4
        end_effector = np.array([effector1, effector2, effector3])
        return end_effector

    def jacobian(self, joints):
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
        jacobian = np.array([[j11, j12, j13], [j21, j22, j23], [j31, j32, j33]])
        return jacobian

    def control(self, joints):
        curr_time = rospy.get_time()
        dt = curr_time - self.time_prev
        self.time_prev = curr_time
        q = joints
        inv_j = np.linalg.pinv(self.jacobian(joints))
        pos = np.array(self.forward_kinematics(joints))
        desired_pos = np.array(self.target)
        self.error = (desired_pos - pos)/dt
        desired_q = q + (dt * np.dot(inv_j, self.error.transpose()))
        return desired_q

    #publish robot joint infornation

    def callback(self, data):
        targetdata = self.target_sub
        rospy.loginfo(data.data)
        self.target.data = data

    def callback1(self, data):
        rospy.loginfo(data.data)
        self.joint1.data = data

    def callback2(self, data):
        rospy.loginfo(data.data)
        self.joint3.data = data

    def callback3(self, data):
        rospy.loginfo(data.data)
        self.joint4.data = data

        q_d = self.control(self.joints)
        self.joint1 = Float64()
        self.joint1.data = q_d[0]
        self.joint3 = Float64()
        self.joint3.data = q_d[1]
        self.joint4 = Float64()
        self.joint4.data = q_d[2]

        try:
            self.robot_joint1_pub.publish(self.joint1)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
        except Exception as e:
            print(e)


if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass



