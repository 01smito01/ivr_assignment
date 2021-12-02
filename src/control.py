#!/usr/bin/env python3

#This is a file
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64



class control:
    # initialise
    target = Float64MultiArray()
    joint1 = Float64()
    joint3 = Float64()
    joint4 = Float64()
    joints = np.array([joint1, joint3, joint4])




    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('control', anonymous=True)
        rate = rospy.Rate(50)

        # subscriptions
        self.joint1_sub = rospy.Subscriber("joint_angle_1", Float64, self.callback1)
        self.joint3_sub = rospy.Subscriber("joint_angle_3", Float64, self.callback2)
        self.joint4_sub = rospy.Subscriber("joint_angle_4", Float64, self.callback3)

        # subscribe to target_pos topic to get target positions for IK implementation
        self.target_sub = rospy.Subscriber("/target_control/target_pos", Float64MultiArray, self.callback)

        # publishers
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        # initialize errors etc
        self.time_initial = rospy.get_time()
        self.time_prev = np.array([rospy.get_time()], dtype='float64')
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.errorderived = np.array([0.0, 0.0, 0.0], dtype='float64')

    def getJoints(self):
        joints = [j.data for j in control.joints]
        return joints

    def getTarget(self):
        targetget = control.target.data
        return np.array(targetget)

    def forward_kinematics(self, joints):
        # calculate each entry for the array to return
        effector1 = 3.2 * np.sin(joints[0]) * np.sin(joints[1]) + 2.8*(np.cos(joints[0])*np.sin(joints[2]) + np.sin(joints[0]*np.sin(joints[1]*np.cos(joints[2]))))
        effector2 = 2.8 * (np.sin(joints[0]) * np.sin(joints[2]) - np.cos(joints[0]) * np.sin(joints[1]) * np.cos(joints[2]))
        effector3 = -3.2 * (np.cos(joints[0]) * np.sin(joints[1])) * 2.8 * np.cos(joints[1]) * np.cos(joints[2]) + 3.2 * np.cos(joints[1]) + 4
        end_effector = np.array([effector1, effector2, effector3])
        return end_effector

    def jacobian(self, joints):
        # calculate each jacobian matrix element individually (for readability)
        j11 = 3.2 * np.cos(joints[0]) * np.sin(joints[1]) - 2.8 * np.sin(joints[0]) * np.sin(joints[2]) + 2.8 * np.cos(joints[0]) * np.sin(joints[1]) * np.cos(joints[2])
        j12 = 3.2 * np.sin(joints[0]) * np.cos(joints[1]) + 2.8 * np.sin(joints[0]) * np.cos(joints[1]) * np.cos(joints[2])
        j13 = 2.8 * np.cos(joints[0]) * np.cos(joints[2]) - 2.8 * np.sin(joints[0]) * np.sin(joints[1]) * np.sin(joints[2])
        j21 = 2.8 * np.cos(joints[0]) * np.sin(joints[2]) + 2.8 * np.sin(joints[0]) * np.sin(joints[1]) * np.cos(joints[2]) + 3.2 * np.sin(joints[0]) * np.sin(joints[1])
        j22 = -2.8 * np.cos(joints[0]) * np.cos(joints[1]) * np.cos(joints[2]) - 3.2* np.cos(joints[0]) * np.cos(joints[1])
        j23 = 2.8 * np.sin(joints[0]) * np.cos(joints[2]) + 2.8 * np.cos(joints[0]) * np.sin(joints[1]) * np.sin(joints[2])
        j31 = 0
        j32 = -2.8 * np.sin(joints[1]) * np.cos(joints[2]) - 3.2 * np.sin(joints[1])
        j33 = -2.8 * np.cos(joints[1]) * np.sin(joints[2])
        row1 = np.array([j11, j12, j13])
        row2 = np.array([j21, j22, j23])
        row3 = np.array([j31, j32, j33])
        jacobian = np.array([row1, row2, row3])
        return jacobian



    def control(self):
        curr_time = rospy.get_time()
        dt = curr_time - self.time_prev
        self.time_prev = curr_time
        q = self.getJoints()
        target = self.getTarget()
        inv_j = np.linalg.pinv(self.jacobian(q))
        pos = np.array(self.forward_kinematics(q))
        desired_pos = target
        self.error = (desired_pos - pos)/dt
        desired_q = q + (dt * np.dot(inv_j, self.error.transpose()))
        return desired_q

    # receive data using callbacks and publish robot joint infornation

    def callback(self, data):
        rospy.loginfo("Target is %s", data.data)
        self.target.data = data.data


    def callback1(self, data):
        rospy.loginfo("Joint1 is %s", data.data)
        self.joint1 = data.data

    def callback2(self, data):
        rospy.loginfo("Joint3 is %s", data.data)
        self.joint3 = data.data

    def callback3(self, data):
        rospy.loginfo("Joint4 is %s", data.data)
        self.joint4 = data.data

        q_d = self.control()

        targetjoint1 = Float64()
        targetjoint1.data = q_d[0]
        targetjoint3 = Float64()
        targetjoint3.data = q_d[1]
        targetjoint4 = Float64()
        targetjoint4.data = q_d[2]

        print(q_d)

        self.robot_joint1_pub.publish(targetjoint1)
        self.robot_joint3_pub.publish(targetjoint3)
        self.robot_joint4_pub.publish(targetjoint4)








def main(args):
    c = control()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


