#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

import numpy as np
import inverse_kinematics

class Inverse_Kinematics_ROS:
    theta = [0.0, 0.0, 0.0, 0.0]
    use_up = True
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4"]

    def callback(self, msg: Point32):
        right_angle = np.pi / 2
        L0, L1, L2, L3_x, L3_y = 100, 117.5, 95, 15, 95
        theta_0, theta_1_down, theta_2_down, theta_3_down, theta_1_up, theta_2_up, theta_3_up = \
            inverse_kinematics.IKin_4R(L0, L1, L2, L3_x, L3_y, msg.x, msg.y, msg.z)

        if self.use_up:
            rospy.loginfo(rospy.get_caller_id() + "\nCalculated Data: T0: %f, T1: %f, T2: %f, T3: %f", theta_0, theta_1_up, theta_2_up, theta_3_up)
            self.theta = [theta_3_up, -theta_2_up, right_angle - theta_1_up, theta_0]
        else:
            rospy.loginfo(rospy.get_caller_id() + "\nCalculated Data: T0: %f, T1: %f, T2: %f, T3: %f", theta_0, theta_1_down, theta_2_down, theta_3_down)
            self.theta = [theta_3_down, -theta_2_down, right_angle - theta_1_down, theta_0]

    def inverse_kinematics_ROS(self):
        rospy.init_node("inverse_kinematics", anonymous=True)
        self.pub = rospy.Publisher("desired_joint_states", JointState, queue_size=20)
        rospy.Subscriber("cube_location", Point32, self.callback)
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for i in range(4):
                current_joint_state = JointState()
                current_joint_state.name = self.joint_names
                current_joint_state.position = self.theta
                self.pub.publish(current_joint_state)
            self.rate.sleep()

try:
    IKROS = Inverse_Kinematics_ROS()
    IKROS.inverse_kinematics_ROS()
except rospy.ROSInterruptException:
    pass
