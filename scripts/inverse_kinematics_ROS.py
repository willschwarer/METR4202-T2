#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

import numpy as np
import inverse_kinematics

class Inverse_Kinematics_ROS:
    # Initialise Variables
    theta = [0.0, 0.0, 0.0, 0.0]
    use_up = True # Use the up angles, give the robot more clearance
    use_reverse = False # Reverse is to position the robot on the other side of the rig (i.e. Y < 0)
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4"]

    def callback(self, msg: Point32):
        # Check if position is zero and set all angles to zero
        if np.abs(msg.x) < 0.01 and np.abs(msg.y) < 0.01 and np.abs(msg.z) < 0.01:
            self.theta = [0, 0, 0, 0]
            return
        # Define constants which are useful for Inverse Kinematics
        right_angle = np.pi / 2
        L0, L1, L2, L3_x, L3_y = 100, 117.5, 95, 15, 102

        # Run the inverse kinematics function
        theta_0, theta_1_down, theta_2_down, theta_3_down, theta_1_up, theta_2_up, theta_3_up = \
            inverse_kinematics.IKin_4R(L0, L1, L2, L3_x, L3_y, msg.x, msg.y, msg.z)

        # Select whether to use the up or down variant
        if self.use_up:
            self.theta = [theta_3_up, -theta_2_up, right_angle - theta_1_up, theta_0]
        else:
            self.theta = [theta_3_down, -theta_2_down, right_angle - theta_1_down, theta_0]

        # Convert theta to reverse if set
        if self.use_reverse:
            self.theta = [-self.theta[0], -self.theta[1], -self.theta[2], self.theta[3]]

    def swap_RK(self, msg:Bool):
        # Update reverse kinematics mode
        self.use_reverse = msg.data

    def inverse_kinematics_ROS(self):
        # Setup Node
        rospy.init_node("inverse_kinematics", anonymous=True)
        self.pub = rospy.Publisher("kinematics_joint_states", JointState, queue_size=20)
        # Setup publishers and subscribers
        rospy.Subscriber("cube_location", Point32, self.callback)
        rospy.Subscriber("use_reverse_kinematics", Bool, self.swap_RK)
        # Set the refresh rate for ROS
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Create and publish Joint States
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
