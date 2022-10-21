#! /usr/bin/env python3

# TODO Report back when movement is completed

import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

class Movement_Switch:

    def source_callback(self, msg: String):
        self.source = msg.data

    def camera_callback(self, msg: Point32):
        self.camera_location = msg

    def kinematics_callback(self, msg: JointState):
        self.kinematics_joint_states = msg

    def movement_switch(self):
        rospy.init_node("movement_switch", anonymous=True)
        self.location_pub = rospy.Publisher("cube_location", Point32, queue_size=20)
        self.joint_pub = rospy.Publisher("desired_joint_states", JointState, queue_size=20)
        self.rk_pub = rospy.Publisher("use_reverse_kinematics", Bool, queue_size=20)
        rospy.Subscriber("movement_source", String, self.source_callback)
        rospy.Subscriber("camera_location", Point32, self.camera_callback)
        rospy.Subscriber("kinematics_joint_states", JointState, self.kinematics_callback)
        rospy.Subscriber("joint_states", JointState, self.kinematics_callback)
        self.source = "Zero"
        self.camera_location = Point32()
        self.kinematics_joint_states = JointState()
        self.kinematics_joint_states.names = ["joint_1", "joint_2", "joint_3", "joint_4"]
        self.kinematics_joint_states.position = [0, 0, 0, 0]
        self.desired_joint_states = kinematics_joint_states
        self.current_joint_states = JointState()
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.source == "Zero":
                self.rk_pub.publish(True)
                self.desired_joint_states = JointState()
                self.desired_joint_states.names = ["joint_1", "joint_2", "joint_3", "joint_4"]
                self.desired_joint_states.position = [0, 0, 0, 0]
            elif self.source == "Camera":
                self.rk_pub.publish(False)
                self.cube_location = self.camera_location
                self.desired_joint_states = self.kinematics_joint_states
            elif self.source == "Up":
                self.rk_pub.publish(False)
                self.cube_location.z = 70.0
                self.desired_joint_states = self.kinematics_joint_states
            elif self.source == "Colour":
                self.rk_pub.publish(True)
                self.desired_joint_states = JointState()
                self.desired_joint_states.names = ["joint_1", "joint_2", "joint_3", "joint_4"]
                self.desired_joint_states.position = [0, 0, 0.7, 0]
            elif self.source == "Zone_1":
                #TODO Rewrite in a more concise way
                self.rk_pub.publish(True)
                self.cube_location = Point32()
                self.cube_location.x = -180.0
                self.cube_location.y = 80.0
                self.cube_location.z = -30.0
                self.desired_joint_states = self.kinematics_joint_states
            elif self.source == "Zone_2":
                self.rk_pub.publish(True)
                self.cube_location = Point32()
                self.cube_location.x = -80.0
                self.cube_location.y = 180.0
                self.cube_location.z = -30.0
                self.desired_joint_states = self.kinematics_joint_states
            elif self.source == "Zone_3":
                self.rk_pub.publish(True)
                self.cube_location = Point32()
                self.cube_location.x = 80.0
                self.cube_location.y = 180.0
                self.cube_location.z = -30.0
                self.desired_joint_states = self.kinematics_joint_states
            elif self.source == "Zone_4":
                self.rk_pub.publish(True)
                self.cube_location = Point32()
                self.cube_location.x = 180.0
                self.cube_location.y = 80.0
                self.cube_location.z = -30.0
                self.desired_joint_states = self.kinematics_joint_states
            self.location_pub.publish(self.cube_location)
            self.joint_pub.publish(self.desired_joint_states)
            self.rate.sleep()
            
try:
    MS = Movement_Switch()
    MS.movement_switch()
except rospy.ROSInterruptException:
    pass
