#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

class Movement_Switch:
    zone_angles = [-1.13, -0.42, 0.42, 1.13] # The angle of the bottom servo
    zone_names = ["Zone_1", "Zone_2", "Zone_3", "Zone_4"] # Source name of the zones

    def source_callback(self, msg: String):
        # Update the source the gripper and kinematics are pulling from
        self.source = msg.data
        rospy.loginfo(self.source)

    def camera_callback(self, msg: Point32):
        # Update the location the camera
        self.camera_location = msg

    def kinematics_callback(self, msg: JointState):
        # Update the output angles from inverse kinematics
        self.kinematics_joint_states = msg
        self.kinematics_joint_states.velocity = [1, 1, 1, 1]

    def current_joint_callback(self, msg: JointState):
        # Update the angles the dynamixels are reporting
        current_joint_states = msg

    def movement_switch(self):
        # Init Node
        rospy.init_node("movement_switch", anonymous=True)
        # Setup publishers and subscribers
        self.location_pub = rospy.Publisher("cube_location", Point32, queue_size=20)
        self.joint_pub = rospy.Publisher("desired_joint_states", JointState, queue_size=20)
        self.rk_pub = rospy.Publisher("use_reverse_kinematics", Bool, queue_size=20)
        rospy.Subscriber("movement_source", String, self.source_callback)
        rospy.Subscriber("camera_location", Point32, self.camera_callback)
        rospy.Subscriber("kinematics_joint_states", JointState, self.kinematics_callback)
        rospy.Subscriber("joint_states", JointState, self.current_joint_callback)
        # Initialise Variables
        self.source = "Zero"
        self.camera_location = Point32()
        self.cube_location = Point32()
        self.kinematics_joint_states = JointState()
        self.kinematics_joint_states.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
        self.kinematics_joint_states.position = [0, 0, 0, 0]
        self.kinematics_joint_states.velocity = [1, 1, 1, 1]
        self.desired_joint_states = self.kinematics_joint_states
        self.current_joint_states = JointState()
        self.last_zone = 0
        # Set the refresh rate for ROS
        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Switch between states
            if self.source == "Zero":
                # Initial State
                self.rk_pub.publish(False)
                self.desired_joint_states = JointState()
                self.desired_joint_states.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
                self.desired_joint_states.position = [0, 0, 0, 0]
                self.desired_joint_states.velocity = [1, 1, 1, 1]
            elif self.source == "Camera_Track":
                # Follow the location the camera is reporting
                self.rk_pub.publish(False)
                self.cube_location = self.camera_location
                self.desired_joint_states = self.kinematics_joint_states
            elif self.source == "Camera":
                # Get a single location the camera reports then hold that position
                if self.camera_counter == 0:
                    self.rk_pub.publish(False)
                    self.cube_location = self.camera_location
                    self.desired_joint_states = self.kinematics_joint_states
                else:
                    self.rk_pub.publish(False)
                    self.desired_joint_states = self.kinematics_joint_states
                self.camera_counter += 1
            elif self.source == "Up":
                # Move up from last reported cube location (Used to reduce forward motion of gripper lift)
                self.rk_pub.publish(False)
                self.cube_location.z = 140.0
                self.desired_joint_states = self.kinematics_joint_states
            elif self.source == "Zup":
                # Move up from last reported zone
                self.rk_pub.publish(True)
                self.desired_joint_states = JointState()
                self.desired_joint_states.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
                self.desired_joint_states.position = [0.78, -1.3, -0.8, self.zone_angles[self.last_zone]]
                self.desired_joint_states.velocity = [1, 1, 1, 1]
            elif self.source == "Colour":
                # Move to camera to test colour
                self.rk_pub.publish(True)
                self.desired_joint_states = JointState()
                self.desired_joint_states.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
                self.desired_joint_states.position = [0, 0, 0.56, 0]
                self.desired_joint_states.velocity = [1, 1, 1, 1]
            elif self.source == "Throw":
                # Position to prepare for a throw
                self.rk_pub.publish(True)
                self.desired_joint_states = JointState()
                self.desired_joint_states.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
                self.desired_joint_states.position = [-1.4, 1.4, 0, 0]
                self.desired_joint_states.velocity = [1, 1, 1, 1]
            elif self.source == "Yeet":
                # Throw the cube
                self.rk_pub.publish(False)
                self.desired_joint_states = JointState()
                self.desired_joint_states.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
                self.desired_joint_states.position = [0, 0, 0, 0]
                self.desired_joint_states.velocity = [3, 3, 3, 3]
            elif self.source in self.zone_names:
                # Move to the cube drop off zones
                self.last_zone = self.zone_names.index(self.source)
                self.rk_pub.publish(True)
                self.desired_joint_states = JointState()
                self.desired_joint_states.name = ["joint_1", "joint_2", "joint_3", "joint_4"]
                self.desired_joint_states.position = [0.78, -1.3, -1.09, self.zone_angles[self.last_zone]]
                self.desired_joint_states.velocity = [1, 1, 1, 1]

            # Reset camera counter if no longer in camera position
            if self.source != "Camera":
                self.camera_counter = 0

            # Publish updated data to inverse kinematics and dynamixels
            self.location_pub.publish(self.cube_location)
            self.joint_pub.publish(self.desired_joint_states)
            # print(self.cube_location)
            # print(self.desired_joint_states)
            self.rate.sleep()

try:
    MS = Movement_Switch()
    MS.movement_switch()
except rospy.ROSInterruptException:
    pass
