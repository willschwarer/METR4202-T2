#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Point32
from std_msgs.msg import String

class Movement_Switch:

    def source_callback(self, msg: String):
        self.source = msg.data

    def camera_callback(self, msg: Point32):
        self.camera_location = msg

    def movement_switch(self):
        rospy.init_node("movement_switch", anonymous=True)
        self.pub = rospy.Publisher("cube_location", Point32, queue_size=20)
        rospy.Subscriber("movement_source", String, self.source_callback)
        rospy.Subscriber("camera_location", Point32, self.camera_callback)
        self.source = "Zero"
        self.camera_location = Point32()
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.source == "Zero":
                self.cube_location = Point32()
                self.cube_location.x = 0.0
                self.cube_location.y = 0.0
                self.cube_location.z = 0.0
            elif self.source == "Camera":
                self.cube_location = self.camera_location
            self.pub.publish(self.cube_location)
            self.rate.sleep()
