#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import numpy as np
from time import sleep
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32, String, Int32
from fiducial_msgs.msg import FiducialTransformArray as FTA
#Conveyor Moving

y_centre = 0.045

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["initial"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)
        self.grip_pub = rospy.Publisher("gripper", Float32, queue_size=10)
        self.rate = rospy.Rate(10)
        
    def execute(self, userdata):
        rospy.loginfo("Moving to Zero position")
        self.move_pub.publish("Yeet")
        sleep(0.68)
        self.grip_pub.publish(0.0)
        sleep(1)
        return "initial"


class Move_To_Cube(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["cube"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)
        self.cube_counter = 0
        self.count_sub = rospy.Subscriber("block_counter", Int32, self.counter_callback)       

    def counter_callback(self, msg: Int32):
        self.cube_counter = msg.data

    def execute(self, userdata):
        while self.cube_counter == 0:
            pass # Wait for a cube

        rospy.loginfo("Moving to cube position")
        self.move_pub.publish("Camera_Track")
        sleep(0.5)
        self.move_pub.publish("Hold")
        sleep(3.5)
        return "cube"
   
class Grip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["grab"])
        self.grip_pub = rospy.Publisher("gripper", Float32, queue_size=10)
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)

    def execute(self, userdata):
        rospy.loginfo("Gripping cube")
        self.move_pub.publish("Hold")
        msg = Float32()
        msg.data = 1.2
        self.grip_pub.publish(msg)
        sleep(0.5)
        return "grab"

class Move_To_Throw(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["throw"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)

    def execute(self, userdata):
        rospy.loginfo("Throwing cube")
        self.move_pub.publish("Throw")
        sleep(0.5)
        return "throw"

def main():  
    rospy.init_node("State_Machine")
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["grip"])
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add("Initial", Initial(), transitions={"initial":"Move_To_Cube"})
        smach.StateMachine.add("Move_To_Cube", Move_To_Cube(), transitions={"cube":"Grip"})
        smach.StateMachine.add("Grip", Grip(), transitions={"grab":"Initial"})
        smach.StateMachine.add("Move_To_Throw", Move_To_Throw(), transitions={"throw":"Initial"})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()


if __name__ == "__main__":
    main()
