#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import numpy as np
from time import sleep
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32, String, Int32
from fiducial_msgs.msg import FiducialTransformArray as FTA

class Initial(smach.State):
    # Robot starts in zero configuration with gripper open
    def __init__(self):
        smach.State.__init__(self, outcomes=["initial"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)
        self.grip_pub = rospy.Publisher("gripper", Float32, queue_size=10)
        self.rate = rospy.Rate(10)
    
    def execute(self, userdata):
        # Move robot and gripper to zero position 
        rospy.loginfo("Moving to Zero position")
        self.move_pub.publish("Zero")
        self.grip_pub.publish(0.0)
        sleep(1)
        return "initial"

class Move_To_Cube(smach.State):
    # Camera gets the cube position and robot uses inverse kinematics to move there 
    def __init__(self):
        smach.State.__init__(self, outcomes=["cube"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)
        self.cube_counter = 0
        self.count_sub = rospy.Subscriber("block_counter", Int32, self.counter_callback)
        self.block_sub = rospy.Subscriber("fiducial_transforms", FTA, self.fiducial_callback)
        self.yaw = 0
        self.spinning = 0

    def fiducial_callback(self, msg: FTA):
        if self.cube_counter > 0:
            orientation = msg.transforms[0].transform.rotation
            orient_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            (roll_next, pitch_next, yaw_next) = euler_from_quaternion(orient_list)

            if np.abs(self.yaw - yaw_next) < 0.01:
                self.spinning += 1
            else:
                self.spinning = 0
            self.yaw = yaw_next

    def counter_callback(self, msg: Int32):
        # Set cube counter to read from the camera the number of cubes
        self.cube_counter = msg.data

    def execute(self, userdata):
        # Wait for a cube to be on conveyor, then move to a position to grab it 
        while self.cube_counter == 0 or self.spinning < 3:
            pass # wait for a cube
        rospy.loginfo("Moving to cube position")
        self.move_pub.publish("Camera")
        sleep(2)
        return "cube"
   
class Grip(smach.State):
    # Gripper closes to grab the cube
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
        sleep(0.3)
        return "grab"

class Move_Up_Conveyor(smach.State):
    # Robot lifts cube upwards to ensure no collisions or other cubes are knocked
    def __init__(self):
        smach.State.__init__(self, outcomes=["cup"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)

    def execute(self, userdata):
        rospy.loginfo("Moving cube up")
        self.move_pub.publish("Up")
        sleep(1.5)
        return "cup"

class Move_To_Camera(smach.State):
    # Robot lifts cube to the camera to detect the colour
    def __init__(self):
        smach.State.__init__(self, outcomes=["camera"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)

    def execute(self, userdata):
        rospy.loginfo("Holding cube to camera")
        self.move_pub.publish("Colour")
        sleep(3)
        return "camera"

class Check_Colour(smach.State):
    # Camera gets the colour of the cube
    def __init__(self):
        smach.State.__init__(self, outcomes=["red","yellow","green","blue","none"])
        self.color_sub = rospy.Subscriber("detected_color",String, self.callback)
        self.zone = "NONE"

    def callback(self, msg:String):
        self.zone = msg.data

    def execute(self, userdata):
        # Determine the colour of the cube 
        rospy.loginfo("Checking colour")
        if self.zone == "RED":
            return "red"
        elif self.zone == "YELLOW":
            return "yellow"
        elif self.zone == "GREEN":
            return "green"
        elif self.zone == "BLUE":
            return "blue"
        return "none"

class Move_To_Zone_1(smach.State):
    # Robot moves red cube to zone 1
    def __init__(self):
        smach.State.__init__(self, outcomes=["zone_1"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)

    def execute(self, userdata):
        # Use inverse kinematics to move cube
        rospy.loginfo("Moving red cube to zone 1")
        self.move_pub.publish("Zone_1")
        sleep(3)
        return "zone_1"

class Move_To_Zone_2(smach.State):
    # Robot moves yellow cube to zone 2
    def __init__(self):
        smach.State.__init__(self, outcomes=["zone_2"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)

    def execute(self, userdata):
        # Use inverse kinematics to move cube
        rospy.loginfo("Moving yellow cube to zone 2")
        self.move_pub.publish("Zone_2")
        sleep(3)
        return "zone_2"

class Move_To_Zone_3(smach.State):
    # Robot moves green cube to zone 3
    def __init__(self):
        smach.State.__init__(self, outcomes=["zone_3"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)

    def execute(self, userdata):
        # Use inverse kinematics to move cube
        rospy.loginfo("Moving green cube to zone 3")
        self.move_pub.publish("Zone_3")
        sleep(3)
        return "zone_3"

class Move_To_Zone_4(smach.State):
    # Robot moves blue cube to zone 4
    def __init__(self):
        smach.State.__init__(self, outcomes=["zone_4"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)

    def execute(self, userdata):
        # Use inverse kinematics to move cube
        rospy.loginfo("Moving blue cube to zone 4")
        self.move_pub.publish("Zone_4")
        sleep(3)
        return "zone_4"

class Release(smach.State):
    # Gripper releases cube in its zone
    def __init__(self):
        smach.State.__init__(self, outcomes=["release"])
        self.grip_pub = rospy.Publisher("gripper", Float32, queue_size=10)
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        # Move gripper position to 0, to release it 
        rospy.loginfo("Releasing cube")
        self.move_pub.publish("Hold")
        self.grip_pub.publish(0.0)
        sleep(0.3)
        return "release"

class Move_Up_Zone(smach.State):
    # Robot lifts gripper upwards to ensure no cube is not knocked over
    def __init__(self):
        smach.State.__init__(self, outcomes=["zup"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)

    def execute(self, userdata):
        # Use inverse kinematics to move upwards 
        rospy.loginfo("Moving cube up from zone")
        self.move_pub.publish("Zup")
        sleep(1)
        return "zup"

def main():  
    # Initialise node
    rospy.init_node("State_Machine")

    # Ceate a SMACH state machine
    sm = smach.StateMachine(outcomes=["grip"])

    # Open container
    with sm:
        # Add each state to container
        # Set up transitions between states. Given a state and outcome, transition to the next state 
        smach.StateMachine.add("Initial", Initial(), transitions={"initial":"Move_To_Cube"})
        smach.StateMachine.add("Move_To_Cube", Move_To_Cube(), transitions={"cube":"Grip"})
        smach.StateMachine.add("Grip", Grip(), transitions={"grab":"Move_Up_Conveyor"})
        smach.StateMachine.add("Move_Up_Conveyor", Move_Up_Conveyor(), transitions={"cup":"Move_To_Camera"})
        smach.StateMachine.add("Move_To_Camera", Move_To_Camera(), transitions={"camera":"Check_Colour"})
        smach.StateMachine.add("Check_Colour", Check_Colour(), transitions={"red":"Move_To_Zone_1",
                "yellow":"Move_To_Zone_2","green":"Move_To_Zone_3","blue":"Move_To_Zone_4","none":"Move_To_Cube"})
        smach.StateMachine.add("Move_To_Zone_1", Move_To_Zone_1(), transitions={"zone_1":"Release"})
        smach.StateMachine.add("Move_To_Zone_2", Move_To_Zone_2(), transitions={"zone_2":"Release"})
        smach.StateMachine.add("Move_To_Zone_3", Move_To_Zone_3(), transitions={"zone_3":"Release"})
        smach.StateMachine.add("Move_To_Zone_4", Move_To_Zone_4(), transitions={"zone_4":"Release"})
        smach.StateMachine.add("Release", Release(), transitions={"release":"Move_Up_Zone"})
        smach.StateMachine.add("Move_Up_Zone", Move_Up_Zone(), transitions={"zup":"Initial"})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()

if __name__ == "__main__":
    main()
