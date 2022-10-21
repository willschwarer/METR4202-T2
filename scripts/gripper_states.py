#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from time import sleep

from std_msgs.msg import Float32, String

#Conveyor Moving

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["initial"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)
        self.grip_pub = rospy.Publisher("gripper", Float32, queue_size=10)
        self.rate = rospy.Rate(10)
        
    def execute(self, userdata):
        rospy.loginfo("Moving to Zero position")
        self.move_pub.publish("Zero")
        self.grip_pub.publish(0.0)
        sleep(3)
        return "initial"

class Move_To_Cube(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["cube"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)
        
    def execute(self, userdata):
        rospy.loginfo("Moving to cube position")
        self.move_pub.publish("Camera")
        
        #Get callback to determine time
        sleep(5)
        return "cube"
   
class Grip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["grab"])
        self.pub = rospy.Publisher("gripper", Float32, queue_size=10)
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)

    def execute(self, userdata):
        rospy.loginfo("Gripping cube")
        self.move_pub.publish("Hold")
        msg = Float32()
        msg.data = 1.2
        self.grip_pub.publish(msg)
        sleep(1)
        return "grab"

class Move_Up(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["up"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)
        
    def execute(self, userdata):
        rospy.loginfo("Moving cube up")
        self.move_pub.publish("Up")
        
        #Get callback to determine time
        sleep(5)
        return "up"

class Move_To_Camera(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["camera"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)

    def execute(self, userdata):
        rospy.loginfo("Holding cube to camera")
        self.move_pub.publish("Colour")
        sleep(2)
        return "camera"

class Check_Colour(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["red","yellow","green","blue","none"])
        self.color_sub = rospy.Subscriber("detected_color",String, self.callback)
        self.zone = "NONE"
    
    def callback(self, msg:String):
        rospy.loginfo("callback works?")
        self.zone = msg.data

    def execute(self, userdata):
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
    def __init__(self):
        smach.State.__init__(self, outcomes=["zone_1"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)
        
    def execute(self, userdata):
        rospy.loginfo("Moving red cube to zone 1")
        self.move_pub.publish("Zone_1")
        
        #Get callback to determine time
        sleep(5)
        return "zone_1"

class Move_To_Zone_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["zone_2"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)
        
    def execute(self, userdata):
        rospy.loginfo("Moving yellow cube to zone 2")
        self.move_pub.publish("Zone_2")
        
        #Get callback to determine time
        sleep(5)
        return "zone_2"

class Move_To_Zone_3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["zone_3"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)
        
    def execute(self, userdata):
        rospy.loginfo("Moving green cube to zone 3")
        self.move_pub.publish("Zone_3")
        
        #Get callback to determine time
        sleep(5)
        return "zone_3"

class Move_To_Zone_4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["zone_4"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)

    def execute(self, userdata):
        rospy.loginfo("Moving blue cube to zone 4")
        self.move_pub.publish("Zone_4")
        
        #Get callback to determine time
        sleep(5)
        return "zone_4"

class Release(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["release"])
        self.move_pub = rospy.Publisher("movement_source", String, queue_size=20)
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        rospy.loginfo("Releasing cube")
        self.move_pub.publish("Hold")
        self.grip_pub.publish(0.0)
        sleep(1)
        return "release"

def main():  
    rospy.init_node("State_Machine")
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["grip"])
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add("Initial", Initial(), transitions={"initial":"Move_To_Cube"})
        smach.StateMachine.add("Move_To_Cube", Move_To_Cube(), transitions={"cube":"Grip"})
        smach.StateMachine.add("Grip", Grip(), transitions={"grab":"Move_Up"})
        smach.StateMachine.add("Move_Up", Move_Up(), transitions={"up":"Move_To_Camera"})
        smach.StateMachine.add("Move_To_Camera", Move_To_Camera(), transitions={"camera":"Check_Colour"})
        smach.StateMachine.add("Check_Colour", Check_Colour(), transitions={"red":"Move_To_Zone_1",
                "yellow":"Move_To_Zone_2","green":"Move_To_Zone_3","blue":"Move_To_Zone_4","none":"Move_To_Cube"})
        smach.StateMachine.add("Move_To_Zone_1", Move_To_Zone_1(), transitions={"zone_1":"Release"})
        smach.StateMachine.add("Move_To_Zone_2", Move_To_Zone_2(), transitions={"zone_2":"Release"})
        smach.StateMachine.add("Move_To_Zone_3", Move_To_Zone_3(), transitions={"zone_3":"Release"})
        smach.StateMachine.add("Move_To_Zone_4", Move_To_Zone_4(), transitions={"zone_4":"Release"})
        smach.StateMachine.add("Release", Release(), transitions={"release":"Initial"})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()


if __name__ == "__main__":
    main()
