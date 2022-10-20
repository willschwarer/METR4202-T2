#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from time import sleep

from std_msgs.msg import Float32
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState


# define gripper grab state
class Grip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grab'])
        print(5)
    def execute(self, userdata):
        print(6)
        rospy.loginfo('Executing state Grip')
        self.pub = rospy.Publisher("gripper", Float32, queue_size=10)
        #self.rate = rospy.Rate(10)
        msg = Float32()
        msg.data = 1.2
        self.pub.publish(msg)
        sleep(1)
        print(7)
        return 'grab'

class Open(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['open'])
        print(8)
    def execute(self, userdata):
        print(9)
        rospy.loginfo('Executing state Open')
        self.pub = rospy.Publisher("gripper", Float32, queue_size=10)
        self.rate = rospy.Rate(10)
        self.pub.publish(0.0)
        sleep(1)
        print(10)
        return 'open'

def main():
    print(12)  
    rospy.init_node('test_state_machine_2')
    print(1)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['grip'])
    print(2)
    # Open the container
    with sm:
        # Add states to the container
        print(3)
        smach.StateMachine.add('Grip', Grip(), 
                               transitions={'grab':'Open'})
        print(4)                       
        smach.StateMachine.add('Open', Open(), 
                               transitions={'open':'Grip'})
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    print(11)   
    main()
