#!/usr/bin/env python3

import rospy
import smach
import smach_ros

import inverse_kinematics_ROS
import servo_interface

from std_msgs.msg import Float32
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

# define zero-configuration state
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['zero_configuration'])
        Inverse_Kinematics_ROS.inverse_kinematics_ROS

    def execute(self, userdata):
        rospy.loginfo('Executing state Initial')
        return 'zero_configuration'

class Random(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next_configuration'])
        Inverse_Kinematics_ROS.inverse_kinematics_ROS

    def execute(self, userdata):
        rospy.loginfo('Executing state Random')
        self.pub = rospy.Publisher("desired_joint_states", JointState, queue_size=20)
        self.sub = rospy.Subscriber("cube_location", Point32, self.callback)
        return 'next_configuration'

# define gripper grab state
class Grip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grab'])
        servo_interface

    def execute(self, userdata):
        rospy.loginfo('Executing state Grip')
        self.pub = rospy.Publisher("gripper", Float32, queue_size=10)
        #self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = Float32()
            msg.data = 1
            self.pub.publish(msg)
            #self.rate.sleep()
        return 'grab'

class Open(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['open'])
        servo_interface

    def execute(self, userdata):
        rospy.loginfo('Executing state Open')
        self.pub = rospy.Publisher("gripper", Float32, queue_size=10)
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(0)
        return 'open'

def main():
    rospy.init_node('test_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['grip'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Initial', Initial(), 
                               transitions={'zero_configuration':'Grip'})
        smach.StateMachine.add('Grip', Grip(), 
                               transitions={'grab':'Initial'})
                               
        smach.StateMachine.add('Random', Initial(), 
                               transitions={'next_configuration':'Initial'})
        smach.StateMachine.add('Open', Open(), 
                               transitions={'open':'Grip'})
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()
