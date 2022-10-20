#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from time import sleep
import threading

from std_msgs.msg import Float32
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initial'])
        
    def execute(self, userdata):
        rospy.loginfo('Moving to Zero position')
        self.pub = rospy.Publisher("cube_location", Point32, queue_size=20)
        #self.rate = rospy.Rate(10)
        msg = Point32()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        self.pub.publish(msg)
        
        self.pub1 = rospy.Publisher("gripper", Float32, queue_size=10)
        self.rate = rospy.Rate(10)
        self.pub1.publish(0.0)
        
        sleep(3)

        return 'initial'

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move'])
        
    def execute(self, userdata):
        rospy.loginfo('Moving to cube position')
        
        self.sub = rospy.Subscriber("camera_location", Point32, self.callback)
        
        #self.threadLock = threading.Lock()
        #self.threadLock.acquire()
        #self.threadLock.acquire(True)
        #self.threadLock.release()
        
        sleep(5)
        
        return 'move'
        
    def callback(self, msg: Point32):
        rospy.loginfo('Recieved Camera Location')
        self.pub = rospy.Publisher("cube_location", Point32, queue_size=10)
        self.pub.publish(msg.x,msg.y,msg.z)
        #self.threadLock.release()
        #wait
        
        sleep(4)
        
        


# define gripper grab state
class Grip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grab'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Grip')
        self.pub = rospy.Publisher("gripper", Float32, queue_size=10)
        #self.rate = rospy.Rate(10)
        msg = Float32()
        msg.data = 1.2
        self.pub.publish(msg)
        sleep(1)
        return 'grab'

class Open(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['open'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Open')
        self.pub = rospy.Publisher("gripper", Float32, queue_size=10)
        self.rate = rospy.Rate(10)
        self.pub.publish(0.0)
        sleep(1)
        return 'open'

class Drop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['drop'])
        
    def execute(self, userdata):
        rospy.loginfo('Dropping to cube at zone')
        
        #self.sub = rospy.Subscriber("camera_location", Point32, self.callback)
        
        #wait
        
        return 'drop'
        

def main():  
    rospy.init_node('test_state_machine_2')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['grip'])
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Initial', Initial(), 
                               transitions={'initial':'Move'})
        
        smach.StateMachine.add('Move', Move(), 
                               transitions={'move':'Grip'})
        
        smach.StateMachine.add('Grip', Grip(), 
                               transitions={'grab':'Open'})
                               
        smach.StateMachine.add('Drop', Drop(), 
                               transitions={'drop':'Open'})
                             
        smach.StateMachine.add('Open', Open(), 
                               transitions={'open':'Initial'})
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    print(11)   
    main()
