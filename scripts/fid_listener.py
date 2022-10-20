#! /usr/bin/env python3
import rospy
from fiducial_msgs.msg import FiducialTransformArray as FTA
import geometry_msgs.msg
from fiducial_msgs.msg import FiducialTransform
import math
import numpy as np
from geometry_msgs.msg import Point32


def callback(data):
    rospy.loginfo(data.transforms[0].transform.translation.x)
    rospy.loginfo(data.transforms[0].transform.translation.y)
    pub = rospy.Publisher("cube_location", Point32, queue_size=20)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    
        if len(data.transforms) > 0:
            offset_translation_x = float(data.transforms[0].transform.translation.x*-1000)
            offset_translation_y = float(data.transforms[0].transform.translation.y*1000 + 225)
            offset_translation_z = float(20)
            print(offset_translation_x,offset_translation_y,offset_translation_z)
            pub.publish(offset_translation_x,offset_translation_y,offset_translation_z)
            
        rate.sleep()

def listener():
    rospy.init_node('fid_listener', anonymous=True)
    pub = rospy.Publisher("cube_location", Point32, queue_size=20)
    rospy.Subscriber("fiducial_transforms", FTA, callback)
    

    rospy.spin()


if __name__ == '__main__':
    listener()
