#! /usr/bin/env python3
import rospy
from fiducial_msgs.msg import FiducialTransformArray as FTA
import geometry_msgs.msg
from fiducial_msgs.msg import FiducialTransform
import math



def callback(data):
    rospy.loginfo(data.transforms[0].transform.translation.x)

def listener():
    rospy.init_node('fid_listener', anonymous=True)

    rospy.Subscriber("fiducial_transforms", FTA, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
