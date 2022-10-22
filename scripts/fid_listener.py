#! /usr/bin/env python3
import rospy
from fiducial_msgs.msg import FiducialTransformArray as FTA
import geometry_msgs.msg
from fiducial_msgs.msg import FiducialTransform
import numpy as np
from geometry_msgs.msg import Point32
from tf.transformations import euler_from_quaternion


def callback(data):
    rospy.loginfo(data.transforms[0].transform.translation.x)
    rospy.loginfo(data.transforms[0].transform.translation.y)
    pub = rospy.Publisher("camera_location", Point32, queue_size=20)
    rate = rospy.Rate(10)

    
    if len(data.transforms) > 0:
        dist_to_block = []
        for fid in data.transforms:
            orientation = fid.transform.rotation
            orient_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            (roll_next, pitch_next, yaw) = euler_from_quaternion(orient_list)
            
            i_yaw = np.arctan2(fid.transform.translation.y+0.22,fid.transform.translation.x)
            
            
            a_yaw = abs(i_yaw - (yaw % np.radians(90)))
            print(np.degrees(a_yaw))
            dist = np.sqrt((fid.transform.translation.x**2) + ((fid.transform.translation.y+.220))**2)
            if a_yaw < np.radians(35) or a_yaw > np.radians(55) or dist < 0.9*0.2125:
                dist_to_block.append(dist)
            else:
                dist_to_block.append(10000)
        print(dist_to_block)
        cb = np.argmin(dist_to_block)
        
        offset_translation_x = float(data.transforms[cb].transform.translation.x*-1000)
        offset_translation_y = float(data.transforms[cb].transform.translation.y*1000 + 220)
        offset_translation_z = float(20)
        print(offset_translation_x,offset_translation_y,offset_translation_z)
        pub.publish(offset_translation_x,offset_translation_y,offset_translation_z)
        

def listener():
    rospy.init_node("fid_listener", anonymous=True)
    pub = rospy.Publisher("camera_location", Point32, queue_size=20)

    global yaw
    yaw = 0
    rospy.Subscriber("fiducial_transforms", FTA, callback)
    

    rospy.spin()


if __name__ == "__main__":
    listener()
