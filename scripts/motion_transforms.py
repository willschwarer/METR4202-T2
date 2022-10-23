#! /usr/bin/env python3
import rospy
from fiducial_msgs.msg import FiducialTransformArray as FTA
from fiducial_msgs.msg import FiducialTransform
import numpy as np
from geometry_msgs.msg import Point32
from std_msgs.msg import Int32

y_cam = 0.19 #0.235
y_centre = 0 #.045

class Motion_Transforms:
    def callback(self, data):
        if len(data.transforms) == 0:
            self.counter_pub.publish(0)
            return
        position = data.transforms[0].transform.translation
        theta_new = np.arctan(position.y/position.x)
        radius = []
        if (theta_new - self.theta) < 0: #clockwise
                direction = 1
        else:
            direction = 0 #anti or stopped

        print(direction, theta_new, self.theta)
        self.theta = theta_new

        for block in data.transforms:
            if direction == 1:
                if block.transform.translation.x < 0 and (block.transform.translation.y + y_centre) > 0: #quad 2
                    r = np.sqrt(block.transform.translation.x**2 + (block.transform.translation.y + y_centre)**2)
                    radius.append(r)
            else:
                if block.transform.translation.x > 0 and (block.transform.translation.y+y_centre) > 0: #quad 1
                    r = np.sqrt(block.transform.translation.x**2 + (block.transform.translation.y + y_centre)**2)
                    radius.append(r)

        try:
            cb = np.argmin(radius)
            if direction == 0:
                offset_translation_x = float(radius[cb]* 1000)
            else:
                offset_translation_x = float(radius[cb]* -1000)
            offset_translation_y = float(190)
            offset_translation_z = float(20)
            self.counter_pub.publish(len(radius))
            self.location_pub.publish(offset_translation_x, offset_translation_y, offset_translation_z)
            print("Update Location")
        except ValueError:
            print("Value Error")
            self.counter_pub.publish(0)

    def motion_transforms(self):
        rospy.init_node("motion_transforms", anonymous=True)
        self.location_pub = rospy.Publisher("camera_location", Point32, queue_size=20)
        self.counter_pub = rospy.Publisher("block_counter", Int32, queue_size=20)
        self.theta = 0
        rospy.Subscriber("fiducial_transforms", FTA, self.callback)
        rate = rospy.Rate(10)
        rospy.spin()


if __name__ == "__main__":
    MT = Motion_Transforms()
    MT.motion_transforms()
