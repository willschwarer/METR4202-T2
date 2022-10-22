#! /usr/bin/env python3
import rospy
from fiducial_msgs.msg import FiducialTransformArray as FTA
from fiducial_msgs.msg import FiducialTransform
import numpy as np
from geometry_msgs.msg import Point32
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32

class Block_Transforms:
    def obstruction(self, x1, y1, x2, y2):
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2) < 50

    def callback(self, data):
        
        if len(data.transforms) > 0:
            dist_to_block = []
            desired = []
            for fid in data.transforms:
                orientation = fid.transform.rotation
                orient_list = [orientation.x, orientation.y, orientation.z, orientation.w]
                (roll, pitch, yaw) = euler_from_quaternion(orient_list)
                
                i_yaw = np.arctan2(fid.transform.translation.y + 0.22, fid.transform.translation.x)
                
                
                a_yaw = abs(i_yaw - (yaw % np.radians(90)))
                print(np.degrees(a_yaw))
                dist = np.sqrt((fid.transform.translation.x**2) + ((fid.transform.translation.y + 0.22))**2)
                if a_yaw < np.radians(35) or a_yaw > np.radians(55) or dist < 0.9 * 0.2125:

                    dist_to_block.append(dist)
                    desired.append(fid)
                
                original_length = len(desired)
                pop_list = []
                for i in range(1, original_length):
                    for j in range(i):
                        if self.obstruction(desired[i].transform.translation.x, desired[i].transform.translation.y + 0.22, 
                        desired[j].transform.translation.x, desired[j].transform.translation.y + 0.22):
                            pop_list.append(i)
                            pop_list.append(j)

                pop_list = sorted(list(dict.fromkeys(pop_list)), reverse=True)
                for i in pop_list:
                    desired.pop(i)
                    dist_to_block.pop(i)

            print(dist_to_block)
            try:
                cb = np.argmin(dist_to_block)
                
                offset_translation_x = float(desired[cb].transform.translation.x * -1000)
                offset_translation_y = float(desired[cb].transform.translation.y * 1000 + 220)
                offset_translation_z = float(20)
                print(offset_translation_x,offset_translation_y,offset_translation_z)
                self.counter_pub.publish(len(desired))
                self.location_pub.publish(offset_translation_x,offset_translation_y,offset_translation_z)
            except ValueError:
                self.counter_pub.publish(0)
                print("All on angle")

        else:
            self.counter_pub.publish(0)
            

    def block_transforms(self):
        rospy.init_node("block_transforms", anonymous=True)
        self.location_pub = rospy.Publisher("camera_location", Point32, queue_size=20)
        self.counter_pub = rospy.Publisher("block_counter", Int32, queue_size=20)
        rospy.Subscriber("fiducial_transforms", FTA, self.callback)
        rate = rospy.Rate(10)
        rospy.spin()


if __name__ == "__main__":
    BT = Block_Transforms()
    BT.block_transforms()