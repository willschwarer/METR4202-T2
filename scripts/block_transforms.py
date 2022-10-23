#! /usr/bin/env python3
import rospy
from fiducial_msgs.msg import FiducialTransformArray as FTA
from fiducial_msgs.msg import FiducialTransform
import numpy as np
from geometry_msgs.msg import Point32
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32

y_cam = 0.19

class Block_Transforms:
    def obstruction(self, x1, y1, x2, y2):
    #    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2) < 0.05
        block_len = 32/1000 # mm

        dist_between = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        dist1 = np.sqrt(x1**2 + y1**2)

        dist2 = np.sqrt(x2**2 + y2**2)

        if dist_between < 1.5 * block_len:
            # Block is inside a potential collision radius
            if dist2 < dist1 + 0.5 * block_len:
                # Block2 is NOT behind block 1 and is a collison
                return True

        return False
  
    def callback(self, data):
        
        if len(data.transforms) > 0:
            desired = data.transforms
            original_length = len(desired)
            pop_list = []
            for i in range(1, original_length):
                for j in range(i):
                    if self.obstruction(desired[i].transform.translation.x, desired[i].transform.translation.y + y_cam, 
                    desired[j].transform.translation.x, desired[j].transform.translation.y + y_cam):
                        pop_list.append(i)
                        pop_list.append(j)

            pop_list = sorted(list(dict.fromkeys(pop_list)), reverse=True)
            print("Pop List", pop_list)
            for i in pop_list:
                desired.pop(i)

            
            dist_to_block = []
            felicitous = [] #look up what this means
            for fid in desired:
                orientation = fid.transform.rotation
                orient_list = [orientation.x, orientation.y, orientation.z, orientation.w]
                (roll, pitch, yaw) = euler_from_quaternion(orient_list)
                
                i_yaw = np.arctan2(fid.transform.translation.y + y_cam, fid.transform.translation.x)
                
                a_yaw = abs(i_yaw - (yaw % np.radians(90)))
                print(np.degrees(a_yaw))
                dist = np.sqrt((fid.transform.translation.x**2) + ((fid.transform.translation.y + y_cam))**2)
                if a_yaw < np.radians(35) or a_yaw > np.radians(55) or dist < 0.9 * 0.2125:
                    dist_to_block.append(dist)
                    felicitous.append(fid)
                
            
            try:
                cb = np.argmin(dist_to_block)
                
                offset_translation_x = float(felicitous[cb].transform.translation.x * -1000)
                offset_translation_y = float(felicitous[cb].transform.translation.y * 1000 + y_cam * 1000)
                offset_translation_z = float(25)
                print(offset_translation_x,offset_translation_y, offset_translation_z)
                self.counter_pub.publish(len(felicitous))
                self.location_pub.publish(offset_translation_x, offset_translation_y, offset_translation_z)
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
