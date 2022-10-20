#!/usr/bin/env python3


import pigpio
import rospy

from std_msgs.msg import Float32

def callback(data):
    global rpi
    val = data.data
    if val < 0:
        val = 0
    elif val > 2:
        val = 2

    rpi.set_servo_pulsewidth(18, int(2000- val * 500))
    rospy.loginfo(rospy.get_caller_id() + " New Val:" + str(val * 500 + 1000))
    


def main():
    global rpi
    rospy.init_node("servo_interface_node", anonymous=True)
    rospy.Subscriber("gripper", Float32, callback)
    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT)
    rospy.spin()


if __name__ == "__main__":
    main()
