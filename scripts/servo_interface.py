#!/usr/bin/env python3

import pigpio
import rospy
from std_msgs.msg import Float32

def callback(data):
    """
    Takes a value between 0 & 2 from gripper node
    and changes the PWM signal for gripper servo
    0 is fully open (2ms pusle width)
    2 is fully closed (1ms pulse width
    """

    global rpi
    val = data.data
    if val < 0:
        val = 0
    elif val > 2:
        val = 2
    # Using gpio 18 on RPI to send PWM signal to Servo
    rpi.set_servo_pulsewidth(18, int(2000 - val * 500))
    rospy.loginfo(rospy.get_caller_id() + " New Val:" + str(val * 500 + 1000))

def main():
    """
    Setup subscriber and gpio settings
    """

    global rpi
    rospy.init_node("servo_interface_node", anonymous=True)
    rospy.Subscriber("gripper", Float32, callback)
    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT)
    rospy.spin()

if __name__ == "__main__":
    main()
