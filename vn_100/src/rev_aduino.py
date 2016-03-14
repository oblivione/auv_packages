#! /usr/bin/env python

import rospy
import serial
import time 
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('depth_val', Float64, queue_size=10)
    rospy.init_node('rev_arduino', anonymous=True)
    s = serial.Serial('/dev/ttyACM0',9600,timeout=.1)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        data = s.readline()
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
