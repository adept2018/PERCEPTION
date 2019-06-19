#!/usr/bin/env python
import importlib

import rospy
import genpy.message
from rospy import ROSException
import sensor_msgs.msg
import ackermann_msgs.msg
import actionlib
import rostopic
import rosservice
from threading import Thread
from rosservice import ROSServiceException

import numpy as np

package, message = "ackermann_msgs/AckermannDriveStamped".split('/')
mod = importlib.import_module(package + '.msg')
etype = getattr(mod, message)

def talker():
    pub = rospy.Publisher('low_level/ackermann_cmd_mux/input/teleop', etype, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        msg.drive = ackermann_msgs.msg.AckermannDrive(-0.3,0.0,-0.2,0.0,0.0)
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
