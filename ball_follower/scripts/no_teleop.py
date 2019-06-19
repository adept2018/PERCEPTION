#!/usr/bin/env python
import importlib

import sys
import rospy
import genpy.message
from rospy import ROSException
import sensor_msgs.msg
import ackermann_msgs.msg
import actionlib
import rostopic
import rosservice
import math
from ball_follower.msg import ballpos
from threading import Thread
from rosservice import ROSServiceException

import numpy as np

package, message = "ackermann_msgs/AckermannDriveStamped".split('/')
mod = importlib.import_module(package + '.msg')
etype = getattr(mod, message)

class no_teleop:

  def __init__(self):
    self.ball_distance_input = 0
    self.ball_angle_input = 0
    self.ballpos_sub = rospy.Subscriber("/output",ballpos,self.callback)
    self.pub = rospy.Publisher('low_level/ackermann_cmd_mux/input/teleop', etype, queue_size=10)
    rospy.init_node('no_teleop',anonymous=True)

  def callback(self, data):
    if data.distance < 1000:
      self.ball_distance_input = data.distance
    else:
      self.ball_distance_input = 0

    if data.angle > -10 and data.angle < 10:
      self.ball_angle_input = data.angle
    #else:
    #  self.ball_angle_input = 0

  def output(self):
    rate = rospy.Rate(25) # 10hz
    while not rospy.is_shutdown():
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        vals = self.control()
        msg.drive = ackermann_msgs.msg.AckermannDrive(vals[0],vals[1],vals[2],vals[3],vals[4])
        self.pub.publish(msg)
        rate.sleep()

  def control(self):
  #Vehicle control code:

    dist_ball = self.ball_distance_input*1000 #Distance to the boll from left camera [mm]
    vhcl_spd = 0 #Vehicle speed
    vhcl_acc = 0 #Vehicle acceleration
    vhcl_jerk = 0 #Vehicle jerk
    steer_ang = 0 #Steer angle, i.e. angle between the vehicle axis and the line between the camera and the ball center
    steer_ang_spd = 0.01 #Steer wheel angular speed (turning the wheels)

    theta_vhcl = 0
    alpha_steer = 0 #From motor controller
    alpha_cam = self.ball_angle_input  #From camera

    dc = 60 #Distance from camera to vehicle center
    excl_dist = 500 #Exclusion distance from the ball
    vhcl_stop_v = 0
    steer_ang_stop = 0
    steer_ang_spd_max = 0.75 #pi/4/s
    dist_max = 10000 #Above this limit we consider the ball out of range [mm]
    vhcl_spd_max = 0.7
    vhcl_acc_max = 0
    steer_angle_max = 0.75 #pi/4
    vhcl_jerk_max = 0

    if dist_ball < excl_dist or dist_ball > dist_max:
      vhcl_spd = vhcl_stop_v
      steer_ang = steer_ang_stop
      vhcl_acc = 0
      vhcl_jerk = 0
      steer_ang_spd = 0
    else:
      theta_vhcl = math.asin(((dist_ball * alpha_cam) + dc)/dist_ball)
      steer_ang = theta_vhcl - alpha_steer

      # Might be needed if we can get steering feedback from vesc
      #if theta_vhcl > 0 and steer_ang < 0:
        #steer_ang = -1 * steer_ang
      #elif theta_vhcl < 0 and steer_ang > 0:
        #steer_ang = -1 * steer_ang

      vhcl_spd = min(vhcl_spd_max, (math.tanh(dist_ball/1000)/2))
      vhcl_acc = vhcl_acc_max
      vhcl_jerk = vhcl_jerk_max
      steer_ang_spd = steer_ang_spd_max
      rospy.loginfo("%f speed is ", vhcl_spd)

    return [steer_ang, steer_ang_spd,vhcl_spd, vhcl_acc, vhcl_jerk]

def main(args):
  ic = no_teleop()
  try:
    ic.output()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == "__main__":
  main(sys.argv)




