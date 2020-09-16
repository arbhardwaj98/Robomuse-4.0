#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
import geometry_msgs.msg
import numpy as np
import math
import time
from cv_bridge import CvBridge, CvBridgeError

class move_base:

  def __init__(self):
    self.pos_sub = rospy.Subscriber("/robomuse/object_pos",geometry_msgs.msg.Point,self.callback,queue_size=None)
    self.pos_pub = rospy.Publisher('/robomuse/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    self.turnspeed = float(0.9)
    self.movespeed = float(1.2)

  def callback(self,data):
    
    row = data.x
    dist = data.z
    F = 1178.7329083650782
    angle = math.atan(abs(row)/F)
    self.turn_angle(3.14)
    self.move_dist(11)
    
  def turn_angle(self, angle):
    tm = angle/self.turnspeed
    print(tm)
    vel = geometry_msgs.msg.Twist()
    vel_s = geometry_msgs.msg.Twist()
    vel.angular.z = self.turnspeed
    self.pos_pub.publish(vel)
    rospy.sleep(tm)
    self.pos_pub.publish(vel_s)
   
  def move_dist(self, dist):
    margin = 0.5
    tm = (dist-margin)/self.movespeed
    print(tm)
    vel = geometry_msgs.msg.Twist()
    vel_s = geometry_msgs.msg.Twist()
    vel.linear.x = self.movespeed
    c = time.time()
    while(True):
      self.pos_pub.publish(vel)
      if time.time()>c+tm:
        break
    self.pos_pub.publish(vel_s)


def main():
  ic = move_base()
  rospy.init_node('move_bot', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
  main()
