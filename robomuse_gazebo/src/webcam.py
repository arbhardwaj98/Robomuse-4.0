#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/screen/image_topic",Image,queue_size=10)
    self.bridge = CvBridge()


  def pub_img(self,cv_image):
    (rows,cols,channels) = cv_image.shape
    #print(rows)
    #print(cols)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def mainloop(cap,ic):
  ret, frame = cap.read()
  if ret:
    ic.pub_img(frame)

if __name__ == '__main__':
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  cap = cv.VideoCapture(0)
  while not rospy.is_shutdown():
    mainloop(cap,ic)
  cap.release()
