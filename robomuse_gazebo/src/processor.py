#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2 as cv
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from PIL import Image as ImageModule
import numpy as np
import face_recognition
import os
import pickle
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
    self.image_pub = rospy.Publisher("/robot/processed_image",Bool,queue_size=10)
    self.image_pos_pub = rospy.Publisher("/robot/processed_image_pos",Point,queue_size=10)
    self.cnt = 0
  def callback(self,data):
    try:
      print(1)
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    state, pixelpos = face_detection(cv_image)
    if state and self.cnt<10:
        self.cnt = self.cnt+1
    if not state and self.cnt>0:
        self.cnt = self.cnt-1

    self.image_pub.publish(self.cnt>=5)
    if self.cnt>=5 and state:
        pos = Point()
        pos.y = -1*pixelpos[0]/400.0 + 0.4; pos.z = -1*pixelpos[1]/400.0 + 0.3
        self.image_pos_pub.publish(pos)

def face_detection(frame):

  small_frame = cv.resize(frame, (0, 0), fx=0.5, fy=0.5)
  rgb_small_frame = small_frame[:, :, ::-1]
  face_landmarks_list = face_recognition.face_landmarks(rgb_small_frame)
  state=False
  pixelpos = np.zeros((1,2))
  if len(face_landmarks_list)>0:
      face_landmarks = face_landmarks_list[0]
      upperlip = np.array(face_landmarks['top_lip'])
      lowerlip = np.array(face_landmarks['bottom_lip'])
      ulpos = np.mean(upperlip[[8,9],1])
      llpos = np.mean(lowerlip[[8,9],1])
      ulw = ulpos- np.mean(upperlip[[2,3],1])
      llw = np.mean(lowerlip[[2,3],1])-llpos
      dist = llpos-ulpos
      pixelpos = (upperlip[8,:] + upperlip[9,:] + lowerlip[8,:] + lowerlip[9,:])/4.0
      threshold = 0.4*(ulw+llw)
      if dist>=threshold:
          state=True
  return (state, pixelpos)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  print(1)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
