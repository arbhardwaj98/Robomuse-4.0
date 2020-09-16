#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/camera/depth/image_raw",Image,self.callback,queue_size=None)
    self.depth_sub = rospy.Subscriber("/camera/camera/ir/image_raw",Image,self.depth_callback,queue_size=None)
    self.dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_250)
    self.arucoParams = cv.aruco.DetectorParameters_create()
    self.sift = cv.xfeatures2d.SIFT_create()
    self.FLANN_INDEX_KDTREE = 1
    self.index_params = dict(algorithm = self.FLANN_INDEX_KDTREE, trees = 5)
    self.search_params = dict(checks=50)   # or pass empty dictionary
    self.flann = cv.FlannBasedMatcher(self.index_params,self.search_params)
    self.load_template()
    self.done = False
    self.depth_image = None
    self.cnt = 0


  def callback(self,data):
    self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
    
  def depth_callback(self,data):
    self.cnt = self.cnt+1
    if (self.cnt > 10):
		try:
		  cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		  #cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
		except CvBridgeError as e:
		  print(e)
		
		src_pts, dst_pts = self.sift_detect(cv_image)
		self.cnt = 0
		'''ret, pts = self.aruco_detect(cv_image)
		if ret:
			cv.polylines(cv_image, [np.int32(pts)], True, (0, 0, 255), 2)'''
		cv.imshow("Robot View", cv_image)
		cv.waitKey(3)
  
  def aruco_detect(self, image):
    imgRemapped_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    res = cv.aruco.detectMarkers(imgRemapped_gray, self.dictionary, parameters = self.arucoParams)  
    ret = False
    pts = [] 
    if len(res[0]) > 0: 
      ret = True
      for p in range(4):
        pts.append([res[0][0][0][p][0], res[0][0][0][p][1]])
    return [ret, pts]  

  def sift_detect(self, image):
    imgRemapped_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    keypoints1, desc1 = self.sift.detectAndCompute(image,None)
	
    matches = self.flann.knnMatch(desc1,self.desc2,k=2)
	# store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)
    
    src_pts = np.float32([ keypoints1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ self.keypoints2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
    return [src_pts, dst_pts]
    
  def find_homography()

  def load_template(self):
    img_path = os.path.join("/home/arjun/catkin_ws/src/robomuse-ros-master/robomuse_gazebo/src/marker.jpg") 
    print(img_path)
    template = cv.imread(img_path)
    self.keypoints2, self.desc2 = self.sift.detectAndCompute(template,None)


def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
  main()
