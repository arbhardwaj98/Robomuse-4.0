#!/usr/bin/env python

import sys
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from phantomx_reactor_arm_controller.srv import UpdatePos

def UpdatePosClient():
    rospy.init_node("UpdatePosClient", anonymous=True)
    rospy.wait_for_service('UpdatePos')
    print ('1a')
    joint_states = rospy.wait_for_message("joint_states", JointState, timeout=None)
    print ('1b')
    try:
        update_pos = rospy.ServiceProxy('UpdatePos', UpdatePos)
        resp = update_pos(joint_states)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def wait(sec):
  c = time.time()
  while(True):
  	if time.time()>c+sec:
  		break

if __name__ == "__main__":

    print("Requesting to move PhantomX Reactor Manipulator")
    wait(10)
    if UpdatePosClient() == 0:
        print("Goal position achieved successfully!")
    else:
        print("Could not move PhantomX Reactor Manipulator")
