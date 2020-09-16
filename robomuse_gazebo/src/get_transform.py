#!/usr/bin/env python
import rospy
import math
import tf
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from robomuse_gazebo.srv import *

def callback2(msg):
    global pos
    pos = msg

def callback1(msg):
    global state
    state = msg.data

def move_arm_client(pn):
    global moving
    rospy.wait_for_service('move_arm')
    try:
        pose = Pose()
        moving = True
        pose.position.x = pn
        ser = rospy.ServiceProxy('move_arm', MoveArm)
        resp1 = ser(pose)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':

    global moved, state, pos, moving

    moved = False
    moving = False
    state = False
    pos = Point()
    pn = [2, 1, 3]
    cnt = 0
    rospy.init_node('screen_tf_listener')
    rospy.Subscriber("/robot/processed_image",Bool,callback1)
    rospy.Subscriber("/robot/processed_image_pos",Point,callback2)
    listener = tf.TransformListener()

    while not rospy.is_shutdown():

        if state and not moved:
            moved = True
            try:
                tp, quat = listener.lookupTransform('arm_base_link', 'display_screen_link', rospy.Time(0))
                print("moved to pos:")
                goal = [tp[0]+pos.x, tp[1]+pos.y, tp[2]+pos.z]
                print(pn[cnt])
                move_arm_client(pn[cnt])
                cnt+=1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        elif moved and not state:
            moved = False
            print("moved Back")
            move_arm_client(0)
            print(0)
            if cnt == 3: exit()
