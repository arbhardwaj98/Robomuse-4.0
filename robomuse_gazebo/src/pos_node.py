#!/usr/bin/env python
# license removed for brevity
import rospy
import geometry_msgs.msg
from std_msgs.msg import String
import time

def wait(sec):
  c = time.time()
  while(True):
    if time.time()>c+sec:
        break


def talker():
    pub = rospy.Publisher('/robomuse/object_pos', geometry_msgs.msg.Point, queue_size=10)
    rospy.init_node('fake_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    p = geometry_msgs.msg.Point()
    p.x = 332
    p.y = 560
    p.z = 12
    pub.publish(p)
    wait(13)
    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
