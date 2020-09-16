#!/usr/bin/env python
import roslib
import rospy
import tf
from nav_msgs.msg import Odometry

def handle_robomuse_pose(msg):
    br = tf.TransformBroadcaster()
    data = msg.pose.pose
    quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    br.sendTransform((data.position.x, data.position.y, data.position.z),
                     quat,
                     rospy.Time.now(),
                     "base_footprint",
                     "world")

if __name__ == '__main__':

    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('/robomuse/base_pose_ground_truth',
                     Odometry,
                     handle_robomuse_pose)
    rospy.spin()
