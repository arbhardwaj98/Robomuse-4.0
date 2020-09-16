#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class commander:

	def callback(self, data):

	    rospy.loginfo(data)
	    self.pub0.publish(data.position[0])
	    self.pub5.publish(data.position[5])
	    self.pub6.publish(data.position[6])
	    self.pub7.publish(data.position[7])
	    self.pub8.publish(data.position[8])
	    
	def __init__(self):

	    rospy.init_node('joint_commander', anonymous=True)

	    
	    self.pub0 = rospy.Publisher('/phantomx_reactor_controller/elbow_pitch_joint/command', Float64, queue_size=10)
	    self.pub5 = rospy.Publisher('/phantomx_reactor_controller/shoulder_pitch_joint/command', Float64, queue_size=10)
	    self.pub6 = rospy.Publisher('/phantomx_reactor_controller/shoulder_yaw_joint/command', Float64, queue_size=10)
	    self.pub7 = rospy.Publisher('/phantomx_reactor_controller/wrist_pitch_joint/command', Float64, queue_size=10)
	    self.pub8 = rospy.Publisher('/phantomx_reactor_controller/wrist_roll_joint/command', Float64, queue_size=10)

	    

	def listener(self):
		rospy.Subscriber("joint_states", JointState, self.callback)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

if __name__ == '__main__':
    try:
        obj = commander()
        obj.listener()
    except rospy.ROSInterruptException:
        pass

