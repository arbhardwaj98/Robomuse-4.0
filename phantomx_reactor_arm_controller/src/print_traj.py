#!/usr/bin/env python
import rospy
from moveit_msgs.msg import DisplayTrajectory

def callback(traj):
    points = traj.trajectory[0].joint_trajectory.points
    for i in range(1,len(points)):
        rospy.sleep(points[i].time_from_start-points[i-1].time_from_start)
        print(points[i].time_from_start.to_sec())
        print("***")
        print(points[i].positions)
        print("-----------------------------")

def listener():
    rospy.init_node('joint_commander', anonymous=True)
    traj = rospy.wait_for_message("move_group/display_planned_path", DisplayTrajectory)
    callback(traj)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
