#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import time
from math import pi
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self, gname, ns):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = gname
    description = ns + "/robot_description"
    group = moveit_commander.MoveGroupCommander(group_name, robot_description = description)
    display_trajectory_publisher = rospy.Publisher(ns + '/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_pose_goal(self, pose_goal):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    self.group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = self.group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    self.group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_name_goal(self, name):
    self.group.set_named_target(name)

    ## Now, we call the planner to compute the plan and execute it.
    plan = self.group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.group.stop()
    self.group.clear_pose_targets()
    return True

  def go_to_joint_state(self, joint_state):
    self.group.go(joint_state, wait=True)
    self.group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

def wait(sec):
  c = time.time()
  while(True):
  	if time.time()>c+sec:
  		break

def server():
    rospy.init_node('move_arm_server')
    main()

def main():
  try:
    print("Received Request")
    moveGroup1 = MoveGroupPythonIntefaceTutorial("arm", "robomuse1")
    moveGroup_gripper1 = MoveGroupPythonIntefaceTutorial("gripper", "robomuse1")
    moveGroup2 = MoveGroupPythonIntefaceTutorial("arm", "robomuse2")
    moveGroup_gripper2 = MoveGroupPythonIntefaceTutorial("gripper", "robomuse2")

    moveGroup1.go_to_name_goal("initial")
    moveGroup2.go_to_name_goal("initial")
    wait(10)


    print("Request Completed")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  server()
