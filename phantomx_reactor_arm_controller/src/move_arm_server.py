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
from robomuse_gazebo.srv import MoveArm
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from phantomx_reactor_arm_controller.srv import UpdatePos

name_list = ["initial", "pose_eat1", "pose_eat2", "pose_eat3"]

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
  def __init__(self, gname):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = gname
    group = moveit_commander.MoveGroupCommander(group_name, robot_description = "robot_description")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
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
def UpdatePosClient(data):
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState, timeout=None)
        #joint_states.position[0] = data
        update_pos = rospy.ServiceProxy('UpdatePos', UpdatePos)
        resp = update_pos()
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def server():
    rospy.init_node("UpdatePosClient", anonymous=True)
    rospy.wait_for_service('UpdatePos')
    print("Server Up")
    handler(1)

def handler(data):
  try:
    print("Received Request")
    moveGroup = MoveGroupPythonIntefaceTutorial("arm")
    moveGroup_gripper = MoveGroupPythonIntefaceTutorial("gripper")
    wait(10)
    moveGroup.go_to_name_goal("initial")
    UpdatePosClient(2)
    wait(6)

    UpdatePosClient(1) #gripper_open

    print("Request Completed")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  server()
