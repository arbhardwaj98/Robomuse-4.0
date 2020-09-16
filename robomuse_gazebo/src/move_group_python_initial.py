#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
import time
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

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
      	print ">>>>>>>>>>>>>"
      	print abs(actual[index] - goal[index])
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    self.define_positions()
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    current_pose = self.group.get_current_pose().pose
    print 1
    print current_pose
    print 2
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def define_positions(self):
    self.pose_initial = geometry_msgs.msg.Pose()
    p1x = 0.217399385107
    p1y = -0.00704575232199
    p1z = 0.171889549293
    p1_ox = 0.543197344747
    p1_oy = 0.0075421261867
    p1_oz = -0.839443910018
    p1_ow = 0.0146178974543
    self.pose_initial.position.x = p1x
    self.pose_initial.position.y = p1y
    self.pose_initial.position.z = p1z
    self.pose_initial.orientation.x = p1_ox
    self.pose_initial.orientation.y = p1_oy
    self.pose_initial.orientation.z = p1_oz
    self.pose_initial.orientation.w = p1_ow

    self.pose_pick1 = geometry_msgs.msg.Pose()
    p1x = 0.68721289542
    p1y = 0.0830350698565
    p1z = 0.341110692822
    p1_ox = 0.0121110425908
    p1_oy = -0.074038232709
    p1_oz = 0.117837142561
    p1_ow = 0.990194965942
    self.pose_pick1.position.x = p1x
    self.pose_pick1.position.y = p1y
    self.pose_pick1.position.z = p1z
    self.pose_pick1.orientation.x = p1_ox
    self.pose_pick1.orientation.y = p1_oy
    self.pose_pick1.orientation.z = p1_oz
    self.pose_pick1.orientation.w = p1_ow

    self.pose_pick2 = geometry_msgs.msg.Pose()
    p1x = 0.714181166386
    p1y = 0.0896521793088
    p1z = 0.184099785793
    p1_ox = -0.00710126495245
    p1_oy = 0.100855704719
    p1_oz = 0.116983780396
    p1_ow = 0.98797393386
    self.pose_pick2.position.x = p1x
    self.pose_pick2.position.y = p1y
    self.pose_pick2.position.z = p1z
    self.pose_pick2.orientation.x = p1_ox
    self.pose_pick2.orientation.y = p1_oy
    self.pose_pick2.orientation.z = p1_oz
    self.pose_pick2.orientation.w = p1_ow

  def go_to_pose_goal(self, posint):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()

    if posint == 0:
    	pose_goal = self.pose_initial
    elif posint == 1:
    	pose_goal = self.pose_pick1
    elif posint == 2:
    	pose_goal = self.pose_pick2

    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    group = self.group
    group.execute(plan, wait=True)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

def wait(sec):
  c = time.time()
  while(True):
    if time.time()>c+sec:
      break

def main():
  try:
    moveGroup = MoveGroupPythonIntefaceTutorial()
    moveGroup.go_to_pose_goal(0)
    wait(6)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

