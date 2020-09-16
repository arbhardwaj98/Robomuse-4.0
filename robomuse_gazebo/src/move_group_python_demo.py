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

pose_initial = geometry_msgs.msg.Pose()
p1x = 0.217399385107
p1y = -0.00704575232199
p1z = 0.171889549293
p1_ox = 0.543197344747
p1_oy = 0.0075421261867
p1_oz = -0.839443910018
p1_ow = 0.0146178974543
pose_initial.position.x = p1x
pose_initial.position.y = p1y
pose_initial.position.z = p1z
pose_initial.orientation.x = p1_ox
pose_initial.orientation.y = p1_oy
pose_initial.orientation.z = p1_oz
pose_initial.orientation.w = p1_ow

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
  def __init__(self, gname):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_demo',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = gname
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()
    print "============ Printing robot state ============="
    print robot.get_current_state()
    print ""
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    if gname == "arm":
	    current_pose = self.group.get_current_pose().pose
	    print 1
	    print current_pose
	    print 2
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def plan_to_pose_goal(self, pose_goal):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.plan(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose
    return [plan, current_pose]

  def go_to_pose_goal(self, pose_goal):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_joint_state(self, joint_goal):
    group = self.group
    #joint_goal = group.get_current_joint_values()
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal_2, current_joints, 0.01)

  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x -= scale * 0.1  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x -= scale * 0.01  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z += scale * 0.05  # Third move sideways (y)
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
    moveGroup = MoveGroupPythonIntefaceTutorial("arm")
    moveGroup_gripper = MoveGroupPythonIntefaceTutorial("gripper")

    #moveGroup.go_to_pose_goal(1)
    plan, fraction = moveGroup.plan_cartesian_path(1)
    print("fraction = " + str(fraction))
    print(plan)
    moveGroup.display_trajectory(plan)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
