#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void my_handler (int s)
{
 //system("gnome-terminal");
 system("cd ~/ && ./dock.sh");
 while(1);
 exit(1);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);


  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::Point initpose;
  
  //int x1 = initpose.x;
  //while(x1 == initpose.x);

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  int i=0;
while(1){
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = -3.0;
  goal.target_pose.pose.orientation.z = -0.71;
  goal.target_pose.pose.orientation.w = 0.70;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");
  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.position.y = -5.3;
  goal.target_pose.pose.orientation.z = -0.60;
  goal.target_pose.pose.orientation.w = 0.79;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");

goal.target_pose.pose.position.x = 0.74;
  goal.target_pose.pose.position.y = -10.1;
  goal.target_pose.pose.orientation.z = -0.99;
  goal.target_pose.pose.orientation.w = 0.03;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");
  goal.target_pose.pose.position.x = -5.79;
  goal.target_pose.pose.position.y = -9.0;
  goal.target_pose.pose.orientation.z = -0.959;
  goal.target_pose.pose.orientation.w = 0.28;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");
  goal.target_pose.pose.position.x = -10.92;
  goal.target_pose.pose.position.y = -13.69;
  goal.target_pose.pose.orientation.z = -0.398;
  goal.target_pose.pose.orientation.w = 0.918;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");
  goal.target_pose.pose.position.x = -10.53;
  goal.target_pose.pose.position.y = -6.39;
  goal.target_pose.pose.orientation.z = -4.49;
  goal.target_pose.pose.orientation.w = -0.878;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");
    i++;
}
  return 0;
}
