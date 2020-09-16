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
   ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/robomuse/cmd_vel", 1000);
  geometry_msgs::Twist myTwistMsg;
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
  goal.target_pose.pose.position.x = -2.71;
  goal.target_pose.pose.position.y = 2.04;
  goal.target_pose.pose.orientation.z = 0.9605;
  goal.target_pose.pose.orientation.w = 0.278;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");
  
  goal.target_pose.pose.position.x = -6.97;
  goal.target_pose.pose.position.y = 4.74;
  goal.target_pose.pose.orientation.z = 0.9605;
  goal.target_pose.pose.orientation.w = 0.278;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");

  goal.target_pose.pose.position.x = -11.53;
  goal.target_pose.pose.position.y = 9.62;
  goal.target_pose.pose.orientation.z = 0.9605;
  goal.target_pose.pose.orientation.w = 0.278;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");

  goal.target_pose.pose.position.x = -15.79;
  goal.target_pose.pose.position.y = 11.18;
  goal.target_pose.pose.orientation.z = 0.9605;
  goal.target_pose.pose.orientation.w = 0.278;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");

  goal.target_pose.pose.position.x = -19.71;
  goal.target_pose.pose.position.y = 13.35;
  goal.target_pose.pose.orientation.z = 0.9605;
  goal.target_pose.pose.orientation.w = 0.278;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");

    goal.target_pose.pose.position.x = -19.71;
  goal.target_pose.pose.position.y = 13.35;
  goal.target_pose.pose.orientation.z = 1;
  goal.target_pose.pose.orientation.w = 1;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");

  ros::Time begiTime = ros::Time::now();
  ros::Duration delay2(5);
  
  while(ros::Time::now() - begiTime <= delay2 )
  {
    pub.publish(myTwistMsg);
    ros::Duration(0.1).sleep();
  }

  myTwistMsg.linear.x = 0;
  myTwistMsg.linear.y = 0;
  myTwistMsg.linear.z = 0;

  myTwistMsg.angular.x = 0;
  myTwistMsg.angular.y = 0;
  myTwistMsg.angular.z = 0.2;

  ros::Time beginTime = ros::Time::now();
  ros::Duration delay1(31.4);
  
  while(ros::Time::now() - beginTime <= delay1 )
  {
    pub.publish(myTwistMsg);
    ros::Duration(0.1).sleep();
  }
  
  goal.target_pose.pose.position.x = -13.79;
  goal.target_pose.pose.position.y = 11.18;
  goal.target_pose.pose.orientation.z = -0.266;
  goal.target_pose.pose.orientation.w = 0.9639;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");

  goal.target_pose.pose.position.x = -10.60;
  goal.target_pose.pose.position.y = 9.23;
  goal.target_pose.pose.orientation.z = -0.266;
  goal.target_pose.pose.orientation.w = 0.9639;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");

  goal.target_pose.pose.position.x = -5.40;
  goal.target_pose.pose.position.y = 5.053;
  goal.target_pose.pose.orientation.z = -0.266;
  goal.target_pose.pose.orientation.w = 0.9639;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");

  goal.target_pose.pose.position.x = 0;
  goal.target_pose.pose.position.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("reached");
  else
    ROS_INFO("not reached");

  return 0;
}
