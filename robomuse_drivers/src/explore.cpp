#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <frontier_exploration/ExploreTaskAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "frontier_exploration_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> ac("explore_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  frontier_exploration::ExploreTaskGoal goal;

  geometry_msgs::PolygonStamped boundary;
  geometry_msgs::Point32 p1, p2, p3, p4, p5;
  float dim = 15.0;  

  p1.x = dim/2; p1.y = dim/2; p1.z = 0.0;
  p2.x = dim/2; p2.y = -dim/2; p2.z = 0.0;
  p3.x = -dim/2; p3.y = -dim/2; p3.z = 0.0;
  p4.x = -dim/2; p4.y = dim/2; p4.z = 0.0;
  p5.x = dim/2; p5.y = dim/2; p5.z = 0.0;

  boundary.polygon.points.reserve(5);
  boundary.header.seq = 0;
  boundary.header.stamp = ros::Time::now();
  boundary.header.frame_id = "map";
  boundary.polygon.points.push_back(p1);
  boundary.polygon.points.push_back(p2);
  boundary.polygon.points.push_back(p3);
  boundary.polygon.points.push_back(p4);
  boundary.polygon.points.push_back(p5);


  goal.explore_boundary = boundary;

  goal.explore_center.header.frame_id = "map";
  goal.explore_center.point.x = 1;
  goal.explore_center.point.y = 0;
  goal.explore_center.point.z = 0;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));


  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
