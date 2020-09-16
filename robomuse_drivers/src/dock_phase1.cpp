#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_listener.h>
#include<math.h>

#define PI 3.14159265
tf::StampedTransform transform;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robomuse_Dock_Phase1");
  ros::NodeHandle node_handle;
  bool obj_found = 0;
  tf::TransformListener listener;
  
  ros::Time t = ros::Time(0);
  double x1,y1,z1,x2,y2,z2,x_mid,y_mid,x,y;
  try{
    listener.waitForTransform("/base_link", "/object_28", t, ros::Duration(4.0));
    listener.lookupTransform("/base_link", "/object_28", t, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  x1 = transform.getOrigin().getX();
  y1 = transform.getOrigin().getY();
  z1 = transform.getOrigin().getZ();
  ROS_INFO("TF %f %f %f",x1,y1,z1);

  try{
    listener.waitForTransform("/base_link", "/object_29", t, ros::Duration(4.0));
    listener.lookupTransform("/base_link", "/object_29", t, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  ROS_INFO("TF");

  x2 = transform.getOrigin().getX();
  y2 = transform.getOrigin().getY();
  z2 = transform.getOrigin().getZ();
  ROS_INFO("TF %f %f %f",x2,y2,z2);

  double d1,d2,constant;
  d1 = x1*x1 + y1*y1;
  d2 = x2*x2 + y2*y2;
  if (d1<d2) constant = -PI/2.0;
  else constant = PI/2.0;

  try{
    listener.waitForTransform("/map", "/object_28", t, ros::Duration(4.0));
    listener.lookupTransform("/map", "/object_28", t, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  
  ROS_INFO("TF");
  x1 = transform.getOrigin().getX();
  y1 = transform.getOrigin().getY();
  z1 = transform.getOrigin().getZ();
  ROS_INFO("TF %f %f %f",x1,y1,z1);

  try{
    listener.waitForTransform("/map", "/object_29", t, ros::Duration(4.0));
    listener.lookupTransform("/map", "/object_29", t, transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  ROS_INFO("TF");
  x2=  transform.getOrigin().getX();
  y2 = transform.getOrigin().getY();
  z2 = transform.getOrigin().getZ();
  ROS_INFO("TF %f %f %f",x2,y2,z2);

  x_mid = (x1 + x2) / 2.0;
  y_mid = (y1 + y2) / 2.0;
  x = x_mid - sqrt((1.5 * 1.5)/((((x1-x2)/(y2-y1))*((x1-x2)/(y2-y1)))+1));
  y = y_mid + ((x1 -x2)/(y2 - y1)) * (x - x_mid);
  ros::Publisher pub = node_handle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000, true);

  geometry_msgs::PoseStamped move_pose;
  move_pose.pose.position.x = x;
  move_pose.pose.position.y = y;
  double result;
  result = atan ((y2-y1)/(x2-x1)) + constant;
  move_pose.pose.orientation = tf::createQuaternionMsgFromYaw(result);
  ROS_INFO("%f ",result);
  move_pose.header.frame_id = "map";

  // Publish the message.how to know move base simple goal has been reached
  pub.publish(move_pose);
  sleep(5);
  ros::shutdown();
  return 0;
}
