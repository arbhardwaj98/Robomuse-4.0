#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float x1,y11,z1,x2,y2,z2,x_mid,y_mid,dmid,x01,y01,z01,xo2,yo2,flag =0,fl=0;

class dock
{
public:
  dock();
  void Callback(void);
private:
  ros::NodeHandle nh;
  ros::Subscriber joy_sub_;
  ros::Publisher dock_vel;
  ros::Rate *rate;
  tf::TransformListener listener;
  tf::StampedTransform obj1;
  tf::StampedTransform obj2;
  geometry_msgs::Twist cmdvel;
  bool obj_found;
  double thresh;
  double dist1, dist2, error, kp;
  bool docked;
  MoveBaseClient *ac;
};

dock::dock()
{
  dock_vel = nh.advertise<geometry_msgs::Twist>("/robomuse/cmd_vel", 1);
  obj_found = 0;
  thresh = 0.50;
  kp = 3.0;
  rate = new ros::Rate(1.0);
  docked = 0;
}
void dock::Callback()
{
  MoveBaseClient ac("move_base", true);
  while (ros::ok() && !docked) {
    obj_found = 0;
    while(ros::ok() && !obj_found) {
      try{
        listener.lookupTransform("map","marker_frame", ros::Time(0), obj1);
        obj_found = 1;
      }
      catch (tf::TransformException &ex){
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    if(fl==0 && obj1.getOrigin() && obj1.getOrigin().y() <100 && obj1.getOrigin().y()!=0 && obj1.getOrigin().x())
    {
      for(int i=0;i<10;i++){    x1 += obj1.getOrigin().x();
    y11 += obj1.getOrigin().y();
    z1 += obj1.getOrigin().z(); }
    x1= round(x1*1000)/10000;
    y11= round(y11*1000)/10000;
    z1= round(z1*1000)/10000;
    ROS_INFO(" %f , %f , %f ",x1,y11,z1);
    fl=1;
    }
    if(fl==1){
      x01=x1;
      y01=y11;
      z01=z1;
      fl=2;
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = (x1-1);
      goal.target_pose.pose.position.y = 0.0;
      goal.target_pose.pose.orientation.w = 1.0;
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);
      ac.waitForResult();
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("reached");
      else
        ROS_INFO("not reached");
      } 
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dock");
  dock Dock;
  Dock.Callback();
  ros::spin();
}
