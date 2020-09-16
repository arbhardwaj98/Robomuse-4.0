#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <fstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float x1,y11,z1,x2,y2,z2,x_mid,y_mid,dmid,xo1,yo1,xo2,yo2;

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
  rate = new ros::Rate(10.0);
  docked = 0;
}
void dock::Callback()
{
    int f=0;
  std::ofstream mf;
  mf.open("/home/robomuse/catkin_ws/src/robomuse-ros-master/robomuse_drivers/marker.csv");
  while (ros::ok() && !docked) {
    obj_found = 0;
    while(ros::ok() && !obj_found) {
      try{
        listener.lookupTransform("odom", "marker1_frame", ros::Time(0), obj1);
        listener.lookupTransform("odom", "marker2_frame", ros::Time(0), obj2);
        obj_found = 1;
      }
      catch (tf::TransformException &ex){
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
	obj_found=0;
      }
      if(obj_found==1){
	mf<<obj1.getOrigin().x()<<','<<obj1.getOrigin().y()<<','<<obj1.getOrigin().z()<<','<<obj1.getRotation().x()<<','<<obj1.getRotation().y()<<','<<obj1.getRotation().z()<<','<<obj1.getRotation().w()<<"\n";
  rate->sleep();
}
    
}}}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "markerlog");
  dock Dock;
  Dock.Callback();
  ros::spin();
}
