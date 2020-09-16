#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

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
  rate = new ros::Rate(1.0);
  docked = 0;
}
void dock::Callback()
{
    int f=0;
  while (ros::ok() && !docked) {
    obj_found = 0;
    while(ros::ok() && !obj_found) {
      try{
        listener.lookupTransform("base_link", "object_29", ros::Time(0), obj1);
        listener.lookupTransform("base_link", "object_28", ros::Time(0), obj2);
        obj_found = 1;
      }
      catch (tf::TransformException &ex){
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    if(obj1.getOrigin() && obj1.getOrigin().y() <100 && obj1.getOrigin().y()!=0 && obj1.getOrigin().x() && f != 1 )
    {
    x1 = obj1.getOrigin().x();
    y11 = obj1.getOrigin().y();
    z1 = obj1.getOrigin().z(); 
    x2 = obj2.getOrigin().x();
    y2 = obj2.getOrigin().y();
    z2 = obj2.getOrigin().z();
    x_mid = (x1 + x2) / 2.0;
    y_mid = (y11 + y2) / 2.0;
    dmid = x_mid * x_mid + y_mid * y_mid;
    dmid = sqrt(dmid);
    ROS_INFO("Dmid : %f ",dmid);
  }
  cmdvel.linear.x = 0.1;
  dock_vel.publish(cmdvel);
  rate->sleep();
}}}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dock");
  dock Dock;
  Dock.Callback();
  ros::spin();
}
