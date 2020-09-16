#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h> 
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
  ros::Publisher correctedpose;
  ros::Publisher inLOS;
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
  int i;
};

dock::dock()
{  
  correctedpose = nh.advertise<geometry_msgs::Pose>("/robomuse/visualcorrection", 1);
  obj_found = 0;
  thresh = 0.50;
  kp = 3.0;
  rate = new ros::Rate(2.0);
  docked = 0;
  i =0;
}
void dock::Callback()
{
    int f=0;
    geometry_msgs::Pose first;
    geometry_msgs::Pose posecor;
    std_msgs::Bool a;
    while(ros::ok() && !docked) {
        while(ros::ok()) {
            obj_found = 0;
            try{
                listener.lookupTransform("marker1_frame", "base_link", ros::Time(0), obj1);
                obj_found = 1;
            }
            catch (tf::TransformException &ex){
                ROS_WARN("%s",ex.what());
	            obj_found=0;
            }
            if(obj_found==1){
                posecor.position.x = obj1.getOrigin().x();
                posecor.position.y = obj1.getOrigin().y();
                posecor.position.z = obj1.getOrigin().z();
                posecor.orientation.x = obj1.getRotation().x();
                posecor.orientation.y = obj1.getRotation().y();
                posecor.orientation.z = obj1.getRotation().z();
                posecor.orientation.w = obj1.getRotation().w();
                if(i==0){
                    first.orientation.w = obj1.getRotation().w();
                }
                if(first.orientation.w/posecor.orientation.w > 0){
                    correctedpose.publish(posecor);
                }
                //std::cout<<obj1.getOrigin().x()<<','<<obj1.getOrigin().y()<<','<<obj1.getOrigin().y();
                rate->sleep();
                i = 1;
            }
            
        } 
    }
    
} 


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kidnapsolver");
  dock Dock;
  Dock.Callback();
  ros::spin();
}
