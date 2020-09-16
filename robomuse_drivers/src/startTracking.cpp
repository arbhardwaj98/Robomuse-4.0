#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>
#include<math.h>

int flag = 0;
float kp = 2.5;
float kp_l = 0.4;
int problem = 0;

int main(int argc, char** argv)
{ ros::init(argc, argv, "tracking_node");
ros::NodeHandle node;

// publisher declaration
ros::Publisher torso_joint = node.advertise<geometry_msgs::Point>("asdf", 1);
ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/robomuse/cmd_vel", 1);

// listener
tf::TransformListener listener;

ros::Rate rate(5.0); // frequency of operation
float angle,x,y,z,vel,prevx,prevy;
prevx = 0;
prevy = 0;

while (node.ok())
{
    // Transforms declared for each joint
    tf::StampedTransform transform_torso;

    try
    {
        // each joint frame to reference frame transforms
        listener.lookupTransform("/base_link", "/torso_1",ros::Time(0), transform_torso);
	problem = 0;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
	problem =1;
        ros::Duration(0.10).sleep();
        continue;
    }

    // geometry points declaration for storing 3D coordinates of joints and then published later
    geometry_msgs::Point torso_pose;
    // joint position extraction and store
    // torso joint
    x = transform_torso.getOrigin().x();
    y = transform_torso.getOrigin().y();
    z = transform_torso.getOrigin().z();

    if((prevx == x) && (prevy == y))
    {
      problem = 1;
      ROS_INFO("problem !!");
    }

    prevx = x;
    prevy = y;

    angle = atan(y/x);
    ROS_INFO("angle: %f",angle);
  //  torso_pose.header.frame_id = "map";
  //  torso_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    // if(abs(angle)<0.01)
    // {
    //   angle = 0;
    // }
    vel = kp*angle;
    if(vel>=0.4)
    {
      vel = 0.4;
    }
    else if(vel<=-0.4){
      vel= -0.4;
    }



    ROS_INFO("vel : %f", vel);

    geometry_msgs::Twist msg;
    msg.angular.z = vel;
    if((sqrt(x*x + y*y))<=1.5)
    {
      msg.linear.x = 0.0;
    }
    else
    {
    	msg.linear.x = kp_l * (sqrt(x*x + y*y) - 1.5);
    	if(msg.linear.x >= 0.6)
    		msg.linear.x = 0.6;
    }
    // joint positions publish
    torso_joint.publish(torso_pose);
    if(problem == 1)
    {
       msg.linear.x = 0.0;
       msg.angular.z = 0.0;
     }
    pub.publish(msg);
  //    ROS_INFO("TF %d %d %d",torso_pose.pose.position.x,torso_pose.pose.position.y,torso_pose.pose.position.z);

    rate.sleep();
}

return 0;
};

