#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "square");

  ros::NodeHandle n;

  // Advertize the publisher on the topic you like
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/robomuse/cmd_vel", 1000);
  ros::Publisher pub2 = n.advertise<std_msgs::Int8>("startencryption", 1000);
  ros::Duration(4).sleep();

  while (ros::ok())
  {

    geometry_msgs::Twist myTwistMsg;
    // Here you build your twist message - turn right 90 degs
    for (int i = 0; i<4; i++)
  {
    myTwistMsg.linear.x = 0.15;
    myTwistMsg.linear.y = 0;
    myTwistMsg.linear.z = 0;

    myTwistMsg.angular.x = 0;
    myTwistMsg.angular.y = 0;
    myTwistMsg.angular.z = 0;

    ros::Time beginTime = ros::Time::now();
    ros::Duration delay2(2);

    while(ros::Time::now() - beginTime <= delay2 )
    {
        pub.publish(myTwistMsg);
        ros::Duration(0.1).sleep();
    }

    myTwistMsg.linear.x = 0;
    myTwistMsg.linear.y = 0;
    myTwistMsg.linear.z = 0;

    myTwistMsg.angular.x = 0;
    myTwistMsg.angular.y = 0;
    myTwistMsg.angular.z = -0.2;


    ros::Duration delay1(7.85);
    beginTime = ros::Time::now();

    while(ros::Time::now() - beginTime <= delay1 )
    {
        pub.publish(myTwistMsg);
        ros::Duration(0.1).sleep();
    }
  }

  std_msgs::Int8 msg;
  msg.data = 1;
  pub2.publish(msg);
  return 0;
  }
}
