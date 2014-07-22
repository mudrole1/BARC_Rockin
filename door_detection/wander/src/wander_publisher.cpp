#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wander_publisher");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    geometry_msgs::Twist msg;

    msg.linear.x=10;
    msg.linear.y=10;
    msg.linear.z=10;
    msg.angular.x=10;
    msg.angular.y=10;
    msg.angular.z=10;

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
