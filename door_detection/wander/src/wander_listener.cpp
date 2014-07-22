#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>

void chatterCallback(const sensor_msgs::LaserScan& msg)
{
  ROS_INFO("I heard: [%f]", msg.ranges[-msg.angle_min/msg.angle_increment]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wander_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/scan", 1000, chatterCallback);

  ros::spin();

  return 0;
}

