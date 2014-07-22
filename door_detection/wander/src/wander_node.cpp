#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

ros::Publisher chatter_pub;

void chatterCallback(const sensor_msgs::LaserScan& msg)
{


geometry_msgs::Twist msgs; 	
if(msg.ranges[-msg.angle_min/msg.angle_increment] > 1){
    msgs.linear.x=.3;
    msgs.linear.y=0;
    msgs.linear.z=0;
    msgs.angular.x=0;
    msgs.angular.y=0;
    msgs.angular.z=0;
} 
else{
    msgs.linear.x=-.1;
    msgs.linear.y=0;
    msgs.linear.z=0;
    msgs.angular.x=0;
    msgs.angular.y=0;
    msgs.angular.z=1.3;
}
    chatter_pub.publish(msgs);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wander_node");

  ros::NodeHandle n;

  chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	  ros::Subscriber sub = n.subscribe("/scan", 1000, chatterCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

