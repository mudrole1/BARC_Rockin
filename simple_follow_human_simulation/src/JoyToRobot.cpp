// "JoyToRobot"
// This program creates a node that reads subscribe to 'joy' msgs 
// and writes a 'cmd_vel' msg for a diff robot.
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
using namespace std;

bool publish_on  = false;
bool robot_on    = false;
double v_x = 0;
double w_z = 0;

void JoyCallback (const sensor_msgs::Joy::ConstPtr& msg)
{
  publish_on = true;
  robot_on = true;


  if( msg->axes[6] == 1 )       
    v_x = 0.1; 
  else if( msg->axes[6] == -1 ) 
    v_x = -0.1;
  else                          
    v_x = 0;   

  if( msg->axes[5] == 1 )       
    w_z = 0.15; 
  else if( msg->axes[5] == -1 ) 
    w_z = -0.15;
  else                          
    w_z = 0;   

  if( msg->axes[5] == 0 && msg->axes[6] == 0 )
    robot_on = false;

  //cout << msg->axes[5] << " " << msg->axes[6] << endl;
  //cout << v_x << " " << w_z << endl;


}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "JoyToRobot");

  ros::NodeHandle n;

  ros::Publisher  node_pub = n.advertise <geometry_msgs::Twist>("robot_0/cmd_vel", 2); // This has to be the HUMAN topic
  ros::Subscriber node_sub = n.subscribe("joy", 2, JoyCallback);

  geometry_msgs::Twist msg; // robot_vel
  
  while( ros::ok() ){
    if( robot_on == true ){
      msg.linear.x  = v_x;
      msg.angular.z = w_z;
      node_pub.publish( msg );
      publish_on = false;
    }
    ros::spinOnce();
  }
  return 0;
}
