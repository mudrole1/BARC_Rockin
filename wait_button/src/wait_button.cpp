#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include <sstream>
#include <wait_button/button.h>

bool b1=false;
bool b2=false;

void chatterCallback(const sensor_msgs::Joy & msg)
{
  if(msg.buttons[4]==1)
    b1=true;
  if(msg.buttons[5]==1)
    b2=true;
  //ROS_INFO("I heard: [%d,%d]", msg.buttons[4], msg.buttons[5]);
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wait_button");

  ros::NodeHandle ns;
  ros::NodeHandle nt; 

  wait_button::button mess;

  ros::Rate loop_rate(10);

  ros::Subscriber sub = ns.subscribe("/joy", 1000, chatterCallback);   
  ros::Publisher chatter_pub = nt.advertise<wait_button::button>("/wait_for_button", 1000);

  while(chatter_pub.getNumSubscribers() == 0)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  while(!b1)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
    ROS_INFO("button 1 send");
    mess.button1 = 1;

    chatter_pub.publish(mess); 
  while(!b2)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("button 2 send");

    mess.button1 = 0;
    mess.button2 = 1;

    chatter_pub.publish(mess); 

  

  return 0;
}

