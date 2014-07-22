#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <sstream>

void chatterCallback(const sensor_msgs::LaserScan& msg)
{
	/*int count = 0;
	for(int i=msg.angle_min;i<msg.angle_max;i+=msg.angle_increment){
		if(msg.ranges[i]>-.785&&msg.ranges[i]<.785){
			count++;
		}
	}	
	int* list = new int[count];
	int nextcount = 0;
	for(int i=msg.angle_min;i<msg.angle_max;i+=msg.angle_increment){
		if(msg.ranges[i]>-.785&&msg.ranges[i]<.785){
			list[nextcount] = msg.ranges[i];
			nextcount++;
		}
	}

	int closed = 0;
	for(int i = 0; i< count;i++){
		if(list[i]==msg.range_max){
			closed++;
		}
		if(i!=0 && i!= count-1){
			if(list[i]>list[i+1]&&list[i]>list[i-1]){
				closed++;
			}
		}	
	}
	ROS_INFO("door closed");
	
	for(int i=0;i<msg.angle_max;i+=msg.angle_increment){
	  ROS_INFO("at [%i]", i);
	  ROS_INFO(" I heard: [%f]", msg.ranges[msg.angle_min + (i*msg.angle_increment)]);
	}*/


	/*
	int maxnum=(msg.angle_max-msg.angle_min)/msg.angle_increment;
	std::cout << "[";
	for(int i=0;i<maxnum;i+=8){
		std::cout << (int)msg.ranges[i];
		std::cout << ",";
	}
	std::cout << "]\n"; 
	
	bool checker = false;
	if(((-M_PI/3)-msg.angle_min) / msg.angle_increment == ((M_PI/3)-msg.angle_min) / msg.angle_increment){
	checker = true;	
	}
	if()
	int anglea = (int)((-M_PI/3)-msg.angle_min) / msg.angle_increment;
	int angleb =  (int)((M_PI/3)-msg.angle_min) / msg.angle_increment;
	std::cout << "[";
	for(int i=anglea;i<angleb+1;i++){
		std::cout << (int)msg.ranges[i];
		std::cout << ",";
	}
	std::cout << "] ";
	std::cout << checker;
	std::cout << "\n"; 
	*/
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wander_listener2");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/scan", 1000, chatterCallback);

  ros::spin();

  return 0;
}

