#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <sstream>

ros::Publisher chatter_pub;

bool keepspinning = true;
float doorwidth = 0.8;
float marginoferror = 0.5;


void chatterCallback(const sensor_msgs::LaserScan& msg)
{
	keepspinning = true;
	bool check = false;
	bool broken = false;
	float centerpoint = msg.ranges[-msg.angle_min / msg.angle_increment];
    float upperandlowerranges = atan ((doorwidth/2)/centerpoint);
	int anglea = (int)(((-upperandlowerranges)-msg.angle_min) / msg.angle_increment);
    int angleb =  (int)(((upperandlowerranges)-msg.angle_min) / msg.angle_increment);
	int size = 0;
	for(int i=anglea;i<angleb+1;i++){
		size++;
	}
	float results [size];
	float predictions [size];
	std::cout << "result:[";
	int notsize = 0;
	for(int i=anglea;i<angleb+1;i++){
		std::cout << msg.ranges[i];
		results[notsize] = msg.ranges[i];
		if(results[notsize]==4)broken=true;
		notsize++;
		std::cout << ",";
	}
	std::cout << "]\n";
	std::cout << "predic:[";
	notsize = 0;
	for(int i=anglea;i<angleb+1;i++){
	float theangle = fabs((i*msg.angle_increment)+msg.angle_min);
	float prediction = centerpoint/cos (theangle);

	if(prediction>4){
		std::cout << "4";
		predictions[notsize] = 4;
		broken =true;
		notsize++;
	}else{
		std::cout << prediction;
		predictions[notsize] = prediction;
		notsize++;
	} 
	std::cout << ",";
	}
	std::cout << "]\n";
	int counter = 0;
	std::cout << size;
	std::cout << "\n";
	std::cout << counter;
	std::cout << "\n";
	std::cout << "checks:[";		
	for(int i =0; i<size;i++){
		float diff = 0;
		diff = fabs(results[i]-predictions[i]);
		if(diff<marginoferror && !broken){
			counter++;
			std::cout << "t";
		}
		else{
			std::cout << "f";
		}
		std::cout << ",";		
	}
	if(size == counter){
		keepspinning = false;
	}

	std::cout << "]\n";
	std::cout << counter;
	std::cout << "\n";
    float newangleresult = upperandlowerranges * 180 / M_PI;
    std::cout << newangleresult;
	geometry_msgs::Twist msgs;
	if(keepspinning){
		std::cout << "door open/no door\n";
		msgs.linear.x=0;
		msgs.linear.y=0;
		msgs.linear.z=0;
		msgs.angular.x=0;
		msgs.angular.y=0;
		msgs.angular.z=0;
	}
    else{
		std::cout << "door closed\n";
		msgs.linear.x=0;
		msgs.linear.y=0;
		msgs.linear.z=0;
		msgs.angular.x=0;
		msgs.angular.y=0;
		msgs.angular.z=0;
	}
		
		chatter_pub.publish(msgs);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "door_checker");

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

