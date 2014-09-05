/*!
 BLABLABLA
 */


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cmath>
#include <iostream>
#include <algorithm>

#include <geometry_msgs/Twist.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include <actionlib_msgs/GoalID.h>


class Controller
{
public:
  Controller() ;

private:

  void modeCallback(const std_msgs::Int8& msg);
  void teleopCallback(const geometry_msgs::Twist& msg);
  void navCallback(const geometry_msgs::Twist& msg);

  int control_mode_;
  bool valid_mode_;

  ros::NodeHandle n_;
  ros::Subscriber control_mode_sub_, vel_sub_telop_, vel_sub_nav_ ;
  ros::Publisher vel_pub_ , cancelGoal_pub_, explorationCancel_pub_;

  geometry_msgs::Twist cmdVel_;
  actionlib_msgs::GoalID cancelGoal_;

};

Controller::Controller()
{
  valid_mode_ = true;
  control_mode_ = 0 ; //  stop/idle mode.

  vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 50,this);
  cancelGoal_pub_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 5,this);
  explorationCancel_pub_ = n_.advertise<actionlib_msgs::GoalID>("/explore_server/cancel", 5,this);

  /* Subscribes to:
   * "/control_mode" to take the relevant autonomy mode
   * "/teleop/cmd_vel" to take the velocity coming from the teleoperation
   * "/navigation/cmd_vel" to take the velocity coming out of a navigation controller
  */
  control_mode_sub_ = n_.subscribe("/control_mode", 100, &Controller::modeCallback,this);
  vel_sub_telop_ = n_.subscribe("/teleop/cmd_vel", 100, &Controller::teleopCallback,this);
  vel_sub_nav_ = n_.subscribe("/navigation/cmd_vel", 100, &Controller::navCallback,this);
}

void Controller::modeCallback(const std_msgs::Int8& msg)
{

  switch (msg.data)
    {
    case 0:
      {
        control_mode_ = 0;
        valid_mode_ = true;
        ROS_INFO("Stop robot");
        break;
      }
    case 1:
      {
        control_mode_ = 1;
        valid_mode_ = true;
        ROS_INFO("Control mode: Teleoperation");
        break;
      }
    case 2:
      {
        control_mode_ = 2;
        valid_mode_ = true;
        ROS_INFO("Control mode: Autonomy");
        break;
      }
    default:
      {
        valid_mode_ = false;
        ROS_INFO("Please choose a valid control mode.");
      }
    }
}

void Controller::navCallback(const geometry_msgs::Twist& msg)
{
  if (control_mode_ == 2)
    {
      cmdVel_.linear.x = msg.linear.x;
      cmdVel_.angular.z = msg.angular.z;
      vel_pub_.publish(cmdVel_);
    }
  if (control_mode_ == 0)
    {
      cmdVel_.linear.x = 0;
      cmdVel_.angular.z = 0;
      vel_pub_.publish(cmdVel_);
      cancelGoal_pub_.publish(cancelGoal_);
      explorationCancel_pub_.publish(cancelGoal_);
    }
}

void Controller::teleopCallback(const geometry_msgs::Twist& msg)
{
  if (control_mode_ == 1)
    {
      cmdVel_.linear.x = msg.linear.x;
      cmdVel_.angular.z = msg.angular.z;
      vel_pub_.publish(cmdVel_);
    }
  if (control_mode_ == 0)
    {
      cmdVel_.linear.x = 0;
      cmdVel_.angular.z = 0;
      vel_pub_.publish(cmdVel_);
      cancelGoal_pub_.publish(cancelGoal_);
      explorationCancel_pub_.publish(cancelGoal_);
    }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "mixed_initiative_controller");
  Controller controller_obj;

  ros::Rate r(20); // 20 hz
  while (ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }

}

