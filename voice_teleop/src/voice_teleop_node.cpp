/*!
 * mixed_initiative_teleop_node.cpp
 * Copyright (c) 2014, Manolis Chiou
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*!

@mainpage
  Joystick teleoperation node for use within the mixed initiative framework. The user can operate the robot in
  teleoperation mode and change on the fly autonomy level/mode. Also a stop button is implimented.
  It was ment to be used with an Xbox 360 joystick but should work with any joystick.
<hr>

@section usage Usage
@par    After start roscore, you need load robot configuration file to parameter server first.
        For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
        then run drrobot_player first. then start ros joy node.
@verbatim
$ mixed_initiative_teleop
@endverbatim

<hr>
@section topic ROS topics

Publishes to (name / type):
-@b /teleop/cmd_vel: will publish to /teleop/cmd_vel a geometry_msgs/Twist.msg type message to drrobot_player.
 For differential robots, linear.x is forward/backward speed (m/sec), and angular.z (rad/sec)is the angular speed.
<hr>
*/


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

class VoiceTeleop
{

public:
    VoiceTeleop();

private:
    void VoiceCmdCallback(const std_msgs::String::ConstPtr& voiceCmd);
    void PoseCmdCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);

    ros::NodeHandle nh_;

    ros::Publisher goal_pub_;
    ros::Subscriber voiceCmd_sub_, currentPose_sub_;

    geometry_msgs::PoseWithCovarianceStamped pose_;

};


VoiceTeleop::VoiceTeleop()
{

    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    voiceCmd_sub_ = nh_.subscribe<std_msgs::String>("/recognizer/output", 1, &VoiceTeleop::VoiceCmdCallback, this);
    currentPose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &VoiceTeleop::PoseCmdCallback, this);

}

void VoiceTeleop::PoseCmdCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
    pose_.pose.pose.position.x = pose->pose.pose.position.x;
    pose_.pose.pose.position.y = pose->pose.pose.position.y;
    pose_.pose.pose.orientation.z = pose->pose.pose.orientation.z;

}

void VoiceTeleop::VoiceCmdCallback(const std_msgs::String::ConstPtr& voiceCmd)
{

    geometry_msgs::PoseStamped goal;

    goal.header.frame_id = "/map";
    goal.header.stamp = ros::Time::now() ;


    // autonomy mode choice
    if (voiceCmd->data == "right")
    {
        goal.pose.position.x = pose_.pose.pose.position.x ;
        goal.pose.position.y = pose_.pose.pose.position.y - 1;
        goal.pose.orientation.z = pose_.pose.pose.orientation.z;

        goal_pub_.publish(goal);
    }
    if (voiceCmd->data == "left")
    {
        goal.pose.position.x = pose_.pose.pose.position.x;
        goal.pose.position.y = pose_.pose.pose.position.y + 1;
        goal.pose.orientation.z = pose_.pose.pose.orientation.z;

        goal_pub_.publish(goal);
    }
    if (voiceCmd->data == "forward")
    {
        goal.pose.position.x = pose_.pose.pose.position.x + 1;
        goal.pose.position.y = pose_.pose.pose.position.y;
        goal.pose.orientation.z = pose_.pose.pose.orientation.z;

        goal_pub_.publish(goal);
    }

    if (voiceCmd->data == "backward")
    {
        goal.pose.position.x = pose_.pose.pose.position.x - 1;
        goal.pose.position.y = pose_.pose.pose.position.y;
        goal.pose.orientation.z = pose_.pose.pose.orientation.z;

        goal_pub_.publish(goal);
    }


}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "voice_teleop");
    VoiceTeleop voice_teleop;

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}
