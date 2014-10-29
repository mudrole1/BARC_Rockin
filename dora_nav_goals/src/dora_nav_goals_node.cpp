#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// global variable to save from rewritting package with classes.
// It stores if the nav goal was successful and if the callback was called
std_msgs::Int32 result;
ros::Publisher  pub;

void NavRequestCallback(const std_msgs::StringConstPtr& destination)
{
    ROS_INFO("I have got a Nav GOAL!!!");

    bool unknown = false;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
//    while(!ac.waitForServer(ros::Duration(5.0))){
//        ROS_INFO("Waiting for the move_base action server to come up");
//    }

    //Create the goal message
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    if (destination->data == "Door") {
        goal.target_pose.pose.position.x = 0.51;
        goal.target_pose.pose.position.y = 0.17;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 0.84;
        goal.target_pose.pose.orientation.w = 0.54;
    } else if (destination->data == "Enter") {
        goal.target_pose.pose.position.x = 0.95;
        goal.target_pose.pose.position.y = -0.83;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 0.78;
        goal.target_pose.pose.orientation.w = 0.62;
    } else if (destination->data == "Kitchen") {
        goal.target_pose.pose.position.x = 0.51;
        goal.target_pose.pose.position.y = 0.17;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 0.84;
        goal.target_pose.pose.orientation.w = 0.54;
    } else if (destination->data == "Bedroom") {
        goal.target_pose.pose.position.x = 4.75;
        goal.target_pose.pose.position.y = 2.16;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = -0.80;
        goal.target_pose.pose.orientation.w = 0.59;
    } else {
        unknown = true;
    }

    if (!unknown) {
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();
    }

    if (unknown) {
        ROS_INFO("Unknown destination: %s", destination->data.c_str());
        result.data = -1;
    } else if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Dora successfully moved to coordinates");
        result.data = 1;
    } else {
        ROS_INFO("Dora failed to move properly for some reason");
        result.data = 0;
    }

    pub.publish(result);
}



int main(int argc, char** argv){

    ros::init(argc, argv, "dora_nav_goals");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/navigation/request", 10, NavRequestCallback);
    pub = n.advertise<std_msgs::Int32>("/navigation/response", 10);

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}
