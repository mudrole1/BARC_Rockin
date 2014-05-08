#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>

// the custom msg containing the name and the coordinates of nav goals
#include <dora_nav_goals/named_coordinates.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher pub;

void NavRequestCallback(const dora_nav_goals::named_coordinatesConstPtr& msg)
{
    ROS_INFO("I have got a Nav GOAL!!!");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("Sending goal");
    ac.sendGoal(msg->goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Dora successfully moved to coordinates");

        std_msgs::Bool response;
        response.data = 1;
        pub.publish(response);
    }
    else {
        ROS_INFO("Dora failed to move properly for some reason");

        std_msgs::Bool response;
        response.data = 0;
        pub.publish(response);
    }
}



int main(int argc, char** argv){

    ros::init(argc, argv, "dora_nav_goals");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/navigate/request", 1, NavRequestCallback);

    pub = n.advertise<std_msgs::Bool>("/navigate/response", 1000);

    ros::spin();

    return 0;
}
