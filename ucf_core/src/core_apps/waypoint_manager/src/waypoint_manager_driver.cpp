#include "ros/ros.h"
#include "waypoint_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_manager_node");
    ros::NodeHandle nh;

    // Dumb test stuff.
    std::vector<geometry_msgs::Pose> initial;
    geometry_msgs::Pose pose;

    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1.0;
    pose.position.x = 5.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;

    initial.push_back(pose);

    WaypointManager manager(nh, initial);
    manager.start();

    return 0;
}