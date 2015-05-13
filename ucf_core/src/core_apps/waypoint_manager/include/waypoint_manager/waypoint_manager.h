#ifndef WAYPOINT_MANAGER_H
#define WAYPOINT_MANAGER_H
#include <stack>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "std_srvs/Empty.h"
#include "waypoint_manager_msgs/NextWaypoint.h"
#include "waypoint_manager_msgs/PushWaypoint.h"

class WaypointManager
{
public:
    WaypointManager(ros::NodeHandle nh);
    WaypointManager(ros::NodeHandle nh, std::vector<geometry_msgs::Pose> initial_waypoints);

    // Start processing callbacks.
    void start();

private:
    // Callback for retrieving the current target waypoint.
    bool nextWaypointCallback(waypoint_manager_msgs::NextWaypoint::Request&, waypoint_manager_msgs::NextWaypoint::Response&);
    // Callback for advancing to next waypoint including stack(s) manipulation.
    bool advanceWaypointCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    // Callback for backtracking to previously visited waypoint including stack(s) manipulation.
    bool backtrackWaypointCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    // Callback for pushing new (generated) waypoint onto the forward stack.
    bool pushNewWaypointCallback(waypoint_manager_msgs::PushWaypoint::Request&, waypoint_manager_msgs::PushWaypoint::Response&);

    // Nodehandle to save and use when needed.
    ros::NodeHandle node_handle_;
    // Stack of poses as waypoints.
    std::stack<geometry_msgs::Pose> forward_waypoint_stack_;
    // Stack of poses we have already visited.
    std::stack<geometry_msgs::Pose> visited_waypoint_stack_;
    // Service server for retrieving next waypoint on forward stack.
    ros::ServiceServer next_waypoint_srv_;
    // Service server for advancing forward to next waypoint.
    ros::ServiceServer forward_waypoint_srv_;
    // Service server for backtracking to previous waypoint.
    ros::ServiceServer backtrack_waypoint_srv_;
    // Service server for pushing brand new waypoint.
    ros::ServiceServer push_new_waypoint_srv_;
};

#endif