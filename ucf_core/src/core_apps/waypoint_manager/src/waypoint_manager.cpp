#include "waypoint_manager.h"

WaypointManager::WaypointManager(ros::NodeHandle nh)
{
    std::vector<geometry_msgs::Pose> empty;
    WaypointManager(nh, empty);
}

WaypointManager::WaypointManager(ros::NodeHandle nh, std::vector<geometry_msgs::Pose> initial_waypoints)
    : node_handle_(nh)
{
    // Create forward waypoint stack.
    std::vector<geometry_msgs::Pose>::iterator iter;
    for (iter = initial_waypoints.begin(); iter != initial_waypoints.end(); ++iter)
    {
        forward_waypoint_stack_.push(*iter);
    }

    // Initialize service servers.
    next_waypoint_srv_ = node_handle_.advertiseService("next_waypoint", &WaypointManager::nextWaypointCallback, this);
    forward_waypoint_srv_ = node_handle_.advertiseService("advance_waypoint", &WaypointManager::advanceWaypointCallback, this);
    backtrack_waypoint_srv_ = node_handle_.advertiseService("backtrack_waypoint", &WaypointManager::backtrackWaypointCallback, this);
    push_new_waypoint_srv_ = node_handle_.advertiseService("push_new_waypoint", &WaypointManager::pushNewWaypointCallback, this);
}

void WaypointManager::start()
{
    // Node exists just to process callbacks.
    ros::spin();
}

bool WaypointManager::nextWaypointCallback(waypoint_manager_msgs::NextWaypoint::Request&, waypoint_manager_msgs::NextWaypoint::Response& resp)
{
    // If forward stack empty, fail service request.
    if (forward_waypoint_stack_.empty())
        return false;

    // Populate the top of the forward waypoint stack into the response message.
    resp.waypoint = forward_waypoint_stack_.top();
    return true;
}

bool WaypointManager::advanceWaypointCallback(waypoint_manager_msgs::AdvanceWaypoint::Request&, waypoint_manager_msgs::AdvanceWaypoint::Response&)
{
    // If forward stack empty, fail service request.
    if (forward_waypoint_stack_.empty())
        return false;

    geometry_msgs::Pose current = forward_waypoint_stack_.top();
    visited_waypoint_stack_.push(current);
    forward_waypoint_stack_.pop();
    return true;
}

bool WaypointManager::backtrackWaypointCallback(waypoint_manager_msgs::BacktrackWaypoint::Request&, waypoint_manager_msgs::BacktrackWaypoint::Response&)
{
    // If visited stack empty, fail service request.
    if (visited_waypoint_stack_.empty())
        return false;

    geometry_msgs::Pose previous = visited_waypoint_stack_.top();
    forward_waypoint_stack_.push(previous);
    visited_waypoint_stack_.pop();
    return true;
}

bool WaypointManager::pushNewWaypointCallback(waypoint_manager_msgs::PushWaypoint::Request& req, waypoint_manager_msgs::PushWaypoint::Response&)
{
    forward_waypoint_stack_.push(req.waypoint);
    return true;
}
