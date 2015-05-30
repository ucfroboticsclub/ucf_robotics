//
// Created by developer on 5/30/15.
//

#include "igvc_cmd_multiplexer/igvc_cmd_multiplexer.h"

IGVCCmdMultiplexer::IGVCCmdMultiplexer(ros::NodeHandle nh, std::string autonomous_topic, std::string teleop_topic,
                                       std::string status_topic, std::string out_cmd_topic)
                        : is_autonomous_(false), nh_(nh)
{
    autonomous_sub_ = nh_.subscribe(autonomous_topic, 1, &IGVCCmdMultiplexer::autonomousCB, this);
    teleop_sub_ = nh_.subscribe(teleop_topic, 1, &IGVCCmdMultiplexer::teleopCB, this);
    status_sub_ = nh_.subscribe(status_topic, 1, &IGVCCmdMultiplexer::statusCB, this);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(out_cmd_topic, 1);
}

void IGVCCmdMultiplexer::autonomousCB(const geometry_msgs::TwistConstPtr msg)
{
    autonomous_msg_.angular = msg->angular;
    autonomous_msg_.linear = msg->linear;
}

void IGVCCmdMultiplexer::teleopCB(const geometry_msgs::TwistConstPtr msg)
{
    teleop_msg_.angular = msg->angular;
    teleop_msg_.linear = msg->linear;
}

void IGVCCmdMultiplexer::statusCB(const roboteq_msgs::StatusConstPtr msg)
{
    is_autonomous_ = msg->autonomous_state;
}

void IGVCCmdMultiplexer::publish()
{
    geometry_msgs::Twist *msg = is_autonomous_ ? &autonomous_msg_ : &teleop_msg_;

    cmd_vel_pub_.publish(*msg);

}