//
// Created by developer on 5/30/15.
//

#ifndef IGVC_CMD_MULTIPLEXER_IGVCCMDMULTIPLEXER_H
#define IGVC_CMD_MULTIPLEXER_IGVCCMDMULTIPLEXER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <roboteq_msgs/Status.h>


class IGVCCmdMultiplexer
{
private:
    bool is_autonomous_;

    geometry_msgs::Twist autonomous_msg_, teleop_msg_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber autonomous_sub_, teleop_sub_;
    ros::Subscriber status_sub_;

    ros::NodeHandle nh_;

    void autonomousCB(const geometry_msgs::TwistConstPtr msg);
    void teleopCB(const geometry_msgs::TwistConstPtr msg);
    void statusCB(const roboteq_msgs::StatusConstPtr msg);

public:
    IGVCCmdMultiplexer(ros::NodeHandle nh, std::string autonomous_topic, std::string teleop_topic,
                        std::string status_topic, std::string out_cmd_topic );

    void publish();

};


#endif //IGVC_CMD_MULTIPLEXER_IGVCCMDMULTIPLEXER_H
