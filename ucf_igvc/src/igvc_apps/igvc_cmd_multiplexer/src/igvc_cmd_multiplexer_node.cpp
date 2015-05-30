//
// Created by developer on 5/30/15.
//

#include <igvc_cmd_multiplexer/igvc_cmd_multiplexer.h>
#include "ros/ros.h"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "cmd_multiplexer");
    ros::NodeHandle nh("~");


    std::string cmd_vel_topic;
    if (!nh.getParam("cmd_vel_topic", cmd_vel_topic))
    {
        cmd_vel_topic = "/cmd_vel";
    }


    std::string auto_cmd_topic;
    if (!nh.getParam("auto_cmd_topic", auto_cmd_topic))
    {
        auto_cmd_topic = "/auto_cmd_vel";
    }


    std::string teleop_cmd_topic;
    if (!nh.getParam("teleop_cmd_topic", teleop_cmd_topic))
    {
        teleop_cmd_topic = "/teleop_cmd_topic";
    }

    std::string status_topic;
    if (!nh.getParam("status_topic", status_topic))
    {
        status_topic = "/status_topic";
    }

    int rate;
    if(!nh.getParam("sleep_rate", rate))
        rate = 15;


    IGVCCmdMultiplexer mp(nh, auto_cmd_topic, teleop_cmd_topic, status_topic, cmd_vel_topic);


    ros::Rate sleep_rate(rate);
    while(ros::ok())
    {
        ros::spinOnce();
        mp.publish();
        sleep_rate.sleep();

    }

    return 0;
}