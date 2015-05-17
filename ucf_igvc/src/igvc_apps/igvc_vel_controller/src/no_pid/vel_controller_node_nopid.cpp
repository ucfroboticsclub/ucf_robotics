/**

Copyright (c) 2015, Robotics Club @ UCF
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.

@author Thomas Watters (thomaswatters@knights.ucf.edu)

 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <roboteq_msgs/Command.h>
#include "igvc_vel_controller_nopid/velocity_controller.h"

int main(int argc, char** argv)
{
	ROS_INFO("Starting vel_controller_node");
	ros::init(argc, argv, "~");
	ros::NodeHandle nh("~");

    std::string cmd_vel_topic;
    double wheel_radius, base_radius;
	if( !nh.getParam("cmd_vel_topic", cmd_vel_topic) )
	{
        ROS_INFO("No Command Velocit, left_motor_topic, right_motor_topic;y topics provided using default -- /cmd_vel");
		cmd_vel_topic = "/cmd_vel" ;
	}

	if(!nh.hasParam("channels"))
	{
		ROS_INFO("No Motor Channels Provided");
		return -1;
    }

	XmlRpc::XmlRpcValue channels;
	nh.param("channels", channels, channels);
	if(channels.size() != 2)
	{
		ROS_INFO("Invalid Number of Motor Channels");
		return -1;
	}

    std::vector< std::string > namespaces;
    namespaces.push_back((std::string)channels[0]);
    namespaces.push_back((std::string)channels[1]);

	if( !nh.getParam("wheel_radius", wheel_radius ) )
	{
		ROS_INFO("No wheel radius provided");
		return -1;
	}

	if( !nh.getParam("base_radius", base_radius ))
	{
		ROS_INFO("No radius prodived for base of robot");
		return  -1;
	}

    igvc::VelocityController vc(cmd_vel_topic,namespaces,wheel_radius, base_radius);

	ros::spin();

	return 0;
}
