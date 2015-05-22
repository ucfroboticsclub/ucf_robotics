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

#include "igvc_vel_controller_nopid/velocity_controller.h"

namespace igvc
{

    VelocityController::VelocityController(std::string cmd_topic, std::vector<std::string> namespaces,
                                           double wheel_radius, double base_radius)
            : nh_(), base_radius_(base_radius), wheel_radius_(wheel_radius), counter(0)
    {
        right_motor_pub_ = nh_.advertise<roboteq_msgs::Command>(namespaces[0] + "/cmd", 1);
        left_motor_pub_ = nh_.advertise<roboteq_msgs::Command>(namespaces[1] + "/cmd", 1);

        cmd_vel_topic_ = nh_.subscribe(cmd_topic, 1, &VelocityController::cmd_vel_callback, this);
    }

    VelocityController::~VelocityController()
    {

    }

    void VelocityController::cmd_vel_callback(const geometry_msgs::Twist &msg)
    {
        //TODO: Add PID Controllers for both velocity vectors

        double linear_velocity_x = -1 * msg.linear.x;	// linear x m/s
        double angular_velocity_z = -1 * msg.angular.z; 	// yaw rad/s

        double linear_contribution = linear_velocity_x / wheel_radius_;
        double angular_contribution = angular_velocity_z * base_radius_ / wheel_radius_;

        roboteq_msgs::Command left_cmd, right_cmd;
        left_cmd.commanded_velocity = 32 * (linear_contribution - angular_contribution);
        right_cmd.commanded_velocity = 32 * (linear_contribution + angular_contribution);

        left_motor_pub_.publish(left_cmd);
        right_motor_pub_.publish(right_cmd);

        counter = 0;
    }


    void VelocityController::zero()
    {
        roboteq_msgs::Command left, right;
        left.commanded_velocity = 0;
        right.commanded_velocity = 0;

        left_motor_pub_.publish(left);
        right_motor_pub_.publish(right);

    }

}
