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
#include <vector>


namespace igvc
{

class VelocityController
{
public:
    VelocityController(std::string, std::vector< std::string >, double, double);
    ~VelocityController();

private:

    ros::NodeHandle nh_;
    //Topic containing geometryTwist for vehicle velocity input
    ros::Subscriber cmd_vel_topic_;

    //Topics that the roboteq driver is listening to
    ros::Publisher left_motor_pub_;
    ros::Publisher right_motor_pub_;

    //Vehicle dimensions required for calculations
    double wheel_radius_, base_radius_;

    //Callbacks
    void cmd_vel_callback(const geometry_msgs::Twist& msg);

};

}
