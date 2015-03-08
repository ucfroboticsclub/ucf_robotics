#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <roboteq_msgs/Command.h>

ros::Publisher left_motor_pub;
ros::Publisher right_motor_pub;
double wheel_radius, base_radius;


void cmd_vel_callback(const geometry_msgs::Twist& msg)
{
	double linear_velocity_x = msg.linear.x;	// linear x m/s
	double angular_velocity_z = msg.angular.z; 	// yaw rad/s

	double linear_contribution = linear_velocity_x / wheel_radius;
	double angular_contribution = angular_velocity_z * base_radius / wheel_radius;

	roboteq_msgs::Command left_cmd, right_cmd;
	left_cmd.commanded_velocity = linear_contribution + angular_contribution;
	right_cmd.commanded_velocity = linear_contribution - angular_contribution;

	left_motor_pub.publish(left_cmd);
	right_motor_pub.publish(right_cmd);


}

int main(int argc, char** argv)
{
	ROS_INFO("Starting vel_controller_node");
	ros::init(argc, argv, "~");
	ros::NodeHandle nh("~");

	std::string cmd_vel_topic, left_motor_topic, right_motor_topic;
	if( !nh.getParam("cmd_vel_topic", cmd_vel_topic) )
	{
		ROS_INFO("No Command Velocity topics provided using default -- /cmd_vel");
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

	left_motor_topic = (std::string)channels[0] + "/cmd";
	right_motor_topic = (std::string)channels[1] + "/cmd";

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

	left_motor_pub = nh.advertise<roboteq_msgs::Command>(left_motor_topic, 1);
	right_motor_pub = nh.advertise<roboteq_msgs::Command>(right_motor_topic, 1);

	ros::Subscriber cmd_vel_sub = nh.subscribe(cmd_vel_topic, 1, cmd_vel_callback);

	ros::spin();

	return 0;
}