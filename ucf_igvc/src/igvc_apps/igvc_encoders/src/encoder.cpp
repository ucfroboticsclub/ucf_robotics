#include "encoder.h"
#include <iostream>

// TODO: Clean up parameter handling between this and the driver node.
// TODO: Discuss merging this entire package into part of the igvc_vel_controller package

Encoder::Encoder(ros::NodeHandle nh)
    : node_handle_(nh), left_speed_(0), right_speed_(0)
{
    initializeParams();

    roboteq_feedback_left_ = node_handle_.subscribe(left_feedback_topic_, 1, &Encoder::leftCallback, this);
    roboteq_feedback_right_ = node_handle_.subscribe(right_feedback_topic_, 1, &Encoder::rightCallback, this);
    encoder_pub_ = node_handle_.advertise<geometry_msgs::TwistWithCovarianceStamped>("twist", 10);

    encoder_msg_.header.frame_id = "base_footprint";
    encoder_msg_.header.seq = 0;
    // TODO: figure out covariance to use
    encoder_msg_.twist.covariance.fill(0);
    encoder_msg_.twist.twist.linear.y = 0;
    encoder_msg_.twist.twist.linear.z = 0;
    encoder_msg_.twist.twist.angular.x = 0;
    encoder_msg_.twist.twist.angular.y = 0;
}

Encoder::~Encoder()
{

}

void Encoder::start()
{
    while (ros::ok())
    {
        ros::spinOnce();
        updateTwistMsg();
        encoder_pub_.publish(encoder_msg_);
    }
}

void Encoder::leftCallback(const roboteq_msgs::FeedbackConstPtr& msg)
{
    left_speed_ = (msg->measured_velocity / gear_ratio_) * wheel_radius_;
}

void Encoder::rightCallback(const roboteq_msgs::FeedbackConstPtr& msg)
{
    right_speed_ = (msg->measured_velocity / gear_ratio_) * wheel_radius_;
}

void Encoder::updateTwistMsg()
{
    // Calculate as member variable for readability.
    linear_velocity_ = (left_speed_ + right_speed_) / 2;
    angular_velocity_ = (right_speed_ - left_speed_) / (2 * base_radius_);

    // Populate the message fields.
    encoder_msg_.twist.twist.angular.z = angular_velocity_;
    encoder_msg_.twist.twist.linear.x = linear_velocity_;

    // Stamp current time.
    encoder_msg_.header.stamp.sec = ros::Time::now().sec;
    encoder_msg_.header.stamp.nsec = ros::Time::now().nsec;
}

void Encoder::initializeParams()
{
    // Set up left motor feedback topic.
    if (!node_handle_.getParam("left_motor_feedback_topic", left_feedback_topic_))
    {
        ROS_WARN("left_motor_feedback_topic not found, defaulting to /left_motor/feedback");
        left_feedback_topic_ = "/left_motor/feedback";
    }
    // Set up right motor feedback topic.
    if (!node_handle_.getParam("right_motor_feedback_topic", right_feedback_topic_))
    {
        ROS_WARN("right_motor_feedback_topic not found, defaulting to /right_motor/feedback");
        right_feedback_topic_ = "/right_motor/feedback";
    }
    // Get wheel radius
    if (!node_handle_.getParam("wheel_radius", wheel_radius_))
    {
        ROS_WARN("wheel_radius not found, defaulting to 0.15875");
        wheel_radius_ = 0.15875;
    }
    // Get base radius.
    if (!node_handle_.getParam("base_radius", base_radius_))
    {
        ROS_WARN("base_radius not found, defaulting to 0.35560");
        base_radius_ = 0.35560;
    }
    // Get motor (gear?) ratio.
    if (!node_handle_.getParam("gear_ratio", gear_ratio_))
    {
        ROS_WARN("gear_ratio not found, defaulting to 32");
        gear_ratio_ = 32;
    }
}