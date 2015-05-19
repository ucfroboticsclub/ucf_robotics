#ifndef ENCODER_H
#define ENCODER_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <roboteq_msgs/Feedback.h>

class Encoder
{
public:
    Encoder(ros::NodeHandle nh);
    ~Encoder();
    void start();


private:
    int gear_ratio_;
    double wheel_radius_;
    double base_radius_;
    std::string left_feedback_topic_;
    std::string right_feedback_topic_;

    ros::NodeHandle node_handle_;
    ros::Subscriber roboteq_feedback_left_;
    ros::Subscriber roboteq_feedback_right_;
    ros::Publisher encoder_pub_;

    geometry_msgs::TwistWithCovarianceStamped encoder_msg_;

    // Linear speeds of each motor at the contact between wheel/ground
    double left_speed_;
    double right_speed_;

    // Linear and rotational velocities of robot body as calculated
    double linear_velocity_;
    double angular_velocity_;

    void leftCallback(const roboteq_msgs::FeedbackConstPtr& msg);
    void rightCallback(const roboteq_msgs::FeedbackConstPtr& msg);
    void updateTwistMsg();
    void initializeParams();
};

#endif