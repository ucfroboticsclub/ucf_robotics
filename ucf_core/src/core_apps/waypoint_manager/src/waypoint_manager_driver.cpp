#include "ros/ros.h"
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include "waypoint_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_manager_node");
    ros::NodeHandle nh("~");

    std::string file_path = "/home/kenneth/temp/waypoints_test.txt";

    nh.getParam("waypoint_file", file_path);

    // Variables for re-use in readfile loop.
    std::string input_line;
    std::vector<double> waypoint_values;
    boost::char_separator<char> separator(", ");
    std::vector<geometry_msgs::Pose> initial_poses;
    geometry_msgs::Pose pose;

    std::ifstream infile;
    infile.open(file_path.c_str());
    int line_number = 0;

    if (infile.is_open())
    {
        ROS_INFO_STREAM("Waypoint file found: " << file_path);

        while (getline(infile, input_line))
        {
            ++line_number;

            // Check for comments or an empty line.  Continue to next line if found.
            if (input_line[0] == '#' || input_line.empty()) { continue; }

            boost::tokenizer< boost::char_separator<char> > list(input_line, separator);
            boost::tokenizer< boost::char_separator<char> >::const_iterator iter;

            // Cast and push all values read as doubles.
            for (iter = list.begin(); iter != list.end(); iter++)
            {
                waypoint_values.push_back(boost::lexical_cast<double>(*iter));
            }

            // If we didn't read 7 values to construct the pose, kill the process.
            if (waypoint_values.size() != 7)
            {
                ROS_ERROR_STREAM("Input file malformed at line " << line_number << ", killing process.");
                return 1;
            }

            // Build pose.
            pose.position.x = waypoint_values[0];
            pose.position.y = waypoint_values[1];
            pose.position.z = waypoint_values[2];
            pose.orientation.x = waypoint_values[3];
            pose.orientation.y = waypoint_values[4];
            pose.orientation.z = waypoint_values[5];
            pose.orientation.w = waypoint_values[6];

            // Push pose and reset waypoint_values vector.
            initial_poses.push_back(pose);
            waypoint_values.clear();
        }

        infile.close();
    }
    else
    {
        ROS_ERROR_STREAM("File not found, tried " << file_path << ", killing process!");
        return 1;
    }

    // Start main node functionality.
    WaypointManager manager(nh, initial_poses);
    manager.start();

    return 0;
}