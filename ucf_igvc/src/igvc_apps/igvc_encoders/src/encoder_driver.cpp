#include "encoder.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "igvc_encoders_node");
    ros::NodeHandle nh("~");

    Encoder encoder(nh);
    encoder.start();

    return 0;
}