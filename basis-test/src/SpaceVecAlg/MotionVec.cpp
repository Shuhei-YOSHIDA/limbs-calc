#include "ros/ros.h"

#include<SpaceVecAlg>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motionvec");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);
    
    while (ros::ok()) {
        ROS_INFO("check");
    }
}
