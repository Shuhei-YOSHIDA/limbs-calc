#include "ros/ros.h"

#include<SpaceVecAlg/SpaceVecAlg> //Summary of header files of SpaceVecAlg

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motionvec");
    ros::NodeHandle n;

    //MotionVec is the Spatial Motion Vector implementation
    Eigen::Vector3d w,v ;
    w << 1, 2, 3;
    v << 0.1, 0.2, 0.3;
    sva::MotionVecd mv1(w, v); // constructor
    w = mv1.angular(); // angular getter
    v = mv1.linear(); // linear getter
    ROS_INFO("%f, %f, %f, angular", w(0), w(1), w(2));
    ROS_INFO("%f, %f, %f, linear", v(0), v(1), v(2));
    
    Eigen::Vector3d w2,v2 ;
    w2 << 3, 2, 1;
    v2 << 0.3, 0.2, 0.1;
    sva::MotionVecd mv2(w, v); // constructor
    w2 = mv2.angular(); // angular getter
    v2 = mv2.linear(); // linear getter
    ROS_INFO("%f, %f, %f, angular", w2(0), w2(1), w2(2));
    ROS_INFO("%f, %f, %f, linear", v2(0), v2(1), v2(2));

    ros::Rate loop_rate(10);
    
    while (ros::ok()) {
    //    ROS_INFO("check");
        std::cout << (mv1+mv2) << std::endl;
        std::cout << (10.*mv1) << std::endl;
        loop_rate.sleep();
    }
}
