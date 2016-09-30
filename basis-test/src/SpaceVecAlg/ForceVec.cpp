#include "ros/ros.h"

#include<SpaceVecAlg/SpaceVecAlg> //Summary of header files of SpaceVecAlg

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forcevec");
    ros::NodeHandle n;

    //MotionVec is the Spatial Motion Vector implementation
    Eigen::Vector3d w,v ;
    w << 1, 2, 3;
    v << 0.1, 0.2, 0.3;
    sva::ForceVecd fv1(w, v); // constructor
    w = fv1.couple(); // couple getter
    v = fv1.force(); // force getter
    ROS_INFO("%f, %f, %f, couple", w(0), w(1), w(2));
    ROS_INFO("%f, %f, %f, force", v(0), v(1), v(2));
    
    Eigen::Vector3d w2,v2 ;
    w2 << 3, 2, 1;
    v2 << 0.3, 0.2, 0.1;
    sva::ForceVecd fv2(w, v); // constructor
    w2 = fv2.couple(); // couple getter
    v2 = fv2.force(); // force getter
    ROS_INFO("%f, %f, %f, couple", w2(0), w2(1), w2(2));
    ROS_INFO("%f, %f, %f, force", v2(0), v2(1), v2(2));

    ros::Rate loop_rate(10);
    
    while (ros::ok()) {
    //    ROS_INFO("check");
        std::cout << (fv1+fv2) << std::endl;
        std::cout << (10.*fv1) << std::endl;
        loop_rate.sleep();
    }
}
