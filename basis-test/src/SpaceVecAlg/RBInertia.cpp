#include "ros/ros.h"

#include<SpaceVecAlg/SpaceVecAlg> //Summary of header files of SpaceVecAlg

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rbinertia");
    ros::NodeHandle n;

    Eigen::Vector3d com(1,0,0); //origin to CoM translation
    double mass = 10; //rigid body mass
    Eigen::Vector3d h = com*mass; //first moment of mass
    Eigen::Matrix3d I; //rigid body at origin
    I << 0.01, 0.00, 0.00, 
         0.00, 0.01, 0.00, 
         0.00, 0.00, 0.01;

    sva::RBInertiad rbi1(mass, h, I); // constructor
    mass = rbi1.mass ();
    h = rbi1.momentum();
    std::cout << h << " momentum" << std::endl;
    I = rbi1.inertia();
    std::cout << I << " inertia" << std::endl;
    
    Eigen::Vector3d com2(1,0,0); //origin to CoM translation
    double mass2 = 10; //rigid body mass
    Eigen::Vector3d h2 = com*mass; //first moment of mass
    Eigen::Matrix3d I2; //rigid body at origin
    I2 << 0.01, 0.00, 0.00, 
         0.00, 0.01, 0.00, 
         0.00, 0.00, 0.01;

    sva::RBInertiad rbi2(mass2, h2, I2);
    std::cout << (rbi1 + rbi2) << std::endl;
    std::cout << (10.*rbi1) << std::endl;
    
    Eigen::Vector3d w,v ;
    w << 1, 2, 3;
    v << 0.1, 0.2, 0.3;
    sva::MotionVecd mv(w, v);
    sva::ForceVecd fv = rbi1*mv; //motion space to force space
    std::cout << fv << " force" << std::endl;


    ros::Rate loop_rate(10);
    
    while (ros::ok()) {
    //    ROS_INFO("check");
        loop_rate.sleep();
    }
}
