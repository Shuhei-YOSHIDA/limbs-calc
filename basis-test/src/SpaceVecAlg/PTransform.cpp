#include "ros/ros.h"

#include<SpaceVecAlg/SpaceVecAlg> //Summary of header files of SpaceVecAlg

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ptransform");
    ros::NodeHandle n;
    
    Eigen::Matrix3d E; // rotation
    E << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0); // rotation
    Eigen::Vector3d r; // translation
    r << 10.0, 0.0, 0.0;

    sva::PTransformd X1(E,r); // constructors
    sva::PTransformd X2(q,r); // quaternion -> matrix
    sva::PTransformd X3 = sva::PTransformd::Identity();
    E = X1.rotation(); // rotation getter
    r = X1.translation();

    // inverse function
    if (E.transpose() == X1.inv().rotation()) 
            std::cout << "rot inv" << std::endl;
    if (-E*r == X1.inv().translation())
            std::cout << "rot trs" << std::endl;

}

