#include "ros/ros.h"

#include<SpaceVecAlg/SpaceVecAlg> //Summary of header files of SpaceVecAlg

int main(int argc, char **argv)
{
    ros::init(argc, argv, "utilities");
    ros::NodeHandle n;

    double theta = 0.0;
    Eigen::Matrix3d Ex = sva::RotX(theta); // X rotation
    Eigen::Matrix3d Ey = sva::RotY(theta); // Y rotation
    Eigen::Matrix3d Ez = sva::RotZ(theta); // Z rotation

    // 3d projection of rotation error
    // x, y, z rotation to go from Ex to Ey
    Eigen::Vector3d err = sva::rotationError(Ex, Ey);
    std::cout << err << std::endl;

    // compute inertia at origin from inertia at CoM
    Eigen::Matrix3d IatCoM, IatO;
    IatCoM << 1.0, 0.0, 0.0, 
              0.0, 0.5, 0.0,
              0.0, 0.0, 0.1; 
    Eigen::Matrix3d E; // rotation from origin to com frame
    E << 1.0, 0.0, 0.0, 
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    Eigen::Vector3d com; // translation from origin to com
    com << 10.0, 0, 0;
    double mass = 1.0;
    IatO = sva::inertiaToOrigin(IatCoM, mass, com, E);
    std::cout << IatO << std::endl;
}

