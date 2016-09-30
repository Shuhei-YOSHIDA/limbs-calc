#include "ros/ros.h"

#include<RBDyn/Joint.h> //Summary of header files of SpaceVecAlg

/*
 * jorisv/sva_rbdyn_presentationのサンプルとは違ってidが無い．nameのみ．
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint");
    ros::NodeHandle n;

    Joint::Type type; // Rev, Prism, Spherical, Planar, 
                      // Cylindrical, Free and Fixed
    Eigen::Vector3d axis; // For Rev, Prism and Cylindrical
    bool direction; // true forward, false backward
    int id;
    std::string name;
    

}
