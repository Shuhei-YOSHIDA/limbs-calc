#include "ros/ros.h"

#include<RBDyn/Joint.h> 

/*
 * jorisv/sva_rbdyn_presentationのサンプルとは違ってidが無い．nameのみ．
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint");
    ros::NodeHandle n;

    rbd::Joint::Type type = rbd::Joint::Rev; // Rev, Prism, Spherical, Planar, 
                            // Cylindrical, Free and Fixed
    Eigen::Vector3d axis(1.0, 0.0, 0.0); // For Rev, Prism and Cylindrical
    bool direction = true; // true forward, false backward
    // int id;
    std::string name = "sample";
    
    // constructors
    rbd::Joint j1(type, axis, direction, name);
    rbd::Joint j2(type, direction, name);
    
    // getter
    if (type == j1.type())
        std::cout << "type gotten:" << j1.type() << std::endl;

    if (direction == j1.forward())
        std::cout << "direction gotten:" << j1.forward() << std::endl;
    
    if (name == j1.name())
        std::cout << "name gotten:" << j1.name() << std::endl;

    std::cout << "q vector size gotten:" << j1.params() << std::endl;

    std::cout << "dof gotten:" << j1.dof() << std::endl;
    
    std::cout << "motion space matrix gotten:" << j1.motionSubspace() << std::endl;

    // transformation and velocity
    std::vector<double> q(j1.params()), alpha(j1.dof()), alphaDot(j1.dof());
    std::cout << j1.pose(q) << std::endl; // X_j transformation matrix
    std::cout << j1.motion(alpha) << std::endl;  // v_j motion vector
    std::cout << j1.tanAccel(alphaDot) << std::endl;  // tangential acceleration

    // initialization
    j1.zeroParam(); // identity q vector
    j1.zeroDof(); // identity alpha, alphaDot vector

}
