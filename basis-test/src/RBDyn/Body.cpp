#include "ros/ros.h"

#include<RBDyn/Body.h> 

/*
jorisv/sva_rbdyn_presentationのBodyのサンプルコードは古いみたい．
RBDyn/Body.hを見ると，idを含めそうなコンストラクタがない．
nameとidで被っているから？
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "body");
    ros::NodeHandle n;

    // int id; // Ver 0.9.0, id is not used?
    std::string name = "name";

    Eigen::Vector3d com(1,0,0); //origin to CoM translation
    double mass = 10; //rigid body mass
    Eigen::Vector3d h = com*mass; //first moment of mass
    Eigen::Matrix3d I; //rigid body at origin
    I << 0.01, 0.00, 0.00, 
         0.00, 0.01, 0.00, 
         0.00, 0.00, 0.01;

    sva::RBInertiad rbi(mass, h, I); //body inertia

    // constructors
    rbd::Body b1(rbi, name);
    rbd::Body b2(mass, com, I, name);

    if (rbi == b1.inertia()) // inertia getter
        std::cout << "inertia gotten:" << rbi << std::endl;
    //if (id == b1.id()) // id getter
       // std::cout << "id gotten:" << id << std::endl;
    if (name == b1.name()) // name getter
        std::cout << "name gotten:" << name << std::endl;
}
