#include "ros/ros.h"

#include<RBDyn/MultiBody.h> 
#include<RBDyn/MultiBodyGraph.h>

// A non oriented graph that model the robot

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multibodygraph");
    ros::NodeHandle n;

    sva::RBInertiad rbi;
    rbd::Body b1(rbi, "b1"), b2(rbi, "b2"), b3(rbi, "b3");
    rbd::Joint j1(rbd::Joint::Rev, Eigen::Vector3d::UnitX(), true, "j1");
    rbd::Joint j2(rbd::Joint::Spherical, true, "j2");
    sva::PTransformd X_12, X_13;
    sva::PTransformd I(sva::PTransformd::Identity());

    rbd::MultiBodyGraph mbg; // constructor
    mbg.addBody(b1); mbg.addBody(b2); mbg.addBody(b3);
    mbg.addJoint(j1); mbg.addJoint(j2);

    // link body b1 to body b2 with joint j1 and static transform X_12
    mbg.linkBodies("b1", X_12, // body1 name, transform from body1 to joint in body 1
                   "b2", I,    // body2 name, transform from body2 to joint in body 2
                   "j1");      // joint name, (default bool is true, 回転方向)
    
    // link body b1 to body b3 with joint j2 and static transform X_13
    mbg.linkBodies("b1", X_13, // body1 name, transform from body1 to joint in body 1
                   "b3", I,    // body3 name, transform from body1 to joint in body 3
                   "j2");      // joint name, (default bool is true, 回転方向)

    // create a multibody with b1 as fixed root body
    rbd::MultiBody mb1 = mbg.makeMultiBody("b1", rbd::Joint::Fixed);

    sva::PTransformd X_O_j0, X_b0_j0;
    // multibody with b2 as planar base
    // X_O_j0 is the transform from the world to the planar joint
    // X_b0_j0 is the transform from b2 to the joint (X_T) 
    rbd::MultiBody mb2 = mbg.makeMultiBody("b2", rbd::Joint::Planar, X_O_j0, X_b0_j0);

    // remove a joint and his subtree
    // mbg.removeJoint(...);

    // merge a subtree in one body
    // mbg.mergeSubBodies(...);

    // compute bodies transformation to them origin
    // mbg.bodiesBaseTransform(...);
    
    std::cout << "num of bodies:" << mb1.nrBodies() << std::endl;
    std::cout << "num of joints:" << mb1.nrJoints() << std::endl;
    for ( auto itr = mb1.bodies().begin(); itr != mb1.bodies().end(); ++itr) 
    {
        std::cout << "bodies of multibody:" << *itr << std::endl;
    }
}
