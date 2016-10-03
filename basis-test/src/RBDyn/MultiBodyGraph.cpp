#include "ros/ros.h"

#include<RBDyn/MultiBody.h> 
#include<RBDyn/MultiBodyGraph.h>
#include<RBDyn/MultiBodyConfig.h> 
//Algorithm
#include<RBDyn/FK.h>
#include<RBDyn/FV.h>
#include<RBDyn/ID.h>
#include<RBDyn/FD.h>
#include<RBDyn/CoM.h>

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
    
    // bodies number : b1, b2, b2 = 3
    // joints number : joint1:root - b1, joint2:b1 - b2, joint3: b1 - b3, for example
    std::cout << "num of bodies:" << mb1.nrBodies() << std::endl;
    std::cout << "num of joints:" << mb1.nrJoints() << std::endl;
    for ( auto itr = mb1.bodies().begin(); itr != mb1.bodies().end(); ++itr) 
    {
        std::cout << "bodies of multibody:" << *itr << std::endl;
    }

    for ( auto itr = mb1.joints().begin(); itr != mb1.joints().end(); ++itr) 
    {
        std::cout << "joints of multibody:" << *itr << std::endl;
    }

    //-------------------------------------------------
    // MultiBody - Kinematics Tree Implementation:
    std::cout << "kinematics sample" << std::endl;
    
    // body that pred/succ a joint
    for (auto itr = mb1.predecessors().begin(); itr != mb1.predecessors().end(); ++itr) 
    {
        std::cout << "predecessors:" << *itr << std::endl; 
    }

    for (auto itr = mb1.successors().begin(); itr != mb1.successors().end(); ++itr) 
    {
        std::cout << "successors:" << *itr << std::endl; 
    }

    for (auto itr = mb1.parents().begin(); itr != mb1.parents().end(); ++itr) 
    {
        std::cout << "parents:" << *itr << std::endl; // lambda
    }

    for (auto itr = mb1.transforms().begin(); itr != mb1.transforms().end(); ++itr) 
    {
        std::cout << "transforms:" << *itr << std::endl; // X_T array
    }

    // body/joint index by id
    std::cout << "body index exam:" << mb1.bodyIndexByName("b1") << std::endl;
    std::cout << "joint index exam:" << mb1.jointIndexByName("j1") << std::endl;
    if (mb1.bodyIndexByName("b1") != mb2.bodyIndexByName("b1")) {
        std::cout << "configuration of multibody1 and 2 is different" << std::endl;
    }
    
    std::cout << "Params = Spherical:4 + Rev:1 + Fixed:0" << std::endl;
    std::cout << "Params:" << mb1.nrParams() << std::endl;
    std::cout << "Dof = Spherical:3 + Rev:1 + Fixed:0" << std::endl;
    std::cout << "Dof:" << mb1.nrDof() << std::endl;

    std::cout << "Params = Spherical:4 + Rev:1 + Planar:3" << std::endl;
    std::cout << "Params:" << mb2.nrParams() << std::endl;
    std::cout << "Dof = Spherical:3 + Rev:1 + Planar:3" << std::endl;
    std::cout << "Dof:" << mb2.nrDof() << std::endl;

    unsigned int count = 0;
    for (auto itr = mb1.jointsPosInParam().begin(); itr != mb1.jointsPosInParam().end(); ++itr) 
    {
        std::cout << count << ":jointPos in flat q:" << *itr << std::endl;
        count++;
    }

    count = 0;
    for (auto itr = mb1.jointsPosInDof().begin(); itr != mb1.jointsPosInDof().end(); ++itr) 
    {
        std::cout << count << ":jointPos in flat alpha:" << *itr << std::endl;
        count++;
    }


    //-------------------------------------------------
    // MultiBodyConfig - configuration of a multi body is separeted from his model
    std::cout << "MultiBodyConfig example" << std::endl;

    rbd::MultiBodyConfig mbc1(mb1);
    mbc1.zero(mb1); // set q, alpha, alphaD and jointTorque to 0

    // indexed by joint
    // std::vector<std::vector<double>>
    count = 0;
    std::cout << "q generalized position vector" << std::endl;
    for (auto itr = mbc1.q.begin(); itr != mbc1.q.end(); ++itr) {
        std::cout << count << std::endl;
        count++;
        for (auto itr2 = itr->begin(); itr2 != itr->end(); ++itr2) {
            std::cout << *itr2 << std::endl;
        }
    }

    count = 0;
    std::cout << "alpha generalized velocity vector" << std::endl;
    for (auto itr = mbc1.alpha.begin(); itr != mbc1.alpha.end(); ++itr) {
        std::cout << count << std::endl;
        count++;
        for (auto itr2 = itr->begin(); itr2 != itr->end(); ++itr2) {
            std::cout << *itr2 << std::endl;
        }
    }

    count = 0;
    std::cout << "q generalized acceleration vector" << std::endl;
    for (auto itr = mbc1.alphaD.begin(); itr != mbc1.alphaD.end(); ++itr) {
        std::cout << count << std::endl;
        count++;
        for (auto itr2 = itr->begin(); itr2 != itr->end(); ++itr2) {
            std::cout << *itr2 << std::endl;
        }
    }

    count = 0;
    std::cout << "joint torque" << std::endl;
    for (auto itr = mbc1.jointTorque.begin(); itr != mbc1.jointTorque.end(); ++itr) {
        std::cout << count << std::endl;
        count++;
        for (auto itr2 = itr->begin(); itr2 != itr->end(); ++itr2) {
            std::cout << *itr2 << std::endl;
        }
    }
    
    count = 0;
    std::cout << "motion subspace matrix" << std::endl;
    for (auto itr = mbc1.motionSubspace.begin(); itr != mbc1.motionSubspace.end(); ++itr) {
        std::cout << count << std::endl;
        count++;
        std::cout << "matrix" << std::endl;
        std::cout << *itr << std::endl;
    }

    // indexed by body
    count = 0;
    std::cout << "Force apply on each body" << std::endl;
    for (auto itr = mbc1.force.begin(); itr != mbc1.force.end(); ++itr) {
        std::cout << count << ":" << *itr << std::endl;
        count++;
    }

    count = 0;
    std::cout << "Body transformation in world coord" << std::endl;
    for (auto itr = mbc1.bodyPosW.begin(); itr != mbc1.bodyPosW.end(); ++itr) {
        std::cout << count << ":" << *itr << std::endl;
        count++;
    }

    count = 0;
    std::cout << "Body velocity in world coord" << std::endl;
    for (auto itr = mbc1.bodyVelW.begin(); itr != mbc1.bodyVelW.end(); ++itr) {
        std::cout << count << ":" << *itr << std::endl;
        count++;
    }

    count = 0;
    std::cout << "Body velocity in body coord" << std::endl;
    for (auto itr = mbc1.bodyVelB.begin(); itr != mbc1.bodyVelB.end(); ++itr) {
        std::cout << count << ":" << *itr << std::endl;
        count++;
    }

    std::cout << "gravity apply on the system:" << std::endl << mbc1.gravity << std::endl;

    //-------------------------------------------------
    // MultiBodyConfig useful functions

    rbd::MultiBodyConfig mbc2(mb2);
    mbc2.zero(mb2); // set q, alpha, alphaD and jointTorque to 0
    // convert q, alpha, alphaD and force vector
    // from mbc1 to mbc2
    // mbc1 and mbc2 must come from the same mbg
    // don't convert root joint
    
    rbd::ConfigConverter(mb1, mb2); //order:from, to, 
    //rbd::configConverter.convertJoint(...)
    
    //Eigen::VectorXd q;
    // to and from flat vector
    //rbd::paramToVector(mbc1.q, q);
    //rbd::vectorToParam(q, mbc1.q);

    //-------------------------------------------------
    // Algorithm
    
    // MultiBodyConfig has member such as bodyPosW, ... and so on. 

    // q -> bodyPosW
    rbd::forwardKinematics(mb1, mbc1); // result is in multibodyconfig

    // alpha, FK -> bodyVelW, bodyVelB, S
    rbd::forwardVelocity(mb1, mbc1);

    // alphaD, gravity, FV -> jointTorque
    rbd::InverseDynamics id(mb1);
    id.inverseDynamics(mb1, mbc1);

    // jointTorque, FV -> alphaD, H, C
    rbd::ForwardDynamics fd(mb1);
    fd.forwardDynamics(mb1, mbc1);

    // FK -> CoM
    std::cout << "com:" << std::endl << 
        //rbd::computeCoM(mb1, mbc1) // return as Eigen::Vector3d
        "Set Mass!" << std::endl;

    // FV -> CoM speed
    std::cout << "com velocity:" << std::endl <<
        //rbd::computeCoMVelocity(mb1, mbc1);
        "Set Mass!" << std::endl;

    //-------------------------------------------------
    // Jacobian
    // Like the 6D vector, the rotation part of the Jacobian is the 3 first
    // row ant the translation part is the 3 last row. 

    Eigen::Vector3d bodyPoint(0.1, 0, 0);
    rbd::Jacobian jac(mb1, "b1", bodyPoint);
    std::cout << "6D bodyName world coord jacobian at bodyPoint" << std::endl;
    std::cout << jac.jacobian(mb1, mbc1) << std::endl;
    std::cout << "bodyName coordinate" << std::endl;
    std::cout << jac.bodyJacobian(mb1, mbc1) << std::endl;

    Eigen::Vector3d vec(0, 0.1, 0);
    std::cout << "3D world coord jacobian of a vector in bodyName" << std::endl;
    std::cout << jac.vectorJacobian(mb1, mbc1, vec);
    std::cout << "bodyName coordinate jacobian" << std::endl;

    
    std::cout << jac.vectorBodyJacobian(mb1, mbc1, vec);

    //std::cout << "translate a jacobian to a specific point" << std::endl;
    //jac.translateJacobian(...);
    //std::cout << "project a jacobian in the robot parameter vector" << std::endl;
    //jac.fullJacobian(...);

    
    //-------------------------------------------------
    // Other Stuff
    // * Jacobian time derivative j_dot computation
    // * Jacobian normal acceleration vector j_dot*q_dot compuation
    // * CoM Jacobian
    // * Centroidal Momentum MAtrix A_g and A_g_dot
    // * Centroidal ZMP
    // * Euler Integration

    //-------------------------------------------------
    // Example: problem
    // Find collisionPossible by using sch library


    //-------------------------------------------------
    // Example: optim
}
