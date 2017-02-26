#include <ros/ros.h>
#include <urdf/model.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/ID.h>
#include <RBDyn/FD.h>
#include <RBDyn/CoM.h>

using namespace Eigen;
using namespace urdf;
using namespace std;
Model model;

bool readUrdf()
{
    if (!model.initParam("robot_description")) {
        return false;
    }
    return true;
}

void treeParse(LinkConstSharedPtr link, int level = 0)
{
    level+=2;//for indent
    int count = 0;
    for (std::vector<LinkSharedPtr>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
        if (*child) {
            for (int j=0;j<level;j++) std::cout <<"  ";//indent
            std::cout << "child(" << (count++)+1 << "): " << (*child)->name << std::endl;
            // next generation
            treeParse(*child, level);
        }
        else { // In case of end link, this part does not processed?
            for (int j=0;j<level;j++) std::cout <<"  ";//indent
            std::cout << "root link: " << link->name << "has no child " << *child << std::endl;
        }
    }

}

rbd::MultiBodyGraph mbg;
void setMultiBodyGraph(LinkConstSharedPtr link);
void setGraph(LinkConstSharedPtr link)
{
    cout << "set graph" << endl;
    //set Root link
    auto iner = link->inertial;
    double mass; Vector3d com; Matrix3d Io;
    if (iner ) {
        cout << "set inertial of Root link" << endl;
        mass = iner->mass;
        com << iner->origin.position.x, 
               iner->origin.position.y, 
               iner->origin.position.z;
        // No rotation may be between joint and CoM frame
        Matrix3d Ic;
        Ic << iner->ixx, iner->ixy, iner->ixz, 
              iner->ixy, iner->iyy, iner->iyz, 
              iner->ixz, iner->iyz, iner->izz;
        Matrix3d E = Matrix3d::Identity();
        Io = sva::inertiaToOrigin(Ic, mass, com, E);
    }

    sva::RBInertiad rbi(mass, mass*com, Io);
    rbd::Body rlink(rbi, link->name);
    mbg.addBody(rlink);
    cout << "Root link " << link->name << " is set " << endl;

    setMultiBodyGraph(link);
}
// Root link is not added to graph. Use root as virtual link or add it otherway
void setMultiBodyGraph(LinkConstSharedPtr link)
{
    for (std::vector<LinkSharedPtr>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
        if (*child) {
            // set body to graph
            auto iner = (*child)->inertial;
            double mass; Vector3d com; Matrix3d Io;
            if (iner) {
                mass = iner->mass;
                com << iner->origin.position.x, 
                       iner->origin.position.y, 
                       iner->origin.position.z;
                // tensor at CoM for URDF is converted to at Origin(Joint frame?)
                // No rotation may be between joint and CoM frame
                Matrix3d Ic;
                Ic << iner->ixx, iner->ixy, iner->ixz, 
                      iner->ixy, iner->iyy, iner->iyz, 
                      iner->ixz, iner->iyz, iner->izz;
                Matrix3d E = Matrix3d::Identity();
                Io = sva::inertiaToOrigin(Ic, mass, com, E);
            }// If added body's inertia is null, what occur?

            sva::RBInertiad rbi(mass, mass*com, Io);
            rbd::Body rlink(rbi, (*child)->name);

            mbg.addBody(rlink);

            // set joint(to parent) to graph
            auto joint = (*child)->parent_joint;
            
            rbd::Joint::Type type;
            Vector3d axis;
            bool typecheck = true;
            switch (joint->type) { //urdf joint define
                case Joint::REVOLUTE: {
                    type = rbd::Joint::Rev;
                    axis << joint->axis.x, joint->axis.y, joint->axis.z;
                }
                break;
                case Joint::FIXED: {
                    type = rbd::Joint::Fixed;
                    axis = Vector3d::UnitZ();
                }
                break;
                case Joint::FLOATING: 
                    type = rbd::Joint::Free;
                    axis = Vector3d::UnitZ();
                break;
                default:
                    typecheck = false;
                break;
            }
            if (typecheck) {//Otherwise, this code will fail?
                // add joint to graph and link joint and body
                rbd::Joint jnt(type, axis, true, joint->name);
                mbg.addJoint(jnt);
                auto pToj = joint->parent_to_joint_origin_transform;
                Vector3d r_pj(pToj.position.x, pToj.position.y, pToj.position.z);
                Quaterniond q_pj(pToj.rotation.w, pToj.rotation.x, pToj.rotation.y, pToj.rotation.z);

                //sva::PTransformd X_PJ(q_pj, r_pj);//parent's link to joint
                sva::PTransformd X_PJ;//parent's link to joint
                sva::PTransformd X_LJ(sva::PTransformd::Identity());//child's link to joint
                
                //Root Link is added before this method
                mbg.linkBodies(link->name, X_PJ, 
                               (*child)->name, X_LJ, 
                               joint->name);
            }
            std::cout << (*child)->name << " is set"  << std::endl;
            // next generation
            setMultiBodyGraph(*child);
        }
    }
   
    
    return;  
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "multiLinkCalc");
    ros::NodeHandle n;
    
    readUrdf();
    LinkConstSharedPtr root_link = model.getRoot();
    treeParse(root_link);
    setGraph(root_link);
    cout << "graph is already set" << endl;

    ros::spin();

    return 0;
}
