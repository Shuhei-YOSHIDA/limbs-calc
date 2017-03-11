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

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <random>


//for compatibility of indigo and kinetic
#include <memory>
typedef boost::shared_ptr<urdf::Link> _LinkSharedPtr;
typedef boost::shared_ptr<const urdf::Link> _LinkConstSharedPtr;
using namespace Eigen;
using namespace urdf;
using namespace std;
Model model;

class task {// interface class
    
    public:
        virtual ~task(){};
        virtual VectorXd g(rbd::MultiBody mb, rbd::MultiBodyConfig mbc) = 0;
        virtual MatrixXd J(rbd::MultiBody mb, rbd::MultiBodyConfig mbc) = 0;
};

class BodyTask : public task {

    public:
        BodyTask(){};
        VectorXd g(rbd::MultiBody mb, rbd::MultiBodyConfig mbc);
        MatrixXd J(rbd::MultiBody mb, rbd::MultiBodyConfig mbc);
};

bool readUrdf()
{
    if (!model.initParam("robot_description")) {
        return false;
    }
    return true;
}

void treeParse(_LinkConstSharedPtr link, int level = 0)
{
    level+=2;//for indent
    int count = 0;
    for (std::vector<_LinkSharedPtr>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
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

void msgFromMultiBodyConfig(rbd::MultiBody mb, rbd::MultiBodyConfig mbc, sensor_msgs::JointState& msg)
{
    //int rbd::MultiBody::sBodyIndexByName(name)
    //int rbd::MultiBody::sJointIndexByName(name)
    //sensor_msgs::JointState msg;
    int count = 0;
    for (auto itr = mbc.q.begin(); itr != mbc.q.end(); ++itr) {
        if (mb.joint(count).type() == rbd::Joint::Type::Rev ||
            mb.joint(count).type() == rbd::Joint::Type::Prism) {// 1dof joint
            msg.name.push_back(mb.joint(count).name());
            msg.position.push_back(*(itr->begin()));
        } 
        count++;
    }
    msg.header.stamp = ros::Time::now();
}

rbd::MultiBodyGraph mbg;
void setMultiBodyGraph(_LinkConstSharedPtr link);
void setGraph(_LinkConstSharedPtr link)
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
void setMultiBodyGraph(_LinkConstSharedPtr link)
{
    for (std::vector<_LinkSharedPtr>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
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
                case Joint::REVOLUTE: { //limits should be added
                    type = rbd::Joint::Rev;
                    axis << joint->axis.x, joint->axis.y, joint->axis.z;
                }
                break;
                case Joint::CONTINUOUS: { //no-limit rotation
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

                sva::PTransformd X_PJ(q_pj, r_pj);//parent's link to joint
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



void sample3(sensor_msgs::JointState &jmsg, visualization_msgs::MarkerArray &amsg)
{
    rbd::MultiBody mb = mbg.makeMultiBody("base_link", rbd::Joint::Fixed);
    rbd::MultiBodyConfig mbc(mb);
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_int_distribution<int> ranj(0,100);
    for (auto itr = mbc.q.begin(); itr != mbc.q.end(); ++itr) {
        for (auto itr2 = itr->begin(); itr2 != itr->end(); ++itr2) {
            *itr2 = ranj(mt)/100.0; 
            //*itr2 = 0.0; 
        }
    }

    rbd::forwardKinematics(mb, mbc);
    int count = 0;
    auto stamp = ros::Time::now();
    for (auto itr = mbc.bodyPosW.begin(); itr != mbc.bodyPosW.end(); ++itr) {
        //cout << mb.body(count).name() << endl;
        //cout << "transration" << endl << itr->translation() << endl;
        //cout << "rotation" << endl << itr->rotation() << endl;

        visualization_msgs::Marker mrk;
        mrk.header.stamp = stamp;
        mrk.header.frame_id = "base_link";
        mrk.id = count;
        mrk.text = mb.body(count).name();
        mrk.type = visualization_msgs::Marker::ARROW;
        mrk.pose.position.x = itr->translation()(0);
        mrk.pose.position.y = itr->translation()(1);
        mrk.pose.position.z = itr->translation()(2);
        auto q = Eigen::Quaterniond(itr->rotation());
        mrk.pose.orientation.x = q.x();
        mrk.pose.orientation.y = q.y();
        mrk.pose.orientation.z = q.z();
        mrk.pose.orientation.w = -q.w();//Consistency for TF
        mrk.scale.x = 0.1; mrk.scale.y = 0.01; mrk.scale.z = 0.01;
        mrk.color.r = 0.5; mrk.color.g = 0.0; mrk.color.b = 0.0; mrk.color.a = 0.5; 
        amsg.markers.push_back(mrk);

        count++;
    }

    msgFromMultiBodyConfig(mb, mbc, jmsg);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "multiLinkCalc2");
    ros::NodeHandle n;
    ros::Publisher j_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher a_pub = n.advertise<visualization_msgs::MarkerArray>("markers", 1);
    
    readUrdf();
    _LinkConstSharedPtr root_link = model.getRoot();
    treeParse(root_link);
    setGraph(root_link);
    cout << "graph is already set" << endl;

    cout << "sample3" << endl;
    while (ros::ok()) {
        sensor_msgs::JointState jmsg;
        visualization_msgs::MarkerArray amsg;
        sample3(jmsg, amsg);
        a_pub.publish(amsg);
        j_pub.publish(jmsg);
        ros::Duration(1.0).sleep();
    }
    cout << "end " << endl;

    ros::spin();

    return 0;
}

