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
#include <RBDyn/EulerIntegration.h>

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <random>

#include "pseudoInverse.h"

// Inspiring and Been Based on Official python-bind tutorial
// http://nbviewer.jupyter.org/github/jorisv/sva_rbdyn_tutorials/blob/master/MyFirstIK.ipynb
//for compatibility of indigo and kinetic
#include <memory>
typedef boost::shared_ptr<urdf::Link> _LinkSharedPtr;
typedef boost::shared_ptr<const urdf::Link> _LinkConstSharedPtr;
using namespace Eigen;
using namespace urdf;
using namespace std;
Model model;

class Task {// interface class

public:
    virtual ~Task(){};
    virtual VectorXd g(rbd::MultiBody mb, rbd::MultiBodyConfig mbc) = 0;
    virtual MatrixXd J(rbd::MultiBody mb, rbd::MultiBodyConfig mbc) = 0;
    string _name;
};

typedef boost::shared_ptr<Task> TaskPtr;
typedef std::vector<pair<double, TaskPtr>> MultiTaskPtr;//priority and Task

class BodyTask : public Task {

public:
    BodyTask(rbd::MultiBody mb, string bodyName, sva::PTransformd X_O_T, 
             sva::PTransformd X_b_p = sva::PTransformd::Identity(), string name = "BodyTask")
    {
        /*
        Compute the error and the jacobian to target a static frame for a body.
        Parameters: 
        - mb: MultiBody
        - bodyId: ID of the body that should move
        - X_0_T: targeted frame (PTransformd)
        - X_b_p: static frame on the body bodyId
        */
        _name = name;
        _bodyName = bodyName;
        _bodyIndex = mb.bodyIndexByName(bodyName);
        _X_O_T = X_O_T;
        _X_b_p = X_b_p;
        _jac = rbd::Jacobian(mb, bodyName);
        _jac_mat_sparse = MatrixXd::Zero(6, mb.nrDof());//mb.nrParams?
        //check
        //cout << "body name is " << _bodyName << endl;
        //cout << "bodyindes is " << _bodyIndex << endl;
        //cout << "XOT translation " << endl << _X_O_T.translation() << endl
        //     << "XOT rotation " << endl << _X_O_T.rotation() << endl;
        //cout << "xbp translation " << endl << _X_b_p.translation() << endl
        //     << "xbp rotation " << endl << _X_b_p.rotation() << endl;
        //cout << "jac_mat_dense " << _jac_mat_sparse << endl;


    }

    sva::PTransformd X_O_p(rbd::MultiBodyConfig mbc) 
    {
        //check
        //cout << "calc xop" << endl;
        //cout << "size " << mbc.bodyPosW.size() << endl;
        sva::PTransformd X_O_b(mbc.bodyPosW[_bodyIndex]); //Segmentation  fault!!!
        //cout << "xop..." << endl;
        sva::PTransformd xop = _X_b_p * X_O_b;
        //check
        //cout << "localxobtra " << endl << X_O_b.translation() << endl
        //     << "localxobrot " << endl << X_O_b.rotation() << endl;
        //cout << "localxoptra " << endl << xop.translation() << endl
        //     << "localxoprot " << endl << xop.rotation() << endl;
        return xop; 
    }

    virtual VectorXd g(rbd::MultiBody mb, rbd::MultiBodyConfig mbc)
    {
        //check
        //cout << "calc g" << endl;
        auto X_O_p = this->X_O_p(mbc);
        auto g_body = sva::transformError(_X_O_T, X_O_p);//MotionVec
        //check
        //cout << "task.g " << endl;
        //cout << g_body.vector() << endl;
        return g_body.vector();
    }

    virtual MatrixXd J(rbd::MultiBody mb, rbd::MultiBodyConfig mbc)
    {
        sva::PTransformd X_O_p = this->X_O_p(mbc);
        // Set transformation in Oringin orientation frame
        sva::PTransformd X_O_p_0 = sva::PTransformd(X_O_p.rotation()).inv() * X_O_p;
        MatrixXd jac_mat_dense = _jac.jacobian(mb, mbc, X_O_p_0); // おかしい？
        _jac.fullJacobian(mb, jac_mat_dense, _jac_mat_sparse);
        //check
        //cout << "Xop transltation " << endl << X_O_p.translation() << endl
        //     << "Xop rotation " << endl << X_O_p.rotation() << endl;
        //cout << "Xop0 transltation " << endl << X_O_p_0.translation() << endl
        //     << "Xop0 rotation " << endl << X_O_p_0.rotation() << endl;
        //cout << "dense " << endl
        //     << jac_mat_dense << endl;
        //cout << "task.J " << endl;
        //cout << _jac_mat_sparse << endl;
        return _jac_mat_sparse;
    }
    string _bodyName;
    int _bodyIndex;
    sva::PTransformd _X_O_T;
    sva::PTransformd _X_b_p;
    rbd::Jacobian  _jac;
    MatrixXd _jac_mat_sparse;

};

class PostureTask : public Task {

public:
    PostureTask(rbd::MultiBody mb, rbd::MultiBodyConfig mbc, string name = "PostureTask")
    {
        _name = name;
        _q_T = mbc; //need only one reference articular position vector

        //use joint type: prism, rev and Sperical
        auto isDefine =[](rbd::Joint j) { 
            if (j.type() == rbd::Joint::Prism || 
                j.type() == rbd::Joint::Rev   ||
                j.type() == rbd::Joint::Spherical) return true;
            else return false;
        };
        
        // take back joint and joint index that are define
        auto alljoints = mb.joints();
        int count = 0;
        for (auto itr = alljoints.begin(); itr != alljoints.end(); ++itr) {
            if (isDefine(*itr)) {
                _jointIndex.push_back(count);
                _joints.push_back(*itr);
            }
            count++;
        }
        int nrDof = 0;
        for (auto itr = _joints.begin(); itr != _joints.end(); ++itr) {
            nrDof += itr->dof();
        }

        // initialize g
        _g_mat = VectorXd::Zero(nrDof);

        // initialize the jacobian
        _J_mat = MatrixXd::Zero(nrDof, mb.nrDof());
        unsigned int posInG = 0;
        count = 0;
        for (auto itr = _joints.begin(); itr != _joints.end(); ++itr) {
            unsigned int posInDof = mb.jointPosInDof(_jointIndex[count]);
            _J_mat.block(posInG, posInDof, itr->dof(), itr->dof()) = 
                MatrixXd::Identity(itr->dof(), itr->dof());
            posInG += itr->dof();
            count++;
        }
    }

    virtual VectorXd g(rbd::MultiBody mb, rbd::MultiBodyConfig mbc)
    {
        auto q = mbc.q; 
        auto jointConfig = mbc.jointConfig;
        unsigned int posInG = 0;
        int count = 0;
        for (auto itr = _joints.begin(); itr != _joints.end(); ++itr) {
            int jIndex = _jointIndex[count];
            if (itr->type() == rbd::Joint::Prism || itr->type() == rbd::Joint::Rev) {
                //rev and prism joints returns 1 as dof() //risky code?
                //cout << "segment prism or rev " << itr->name()  
                //     << " dof is " << itr->dof() 
                //     << " jIndex is " << jIndex 
                //     << " q[index] size is " << q[jIndex].size()<<  endl;
                VectorXd tmp(itr->dof()); tmp << q[jIndex][0] - _q_T.q[jIndex][0];
                _g_mat.segment(posInG, itr->dof()) = tmp; 
            }
            else if (itr->type() == rbd::Joint::Spherical) {
                //cout << "segment spherical " << itr->name() << endl;
                // spherical joint returns quaternion //risky code?
                auto orid = Quaterniond(_q_T.q[jIndex][0], _q_T.q[jIndex][1], 
                                        _q_T.q[jIndex][2], _q_T.q[jIndex][3]).inverse().matrix();
                // spherical joint returns 3 as dof() //risky code?
                _g_mat.segment(posInG, itr->dof()) = 
                    sva::rotationError(orid, jointConfig[jIndex].rotation());
            }

            posInG+=itr->dof();
            count++;
        }
        return _g_mat;
    }

    virtual MatrixXd J(rbd::MultiBody mb, rbd::MultiBodyConfig mbc)
    {
        return _J_mat;
    }


    std::vector<unsigned int> _jointIndex;
    std::vector<rbd::Joint> _joints;
    rbd::MultiBodyConfig _q_T;
    VectorXd _g_mat;
    MatrixXd _J_mat;
};

void oneTaskMin(rbd::MultiBody mb, rbd::MultiBodyConfig &mbc, TaskPtr task,
                double delta = 1.0, unsigned int maxIter = 100, double prec = 1e-8)
{
    auto q = rbd::paramToVector(mb, mbc.q);
    unsigned int iterate = 0;
    bool minimizer = false;
    while (iterate < maxIter && !minimizer) {
        // compute task data
        auto g = task->g(mb, mbc);
        auto J = task->J(mb, mbc);
        
        // compute alpha
        // J*alpha = -g
        //cout << "alpha calc" << endl;
        VectorXd alpha;
        alpha = -PseudoInverse(J, 1e-9)*g;//least square ?
    
        // integrate and run the forward kinematic
        //cout << "integration calc" << endl;
        mbc.alpha = rbd::vectorToDof(mb, alpha);
        rbd::eulerIntegration(mb, mbc, delta);
        rbd::forwardKinematics(mb, mbc);
    
        // take the new q vector
        //cout << "new q vector calc" << endl;
        q = rbd::paramToVector(mb, mbc.q);
    
        //H-infinite norm?
        //cout << "infinite norm calc" << endl;
        auto alphaInf = alpha.lpNorm<Infinity>();
    
        // yield the current state
        cout << "iterate " << iterate << endl;
        //cout << " --------------------------- " << endl;
        //cout << "iterate " << iterate << endl
        //     << "q " << endl << q.transpose() << endl
        //     << "alpha " << alpha.transpose() << endl 
        //     << "alphainf " << alphaInf << endl;
        //cout << "g " << endl << g << endl;
        //cout << "J " << endl << J << endl;
        //cout << " --------------------------- " << endl;
    
        // check if the current alpha is a minimizer
        if (alphaInf < prec) minimizer = true;
        iterate++;
    }
}

void manyTaskMin(rbd::MultiBody mb, rbd::MultiBodyConfig &mbc, MultiTaskPtr tasks, 
                 double delta = 1.0, unsigned int maxIter = 100, double prec = 1e-8)
{
    auto q = rbd::paramToVector(mb, mbc.q);
    unsigned int iterate = 0;
    bool minimizer = false;
    while (iterate < maxIter && !minimizer) {
        // compute task data such as w*g, w*J
        std::vector<VectorXd> gList;
        std::vector<MatrixXd> JList;
        int count = 0;
        int height = 0;
        for (auto itr = tasks.begin(); itr != tasks.end(); ++itr) {
            VectorXd gi = itr->first * itr->second->g(mb, mbc);
            MatrixXd Ji = itr->first * itr->second->J(mb, mbc);

            //test check: undefined value causes crash
            for (double var : vector<double>(gi.data(), gi.data()+gi.size())) {
                if (std::isnan(var)) {
                    cout << "gi nan " << endl;
                    while(1);
                }
            }
            for (double var : vector<double>(Ji.data(), Ji.data()+Ji.size())) {
                if (std::isnan(var)) {
                    cout << "Ji nan " << endl;
                    while(1);
                }
            }
            gList.push_back(gi);
            JList.push_back(Ji);
            height+=gi.size();//J.rows == g.rows
            count++;
        }
        //concatinate gList, JList
        int width = mb.nrDof(); 
        VectorXd g(height);
        MatrixXd J(height, width);
        int len = 0;
        for (auto itr = gList.begin(); itr != gList.end(); ++itr) {
            g.segment(len, itr->size()) = *itr;
            len+=itr->size();
        }
        len = 0;
        for (auto itr = JList.begin(); itr != JList.end(); ++itr) {
            J.block(len, 0, itr->rows(), width) = *itr;
            len+=itr->rows();
        }

        
        // compute alpha
        // J*alpha = -g
        VectorXd alpha;
        alpha = -PseudoInverse(J, 1e-9)*g;//least square ?

        // integrate and run the forward kinematics
        mbc.alpha = rbd::vectorToDof(mb, alpha);
        rbd::eulerIntegration(mb, mbc, delta);
        rbd::forwardKinematics(mb, mbc);

        // take the new q vector
        q = rbd::paramToVector(mb, mbc.q);
    
        //H-infinite norm?
        auto alphaInf = alpha.lpNorm<Infinity>();

        // yield the current state
        cout << " iterate "  << iterate << endl;
        //cout << " --------------------------- " << endl;
        //cout << "iterate " << iterate << endl
        //     << "q " << endl << q.transpose() << endl
        //     << "alpha " << alpha.transpose() << endl 
        //     << "alphainf " << alphaInf << endl;
        //cout << "g " << endl << g << endl;
        //cout << "J " << endl << J << endl;
        //cout << " --------------------------- " << endl;

        // check if the current alpha is a minimizer
        if (alphaInf < prec) minimizer = true;
        iterate++;
    }
}

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
            //check
            cout << mb.joint(count).name() << " is " << *(itr->begin()) << endl;
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

            sva::PTransformd X_PJ(q_pj.inverse(), r_pj);//parent's link to joint
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

void sample4(sensor_msgs::JointState &jmsg, visualization_msgs::MarkerArray &amsg)
{
    rbd::MultiBody mb = mbg.makeMultiBody("base_link", rbd::Joint::Fixed);
    rbd::MultiBodyConfig mbcIK(mb);
    mbcIK.zero(mb);
    cout << "mbc q size is " << mbcIK.q.size() << endl;
    cout << "FK " << endl;
    rbd::forwardKinematics(mb, mbcIK);
    rbd::forwardVelocity(mb, mbcIK);//For motionSubspace


    //Unreachable
    //sva::PTransformd X_O_T = sva::PTransformd(sva::RotY(M_PI/2), Vector3d(0.5, 0.5, 0.5));
    //Reachable
    sva::PTransformd X_O_T = sva::PTransformd(sva::RotY(M_PI/4), Vector3d(0.3, 0.1, 0.0));
    //Task *bodytask = new BodyTask(mb, "r_wrist", X_O_T);
    TaskPtr bodytask(new BodyTask(mb, "r_wrist", X_O_T));

    //calculation
    // copy the initial configuration to avoid the algorithm to change it
    rbd::MultiBodyConfig mbcIKSolve = rbd::MultiBodyConfig(mbcIK);
    oneTaskMin(mb, mbcIKSolve, bodytask);

    //marker
    visualization_msgs::Marker mrk;
    mrk.header.frame_id = "base_link";
    mrk.pose.position.x = X_O_T.translation()(0);
    mrk.pose.position.y = X_O_T.translation()(1);
    mrk.pose.position.z = X_O_T.translation()(2);
    auto q = Eigen::Quaterniond(X_O_T.rotation());
    mrk.pose.orientation.x = q.x();
    mrk.pose.orientation.y = q.y();
    mrk.pose.orientation.z = q.z();
    mrk.pose.orientation.w = -q.w();
    mrk.id = 0;
    mrk.type = visualization_msgs::Marker::ARROW;
    mrk.scale.x = 0.1; mrk.scale.y = 0.01; mrk.scale.z = 0.01;
    mrk.color.r = 0.5; mrk.color.g = 0.0; mrk.color.b = 0.0; mrk.color.a = 0.5; 
    amsg.markers.push_back(mrk);

    //joint state
    msgFromMultiBodyConfig(mb, mbcIKSolve, jmsg);
}

void sample5(sensor_msgs::JointState &jmsg, visualization_msgs::MarkerArray &amsg)
{
    rbd::MultiBody mb = mbg.makeMultiBody("base_link", rbd::Joint::Fixed);
    rbd::MultiBodyConfig mbcIK(mb);
    //mbcIK.zero(mb);
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_int_distribution<int> ranj(0,100);
    for (auto itr = mbcIK.q.begin(); itr != mbcIK.q.end(); ++itr) {
        for (auto itr2 = itr->begin(); itr2 != itr->end(); ++itr2) {
            *itr2 = ranj(mt)/100.0; 
        }
    }
    rbd::forwardKinematics(mb, mbcIK);
    rbd::forwardVelocity(mb, mbcIK);//For motionSubspace
    rbd::MultiBodyConfig mbcIKSolve = rbd::MultiBodyConfig(mbcIK);

    //BodyTask
    sva::PTransformd X_O_T = sva::PTransformd(sva::RotY(M_PI/4), Vector3d(0.3, 0.1, 0.0));
    TaskPtr bodytask(new BodyTask(mb, "r_wrist", X_O_T));
    //PostureTask
    TaskPtr posturetask(new PostureTask(mb, mbcIK));

    //calculation
    //Each task should have Consistency
    MultiTaskPtr tasks;
    tasks.push_back(pair<double, TaskPtr>(10000000, bodytask));
    tasks.push_back(pair<double, TaskPtr>(1.0, posturetask));
    manyTaskMin(mb, mbcIKSolve, tasks, 1.0, 200);

    //marker: target body
    visualization_msgs::Marker mrk;
    mrk.header.frame_id = "base_link";
    mrk.header.stamp = ros::Time::now();
    mrk.pose.position.x = X_O_T.translation()(0);
    mrk.pose.position.y = X_O_T.translation()(1);
    mrk.pose.position.z = X_O_T.translation()(2);
    auto q = Eigen::Quaterniond(X_O_T.rotation());
    mrk.pose.orientation.x = q.x();
    mrk.pose.orientation.y = q.y();
    mrk.pose.orientation.z = q.z();
    mrk.pose.orientation.w = -q.w();
    mrk.id = -1;
    mrk.type = visualization_msgs::Marker::ARROW;
    mrk.scale.x = 0.2; mrk.scale.y = 0.02; mrk.scale.z = 0.02;
    mrk.color.r = 0.5; mrk.color.g = 0.0; mrk.color.b = 0.0; mrk.color.a = 0.5;
    amsg.markers.push_back(mrk);

    //marker: Target posture 
    int idcount = 0;
    auto stamp = ros::Time::now();
    for (auto itr = mbcIK.bodyPosW.begin(); itr != mbcIK.bodyPosW.end(); ++itr) {
        visualization_msgs::Marker mrk;
        mrk.header.stamp = stamp;
        mrk.header.frame_id = "base_link";
        mrk.id = idcount;
        mrk.text = mb.body(idcount).name();
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
        mrk.color.r = 0.0; mrk.color.g = 0.8; mrk.color.b = 0.0; mrk.color.a = 0.5; 
        amsg.markers.push_back(mrk);

        idcount++;
    }



    //joint state
    msgFromMultiBodyConfig(mb, mbcIKSolve, jmsg);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "multiLinkCalc3");
    ros::NodeHandle n;
    ros::Publisher j_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher a_pub = n.advertise<visualization_msgs::MarkerArray>("markers", 1);
    
    readUrdf();
    _LinkConstSharedPtr root_link = model.getRoot();
    treeParse(root_link);
    setGraph(root_link);
    cout << "graph is already set" << endl;

    visualization_msgs::MarkerArray amsg, amsg2;
    sensor_msgs::JointState jmsg, jmsg2;
    sample4(jmsg, amsg);
    sample5(jmsg2, amsg2);
    while (ros::ok()) {
        cout << "sample4" << endl;
        for (int i = 0; i < 50; i++) {
            amsg.markers[0].header.stamp = ros::Time::now();
            jmsg.header.stamp = ros::Time::now();
            a_pub.publish(amsg);
            j_pub.publish(jmsg);
            ros::Duration(0.1).sleep();
        }
        auto damsg = amsg;
        for (auto&& var : damsg.markers) {
            var.action = visualization_msgs::Marker::DELETE;
            var.header.stamp = ros::Time::now();
        }
        a_pub.publish(damsg);
        ros::Duration(0.1).sleep();

        cout << "sample5" << endl;
        for (int i = 0; i < 50; i++) {
            for (auto itr = amsg2.markers.begin(); itr != amsg2.markers.end(); ++itr)
                itr->header.stamp = ros::Time::now();
            jmsg2.header.stamp = ros::Time::now();
            a_pub.publish(amsg2);
            j_pub.publish(jmsg2);
            ros::Duration(0.1).sleep();
        }
        auto damsg2 = amsg2;
        for (auto&& var : damsg2.markers) {
            var.action = visualization_msgs::Marker::DELETE;
            var.header.stamp = ros::Time::now();
        }
        a_pub.publish(damsg2);
        ros::Duration(0.1).sleep();
    }
    
    cout << "end " << endl;

    return 0;
}

