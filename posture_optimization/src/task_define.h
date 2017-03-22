#pragma once
#include <Eigen/Core>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/Jacobian.h>
using namespace std;
using namespace Eigen;

class Task {// interface class

public:
    virtual ~Task(){};
    virtual VectorXd g(rbd::MultiBody mb, rbd::MultiBodyConfig mbc) = 0;
    virtual MatrixXd J(rbd::MultiBody mb, rbd::MultiBodyConfig mbc) = 0;
    string _name;
    string _type;
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
        _type = "BodyTask";
    }

    sva::PTransformd X_O_p(rbd::MultiBodyConfig mbc) 
    {
        sva::PTransformd X_O_b(mbc.bodyPosW[_bodyIndex]);
        sva::PTransformd xop = _X_b_p * X_O_b;
        return xop; 
    }

    virtual VectorXd g(rbd::MultiBody mb, rbd::MultiBodyConfig mbc)
    {
        auto X_O_p = this->X_O_p(mbc);
        auto g_body = sva::transformError(_X_O_T, X_O_p);//MotionVec
        return g_body.vector();
    }

    virtual MatrixXd J(rbd::MultiBody mb, rbd::MultiBodyConfig mbc)
    {
        sva::PTransformd X_O_p = this->X_O_p(mbc);
        // Set transformation in Origin orientation frame
        sva::PTransformd X_O_p_0 = sva::PTransformd(X_O_p.rotation()).inv() * X_O_p;
        MatrixXd jac_mat_dense = _jac.jacobian(mb, mbc, X_O_p_0); // おかしい？
        _jac.fullJacobian(mb, jac_mat_dense, _jac_mat_sparse);
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
        
        _type = "PostureTask";
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
                VectorXd tmp(itr->dof()); tmp << q[jIndex][0] - _q_T.q[jIndex][0];
                _g_mat.segment(posInG, itr->dof()) = tmp; 
            }
            else if (itr->type() == rbd::Joint::Spherical) {
                auto orid = Quaterniond(_q_T.q[jIndex][0], _q_T.q[jIndex][1], 
                                        _q_T.q[jIndex][2], _q_T.q[jIndex][3]).inverse().matrix();
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


