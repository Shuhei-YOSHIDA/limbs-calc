#pragma once
#include <roboptim/core/linear-function.hh>
#include <roboptim/core/differentiable-function.hh>
#include <roboptim/core/twice-differentiable-function.hh>
#include <roboptim/core/io.hh>
#include <roboptim/core/solver.hh>
#include <roboptim/core/solver-factory.hh>

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include "task_define.h"

/*
 * Convex problem define for inverse kinematics for iterated time
 */

using namespace roboptim;
using namespace std;

// Evaluation function that is minimized
// configuration velocity
// * PostureTask
// * BodyTask
struct F_ik : public TwiceDifferentiableFunction
{
    F_ik(int nrDof, rbd::MultiBody mb, rbd::MultiBodyConfig mbc)
        : TwiceDifferentiableFunction(nrDof, 1, "evaluaion")
    {
        _nrDof = nrDof;
        _mb = mb;
        rbd::forwardKinematics(_mb, mbc); // For bodyPosW
        rbd::forwardVelocity(_mb, mbc); // For motionSubSpace
        _mbc = mbc;

        //For length of q:x
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
    }

    // Keep consistency:impl_compute, impl_gradient, impl_hessian
    void impl_compute (result_ref result, const_argument_ref x) const
    {
        //calc by using task-ptr
        VectorXd xq(_nrDof);
        for (int i = 0; i < _nrDof; i++) {
            xq[i] = x[i];
        }
        result[0] = 0;
        for (auto task : _tasks) {
            VectorXd e;
            e = task.second->J(_mb, _mbc)*xq + task.second->g(_mb, _mbc);

            // calc evaluation
            result[0]+=e.squaredNorm();
        }
    }

    void impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const
    {
        //calc by using task-ptr
        VectorXd xq(_nrDof);
        for (int i = 0; i < _nrDof; i++) {
            xq[i] = x[i];
        }
        //VectorXd all(_nrDof);
        VectorXd all = VectorXd::Zero(_nrDof);
        for (auto task : _tasks) {
            VectorXd diff(_nrDof);
            auto J = task.second->J(_mb, _mbc);
            auto g = task.second->g(_mb, _mbc);
            diff = 2*J.transpose()*(J*xq + g);
            // calc evaluation
            all+=diff;
        }
        grad = all;
    }

    void impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
    {
        for (auto task : _tasks) {
            auto J = task.second->J(_mb, _mbc);
            h += 2*J.transpose()*J;
        }
    }

public:
    void xToMBC(std::vector<double> xq, rbd::MultiBodyConfig& mbc) const
    {
        //Consider fixed-joint in mbc.q
        mbc.zero(_mb);
        int jcount = 0;
        int xcount = 0;
        for (auto itr = _joints.begin(); itr != _joints.end(); ++itr) {
            int jIndex = _jointIndex[jcount];
            if (itr->type() == rbd::Joint::Prism || itr->type() == rbd::Joint::Rev) {
                mbc.q[jIndex][0] = xq[xcount];
                xcount++;
            }
            else if (itr->type() == rbd::Joint::Spherical) {
                mbc.q[jIndex][0] = xq[xcount];
                mbc.q[jIndex][1] = xq[xcount+1];
                mbc.q[jIndex][2] = xq[xcount+2];
                mbc.q[jIndex][3] = xq[xcount+3];
                xcount+=4;
            }

            jcount++;
        }
    }
    void xToMBC(argument_ref x, rbd::MultiBodyConfig& mbc) const
    {
        std::vector<double> xq;
        for (int i = 0; i < _nrDof; i++) {
            xq.push_back(x[i]);
        }
        xToMBC(xq, mbc);
    }


public:
    MultiTaskPtr _tasks;
    int _nrDof;
    rbd::MultiBody _mb;
    rbd::MultiBodyConfig _mbc;
    std::vector<unsigned int> _jointIndex;
    std::vector<rbd::Joint> _joints;
};

// Constraint Function, equality or inequality
// Velocity damper for joint angle and velocity For upper limit
struct G_q_vd_up : public DifferentiableFunction
{
    G_q_vd_up (int nrDof, rbd::MultiBody mb, rbd::MultiBodyConfig mbc)
        : DifferentiableFunction(nrDof, nrDof, "q_vd_upper_limit")
    {
        _nrDof = nrDof;
        _mb = mb;
        rbd::forwardKinematics(_mb, mbc); // For bodyPosW
        rbd::forwardVelocity(_mb, mbc); // For motionSubSpace
        _mbc = mbc;

        //For length of q:x
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
    }

    void impl_compute (result_ref result, const_argument_ref x) const
    {
        // Velocity damper
        VectorXd q_dot_max(_nrDof);
        VectorXd q_angle(_nrDof);
        double qi = 10*M_PI/180.0; //rad
        double qs =  5*M_PI/180.0;
        double qdlimit = 7*M_PI/180.0; //rad/s
        double xi = qdlimit;
        double qmax = 150*M_PI/180.0;

        int jcount = 0;
        int xcount = 0;
        for (auto itr = _joints.begin(); itr != _joints.end(); ++itr) {
            int jIndex = _jointIndex[jcount];
            if (itr->type() == rbd::Joint::Prism || itr->type() == rbd::Joint::Rev) {
                q_angle[xcount] = _mbc.q[jIndex][0];
                xcount++;
            }
            else if (itr->type() == rbd::Joint::Spherical) {
                q_angle[xcount]   = _mbc.q[jIndex][0];
                q_angle[xcount+1] = _mbc.q[jIndex][1];
                q_angle[xcount+2] = _mbc.q[jIndex][2];
                q_angle[xcount+3] = _mbc.q[jIndex][3];
                xcount+=4;
            }
            jcount++;
        }

        for (int i = 0; i < _nrDof; i++) {
            if (qmax - q_angle[i] <= qi) {
                q_dot_max[i] = xi*((qmax - q_angle[i])-qs)/(qi-qs);
            }
            else {
                q_dot_max[i] = qdlimit;
            }
        }
        result = x - q_dot_max;



    }

    void impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const {}

    void impl_jacobian (jacobian_ref jaco, const_argument_ref x) const
    {
        MatrixXd j = MatrixXd::Identity(_nrDof, _nrDof);
        jaco = j;
    }

public:
    int _nrDof;
    rbd::MultiBody _mb;
    rbd::MultiBodyConfig _mbc;
    std::vector<unsigned int> _jointIndex;
    std::vector<rbd::Joint> _joints;
};

// Velocity damper for joint angle and velocity For lower limit
struct G_q_vd_lo : public DifferentiableFunction
{
    G_q_vd_lo (int nrDof, rbd::MultiBody mb, rbd::MultiBodyConfig mbc)
        : DifferentiableFunction(nrDof, nrDof, "q_vd_lower_limit")
    {
        _nrDof = nrDof;
        _mb = mb;
        rbd::forwardKinematics(_mb, mbc); // For bodyPosW
        rbd::forwardVelocity(_mb, mbc); // For motionSubSpace
        _mbc = mbc;

        //For length of q:x
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
    }

    void impl_compute (result_ref result, const_argument_ref x) const
    {
        // Velocity damper
        VectorXd q_dot_min(_nrDof);
        VectorXd q_angle(_nrDof);
        double qi = 10*M_PI/180.0; //rad
        double qs =  5*M_PI/180.0;
        double qdlimit = -7*M_PI/180.0; //rad/s
        double xi = qdlimit;
        double qmin = -150*M_PI/180.0;

        int jcount = 0;
        int xcount = 0;
        for (auto itr = _joints.begin(); itr != _joints.end(); ++itr) {
            int jIndex = _jointIndex[jcount];
            if (itr->type() == rbd::Joint::Prism || itr->type() == rbd::Joint::Rev) {
                q_angle[xcount] = _mbc.q[jIndex][0];
                xcount++;
            }
            else if (itr->type() == rbd::Joint::Spherical) {
                q_angle[xcount]   = _mbc.q[jIndex][0];
                q_angle[xcount+1] = _mbc.q[jIndex][1];
                q_angle[xcount+2] = _mbc.q[jIndex][2];
                q_angle[xcount+3] = _mbc.q[jIndex][3];
                xcount+=4;
            }
            jcount++;
        }

        for (int i = 0; i < _nrDof; i++) {
            if (q_angle[i] - qmin <= qi) {
                q_dot_min[i] = -xi*((q_angle[i]-qmin)-qs)/(qi-qs);
            }
            else {
                q_dot_min[i] = qdlimit;
            }
        }
        result = x - q_dot_min;



    }

    void impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const {}

    void impl_jacobian (jacobian_ref jaco, const_argument_ref x) const
    {
        MatrixXd j = MatrixXd::Identity(_nrDof, _nrDof);
        jaco = j;
    }

public:
    int _nrDof;
    rbd::MultiBody _mb;
    rbd::MultiBodyConfig _mbc;
    std::vector<unsigned int> _jointIndex;
    std::vector<rbd::Joint> _joints;
};
