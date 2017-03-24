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
 * problem define for posture optimization
 */

using namespace roboptim;
using namespace std;

// Evaluation function that is minimized
// * PostureTask
// * BodyTask
struct F : public DifferentiableFunction
//struct F : public TwiceDifferentiableFunction
{
    F(int nrDof, rbd::MultiBody mb) : DifferentiableFunction(nrDof, 1, "evaluaion")
    //F(int nrDof, rbd::MultiBody mb) : TwiceDifferentiableFunction(nrDof, 1, "evaluaion")
    {
        _nrDof = nrDof;
        _mb = mb;
        cout << "_nrdof is " << _nrDof << endl;

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
        cout << "impl_compute" << endl;
        //calc by using task-ptr
        std::vector<double> xq;
        for (int i = 0; i < _nrDof; i++) {
            xq.push_back(x[i]);
        }
        for (auto task : _tasks) {
            double err;
            rbd::MultiBodyConfig mbc(_mb);
            xToMBC(xq, mbc);
            cout << " value check" << endl;
            cout << " mbc joint size is " << mbc.q.size() << endl;
            //for (auto itr = mbc.q.begin(); itr != mbc.q.end(); itr++) {
            //    for (auto itr2 = itr->begin(); itr2 != itr->end(); itr2++) {
            //        cout << *itr2 << ", ";
            //    }
            //}
            // pre-prosess
            if (task.second->_type == "PostureTask") {
                err = task.second->g(_mb, mbc).squaredNorm() * task.first;
            }
            else if (task.second->_type == "BodyTask") {
                cout << "fk bodytask compute" << endl;
                rbd::forwardKinematics(_mb, mbc); // For bodyPosW
                rbd::forwardVelocity(_mb, mbc); // For motionSubSpace
                err = task.second->g(_mb, mbc).squaredNorm() * task.first;
            }
            //else if (force task)
            else {
                while(1) cerr << "unknown task in func F" << endl;
            }

            // calc evaluation
            result[0]+=err;
        }
        cout << "x ";
        for (int i = 0; i < x.size(); i++) {
            cout << x;
        }
        cout << endl;
        cout << "com " << result[0] << endl;

    }

    void impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const
    {
        cout << "impl_gradient" << endl;
        //grad << x[0], x[1], x[2], x[3]; 
        //calc by using task-ptr
        std::vector<double> xq;
        for (int i = 0; i < _nrDof; i++) {
            xq.push_back(x[i]);
        }
        VectorXd all(_nrDof);
        for (auto task : _tasks) {
            VectorXd diff(_nrDof);
            rbd::MultiBodyConfig mbc(_mb); 
            xToMBC(xq, mbc);
            // pre-prosess
            if (task.second->_type == "PostureTask") {
                diff = task.second->g(_mb, mbc)* task.first * 2;//g size is different!
                //if there are some joint except rev, prism and spherecal, this code has problem
            }
            else if (task.second->_type == "BodyTask") {
                cout << "fk bodytask grad " << endl;
                cout << "mbc q size is " << mbc.q.size() << endl;
                rbd::forwardKinematics(_mb, mbc); // For bodyPosW
                rbd::forwardVelocity(_mb, mbc); // For motionSubSpace
                diff = task.second->J(_mb, mbc).transpose() * 
                    task.second->g(_mb, mbc) * task.first * 2;
            }
            //else if (force task)
            else {
                while(1) cerr << "unknown task in func F" << endl;
            }

            // calc evaluation
            all+=diff;
        }
        grad = all;
        cout << "x ";
        for (int i = 0; i < x.size(); i++) {
            cout << x;
        }
        cout << endl;
        cout << "grad " << all << endl;
    }

    //void impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
    //{
    //    h << 1, 0, 0, 0, 
    //         0, 1, 0, 0, 
    //         0, 0, 1, 0, 
    //         0, 0, 0, 1; 
    //}

public:
    void xToMBC(std::vector<double> xq, rbd::MultiBodyConfig& mbc) const
    {
        cout << "xToMBC" << endl;
        //Consider fixed-joint in mbc.q
        //rbd::MultiBodyConfig mbc(_mb);
        mbc.zero(_mb);
        cout << "mbc.q size is " << mbc.q.size() << endl;
        cout << "mb check : nrDof is" << _mb.nrDof() << endl;
        //int count = 0;
        //for (auto itr = mbc.q.begin(); itr != mbc.q.end(); itr++) {
        //    for (auto itr2 = itr->begin(); itr2 != itr->end(); itr2++) {
        //        *itr2 = xq[count];
        //        count++;
        //    }
        //}
        //return mbc;
        int jcount = 0;
        int xcount = 0;
        for (auto itr = _joints.begin(); itr != _joints.end(); ++itr) {
            cout << "jcount, xcount " << jcount << ", " << xcount << endl;
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
        cout << "xToMBC q size is " << mbc.q.size() << endl;
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
    //TaskPtr bodytask;
    //TaskPtr posturetask;
    MultiTaskPtr _tasks;
    int _nrDof;
    rbd::MultiBody _mb;
    std::vector<unsigned int> _jointIndex;
    std::vector<rbd::Joint> _joints;
};

// Constraint Function, equality or inequality
// * joint limit -> implement as bounds, not function
