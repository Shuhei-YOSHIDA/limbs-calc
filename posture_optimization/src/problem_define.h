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
    }

    // Keep consistency:impl_compute, impl_gradient, impl_hessian
    void impl_compute (result_ref result, const_argument_ref x) const
    {
        //result[0] = 0.5*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]); 
        //calc by using task-ptr
        std::vector<double> xq;
        for (int i = 0; i < _nrDof; i++) {
            xq.push_back(x[i]);
        }
        for (auto task : _tasks) {
            double err;
            rbd::MultiBodyConfig mbc = xToMBC(xq);
            // pre-prosess
            if (task.second->_type == "PostureTask") {
                err = task.second->g(_mb, mbc).squaredNorm() * task.first;
            }
            else if (task.second->_type == "BodyTask") {
                rbd::forwardKinematics(_mb, mbc); // For bodyPosW
                err = task.second->g(_mb, mbc).squaredNorm() * task.first;
            }
            //else if (force task)
            else {
                while(1) cerr << "unknown task in func F" << endl;
            }

            // calc evaluation
            result[0]+=err;
        }

    }

    void impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const
    {
        //grad << x[0], x[1], x[2], x[3]; 
        //calc by using task-ptr
        std::vector<double> xq;
        for (int i = 0; i < _nrDof; i++) {
            xq.push_back(x[i]);
        }
        VectorXd all(_nrDof);
        for (auto task : _tasks) {
            VectorXd diff(_nrDof);
            rbd::MultiBodyConfig mbc = xToMBC(xq);
            // pre-prosess
            if (task.second->_type == "PostureTask") {
                diff = task.second->g(_mb, mbc)* task.first * 2;//g size is different!
                //if there are some joint except rev, prism and spherecal, this code has problem
            }
            else if (task.second->_type == "BodyTask") {
                rbd::forwardKinematics(_mb, mbc); // For bodyPosW
                diff = task.second->J(_mb, mbc).transpose() * 
                    task.second->g(_mb, mbc).transpose() * task.first * 2;
            }
            //else if (force task)
            else {
                while(1) cerr << "unknown task in func F" << endl;
            }

            // calc evaluation
            all+=diff;
        }
        grad = all;
    }

    //void impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
    //{
    //    h << 1, 0, 0, 0, 
    //         0, 1, 0, 0, 
    //         0, 0, 1, 0, 
    //         0, 0, 0, 1; 
    //}

    rbd::MultiBodyConfig xToMBC(std::vector<double> xq) const
    {
        rbd::MultiBodyConfig mbc(_mb);
        mbc.zero(_mb);
        int count = 0;
        for (auto itr = mbc.q.begin(); itr != mbc.q.end(); itr++) {
            for (auto itr2 = itr->begin(); itr2 != itr->end(); itr2++) {
                *itr2 = xq[count];
                count++;
            }
        }
        return mbc;
    }


public:
    //TaskPtr bodytask;
    //TaskPtr posturetask;
    MultiTaskPtr _tasks;
    int _nrDof;
    rbd::MultiBody _mb;
};

// Constraint Function, equality or inequality
// * joint limit -> implement as bounds, not function
