#pragma once
#include <roboptim/core/twice-differentiable-function.hh>
#include <roboptim/core/io.hh>
#include <roboptim/core/solver.hh>
#include <roboptim/core/solver-factory.hh>

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include "problem_define.h"

/*
 * problem define for posture optimization
 */

using namespace roboptim;

// Evaluation function that is minimized
// * PostureTask
// * BodyTask
struct F : public TwiceDifferentiableFunction
{
    F() : TwiceDifferentiableFunction(4, 1, "evaluaion")
    {
    }

    void impl_compute (result_ref result, const_argument_ref x) const
    {
        result[0] = 0.5*(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]); 
    }

    void impl_gradient (gradient_ref grad, const_argument_ref x, size_type) const
    {
        grad << x[0], x[1], x[2], x[3]; 
    }

    void impl_hessian (hessian_ref h, const_argument_ref x, size_type) const
    {
        h << 1, 0, 0, 0, 
             0, 1, 0, 0, 
             0, 0, 1, 0, 
             0, 0, 0, 1; 
    }
public:
    TaskPtr bodytask;
    TaskPtr posturetask;
};

// Constraint Function, equality or inequality
// * joint limit
