/**
 * @file sample1.cpp
 */
#include <ros/ros.h>
#include "motion_optimization/preparation.h"
#include <SpaceVecAlg/SpaceVecAlg>
#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FE.h>
#include <RBDyn/ID.h>

#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Object/S_Box.h>
#include <sch/CD/CD_Pair.h>

#include <Tasks/Bound.h>
#include <Tasks/QPConstr.h>
#include <Tasks/QPContactConstr.h>
#include <Tasks/QPSolver.h>
#include <Tasks/QPTasks.h>

int main(int argc, char** argv)
{
  MultiBody mb;
  init(mb);
  
  return 0;
}
