/**
 * @file ik_methods4.cpp
 * @brief Inverse kinematics with priority
 */

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
#include "preparation.h"

using namespace std;
using namespace Eigen;

// header にするべき
class Task {// interface class
public:
    virtual ~Task(){};
    virtual VectorXd g(rbd::MultiBody mb, rbd::MultiBodyConfig mbc) = 0;
    virtual MatrixXd J(rbd::MultiBody mb, rbd::MultiBodyConfig mbc) = 0;
    string _name;
};

typedef boost::shared_ptr<Task> TaskPtr;
typedef std::vector<pair<double, TaskPtr>> MultiTaskPtr;//priority and Task



//Vector3d calcPriorityIK(MultiTaskPtr tasks)
//{
//
//}

int main(int argc, char** argv)
{
  
  return 0;
}
