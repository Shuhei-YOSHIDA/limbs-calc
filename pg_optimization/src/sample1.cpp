/**
 * @file sample1.cpp
 */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include "pg_optimization/preparation.h"
#include <SpaceVecAlg/SpaceVecAlg>
#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/ID.h>

#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Object/S_Box.h>
#include <sch/CD/CD_Pair.h>

#include <PG/ConfigStruct.h>
#include <PG/PostureGenerator.h>
#include <PG/CollisionConstr.h>

using namespace std;
using namespace rbd;
using namespace sva;
using namespace Eigen;
using namespace sensor_msgs;
using namespace visualization_msgs;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample1");
  ros::NodeHandle nh;
  ros::Publisher js_pub = nh.advertise<JointState>("joint_states", 1);
  ros::Publisher ma_pub = nh.advertise<MarkerArray>("markers", 1);

  MultiBody mb;
  Limits limits;
  MultiBodyGraph mbg;
  if(!init(mb, limits, mbg, "l_sole")) return -1;
  MultiBodyConfig mbc(mb);
  mbc.zero(mb);
  // gravity
  mbc.gravity = Vector3d(0, 0, 9.81);

  forwardKinematics(mb, mbc);
  forwardVelocity(mb, mbc);

  pg::PostureGenerator pgPb;
  pg::RobotConfig rc(mb);
  pgPb.param("ipopt.print_level", 0);
  pgPb.param("ipopt.linear_solver", "mumps");
 
  // Set configuration limit
  vector<vector<double>> ql(mbc.q), qu(mbc.q);
  for (int i = 0; i < ql.size(); i++)
  {
    double q_lo, q_up;
    if (mb.joint(i).dof() == 1)
    {
      string name = mb.joint(i).name();
      q_lo = limits.lower.at(name)[0];
      q_up = limits.upper.at(name)[0];
    }
    else //else if (_mbs[0].joint(i).type() == Joint::Type::Free)
    { // Non limit
      q_lo = std::numeric_limits<double>::infinity() * -1;
      q_up = std::numeric_limits<double>::infinity();
    }

    // Set limit value
    for (int j = 0; j < ql[i].size(); j++) // Joint displacement limit
    {
      ql[i][j] = q_lo;
      qu[i][j] = q_up;
    }
  }
  rc.ql = ql;
  rc.qu = qu;

  rc.fixedPosContacts.push_back({"l_sole"/*bodyName*/, Vector3d::Zero()/*target position in world coordinate*/,
                                  PTransformd::Identity()/*body surface frame in body coordinate*/});

  pgPb.robotConfigs({rc}, mbc.gravity);
  bool isSolved = pgPb.run({{mbc.q, {}, {}}});
  if (!isSolved)
  {
    ROS_FATAL("PG could not be solved");
    return -1;
  }

  MultiBodyConfig mbc_res(mb);
  mbc_res.q = pgPb.q();
  forwardKinematics(mb, mbc_res);
  forwardVelocity(mb, mbc_res);

  ros::Rate loop(100);
  while (ros::ok())
  {
    JointState js_msg;
    jointStateFromMBC(mb, mbc_res, js_msg);
    js_pub.publish(js_msg);
    broadcastRoot(mbc.bodyPosW[mb.bodyIndexByName("base_link")], "odom", "base_link");
    loop.sleep();
  }

  return 0;
}
