/**
 * @file sample1.cpp
 */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include "motion_optimization/preparation.h"
#include <SpaceVecAlg/SpaceVecAlg>
#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/ID.h>

#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Object/S_Box.h>
#include <sch/CD/CD_Pair.h>

#include <Tasks/Bounds.h>
#include <Tasks/QPConstr.h>
#include <Tasks/QPContactConstr.h>
#include <Tasks/QPSolver.h>
#include <Tasks/QPTasks.h>

using namespace std;
using namespace rbd;
using namespace sva;
using namespace Eigen;
using namespace tasks;
using namespace sensor_msgs;
using namespace visualization_msgs;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample1");
  ros::NodeHandle nh;
  ros::Publisher js_pub = nh.advertise<JointState>("joint_states", 1);
  ros::Publisher ma_pub = nh.advertise<MarkerArray>("markers", 1);

  MultiBody mb;
  if(!init(mb)) return -1;
  MultiBodyConfig mbc(mb);
  mbc.zero(mb);

  forwardKinematics(mb, mbc);
  forwardVelocity(mb, mbc);
  
  vector<MultiBody> mbs = {mb};
  vector<MultiBodyConfig> mbcs(1);

  qp::QPSolver solver;
  solver.nrVars(mbs, {/*接触情報?*/}, {});

  solver.updateConstrSize();

  auto pos0 = Vector3d::Zero();
  qp::PositionTask posTask0(mbs, 0, "base_link", pos0);
  qp::SetPointTask posTaskSp0(mbs, 0, &posTask0, 10., 1);
  solver.addTask(&posTaskSp0);

  auto pos1 = Vector3d(0.300, 0.300, 0.300);
  qp::PositionTask posTask1(mbs, 0, "l_wrist", pos1);
  qp::SetPointTask posTaskSp1(mbs, 0, &posTask1, 10., 1);
  solver.addTask(&posTaskSp1);

  mbcs[0] = mbc;
  vector<MultiBodyConfig> mbc_array(10000);
  vector<JointState> js_array(10000);
  vector<PTransformd> root_pose_array(10000);
  for (int i = 0; i < 10000; i++)
  {
    if(!solver.solve(mbs, mbcs))
    {
      ROS_FATAL("Could not solved");
      ros::shutdown();
      return -1;
    }
    eulerIntegration(mb, mbcs[0], 0.001);

    forwardKinematics(mb, mbcs[0]);
    forwardVelocity(mb, mbcs[0]);

    mbc_array[i] = mbcs[0];
    JointState js_msg;
    jointStateFromMBC(mb, mbcs[0], js_msg);
    js_array[i] = js_msg;
    root_pose_array[i] = mbcs[0].bodyPosW[mb.bodyIndexByName("base_link")];
  }

  ROS_INFO_STREAM("Task error: " << posTask1.eval().norm());

  Marker m_msg;
  markerSet(PTransformd(pos1), m_msg, 0, "odom");
  MarkerArray ma_msg;
  ma_msg.markers.push_back(m_msg);

  ros::Rate loop(1000);
  while(ros::ok())
  {
    ROS_INFO("replay motion");
    for (auto&& m : ma_msg.markers) m.header.stamp = ros::Time::now();
    ma_pub.publish(ma_msg);
    for (int i = 0; i < 10000; i++)
    {
      auto stamp = ros::Time::now();
      js_array[i].header.stamp = stamp;
      js_pub.publish(js_array[i]);
      broadcastRoot(root_pose_array[i]);
      loop.sleep();
    }
    ROS_INFO("finished replay");
  }

  ros::spin();

  return 0;
}
