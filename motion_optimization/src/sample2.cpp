/**
 * @file sample2.cpp
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
  ros::init(argc, argv, "sample2");
  ros::NodeHandle nh;
  ros::Publisher js_pub = nh.advertise<JointState>("joint_states", 1);
  ros::Publisher ma_pub = nh.advertise<MarkerArray>("markers", 1);

  MultiBody mb;
  if(!init(mb)) return -1;
  MultiBodyConfig mbc(mb);
  mbc.zero(mb);

  forwardKinematics(mb, mbc);
  forwardVelocity(mb, mbc);

  MultiBody mb_env = makeEnv();
  MultiBodyConfig mbc_env(mb_env);
  mbc_env.zero(mb_env);
  
  vector<MultiBody> mbs = {mb, mb_env};
  vector<MultiBodyConfig> mbcs = {mbc, mbc_env};

  vector<qp::UnilateralContact> contVec =
    {qp::UnilateralContact(0/*r1Index*/,
                           1/*r2Index*/,
                           "r_wrist"/*r1BodyName*/,
                           "A"/*r2BodyName*/,
                           {Vector3d::Zero()}/*r1Points*/,       // r1BodyIdでの接触点
                           Matrix3d::Identity()/*r1Frame*/,      // rbBodyId 座標系での摩擦円錐の座標系
                           PTransformd::Identity()/*X_b1_b2*/,   // r1BodyId，r2BodyId間の変換
                           3/*nrGen*/,                           // 直線母線の数
                           std::tan(M_PI/4.)/*mu*/) // 摩擦係数
    };

  qp::QPSolver solver;
  solver.nrVars(mbs, {}, {});
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

  /// qp::Contact[Acc/Speed/Pos]Constr is kinematic and equality constraint
  //qp::ContactAccConstr contCstrAcc; // initial velocity should be zero
  //qp::ContactSpeedConstr contCstrSpeed(0.001);
  qp::ContactPosConstr contCstrPos(0.001);
  contCstrPos.addToSolver(solver);
  solver.nrVars(mbs, contVec, {/*vector<BilateralContact>*/});
  solver.updateConstrSize();

  vector<MultiBodyConfig> mbc_array2(10000);
  vector<JointState> js_array2(10000);
  vector<PTransformd> root_pose_array2(10000);
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

    mbc_array2[i] = mbcs[0];
    JointState js_msg;
    jointStateFromMBC(mb, mbcs[0], js_msg);
    js_array2[i] = js_msg;
    root_pose_array2[i] = mbcs[0].bodyPosW[mb.bodyIndexByName("base_link")];
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
    ROS_INFO("replay motion 2nd");
    for (auto&& m : ma_msg.markers) m.header.stamp = ros::Time::now();
    ma_pub.publish(ma_msg);
    for (int i = 0; i < 10000; i++)
    {
      auto stamp = ros::Time::now();
      js_array2[i].header.stamp = stamp;
      js_pub.publish(js_array2[i]);
      broadcastRoot(root_pose_array2[i]);
      loop.sleep();
    }
    ROS_INFO("finished replay");
  }

  ros::spin();

  return 0;
}
