/**
 * @file sample3.cpp
 * @brief force contact constraint and motion constraint
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
  ros::init(argc, argv, "sample3");
  ros::NodeHandle nh;
  ros::Publisher js_pub = nh.advertise<JointState>("joint_states", 1);
  ros::Publisher ma_pub = nh.advertise<MarkerArray>("markers", 1);

  MultiBody mb;
  Limits limits;
  if(!init(mb, limits)) return -1;
  MultiBodyConfig mbc(mb);
  mbc.zero(mb);
  // gravity
  mbc.gravity = Vector3d(0, 0, 9.81);

  forwardKinematics(mb, mbc);
  forwardVelocity(mb, mbc);

  // initial standing: Free joint q[free] = [qw, qx, qy, qz, x, y, z]
  mbc.q[mb.jointIndexByName("Root")][0] = 1;
  mbc.q[mb.jointIndexByName("Root")][1] = 0;
  mbc.q[mb.jointIndexByName("Root")][2] = 0;
  mbc.q[mb.jointIndexByName("Root")][3] = 0;
  mbc.q[mb.jointIndexByName("Root")][4] = -mbc.bodyPosW[mb.bodyIndexByName("l_sole")].translation()[0];
  mbc.q[mb.jointIndexByName("Root")][5] = -mbc.bodyPosW[mb.bodyIndexByName("l_sole")].translation()[1]; 
  mbc.q[mb.jointIndexByName("Root")][6] = -mbc.bodyPosW[mb.bodyIndexByName("l_sole")].translation()[2];
  // set arms on side of the body
  mbc.q[mb.jointIndexByName("RShoulderPitch")][0] = 1.75;
  mbc.q[mb.jointIndexByName("LShoulderPitch")][0] = 1.75;

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
                           "l_sole"/*r1BodyName*/,
                           "A"/*r2BodyName*/,
                           {Vector3d::Zero()}/*r1Points*/,       // r1BodyIdでの接触点
                           Matrix3d::Identity()/*r1Frame*/,      // rbBodyId 座標系での摩擦円錐の座標系
                           PTransformd::Identity()/*X_b1_b2*/,   // r1BodyId，r2BodyId間の変換
                           3/*nrGen*/,                           // 直線母線の数
                           std::tan(M_PI/4.)/*mu*/) // 摩擦係数
     ,
     qp::UnilateralContact(0/*r1Index*/,
                           1/*r2Index*/,
                           "r_sole"/*r1BodyName*/,
                           "A"/*r2BodyName*/,
                           {Vector3d::Zero()}/*r1Points*/,       // r1BodyIdでの接触点
                           Matrix3d::Identity()/*r1Frame*/,      // rbBodyId 座標系での摩擦円錐の座標系
                           PTransformd::Identity()/*X_b1_b2*/,   // r1BodyId，r2BodyId間の変換
                           3/*nrGen*/,                           // 直線母線の数
                           std::tan(M_PI/4.)/*mu*/) // 摩擦係数
    };

  qp::QPSolver solver;

  auto pos0 = mbc.bodyPosW[mb.bodyIndexByName("base_link")].translation();
  qp::PositionTask posTask0(mbs, 0, "base_link", pos0);
  qp::SetPointTask posTaskSp0(mbs, 0, &posTask0, 1., 1);
  solver.addTask(&posTaskSp0);

  qp::OrientationTask oriTask0(mbs, 0, "base_link", Matrix3d::Identity());
  qp::SetPointTask oriTaskSp0(mbs, 0, &oriTask0, 10., 1);
  solver.addTask(&oriTaskSp0);

  auto q = mbc.q;
  qp::PostureTask postureTask0(mbs, 0, q, 0.1, 1);
  solver.addTask(&postureTask0);

  auto pos1 = Vector3d(0.300, 0.300, 0.500);
  //qp::PositionTask posTask1(mbs, 0, "l_wrist", pos1);
  //qp::SetPointTask posTaskSp1(mbs, 0, &posTask1, 10., 1);
  //solver.addTask(&posTaskSp1);

  qp::ContactSpeedConstr contCstrSpeed(0.001);
  contCstrSpeed.addToSolver(solver);

  vector<vector<double>> torqueMin(mb.nrJoints()), torqueMax(mb.nrJoints());
  for (auto&& pair : limits.torque)
  {
    auto max = pair.second;
    auto min = pair.second;
    for (int i = 0; i < min.size(); i++) min[i] *= -1;
    torqueMin[mb.jointIndexByName(pair.first)] = min;
    torqueMax[mb.jointIndexByName(pair.first)] = max;
    //cout << pair.first << ":" << mb.jointIndexByName(pair.first) << ":";
    //for (auto&& m : min) cout << m << ", "; cout << ": ";
    //for (auto&& m : max) cout << m << ", "; cout << endl;
  }
  const double INF  = std::numeric_limits<double>::infinity();
  // Root jointの制限が無いので，接触力発生しないような補正力が発生することがある?
  //torqueMin[mb.jointIndexByName("Root")] = {-INF, -INF, -INF, -INF, -INF, -INF};
  //torqueMax[mb.jointIndexByName("Root")] = {+INF, +INF, +INF, +INF, +INF, +INF};
  // Root joint は仮想なので，torqueを発生することができない
  torqueMin[mb.jointIndexByName("Root")] = {-0, -0, -0, -0, -0, -0};
  torqueMax[mb.jointIndexByName("Root")] = {+0, +0, +0, +0, +0, +0};
  qp::MotionConstr motionCstr(mbs, 0, {torqueMin, torqueMax});
  motionCstr.addToSolver(solver);
  qp::PositiveLambda plCstr;
  plCstr.addToSolver(solver);

  // Set Joint limits constraint
  vector<vector<double>> ql(mbcs[0].q), qu(mbcs[0].q);
  vector<vector<double>> qlvel(mbcs[0].alpha), quvel(mbcs[0].alpha);
  for (int i = 0; i < ql.size(); i++)
  {
    double q_lo, q_up, qv_lo, qv_up;
    if (mbs[0].joint(i).dof() == 1)
    {
      string name = mbs[0].joint(i).name();
      q_lo = limits.lower.at(name)[0];
      q_up = limits.upper.at(name)[0];
      qv_lo = -limits.velocity.at(name)[0];
      qv_up = +limits.velocity.at(name)[0];
    }
    else //else if (_mbs[0].joint(i).type() == Joint::Type::Free)
    { // Non limit
      q_lo = std::numeric_limits<double>::infinity() * -1;
      q_up = std::numeric_limits<double>::infinity();
      qv_lo = std::numeric_limits<double>::infinity() * -1;
      qv_up = std::numeric_limits<double>::infinity();
    }

    // Set limit value
    for (int j = 0; j < ql[i].size(); j++) // Joint displacement limit
    {
      ql[i][j] = q_lo;
      qu[i][j] = q_up;
    }

    for (int j = 0; j < qlvel[i].size(); j++) // Joint velocity limit
    {
      qlvel[i][j] = qv_lo;
      quvel[i][j] = qv_up;
    }
  }
  qp::DamperJointLimitsConstr dampJointConstr(mbs, 0, {ql, qu}, {qlvel, quvel}, 0.1, 0.01, 0.5, 0.001);
  dampJointConstr.addToSolver(solver);


  cout << "initial com: " << computeCoM(mb, mbcs[0]).transpose() << endl;
  qp::CoMIncPlaneConstr comPlaneConstr(mbs, 0, 0.001);
  comPlaneConstr.addToSolver(solver);
  comPlaneConstr.addPlane(10, Vector3d(1, 0, 0), 0.0, 0.15, 0.1, 0, 0.); // x dir of com
  comPlaneConstr.addPlane(20, Vector3d(-1, 0, 0), 0.0, 0.15, 0.1, 0, 0.); // -x dir of com
  comPlaneConstr.updateNrPlanes();


  solver.nrVars(mbs, contVec, {/*vector<BilateralContact>*/});
  solver.updateConstrSize();

  cout << "num of Bound constraints : " << solver.nrBoundConstraints() << endl;
  cout << "num of Inequality constraints : " << solver.nrInequalityConstraints() << endl;
  cout << "num of Equality constraints : " << solver.nrEqualityConstraints() << endl;
  cout << "num of Geninequality constraints : " << solver.nrGenInequalityConstraints() << endl;
  cout << "num of Constraints : " << solver.nrConstraints() << endl;

  cout << "num of Tasks : " << solver.nrTasks() << endl;

  mbcs[0] = mbc;
  vector<MultiBodyConfig> mbc_array(10000);
  vector<JointState> js_array(10000);
  vector<PTransformd> root_pose_array(10000);

  ROS_INFO("Waiting RViz for 5 sec");
  ros::Duration(5).sleep(); // wait for RViz
  Marker m_msg;
  markerSet(PTransformd(pos0), m_msg, 0, "odom");
  MarkerArray ma_msg;
  ma_msg.markers.resize(3);
  ma_msg.markers[0] = m_msg;

  for (auto&& m : ma_msg.markers) m.header.stamp = ros::Time::now();
  ma_pub.publish(ma_msg);

  ros::Rate loop(1000);
  for (int i = 0; i < 10000; i++)
  {
    if(!solver.solve(mbs, mbcs))
    {
      ROS_FATAL("Could not solved at %d", i);
      mbc_array.resize(i);
      js_array.resize(i);
      root_pose_array.resize(i);
      break;
      //ros::shutdown();
      //return -1;
    }
    eulerIntegration(mb, mbcs[0], 0.001);

    forwardKinematics(mb, mbcs[0]);
    forwardVelocity(mb, mbcs[0]);

    mbc_array[i] = mbcs[0];
    JointState js_msg;
    jointStateFromMBC(mb, mbcs[0], js_msg);
    js_array[i] = js_msg;
    root_pose_array[i] = mbcs[0].bodyPosW[mb.bodyIndexByName("base_link")];

    auto com = computeCoM(mb, mbcs[0]);
    ma_msg.markers[1] = comMarkerSet(com, 1, "odom");
    com[2] = 0;
    ma_msg.markers[2] = comMarkerSet(com, 2, "odom");
    ma_pub.publish(ma_msg);

    //motionCstr.computeTorque(solver.alphaDVec(), solver.lambdaVec());
    //auto dof_torque = vectorToDof(mb, motionCstr.torque());
    //cout << "torques======" << endl;
    //for (int j = 0; j < dof_torque.size(); j++)
    //{
    //  cout << mb.joint(j).name() << ":";
    //  for (auto&& t : dof_torque[j]) cout << t << ", "; cout << endl;
    //}
    
    //cout << "lambdas======" << endl;
    //cout << solver.lambdaVec() << endl;

    auto stamp = ros::Time::now();
    js_array[i].header.stamp = stamp;
    js_pub.publish(js_array[i]);
    broadcastRoot(root_pose_array[i]);
    loop.sleep();
  }

  ros::Duration(1).sleep();

  while(ros::ok())
  {
    ROS_INFO("replay motion");
    for (auto&& m : ma_msg.markers) m.header.stamp = ros::Time::now();
    ma_pub.publish(ma_msg);
    for (int i = 0; i < mbc_array.size(); i++)
    {
      auto com = computeCoM(mb, mbc_array[i]);
      ma_msg.markers[1] = comMarkerSet(com, 1, "odom");
      com[2] = 0;
      ma_msg.markers[2] = comMarkerSet(com, 2, "odom");
      ma_pub.publish(ma_msg);

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
