/**
 * @file workspace_representation.cpp
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/ros/conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_types.h>
#include <RBDyn/FK.h>
#include "workspace_representation/preparation.h"
#include <SpaceVecAlg/SpaceVecAlg>
#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Object/S_Box.h>
#include <sch/CD/CD_Pair.h>
#include <random>
#include <chrono>

using namespace std;
using namespace Eigen;
using namespace ros;
using namespace pcl;
using namespace rbd;
using namespace sva;
using namespace mc_rbdyn_urdf;
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
  if (!init(mb, limits)) return -1;
  MultiBodyConfig mbc(mb);
  mbc.zero(mb);
  // gravity
  mbc.gravity = Vector3d(0, 0, 9.81);

  forwardKinematics(mb, mbc);

  vector<vector<vector<double>>> q_database;// std::map

  auto start_time = chrono::system_clock::now();
  random_device rnd;
  mt19937 mt(rnd());
  uniform_real_distribution<> r_rand(-150.0, +150.0);
  int max_trial = 100000;
  q_database.reserve(max_trial);
  PointCloud<PointXYZ> cloud;
  for (int i = 0; i < max_trial; i++)
  {
    //for (auto&& joint : mb.joints())
    //{
    //  if (joint.name() == "Root") continue;
    //  if (joint.type() != Joint::Type::Rev) continue;
    //  vector<double> qj;
    //  for (int j = 0; j < joint.dof(); j++)
    //  {
    //    double x = r_rand(mt);
    //    qj.push_back(x);
    //  }
    //  mbc.q[mb.jointIndexByName(joint.name())] = qj;
    //}
    for (int j = 1; j < mbc.q.size(); j++)
    {
      if (mb.joint(j).type() != Joint::Type::Rev) continue;
      for (int k = 0; k < mbc.q[j].size(); k++)
      {
        mbc.q[j][k] = r_rand(mt);
      }
    }
    forwardKinematics(mb, mbc);
    q_database.push_back(mbc.q);
    auto p = mbc.bodyPosW[mb.bodyIndexByName("l_sole")].translation();
    cloud.push_back(PointXYZ(p[0], p[1], p[2]));
  }
  auto end_time = chrono::system_clock::now();
  double elapsed = chrono::duration_cast<chrono::milliseconds>(end_time-start_time).count();
  cout << "finished " << max_trial << " number calculation" << endl;
  cout << elapsed << " ms was elapsed" << endl;

  Publisher point_pub = nh.advertise<sensor_msgs::PointCloud2>("point_ws", 1);
  ros::Rate loop(100);
  JointState js_msg;
  auto mbczero = mbc;
  mbczero.zero(mb);
  jointStateFromMBC(mb, mbczero, js_msg);
  while (ros::ok())
  {
    broadcastRoot(mbc.bodyPosW[mb.bodyIndexByName("base_link")]);
    std_msgs::Header h;
    h.stamp = ros::Time::now();
    h.frame_id = "odom";
    cloud.header = pcl_conversions::toPCL(h);
    point_pub.publish(cloud);
    js_msg.header.stamp = ros::Time::now();
    js_pub.publish(js_msg);
    loop.sleep();
  }

  return 0;
}
