/**
 * @file Simulation.cpp
 * @brief sandbox for dynamic simulation
 */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/FD.h>
#include <RBDyn/EulerIntegration.h>
#include <mc_rbdyn_urdf/urdf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;
using namespace mc_rbdyn_urdf;
using namespace Eigen;

rbd::MultiBody G_mb;

void jointStateFromMBC(rbd::MultiBody mb, rbd::MultiBodyConfig mbc,
                       sensor_msgs::JointState& msg)
{
  auto jIdxMap = mb.jointIndexByName();
  for (auto itr = jIdxMap.begin(); itr != jIdxMap.end(); ++itr) {
    if (mb.joint(itr->second).dof() == 1) {
      msg.name.push_back(itr->first);
      msg.position.push_back(mbc.q[itr->second][0]);
    }
  }
  msg.header.stamp = ros::Time::now();
}

void mbcFromJointState(rbd::MultiBody mb, sensor_msgs::JointState msg,
                       rbd::MultiBodyConfig& mbc)
{
  rbd::MultiBodyConfig mbc_(mb);
  auto jIdxMap = mb.jointIndexByName();
  for (int i = 0; i < msg.name.size(); i++) {
    string name = msg.name[i];
    auto nItr = jIdxMap.find(name);
    if (nItr == jIdxMap.end()) continue;
    mbc_.q[nItr->second][0] = msg.position[i];
  }

  mbc = mbc_;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulation");
  ros::NodeHandle n;

  URDFParserResult res;

  string urdf_string;
  if (n.getParam("robot_description", urdf_string)) {
      bool fixed = false;
      vector<string> filteredLinksIn = {};
      bool transformInertia = true;
      string baseLinkIn = "base_link";
      res = rbdyn_from_urdf(urdf_string, fixed, filteredLinksIn, transformInertia, baseLinkIn);
  }
  else {
      cerr << "urdf was not loaded from parameter " << endl;
      return -1;
  }

  G_mb = res.mb;
  auto mbcEI = rbd::MultiBodyConfig(G_mb);
  mbcEI.zero(G_mb);
  mbcEI.gravity = Vector3d(0, 0, 9.81);
  int iterations = 1000;
  double dt = 0.005;
  rbd::ForwardDynamics FD(G_mb);

  double slow_factor = 0.1;
  ros::Rate loop(1./dt * slow_factor);
  ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  tf2_ros::TransformBroadcaster br;

  while(ros::ok())
  {
    auto mbc = mbcEI;;
    for (int i = 0; i < iterations; i++)
    {
      rbd::forwardKinematics(G_mb, mbc);
      rbd::forwardVelocity(G_mb, mbc);
      FD.forwardDynamics(G_mb, mbc);
      rbd::eulerIntegration(G_mb, mbc, dt);

      auto stamp = ros::Time::now();
      sensor_msgs::JointState js_msg;
      jointStateFromMBC(G_mb, mbc, js_msg);
      js_msg.header.stamp = stamp;
      geometry_msgs::TransformStamped tfs;
      tfs.header.frame_id = "world";
      tfs.header.stamp = stamp;
      tfs.child_frame_id = "base_link";
      auto p = mbc.bodyPosW[G_mb.bodyIndexByName("base_link")];
      tfs.transform.translation.x = p.translation()[0];
      tfs.transform.translation.y = p.translation()[1];
      tfs.transform.translation.z = p.translation()[2];
      auto q = Quaterniond(p.rotation());
      tfs.transform.rotation.x = q.x();
      tfs.transform.rotation.y = q.y();
      tfs.transform.rotation.z = q.z();
      tfs.transform.rotation.w = q.w() * -1;

      js_pub.publish(js_msg);
      br.sendTransform(tfs);

      loop.sleep();
    }
    ROS_INFO("rebirth");
  }

  return 0;
}
