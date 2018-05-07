/**
 * @file ForDyn.cpp
 * @brief sandbox for forward dynamics
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
#include <mc_rbdyn_urdf/urdf.h>

using namespace std;
using namespace mc_rbdyn_urdf;
using namespace Eigen;

ros::Publisher G_a_pub;
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

void jsCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  rbd::MultiBodyConfig mbc(G_mb);
  mbcFromJointState(G_mb, *msg, mbc);
  rbd::forwardKinematics(G_mb, mbc);
  rbd::forwardVelocity(G_mb, mbc);
  //cout << "gravity " << mbc.gravity << endl;
  mbc.gravity = Vector3d(0, 0, -9.81);
  rbd::ForwardDynamics FD(G_mb);
  FD.forwardDynamics(G_mb, mbc);
  cout << "alphaD-- " << endl;;
  //for (auto&& var : mbc.jointTorque)
  for (int i = 0; i < mbc.alphaD.size(); i++)
  {
    auto var = mbc.alphaD[i];
    cout << G_mb.joint(i).name() << ":";
    for (auto && var2 : var) cout << var2 <<", ";
    cout << endl;
  }
  cout << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "invdyn");
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

  // inverse dynamics sample
  ros::Subscriber j_sub = n.subscribe("joint_states", 1, jsCallback);
  ros::spin();

  return 0;
}
