/**
 * @file InvDyn.cpp
 * @brief sandbox for inverse dynamics
 */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/ID.h>
#include <mc_rbdyn_urdf/urdf.h>

using namespace std;
using namespace mc_rbdyn_urdf;

ros::Publisher G_a_pub;
rbd::MultiBody G_mb;
rbd::Jacobian G_jacobian;

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
  ros::init(argc, argv, "invdyn");
  ros::NodeHandle n;

  URDFParserResult res;

  string urdf_string;
  if (n.getParam("robot_description", urdf_string)) {
      bool fixed = true;
      vector<string> filteredLinksIn = {};
      bool transformInertia = true;
      string baseLinkIn = "base_link";
      res = rbdyn_from_urdf(urdf_string, fixed, filteredLinksIn, transformInertia, baseLinkIn);
  }
  else {
      cerr << "urdf was not loaded from parameter " << endl;
      return -1;
  }

  // inverse dynamics sample

  return 0;
}
