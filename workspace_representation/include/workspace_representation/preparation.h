/**
 * @file preparation.h
 */
#pragma once
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <mc_rbdyn_urdf/urdf.h>

using namespace std;
using namespace rbd;
using namespace sva;
using namespace mc_rbdyn_urdf;
using namespace sensor_msgs;
using namespace visualization_msgs;

bool init(MultiBody& mb, Limits& limits, string base_link_name="base_link", bool isFixed=false)
{
  // Read ros parameter and prepair RobotConfiguration class
  string urdf_string;
  ros::NodeHandle nh;
  if (!nh.getParam("robot_description", urdf_string))
  {
    ROS_ERROR("Urdf is not read: RobotConfiguration::RobotConfiguration()");
    return false;
  }
  vector<string> filteredLinksIn = {};
  bool transformInertia = true;
  auto urdf_parser_res = rbdyn_from_urdf(urdf_string, isFixed, filteredLinksIn, transformInertia, base_link_name);
  mb = urdf_parser_res.mb;
  limits = urdf_parser_res.limits;

  return true;
}

bool init(MultiBody& mb)
{
  Limits limits;
  return init(mb, limits);
}

void jointStateFromMBC(MultiBody mb, MultiBodyConfig mbc,
                       JointState& msg)
{
  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();

  int count = 0;
  for (auto itr = mbc.q.begin(); itr != mbc.q.end(); ++itr)
  {
    if (mb.joint(count).type() == rbd::Joint::Type::Rev ||
      mb.joint(count).type() == rbd::Joint::Type::Prism) // 1dof joint
    {
      msg.name.push_back(mb.joint(count).name());
      msg.position.push_back(*(itr->begin()));
    }
    count++;
  }
  msg.header.stamp = ros::Time::now();
}

void markerSet(PTransformd marker_pose, Marker& msg,
               int marker_id, string frame_id)
{
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  msg.id = marker_id;
  msg.type = Marker::ARROW;
  tf::pointEigenToMsg(marker_pose.translation(), msg.pose.position);
  auto q = Eigen::Quaterniond(marker_pose.rotation());
  q.w() = -q.w();//Consistency for TF
  tf::quaternionEigenToMsg(q, msg.pose.orientation);
  msg.scale.x = 0.2; msg.scale.y = 0.02; msg.scale.z = 0.02;
  msg.color.r = 0.5; msg.color.g = 0.0; msg.color.b = 0.0; msg.color.a = 1.0;
}

Marker comMarkerSet(Vector3d com, int marker_id, string frame_id)
{
  Marker msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  msg.id = marker_id;
  msg.type = Marker::SPHERE;
  tf::pointEigenToMsg(com, msg.pose.position);
  msg.scale.x = 0.1; msg.scale.y = 0.1; msg.scale.z = 0.1;
  msg.color.r = 0.0; msg.color.g = 0.3; msg.color.b = 0.9; msg.color.a = 1.0;

  return msg;
}

void broadcastRoot(PTransformd root_pose, string root_id="odom", string base_link_id="base_link")
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = root_id;
  transformStamped.child_frame_id = base_link_id;
  auto tra = root_pose.translation();
  transformStamped.transform.translation.x = tra[0];
  transformStamped.transform.translation.y = tra[1];
  transformStamped.transform.translation.z = tra[2];
  auto q = Eigen::Quaterniond(root_pose.rotation());
  q.w() = -q.w();
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

// sch object is necessary for collision calc
MultiBody makeEnv()
{
  MultiBodyGraph mbg;

  double mass = 1.0;
  Matrix3d I = Matrix3d::Identity();
  Vector3d h = Vector3d::Zero();

  RBInertiad rbi(mass, h, I);

  Body b0(rbi, "A");

  mbg.addBody(b0);

  bool fixed = true;
  MultiBody mb = mbg.makeMultiBody("A", fixed);

  return mb;
}
