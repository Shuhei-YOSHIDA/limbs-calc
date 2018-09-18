/**
 * @file preparation.h
 */
#pragma once
#include <ros/ros.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <mc_rbdyn_urdf/urdf.h>

using namespace std;
using namespace rbd;
using namespace mc_rbdyn_urdf;

void init(MultiBody& mb)
{
  // Read ros parameter and prepair RobotConfiguration class
  string urdf_string;
  ros::NodeHandle nh;
  if (!nh.getParam("robot_description", urdf_string))
  {
    ROS_ERROR("Urdf is not read: RobotConfiguration::RobotConfiguration()");
    ros::shutdown();
    return;
  }
  bool fixed = false;
  vector<string> filteredLinksIn = {};
  bool transformInertia = true;
  string baseLinkIn = "base_link";
  auto urdf_parser_res = rbdyn_from_urdf(urdf_string, fixed, filteredLinksIn, transformInertia, baseLinkIn);
  mb = urdf_parser_res.mb;
}


