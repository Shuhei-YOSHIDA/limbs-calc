#pragma once
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

void JointStateFromMBC(rbd::MultiBody mb, rbd::MultiBodyConfig mbc, sensor_msgs::JointState& msg)
{
    int count = 0;
    for (auto itr = mbc.q.begin(); itr != mbc.q.end(); ++itr) {
        if (mb.joint(count).type() == rbd::Joint::Type::Rev ||
            mb.joint(count).type() == rbd::Joint::Type::Prism) {// 1dof joint
            msg.name.push_back(mb.joint(count).name());
            msg.position.push_back(*(itr->begin()));
        } 
        count++;
    }
    msg.header.stamp = ros::Time::now();
}

void MarkerArrayFromMBC(rbd::MultiBody mb, rbd::MultiBodyConfig mbc, 
                        visualization_msgs::MarkerArray& msg, 
                        std::string frame_id = "base_link")
{
    int count = 0;
    auto stamp = ros::Time::now();
    for (auto itr = mbc.bodyPosW.begin(); itr != mbc.bodyPosW.end(); ++itr) {
        visualization_msgs::Marker mrk;
        mrk.header.stamp = stamp;
        mrk.header.frame_id = frame_id;
        mrk.id = count;
        mrk.text = mb.body(count).name();
        mrk.type = visualization_msgs::Marker::ARROW;
        mrk.pose.position.x = itr->translation()(0);
        mrk.pose.position.y = itr->translation()(1);
        mrk.pose.position.z = itr->translation()(2);
        auto q = Eigen::Quaterniond(itr->rotation());
        mrk.pose.orientation.x = q.x();
        mrk.pose.orientation.y = q.y();
        mrk.pose.orientation.z = q.z();
        mrk.pose.orientation.w = -q.w();//Consistency for TF
        mrk.scale.x = 0.1; mrk.scale.y = 0.01; mrk.scale.z = 0.01;
        mrk.color.r = 0.0; mrk.color.g = 0.5; mrk.color.b = 0.0; mrk.color.a = 0.5; 
        msg.markers.push_back(mrk);

        count++;
    }
}

void MarkerSet(sva::PTransformd x, visualization_msgs::Marker& msg,
               int id_count = 0, 
               std::string frame_id = "base_link")
{
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.id = id_count;
    msg.type = visualization_msgs::Marker::ARROW;
    msg.pose.position.x = x.translation()(0);
    msg.pose.position.y = x.translation()(1);
    msg.pose.position.z = x.translation()(2);
    auto q = Eigen::Quaterniond(x.rotation());
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = -q.w();//Consistency for TF
    msg.scale.x = 0.2; msg.scale.y = 0.02; msg.scale.z = 0.02;
    msg.color.r = 0.5; msg.color.g = 0.0; msg.color.b = 0.0; msg.color.a = 0.5; 
    
}
