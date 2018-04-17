/**
 * @file Jacobian.cpp
 * @brief sandbox for jacobian
 */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Jacobian.h>
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

// 可操作楕円体を表示するマーカーを返す
void calcOperationability(rbd::MultiBody mb, sensor_msgs::JointState jmsg,
                          rbd::Jacobian jac,
                          visualization_msgs::MarkerArray &amsg)
{
    amsg.markers.clear();
    rbd::MultiBodyConfig mbc;
    mbcFromJointState(mb, jmsg, mbc);
    rbd::forwardKinematics(mb, mbc);
    rbd::forwardVelocity(mb, mbc);

    auto jd = jac.jacobian(mb, mbc);
    cout << "dense matrix" << endl;
    // 可操作度
    cout << "operationability " << sqrt((jd*jd.transpose()).determinant()) << endl;
    cout << jd << endl;

    // 可操作性楕円
    {
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(jd, Eigen::ComputeThinU|Eigen::ComputeThinV);
      Eigen::MatrixXd s = svd.singularValues();
      cout << "singluar values" << endl << s << endl;
      //Eigen::MatrixXd sd = s.asDiagonal();
      //cout << "singular values as diagonal matrix" << endl << sd << endl;
      cout << "left singular vector: U" << endl << svd.matrixU() << endl;
      cout << "right singular vector: V" << endl << svd.matrixV() << endl;
    }

    visualization_msgs::Marker m;
    {
      Eigen::MatrixXd jd_l = jd.block(3, 0, 3, jd.cols());
      cout << "linear part" << endl;
      cout << "operationability " << sqrt((jd_l*jd_l.transpose()).determinant()) << endl;
      cout << jd_l << endl;
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(jd_l, Eigen::ComputeThinU|Eigen::ComputeThinV);
      Eigen::MatrixXd s = svd.singularValues();
      cout << "singluar values" << endl << s << endl;
      cout << "left singular vector: U" << endl << svd.matrixU() << endl;
      cout << "right singular vector: V" << endl << svd.matrixV() << endl;

      m.header.stamp = ros::Time::now();
      m.header.frame_id = "base_link";
      m.id = 0;
      m.type = visualization_msgs::Marker::SPHERE;
      auto trans = mbc.bodyPosW[mb.bodyIndexByName("r_wrist")];
      m.pose.position.x = trans.translation()[0];
      m.pose.position.y = trans.translation()[1];
      m.pose.position.z = trans.translation()[2];
      auto q = Eigen::Quaterniond(Eigen::Matrix3d(svd.matrixU()));
      m.pose.orientation.w = q.w() * -1;
      m.pose.orientation.x = q.x();
      m.pose.orientation.y = q.y();
      m.pose.orientation.z = q.z();
      m.scale.x = s(0, 0);
      m.scale.y = s(1, 0);
      m.scale.z = s(2, 0);
      m.color.r = 0.5; m.color.g = 0.3; m.color.b = 0.1; m.color.a = 0.7;
      amsg.markers.push_back(m);
    }

    {
      Eigen::MatrixXd jd_r = jd.block(0, 0, 3, jd.cols());
      cout << "rotation part" << endl;
      cout << "operationability " << sqrt((jd_r*jd_r.transpose()).determinant()) << endl;
      cout << jd_r << endl;
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(jd_r, Eigen::ComputeThinU|Eigen::ComputeThinV);
      Eigen::MatrixXd s = svd.singularValues();
      cout << "singluar values" << endl << s << endl;
      m.id = 1;
      m.type = visualization_msgs::Marker::SPHERE;
      auto trans = mbc.bodyPosW[mb.bodyIndexByName("r_wrist")];
      m.pose.position.x = trans.translation()[0];
      m.pose.position.y = trans.translation()[1];
      m.pose.position.z = trans.translation()[2];
      auto q = Eigen::Quaterniond(Eigen::Matrix3d(svd.matrixU()));
      m.pose.orientation.w = q.w() * -1;
      m.pose.orientation.x = q.x();
      m.pose.orientation.y = q.y();
      m.pose.orientation.z = q.z();
      m.scale.x = s(0, 0);
      m.scale.y = s(1, 0);
      m.scale.z = s(2, 0);
      m.color.r = 0.1; m.color.g = 0.3; m.color.b = 0.6; m.color.a = 0.7;
      amsg.markers.push_back(m);
    }

    {
      Eigen::MatrixXd js;
      js = Eigen::MatrixXd::Zero(6, mb.nrDof());
      jac.fullJacobian(mb, jd, js);
      cout << "sparse matrix" << endl;
      cout << "operationability " << sqrt((js*js.transpose()).determinant()) << endl;
      //cout << js << endl;
    }
}

void jsCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  visualization_msgs::MarkerArray amsg;
  calcOperationability(G_mb, *msg, G_jacobian, amsg);
  G_a_pub.publish(amsg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sample1");
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

  G_mb = res.mb;

  // Jacobian class
  string bodyName = "r_wrist"; // Set name properly
  G_jacobian = rbd::Jacobian(G_mb, bodyName);

  G_a_pub = n.advertise<visualization_msgs::MarkerArray>("markers", 1);
  ros::Subscriber j_sub = n.subscribe("joint_states", 1, jsCallback);

  // subscrive JointState and publish information of jacobian
  ros::spin();

  return 0;
}
